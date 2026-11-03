import math
import os
import struct
from pathlib import Path

import pytest
import yaml

from argus import profile as profiles
from argus.__main__ import main
from argus.setup.importer import ANSWERS_HINT, build_profile
from argus.setup.urdf import UrdfError, read_urdf

URDF = """<?xml version="1.0"?>
<robot name="test_rover">
  <link name="base_footprint"/>
  <joint name="foot" type="fixed">
    <parent link="base_footprint"/><child link="base_link"/><origin xyz="0 0 0.05"/>
  </joint>
  <link name="base_link">
    <collision><origin xyz="0 0 0.1"/><geometry><box size="0.6 0.4 0.2"/></geometry></collision>
  </link>
  <link name="wheel_left">
    <collision><geometry><cylinder radius="0.05" length="0.04"/></geometry></collision>
  </link>
  <joint name="wl" type="continuous">
    <parent link="base_link"/><child link="wheel_left"/>
    <origin xyz="0 0.22 0" rpy="-1.5708 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <link name="mast">
    <collision><origin xyz="0 0 0.1"/><geometry><mesh filename="mast.stl" scale="1 1 2"/></geometry></collision>
  </link>
  <joint name="m" type="fixed">
    <parent link="base_link"/><child link="mast"/><origin xyz="-0.2 0 0.2" rpy="0 0 1.5708"/>
  </joint>
  <link name="laser"/>
  <joint name="l" type="fixed">
    <parent link="mast"/><child link="laser"/><origin xyz="0 0 0.3"/>
  </joint>
  <gazebo reference="laser">
    <sensor name="front_scan" type="gpu_lidar">
      <update_rate>12</update_rate><topic>scan</topic>
      <lidar><scan><horizontal><samples>360</samples><min_angle>-1.5708</min_angle><max_angle>1.5708</max_angle></horizontal></scan>
      <range><min>0.1</min><max>8.0</max></range></lidar>
    </sensor>
  </gazebo>
</robot>
"""

ANSWERS = yaml.safe_load(ANSWERS_HINT)


def _stl(path: Path, lo, hi):
    """A binary STL box from lo to hi (two triangles is enough for bounds)."""
    tris = [((lo[0], lo[1], lo[2]), (hi[0], lo[1], lo[2]), (hi[0], hi[1], hi[2])),
            ((lo[0], lo[1], lo[2]), (lo[0], hi[1], hi[2]), (hi[0], hi[1], hi[2]))]
    data = b"\0" * 80 + struct.pack("<I", len(tris))
    for t in tris:
        data += struct.pack("<3f", 0, 0, 0) + b"".join(struct.pack("<3f", *v) for v in t) + b"\0\0"
    path.write_bytes(data)


@pytest.fixture
def urdf(tmp_path):
    _stl(tmp_path / "mast.stl", (-0.05, -0.02, 0.0), (0.05, 0.02, 0.1))
    p = tmp_path / "rover.urdf"
    p.write_text(URDF)
    return p


def test_shapes_frames_and_sensors_come_from_the_urdf(urdf):
    body = read_urdf(urdf)
    assert body.base == "base_link"
    assert set(body.frames) == {"base_footprint", "base_link", "wheel_left", "mast", "laser"}
    assert body.frames["base_footprint"][2, 3] == pytest.approx(-0.05)
    assert body.frames["laser"][:3, 3] == pytest.approx([-0.2, 0.0, 0.5])

    box, wheel, mast = body.shapes
    assert box["pose"]["xyz_m"] == pytest.approx([0, 0, 0.1])
    assert wheel["pose"]["xyz_m"] == pytest.approx([0, 0.22, 0]) and wheel["pose"]["rpy_rad"][0] == pytest.approx(-1.5708)
    # Mesh: bounds scaled by (1, 1, 2), centred in the mesh frame, then the link turned 90 deg.
    assert mast["type"] == "box" and mast["size_m"] == pytest.approx([0.1, 0.04, 0.2])
    assert mast["pose"]["xyz_m"] == pytest.approx([-0.2, 0.0, 0.4])
    assert mast["pose"]["rpy_rad"][2] == pytest.approx(1.5708)

    (scan,) = body.sensors
    assert (scan.kind, scan.rate_hz, scan.topic, scan.range_m) == ("range_2d", 12.0, "scan", (0.1, 8.0))
    assert scan.fov_deg[0] == pytest.approx(180.0, abs=0.01)
    assert scan.angular_resolution_deg == pytest.approx(180.0 / 359, abs=1e-3)  # 360 rays over 180 deg


def test_import_with_answers_gives_a_valid_profile_with_provenance(urdf):
    doc, problems, _ = build_profile(urdf, ANSWERS)
    assert problems == []
    assert profiles.problems(doc) == []
    assert doc["geometry"]["ground_z_m"] == pytest.approx(-0.05)  # wheel bottom
    sensors = {s["id"]: s for s in doc["sensors"]}
    assert sensors["front_scan"]["source"]["endpoint"] == "/scan"
    assert sensors["front_scan"]["frame"] == "laser"
    assert doc["provenance"]["geometry.shapes"] == "cad"
    assert doc["provenance"]["sensors.front_scan"] == "cad"
    assert doc["provenance"]["motion"] == "declared"
    assert profiles.derive(doc).safety_ready

    # The answers override a URDF sensor by id.
    answers = dict(ANSWERS, sensors=ANSWERS["sensors"] + [{"id": "front_scan", "rate_hz": 15}])
    doc, problems, _ = build_profile(urdf, answers)
    assert problems == []
    assert {s["id"]: s for s in doc["sensors"]}["front_scan"]["rate_hz"] == 15
    assert doc["provenance"]["sensors.front_scan"] == "declared"


def test_missing_answers_are_listed_not_guessed(urdf):
    doc, problems, _ = build_profile(urdf, {})
    text = "\n".join(problems)
    for section in ("motion", "control", "failsafe", "compute"):
        assert section in text
    assert "odometry" in text
    bad = dict(ANSWERS, sensors=[{"id": "odom", "kind": "wheel_odom", "frame": "nowhere",
                                  "source": {"transport": "ros2", "endpoint": "/odom"}, "rate_hz": 50}])
    _, problems, _ = build_profile(urdf, bad)
    assert any("no link 'nowhere'" in p for p in problems)


def test_unresolvable_mesh_and_bad_files_say_why(tmp_path):
    p = tmp_path / "r.urdf"
    p.write_text(URDF.replace("mast.stl", "package://nowhere_pkg/m.stl"))
    with pytest.raises(UrdfError, match="--package nowhere_pkg="):
        read_urdf(p)
    p.write_text("<robot><link name='a'/></robot>")
    with pytest.raises(UrdfError, match="no collision geometry"):
        read_urdf(p)


def test_cli_writes_the_profile_and_report(urdf, tmp_path, capsys):
    answers = tmp_path / "answers.yaml"
    answers.write_text(ANSWERS_HINT)
    out = tmp_path / "profile.yaml"
    assert main(["setup", "import", str(urdf), "-a", str(answers), "-o", str(out)]) == 0
    report = capsys.readouterr().out
    assert "Sensing" in report and "Ready to drive: yes" in report
    assert profiles.problems(profiles.load(out)) == []
    assert main(["setup", "report", str(out)]) == 0
    assert main(["setup", "import", str(urdf), "-o", str(tmp_path / "x.yaml")]) == 1
    assert "setup template" in capsys.readouterr().err


TB3 = Path("/opt/ros/jazzy/share/nav2_minimal_tb3_sim/urdf/turtlebot3_waffle.urdf")


@pytest.mark.skipif(not TB3.is_file(), reason="TurtleBot3 description not installed")
def test_turtlebot3_waffle_imports():
    lidar = {"id": "lidar", "kind": "range_2d", "frame": "base_scan", "rate_hz": 5, "range_m": [0.12, 3.5],
             "source": {"transport": "ros2", "endpoint": "/scan"}, "fov_deg": {"h": 360, "v": 1}}
    doc, problems, _ = build_profile(TB3, dict(ANSWERS, sensors=ANSWERS["sensors"] + [lidar]))
    assert problems == []
    lo = profiles.derive(doc).collision.aabb_min
    assert doc["geometry"]["ground_z_m"] == pytest.approx(-0.010, abs=1e-3)
    assert math.isclose(lo[2], -0.010, abs_tol=0.01)


@pytest.mark.skipif(not TB3.is_file(), reason="TurtleBot3 description not installed")
def test_a_camera_on_an_optical_frame_looks_forward():
    # The Waffle's camera_depth_frame is rotated the ROS optical way (z forward, y down), as
    # depth images are stamped. Read as x forward it pointed at the floor to the robot's right,
    # and in Gazebo Argo saw phantom obstacles everywhere.
    answers = yaml.safe_load((Path(__file__).parent / "designs" / "turtlebot3_waffle_answers.yaml").read_text())
    doc, problems, notes = build_profile(TB3, answers)
    assert problems == []
    depth = next(s for s in doc["sensors"] if s["kind"] == "depth")
    pose = doc["geometry"]["frames"][depth["frame"]]
    assert depth["frame"] == "camera_depth_frame_x_forward"
    assert all(abs(a) < 0.01 for a in pose["rpy_rad"]), pose
    assert pose["xyz_m"] == doc["geometry"]["frames"]["camera_depth_frame"]["xyz_m"]
    assert any("optical frame" in n for n in notes)
    # Frames already looking along x are left alone.
    assert next(s for s in doc["sensors"] if s["kind"] == "range_2d")["frame"] == "base_scan"
