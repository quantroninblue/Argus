"""The user's own robot in Gazebo (hub plan H3): their Gazebo model as given, or their URDF
through Gazebo's conversion, with only what a run needs added and every addition listed.
Uses the TurtleBot3 Waffle that ships with ROS 2 Jazzy's Nav2 packages."""

import shutil
from pathlib import Path

import pytest

from argus.gazebo.design_model import DesignModelError, design_model
from argus.hub.designs import DesignStore, collect

SHARE = Path("/opt/ros/jazzy/share/nav2_minimal_tb3_sim")
ANSWERS = Path(__file__).parent / "designs" / "turtlebot3_waffle_answers.yaml"

pytestmark = pytest.mark.skipif(not (SHARE / "urdf").exists() or shutil.which("gz") is None or shutil.which("xacro") is None,
                                reason="needs ROS 2 Jazzy with nav2_minimal_tb3_sim and Gazebo, sourced")


def _design(tmp_path, gazebo_model: bool):
    store = DesignStore(tmp_path / "designs")
    v, _ = store.add(collect(SHARE / "urdf" / "turtlebot3_waffle.urdf", ANSWERS,
                             SHARE / "urdf" / "gz_waffle.sdf.xacro" if gazebo_model else None))
    assert v.runnable, [str(f) for f in v.findings]
    return v, store.profile(v)


def test_the_users_gazebo_model_keeps_its_drive_and_sensors(tmp_path):
    v, doc = _design(tmp_path, gazebo_model=True)
    dm = design_model(v.path, doc)
    assert (dm.topics.cmd_vel, dm.topics.odom) == ("/cmd_vel", "/odom")  # its own diff drive
    assert dm.topics.sensors["scan"] == "/scan" and dm.topics.sensors["imu"] == "/imu"
    # Its depth camera names no topic: Argus names one and says so; nothing else is added
    # but the true pose for judging.
    assert len(dm.added) == 2, dm.added
    assert any("depth" in a and "topic" in a for a in dm.added)
    assert "model://nav2_minimal_tb3_sim/" in dm.sdf and dm.resource_path.endswith("packages")


def test_a_urdf_alone_gets_a_drive_and_its_declared_sensors(tmp_path):
    v, doc = _design(tmp_path, gazebo_model=False)
    dm = design_model(v.path, doc)
    drive = next(a for a in dm.added if "differential drive" in a)
    assert "wheel_left_joint" in drive and "0.288 m" in drive and "0.033 m" in drive
    assert sum("does not simulate" in a for a in dm.added) == 3  # scan, depth, imu
    assert dm.topics.cmd_vel == "/model/robot/cmd_vel"
    # Nav2's URDF names visual meshes its package does not have; Gazebo refuses a model whose
    # meshes it cannot find, so those visuals are left out and listed.
    left_out = [a for a in dm.added if a.startswith("left out visual")]
    assert left_out and all("physics is unchanged" in a for a in left_out)
    assert "waffle_base.dae" not in dm.sdf


def test_a_drivetrain_argus_cannot_add_is_refused_with_what_to_do(tmp_path):
    v, doc = _design(tmp_path, gazebo_model=False)
    doc["motion"]["min_turn_radius_m"] = 0.5
    with pytest.raises(DesignModelError, match="add your drivetrain's Gazebo plugin"):
        design_model(v.path, doc)


def test_a_sensor_the_gazebo_model_puts_elsewhere_is_reported(tmp_path):
    # Nav2's Waffle model gives the depth sensor its link's offset twice, so Gazebo renders it
    # about 10 cm above the URDF's camera frame; the lidar is within 3 cm.
    from argus.gazebo.design_model import mount_findings

    v, doc = _design(tmp_path, gazebo_model=True)
    found = mount_findings(v.path, doc)
    assert len(found) == 1 and found[0].startswith("depth:") and "10 cm higher" in found[0]
    assert any(f.severity == "warning" and "10 cm higher" in f.text for f in v.findings)
