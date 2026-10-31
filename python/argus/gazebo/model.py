"""A Gazebo (SDF 1.9) model for a body profile, with physics that can prove something.

The body's collision shapes come from the profile. Profiles do not describe wheels, so a
drivetrain that fits the footprint is made up from the motion capabilities: two driven wheels
and two frictionless casters for bodies that steer by turning. The wheels are real joints with
ground friction and an effort limit, driven by Gazebo's drive system the way a motor controller
drives real wheels (a velocity servo that can only push as hard as its torque allows), so wheel
slip, inertia and too little torque all show up. Its odometry comes from the wheels and drifts
like real wheel odometry. The true pose goes out on a separate topic, for judging runs only.

Sensors are Gazebo's lidar and depth camera at the profile's frames, rates, ranges and fields
of view, with Gaussian noise. A robot with a real URDF should be simulated from that instead.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from xml.sax.saxutils import escape

# Assumed body density when the profile has no mass (kg/m^3; a hollow robot shell with parts).
DENSITY = 300.0
WHEEL_MASS_KG = 0.5
WHEEL_WIDTH_M = 0.04
RANGE_NOISE_M = 0.01
DEPTH_NOISE_M = 0.005


@dataclass
class Topics:
    """Gazebo topic names for one robot, mapped to ROS 2 names by the bridge."""

    model: str
    cmd_vel: str = ""
    odom: str = ""
    truth: str = ""
    sensors: dict[str, str] = field(default_factory=dict)  # sensor id -> gz topic
    kinds: dict[str, str] = field(default_factory=dict)  # sensor id -> profile kind


def _pose(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> str:
    return "<pose>" + " ".join(f"{v:.6g}" for v in (*xyz, *rpy)) + "</pose>"


def _shape_geometry(s: dict) -> str:
    t = s["type"]
    if t == "box":
        x, y, z = s["size_m"]
        return f"<box><size>{x:.6g} {y:.6g} {z:.6g}</size></box>"
    if t == "cylinder":
        return f"<cylinder><radius>{s['radius_m']:.6g}</radius><length>{s['length_m']:.6g}</length></cylinder>"
    if t == "sphere":
        return f"<sphere><radius>{s['radius_m']:.6g}</radius></sphere>"
    raise ValueError(f"geometry.shapes: unsupported type {t}")


def _extent(s: dict) -> tuple[tuple[float, float, float], tuple[float, float, float], float]:
    """Axis-aligned bounds (ignoring rotation) and volume of a shape."""
    c = s.get("pose", {}).get("xyz_m", [0.0, 0.0, 0.0])
    t = s["type"]
    if t == "box":
        h = [v / 2 for v in s["size_m"]]
        vol = s["size_m"][0] * s["size_m"][1] * s["size_m"][2]
    elif t == "cylinder":
        h = [s["radius_m"], s["radius_m"], s["length_m"] / 2]
        vol = math.pi * s["radius_m"] ** 2 * s["length_m"]
    else:
        h = [s["radius_m"]] * 3
        vol = 4 / 3 * math.pi * s["radius_m"] ** 3
    return tuple(c[i] - h[i] for i in range(3)), tuple(c[i] + h[i] for i in range(3)), vol


def _box_inertia(m: float, x: float, y: float, z: float) -> str:
    return (f"<inertia><ixx>{m * (y * y + z * z) / 12:.6g}</ixx><iyy>{m * (x * x + z * z) / 12:.6g}</iyy>"
            f"<izz>{m * (x * x + y * y) / 12:.6g}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>")


def _surface(mu: float) -> str:
    return (f"<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction>"
            "<contact><ode><kp>1e6</kp><kd>100</kd></ode></contact></surface>")


def _sensor(s: dict, frame: dict, topic: str) -> str:
    kind = s["kind"]
    fov = s.get("fov_deg", {})
    h = math.radians(fov.get("h", 90.0))
    v = math.radians(fov.get("v", 1.0))
    rmin, rmax = s.get("range_m", [0.1, 10.0])
    pose = _pose(frame.get("xyz_m", [0, 0, 0]), frame.get("rpy_rad", [0, 0, 0]))
    head = (f'<sensor name="{escape(s["id"])}" type="{{t}}">{pose}<topic>{escape(topic)}</topic>'
            f'<gz_frame_id>{escape(s["id"])}</gz_frame_id><update_rate>{s.get("rate_hz", 10)}</update_rate>'
            "<always_on>true</always_on>")
    if kind in ("range_2d", "range_3d"):
        h_samples = max(2, int(round(math.degrees(h) / 0.5)) + 1)
        v_samples = 1 if kind == "range_2d" else max(2, int(round(math.degrees(v) / 2.0)) + 1)
        vertical = "" if kind == "range_2d" else (
            f"<vertical><samples>{v_samples}</samples><min_angle>{-v / 2:.6g}</min_angle>"
            f"<max_angle>{v / 2:.6g}</max_angle></vertical>")
        return (head.format(t="gpu_lidar")
                + f"<lidar><scan><horizontal><samples>{h_samples}</samples><min_angle>{-h / 2:.6g}</min_angle>"
                f"<max_angle>{h / 2:.6g}</max_angle></horizontal>{vertical}</scan>"
                f"<range><min>{rmin}</min><max>{rmax}</max><resolution>0.01</resolution></range>"
                f"<noise><type>gaussian</type><mean>0</mean><stddev>{RANGE_NOISE_M}</stddev></noise>"
                "</lidar></sensor>")
    if kind == "depth":
        width = 160
        height = max(2, int(round(width * math.tan(v / 2) / math.tan(h / 2))))
        return (head.format(t="depth_camera")
                + f"<camera><horizontal_fov>{h:.6g}</horizontal_fov><image><width>{width}</width>"
                f"<height>{height}</height><format>R_FLOAT32</format></image>"
                f"<clip><near>{rmin}</near><far>{rmax}</far></clip>"
                f"<noise><type>gaussian</type><mean>0</mean><stddev>{DEPTH_NOISE_M}</stddev></noise>"
                "</camera></sensor>")
    return ""


def model_sdf(doc: dict, name: str = "robot", pose=(0.0, 0.0, 0.0, 0.0)) -> tuple[str, Topics]:
    """SDF for the body in `doc`, placed at (x, y, z, yaw), and the topics it uses."""
    motion = doc["motion"]
    if motion["space"] != "planar":
        raise ValueError("Gazebo models for aerial bodies are not supported yet")
    if motion.get("can_strafe") or motion.get("min_turn_radius_m", 0.0) > 0.0:
        raise ValueError("Gazebo models cover bodies that turn in place without strafing so far")

    shapes = doc["geometry"]["shapes"]
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    volume = 0.0
    for s in shapes:
        a, b, vol = _extent(s)
        lo = [min(lo[i], a[i]) for i in range(3)]
        hi = [max(hi[i], b[i]) for i in range(3)]
        volume += vol
    ground = doc["geometry"].get("ground_z_m", 0.0)
    mass = max(1.0, DENSITY * volume)
    size = [hi[i] - lo[i] for i in range(3)]
    centre = [(hi[i] + lo[i]) / 2 for i in range(3)]

    # Wheels inside the footprint, big enough to reach the ground under the body.
    radius = max(0.04, (lo[2] - ground) + 0.03)
    separation = max(0.1, size[1] - WHEEL_WIDTH_M - 0.02)
    caster = radius / 2
    caster_x = max(0.05, size[0] / 2 - caster - 0.02)
    accel = motion["limits"]["x"].get("accel_mps2", 1.0)
    decel = motion["limits"]["x"].get("decel_mps2", accel)
    vmax = max(abs(v) for v in motion["limits"]["x"]["velocity_mps"])
    wmax = motion["limits"]["yaw"]["rate_radps"]
    # Enough torque for the declared acceleration with some to spare, no more.
    effort = 1.5 * (mass + 2 * WHEEL_MASS_KG) * max(accel, decel) * radius / 2

    t = Topics(model=name, cmd_vel=f"/model/{name}/cmd_vel", odom=f"/model/{name}/odometry",
               truth=f"/model/{name}/truth")
    base = [f'<link name="base_link">',
            f"<inertial>{_pose(centre)}<mass>{mass:.6g}</mass>{_box_inertia(mass, *size)}</inertial>"]
    for i, s in enumerate(shapes):
        p = s.get("pose", {})
        geo = _shape_geometry(s)
        sp = _pose(p.get("xyz_m", [0, 0, 0]), p.get("rpy_rad", [0, 0, 0]))
        base.append(f'<collision name="body_{i}">{sp}<geometry>{geo}</geometry>{_surface(0.5)}</collision>')
        base.append(f'<visual name="body_{i}">{sp}<geometry>{geo}</geometry></visual>')
    for sign, label in ((1, "front"), (-1, "rear")):
        cp = _pose((sign * caster_x, 0.0, ground + caster))
        base.append(f'<collision name="caster_{label}">{cp}<geometry><sphere><radius>{caster:.6g}</radius>'
                    f"</sphere></geometry>{_surface(0.0)}</collision>")
    frames = doc["geometry"].get("frames", {})
    for s in doc.get("sensors", []):
        if s["kind"] in ("range_2d", "range_3d", "depth"):
            topic = f"/model/{name}/{s['id']}" + ("/image" if s["kind"] == "depth" else "")
            t.sensors[s["id"]] = topic
            t.kinds[s["id"]] = s["kind"]
            base.append(_sensor(s, frames.get(s["frame"], {}), topic))
    base.append("</link>")

    wheels = []
    for side, y in (("left", separation / 2), ("right", -separation / 2)):
        wheels.append(
            f'<link name="{side}_wheel">{_pose((0.0, y, ground + radius), (-math.pi / 2, 0.0, 0.0))}'
            f"<inertial><mass>{WHEEL_MASS_KG}</mass>"
            f"<inertia><ixx>{WHEEL_MASS_KG * (3 * radius ** 2 + WHEEL_WIDTH_M ** 2) / 12:.6g}</ixx>"
            f"<iyy>{WHEEL_MASS_KG * (3 * radius ** 2 + WHEEL_WIDTH_M ** 2) / 12:.6g}</iyy>"
            f"<izz>{WHEEL_MASS_KG * radius ** 2 / 2:.6g}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>"
            f"</inertial><collision name=\"tyre\"><geometry><cylinder><radius>{radius:.6g}</radius>"
            f"<length>{WHEEL_WIDTH_M}</length></cylinder></geometry>{_surface(1.0)}</collision>"
            f"<visual name=\"tyre\"><geometry><cylinder><radius>{radius:.6g}</radius>"
            f"<length>{WHEEL_WIDTH_M}</length></cylinder></geometry></visual></link>"
            f'<joint name="{side}_wheel_joint" type="revolute"><parent>base_link</parent>'
            f"<child>{side}_wheel</child><axis><xyz>0 0 1</xyz>"
            f"<limit><lower>-1e16</lower><upper>1e16</upper><effort>{effort:.6g}</effort>"
            f"<velocity>{1.2 * vmax / radius:.6g}</velocity></limit>"
            "<dynamics><damping>0.05</damping><friction>0.02</friction></dynamics></axis></joint>")

    plugins = (
        '<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">'
        "<left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint>"
        f"<wheel_separation>{separation:.6g}</wheel_separation><wheel_radius>{radius:.6g}</wheel_radius>"
        f"<max_linear_acceleration>{accel}</max_linear_acceleration>"
        f"<min_linear_acceleration>{-decel}</min_linear_acceleration>"
        f"<max_angular_acceleration>{motion['limits']['yaw'].get('accel_radps2', 2.0)}</max_angular_acceleration>"
        f"<max_velocity>{vmax}</max_velocity><max_angular_velocity>{wmax}</max_angular_velocity>"
        f"<topic>{t.cmd_vel}</topic><odom_topic>{t.odom}</odom_topic><odom_publish_frequency>50"
        "</odom_publish_frequency><frame_id>odom</frame_id><child_frame_id>base_link</child_frame_id>"
        "</plugin>"
        '<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">'
        f"<odom_topic>{t.truth}</odom_topic><odom_frame>world</odom_frame>"
        "<robot_base_frame>base_link</robot_base_frame><odom_publish_frequency>50</odom_publish_frequency>"
        "<dimensions>3</dimensions></plugin>")
    x, y, z, yaw = pose
    sdf = (f'<model name="{escape(name)}">{_pose((x, y, z), (0.0, 0.0, yaw))}'
           + "".join(base) + "".join(wheels) + plugins + "</model>")
    return sdf, t
