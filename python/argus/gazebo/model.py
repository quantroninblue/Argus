"""A Gazebo (SDF 1.9) model for a body profile, with physics that can prove something.

The body's collision shapes come from the profile. Profiles do not describe wheels, so a
drivetrain that fits the footprint is made up from the motion capabilities:

- turns in place, no strafing: two driven wheels and two frictionless casters;
- strafes: four mecanum wheels, each with friction only along its rollers' driving direction
  (the usual way to model rollers in Gazebo);
- has a minimum turn radius: two driven rear wheels and two steered front wheels, with the
  steering limit set from the turn radius.

The wheels are real joints with ground friction and an effort limit, driven by Gazebo's drive
systems the way a motor controller drives real wheels (a velocity servo that can only push as
hard as its torque allows), so wheel slip, inertia and too little torque all show up. Odometry
comes from the wheels and drifts like real wheel odometry. The true pose goes out on a separate topic, for judging runs only.

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
    drive: dict = field(default_factory=dict)  # the drivetrain that was made up for the body


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


def _inertia(m: float, ixx: float, iyy: float, izz: float) -> str:
    return (f"<inertial><mass>{m:.6g}</mass><inertia><ixx>{ixx:.6g}</ixx><iyy>{iyy:.6g}</iyy>"
            f"<izz>{izz:.6g}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>")


def _wheel(name: str, xyz, radius: float, surface: str, sphere: bool = False) -> str:
    """A wheel link with its axle along the body's y axis."""
    m = WHEEL_MASS_KG
    across = m * (3 * radius ** 2 + WHEEL_WIDTH_M ** 2) / 12
    cyl = f"<cylinder><radius>{radius:.6g}</radius><length>{WHEEL_WIDTH_M}</length></cylinder>"
    collision = f"<sphere><radius>{radius:.6g}</radius></sphere>" if sphere else cyl
    return (f'<link name="{name}">{_pose(xyz, (-math.pi / 2, 0.0, 0.0))}{_inertia(m, across, across, m * radius ** 2 / 2)}'
            f'<collision name="tyre"><geometry>{collision}</geometry>{surface}</collision>'
            f'<visual name="tyre_visual"><geometry>{cyl}</geometry></visual></link>')


def _joint(name: str, parent: str, child: str, effort: float, velocity: float,
           lower: float = -1e16, upper: float = 1e16, damping: float = 0.05) -> str:
    """A revolute joint about the child's z axis (the axle of a wheel link from `_wheel`)."""
    return (f'<joint name="{name}" type="revolute"><parent>{parent}</parent><child>{child}</child>'
            f"<axis><xyz>0 0 1</xyz><limit><lower>{lower:.6g}</lower><upper>{upper:.6g}</upper>"
            f"<effort>{effort:.6g}</effort><velocity>{velocity:.6g}</velocity></limit>"
            f"<dynamics><damping>{damping}</damping><friction>0.02</friction></dynamics></axis></joint>")


def _roller_surface(fdir: str) -> str:
    """Friction along `fdir` (in the body frame) only, so the rollers turn freely across it."""
    return (f'<surface><friction><ode><mu>1.0</mu><mu2>0.0</mu2><fdir1 gz:expressed_in="base_link">{fdir}'
            "</fdir1></ode></friction><contact><ode><kp>1e6</kp><kd>100</kd></ode></contact></surface>")


def _limits(lim: dict) -> str:
    """Velocity and acceleration limits for a Gazebo drive system from one profile axis."""
    lo, hi = lim["velocity_mps"]
    accel = lim.get("accel_mps2", 1.0)
    decel = lim.get("decel_mps2", accel)
    return (f"<min_velocity>{lo:.6g}</min_velocity><max_velocity>{hi:.6g}</max_velocity>"
            f"<min_acceleration>{-decel:.6g}</min_acceleration>"
            f"<max_acceleration>{max(accel, decel):.6g}</max_acceleration>")


def model_sdf(doc: dict, name: str = "robot", pose=(0.0, 0.0, 0.0, 0.0)) -> tuple[str, Topics]:
    """SDF for the body in `doc`, placed at (x, y, z, yaw), and the topics it uses."""
    motion = doc["motion"]
    if motion["space"] != "planar":
        raise ValueError("Gazebo models for aerial bodies are not supported yet")
    turn_radius = motion.get("min_turn_radius_m", 0.0)
    if motion.get("can_strafe"):
        drive = "mecanum"
    elif turn_radius > 0.0:
        drive = "ackermann"
    else:
        drive = "differential"

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
    wheelbase = max(0.1, size[0] - 2 * radius - 0.04)
    axle_z = ground + radius
    lx = motion["limits"]["x"]
    accel = lx.get("accel_mps2", 1.0)
    decel = lx.get("decel_mps2", accel)
    vmax = max(abs(v) for v in lx["velocity_mps"])
    if drive == "mecanum":
        vmax = max(vmax, max(abs(v) for v in motion["limits"]["y"]["velocity_mps"]))
    wheel_speed = 1.2 * vmax / radius
    driven = 4 if drive == "mecanum" else 2
    # Enough torque for the declared acceleration with some to spare, no more.
    effort = 1.5 * (mass + 4 * WHEEL_MASS_KG) * max(accel, decel) * radius / driven
    yaw = motion["limits"]["yaw"]

    t = Topics(model=name, cmd_vel=f"/model/{name}/cmd_vel", odom=f"/model/{name}/odometry",
               truth=f"/model/{name}/truth")
    base = ['<link name="base_link">',
            f"<inertial>{_pose(centre)}<mass>{mass:.6g}</mass>{_box_inertia(mass, *size)}</inertial>"]
    for i, s in enumerate(shapes):
        p = s.get("pose", {})
        geo = _shape_geometry(s)
        sp = _pose(p.get("xyz_m", [0, 0, 0]), p.get("rpy_rad", [0, 0, 0]))
        base.append(f'<collision name="body_{i}">{sp}<geometry>{geo}</geometry>{_surface(0.5)}</collision>')
        base.append(f'<visual name="body_{i}_visual">{sp}<geometry>{geo}</geometry></visual>')
    if drive == "differential":
        caster = radius / 2
        caster_x = max(0.05, size[0] / 2 - caster - 0.02)
        for sign, label in ((1, "front"), (-1, "rear")):
            cp = _pose((centre[0] + sign * caster_x, 0.0, ground + caster))
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

    parts = []
    common = (f"<topic>{t.cmd_vel}</topic><odom_topic>{t.odom}</odom_topic>"
              "<odom_publish_frequency>50</odom_publish_frequency><frame_id>odom</frame_id>"
              "<child_frame_id>base_link</child_frame_id>")
    if drive == "differential":
        for side, y in (("left", separation / 2), ("right", -separation / 2)):
            parts.append(_wheel(f"{side}_wheel", (centre[0], y, axle_z), radius, _surface(1.0)))
            parts.append(_joint(f"{side}_wheel_joint", "base_link", f"{side}_wheel", effort, wheel_speed))
        drive_plugin = (
            '<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">'
            "<left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint>"
            f"<wheel_separation>{separation:.6g}</wheel_separation><wheel_radius>{radius:.6g}</wheel_radius>"
            f"<max_linear_acceleration>{accel}</max_linear_acceleration>"
            f"<min_linear_acceleration>{-decel}</min_linear_acceleration>"
            f"<max_angular_acceleration>{yaw.get('accel_radps2', 2.0)}</max_angular_acceleration>"
            f"<max_velocity>{vmax}</max_velocity><max_angular_velocity>{yaw['rate_radps']}</max_angular_velocity>"
            + common + "</plugin>")
    elif drive == "mecanum":
        # Roller directions for an X layout seen from above (rollers on the ground form an O).
        wheels = (("front_left", 1, 1, "1 -1 0"), ("front_right", 1, -1, "1 1 0"),
                  ("back_left", -1, 1, "1 1 0"), ("back_right", -1, -1, "1 -1 0"))
        for label, sx, sy, fdir in wheels:
            xyz = (centre[0] + sx * wheelbase / 2, centre[1] + sy * separation / 2, axle_z)
            parts.append(_wheel(f"{label}_wheel", xyz, radius, _roller_surface(fdir), sphere=True))
            parts.append(_joint(f"{label}_joint", "base_link", f"{label}_wheel", effort, wheel_speed))
        drive_plugin = (
            '<plugin filename="gz-sim-mecanum-drive-system" name="gz::sim::systems::MecanumDrive">'
            "<front_left_joint>front_left_joint</front_left_joint>"
            "<front_right_joint>front_right_joint</front_right_joint>"
            "<back_left_joint>back_left_joint</back_left_joint><back_right_joint>back_right_joint</back_right_joint>"
            f"<wheel_separation>{separation:.6g}</wheel_separation><wheelbase>{wheelbase:.6g}</wheelbase>"
            f"<wheel_radius>{radius:.6g}</wheel_radius>" + _limits({"velocity_mps": [-vmax, vmax],
                                                                   "accel_mps2": accel, "decel_mps2": decel})
            + common + "</plugin>")
    else:
        # Rear axle driven, front wheels steered. The tightest turn the profile allows sets the
        # steering limit, measured at the rear axle.
        steer_limit = math.atan(wheelbase / turn_radius)
        rear_x = centre[0] - wheelbase / 2
        front_x = centre[0] + wheelbase / 2
        for side, y in (("left", separation / 2), ("right", -separation / 2)):
            parts.append(_wheel(f"rear_{side}_wheel", (rear_x, y, axle_z), radius, _surface(1.0)))
            parts.append(_joint(f"rear_{side}_wheel_joint", "base_link", f"rear_{side}_wheel", effort, wheel_speed))
            hub_mass = 0.2
            parts.append(f'<link name="front_{side}_steering">{_pose((front_x, y, axle_z))}'
                         f"{_inertia(hub_mass, 1e-3, 1e-3, 1e-3)}</link>")
            parts.append(_joint(f"front_{side}_steering_joint", "base_link", f"front_{side}_steering",
                                effort=5.0, velocity=2.0, lower=-steer_limit, upper=steer_limit, damping=0.5))
            parts.append(_wheel(f"front_{side}_wheel", (front_x, y, axle_z), radius, _surface(1.0)))
            parts.append(_joint(f"front_{side}_wheel_joint", f"front_{side}_steering", f"front_{side}_wheel",
                                effort=0.0, velocity=wheel_speed))
        drive_plugin = (
            '<plugin filename="gz-sim-ackermann-steering-system" name="gz::sim::systems::AckermannSteering">'
            "<left_joint>rear_left_wheel_joint</left_joint><right_joint>rear_right_wheel_joint</right_joint>"
            "<left_steering_joint>front_left_steering_joint</left_steering_joint>"
            "<right_steering_joint>front_right_steering_joint</right_steering_joint>"
            f"<wheel_separation>{separation:.6g}</wheel_separation><kingpin_width>{separation:.6g}</kingpin_width>"
            f"<wheel_base>{wheelbase:.6g}</wheel_base><steering_limit>{steer_limit:.6g}</steering_limit>"
            f"<wheel_radius>{radius:.6g}</wheel_radius>" + _limits(lx) + common + "</plugin>")

    truth = ('<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">'
             f"<odom_topic>{t.truth}</odom_topic><odom_frame>world</odom_frame>"
             "<robot_base_frame>base_link</robot_base_frame><odom_publish_frequency>50</odom_publish_frequency>"
             "<dimensions>3</dimensions></plugin>")
    t.drive = {"kind": drive, "mass_kg": round(mass, 3), "wheel_radius_m": round(radius, 4),
               "wheel_separation_m": round(separation, 4), "wheel_effort_nm": round(effort, 4),
               "wheel_speed_limit_radps": round(wheel_speed, 3)}
    if drive != "differential":
        t.drive["wheelbase_m"] = round(wheelbase, 4)
    if drive == "ackermann":
        t.drive["steering_limit_rad"] = round(steer_limit, 4)
    x, y, z, heading = pose
    sdf = (f'<model name="{escape(name)}">{_pose((x, y, z), (0.0, 0.0, heading))}'
           + "".join(base) + "".join(parts) + drive_plugin + truth + "</model>")
    return sdf, t
