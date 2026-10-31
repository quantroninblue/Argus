"""Gazebo worlds from the simulator's box worlds, and the ros_gz bridge config for a robot."""

from __future__ import annotations

from argus.sim.world import World

from .model import Topics


def world_sdf(world: World, models: list[str], step_s: float = 0.001) -> str:
    """A world with DART physics at `step_s`, a frictional floor, the boxes as static walls and
    `models` (SDF strings) placed in it. Real-time factor 1, so a run takes as long as it would
    on a real robot."""
    boxes = []
    for i, b in enumerate(world.boxes):
        lo, hi = b[:3], b[3:]
        size = " ".join(f"{hi[k] - lo[k]:.6g}" for k in range(3))
        centre = " ".join(f"{(hi[k] + lo[k]) / 2:.6g}" for k in range(3))
        geo = f"<geometry><box><size>{size}</size></box></geometry>"
        boxes.append(f'<model name="box_{i}"><static>true</static><pose>{centre} 0 0 0</pose>'
                     f'<link name="link"><collision name="c">{geo}</collision>'
                     f'<visual name="v">{geo}</visual></link></model>')
    floor = ('<model name="floor"><static>true</static><link name="link"><collision name="c">'
             "<geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>"
             "<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>"
             '<visual name="v"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane>'
             "</geometry></visual></link></model>")
    return ('<?xml version="1.0"?><sdf version="1.9">'
            f'<world name="{world.name}">'
            f'<physics name="dart" type="dart"><max_step_size>{step_s}</max_step_size>'
            "<real_time_factor>1.0</real_time_factor></physics>"
            '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>'
            '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
            "<render_engine>ogre2</render_engine></plugin>"
            '<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>'
            '<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>'
            '<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>'
            '<light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><direction>-0.5 0.1 -0.9</direction>'
            "</light>"
            + floor + "".join(boxes) + "".join(models) + "</world></sdf>")


def bridge_entries(doc: dict, topics: Topics, command_topic: str | None = None) -> list[tuple[str, str, str, str, str]]:
    """(ROS 2 topic, Gazebo topic, ROS 2 type, Gazebo type, direction) for everything bridged:
    sensors and wheel odometry to the profile's ROS 2 topics, the command from ROS 2, simulated
    time, and the true pose on a topic Argo does not read. Commands come from `command_topic`
    (default: the profile's command endpoint)."""
    entries = [("/clock", "/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "GZ_TO_ROS")]
    sensors = {s["id"]: s for s in doc.get("sensors", [])}
    for sid, gz in topics.sensors.items():
        ros = sensors[sid]["source"]["endpoint"]
        kind = topics.kinds[sid]
        if kind == "range_2d":
            entries.append((ros, gz, "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan", "GZ_TO_ROS"))
        elif kind == "range_3d":
            entries.append((ros, gz + "/points", "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked",
                            "GZ_TO_ROS"))
        elif kind == "imu":
            entries.append((ros, gz, "sensor_msgs/msg/Imu", "gz.msgs.IMU", "GZ_TO_ROS"))
        elif kind == "depth":
            entries.append((ros, gz, "sensor_msgs/msg/Image", "gz.msgs.Image", "GZ_TO_ROS"))
            info_ros = ros.rsplit("/", 1)[0] + "/camera_info"
            info_gz = gz.rsplit("/", 1)[0] + "/camera_info"
            entries.append((info_ros, info_gz, "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "GZ_TO_ROS"))
    odom = next((s for s in doc["sensors"] if s["kind"] == "odometry"), None) or next(
        s for s in doc["sensors"] if s["kind"] == "wheel_odom")
    entries.append((odom["source"]["endpoint"], topics.odom, "nav_msgs/msg/Odometry", "gz.msgs.Odometry",
                    "GZ_TO_ROS"))
    entries.append(("/argus_truth", topics.truth, "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS"))
    entries.append((command_topic or doc["control"]["endpoint"], topics.cmd_vel, "geometry_msgs/msg/Twist",
                    "gz.msgs.Twist", "ROS_TO_GZ"))
    return entries


def bridge_yaml(entries: list[tuple[str, str, str, str, str]]) -> str:
    """ros_gz parameter_bridge config for `bridge_entries`."""
    lines = []
    for ros, gz, ros_type, gz_type, direction in entries:
        lines += [f'- ros_topic_name: "{ros}"', f'  gz_topic_name: "{gz}"', f'  ros_type_name: "{ros_type}"',
                  f'  gz_type_name: "{gz_type}"', f"  direction: {direction}"]
    return "\n".join(lines) + "\n"
