# Argus

Argus is navigation software meant to work on robots it was not written for. You describe a
robot once (its shape, how it moves, how it is commanded, what sensors it has and where they
are mounted), and Argus works out what that body can do and plans for it. Ground robots and
multirotors are both in scope.

The system has two parts:

- **Argus**, the hub, runs in the cloud. It holds the site map, plans routes and exploration,
  and runs the heavier perception and reasoning models.
- **Argo**, the node, runs on each robot (Vicharak Axon first, Raspberry Pi and Jetson later).
  It turns the hub's waypoints into motion, keeps the robot safe on its own, and has a defined
  fallback for every body if the connection to the hub drops. Every connected robot is its own
  Argo instance.

The hub only ever sends waypoints. Motor commands are produced on the robot, and Argo has the
final say on safety.

## Status

Early development. Argus and Argo drive simulated robots end to end, including over a real
network link; nothing has run on real hardware yet.

What exists today:

- **Body profile format** (`schemas/`): a JSON Schema and example profiles for a differential
  base, an omnidirectional base and a multirotor.
- **Capability library** (`cpp/capabilities/`): reads a profile and derives motion limits,
  stopping distances, the collision footprint, sensor coverage in every direction (including
  blind spots caused by the robot's own body, obstacle heights no sensor can see, and how far
  out every height is still seen), localization sources and available inference backends.
  Usable from C++ and Python.
- **Argo** (`cpp/node/`): the on-robot core. A local voxel map built from range sensors, a
  local planner, a safety supervisor that can only slow or stop the robot, and command
  adapters for velocity and pose-setpoint controllers. It runs as a step API, so the same code
  is driven by the simulator now and by drivers on the robot later.
- **Hub-node protocol** (`proto/`): protobuf messages for sessions, telemetry, events, mission
  plans, map updates and stream control, carried over gRPC with mutual TLS. Robots always
  open the connection.
- **Argus** (`python/argus/hub/`): the hub. A registry of robots and their profiles, one
  persistent site map per site built from every robot's evidence (`cpp/sitemap/`), a route
  planner that works in each body's own configuration space, and a mission manager per robot
  that keeps a short list of waypoints up to date and remembers branches that turned out to be
  dead ends.
- **Link handling on Argo**: heartbeats, link quality, a speed cap while the link is degraded,
  and the profile's failsafe when it is lost (stop, hold, finish the current leg, return home
  along its own trail, or land).
- **Simulator** (`python/argus/sim/`): box worlds, ray-cast range sensors and a model of each
  control interface, plus a generator of random valid robot bodies. Collisions, limits and
  coverage are judged from simulator truth, not from Argo's own estimate.

What it does today, tested on robots modelled on common platforms (`tests/bodies/`: TurtleBot3
and TurtleBot4 class, Jackal and Husky class skid-steer bases, a 1:10 scale car, a mecanum base
and an X500 class quadcopter), which must reach their goals, and on randomly generated bodies
of every drive type, which must stay safe:

- Reaches waypoints that need a detour, on ground robots and multirotors.
- Never commands speeds, accelerations or turns the body cannot do.
- Never moves faster in a direction than its sensors can see far enough to stop for, and
  never moves where no sensor covers, unless the profile allows a creep speed there.
- Stops when sensor data goes stale or odometry fails, and refuses to move while the profile
  has unreviewed fields.
- Reaches goals through the hub in worlds that need backtracking out of a dead end, a detour
  that first leads away from the goal, or a new route when a gap is blocked mid-run.
- When the link drops for longer than the profile allows, runs that profile's failsafe, and
  carries on with the mission once the link is back.

## Roadmap

1. Done: Argo running in a headless simulator: local map, local planner, safety supervisor,
   command adapters.
2. Done: hub loop: sessions, site map, global planning and exploration with backtracking,
   tested under network loss.
3. Setup tool for importing and calibrating a real robot, the on-robot runtime with ROS 2
   drivers, and first hardware runs on the Axon.
4. Perception and language models on the hub, each with a smaller fallback.
5. A graphical interface for the hub, a fuller command line, and polish for a first release.

## Building

Tested on Ubuntu 24.04.

```sh
sudo apt install cmake g++ libeigen3-dev libyaml-cpp-dev libprotobuf-dev protobuf-compiler \
    libgrpc++-dev protobuf-compiler-grpc libgtest-dev pybind11-dev python3-dev python3-yaml \
    python3-jsonschema python3-protobuf python3-grpcio python3-numpy python3-pytest openssl
tools/check.sh
```

`tools/check.sh` builds everything into `build/` and runs the C++ and Python tests (a few
minutes; some tests run over a real local network link in wall time).

With ROS 2 Jazzy installed (desktop, with `ros-jazzy-ros-gz` for Gazebo Harmonic), the build
also includes Argo's ROS 2 driver, and `tools/check.sh --gazebo` additionally drives robots
through Gazebo.

## Trying the capability library

After building:

```sh
export PYTHONPATH=python:build/python
python3 - <<'EOF'
from argus import capabilities
caps = capabilities.derive_from_file("schemas/examples/planar_differential.yaml")
print("stops from 1 m/s in", round(caps.motion.stopping_distance_m("x", 1.0), 2), "m")
print("sees", round(100 * caps.sensing.seen_fraction), "% of directions")
print("blind bearings:", [b.azimuth_deg for b in caps.sensing.bins if not b.seen])
EOF
```

To write a profile for your own robot, start from one of the files in `schemas/examples/`.
Validation reports every problem with the path of the field at fault.

## Describing your robot

A profile can be built from the robot's URDF (or xacro) plus a short answers file with what a
URDF cannot say: speed and braking limits, the command topic, the controller timeout, the
failsafe and odometry. Collision shapes, link frames and any sensors declared in Gazebo tags
come from the URDF; meshes become bounding boxes.

```sh
export PYTHONPATH=python:build/python
python3 -m argus setup template > answers.yaml   # then fill it in
python3 -m argus setup import robot.urdf -a answers.yaml -o profile.yaml
python3 -m argus setup report profile.yaml
```

Import lists anything missing instead of guessing it, and prints a capability report: stopping
distances, the bearings and heights the sensors cover, and which values are still declared
rather than measured. `package://` mesh paths resolve through a sourced ROS 2 environment or
`--package NAME=DIRECTORY`.

On the robot, with ROS 2 sourced and the robot's drivers running, `setup check` confirms every
declared topic exists with the right type and publishes at its declared rate. With `--motion`
it also confirms the robot's own controller stops the robot when commands stop: the robot
crawls forward at 0.1 m/s for a second (only with clear space ahead on the lidar, or with the
drive wheels lifted and `--wheels-up`), then commands stop and the time to standstill is
measured. Argo relies on that timeout, so setup does not pass without it.

Motion tests go through Argo in setup mode, so its safety supervisor checks every request the
same way it checks normal driving. Start Argo with `--setup` on the robot first; it then moves
only on setup requests and needs no hub. `--direct` sends commands straight to the robot's
command topic instead, without the supervisor.

```sh
argo --config argo.yaml --setup        # on the robot
python3 -m argus setup check profile.yaml --motion
```

`setup calibrate` then measures command latency, acceleration, top speed and braking on
every axis the robot moves along (forward, backward if it may reverse, sideways if it can
strafe), and the yaw rate or, for a car-like robot, the real turning radius, by stepped motion
tests watched through odometry. Before each run it checks the room in that direction with
whatever range sensors the robot has (scans, point clouds or depth images). The forward runs
also compare the distance odometry reports with the change in range to what is ahead; an
odometry scale more than 5% off is reported as a problem to fix in the robot's driver.
Measured values replace declared ones only where they are worse (slower, weaker braking,
wider turns, more latency), and are marked `calibrated` in the profile.

Argo will not move into space next to the robot that no sensor has looked at yet, except
space only above or below its sensors' vertical view. Set `assumptions.clear_at_start_m` in
the profile to declare how much room around the robot's starting spot is clear (0.5 m is
typical), and start it there; the setup report says if it is missing.

```sh
python3 -m argus setup calibrate profile.yaml -o profile.calibrated.yaml
```

## Driving a simulated robot

```sh
export PYTHONPATH=python:build/python
python3 - <<'EOF'
from argus import profile
from argus.sim import world
from argus.sim.episode import run_episode

doc = profile.load("schemas/examples/planar_differential.yaml")
goal = [{"id": "goal", "position": (5.0, 2.0, 0.0), "tolerance_m": 0.4}]
ep = run_episode(doc, world.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), goal, 40.0)
print("reached" if ep.complete else "did not reach", "the goal in", round(ep.time_s, 1), "s")
print("collided:", ep.collided, "rule violations:", len(ep.violations))
EOF
```

## Sending a simulated robot through the hub

```sh
export PYTHONPATH=python:build/python
python3 - <<'EOF'
from argus import profile
from argus.sim import world
from argus.sim.channel import LinkFaults
from argus.sim.hub_episode import run_hub_episode

doc = profile.load("schemas/examples/planar_omnidirectional.yaml")
# The link stalls from 3 s to 9 s; this profile stops after 3 s without the hub.
ep = run_hub_episode(doc, world.dead_end_branch(), (-5.0, 0.0, 0.0, 0.0), (5.0, 5.5, 0.0), 120.0,
                     faults=LinkFaults(stalls=[(3.0, 9.0)]))
print("reached" if ep.goal_reached else "did not reach", "the goal at", ep.reached_at_s, "s")
print("failsafe:", [(round(e["stamp"], 1), e["kind"]) for e in ep.failsafe_events()])
EOF
```

## Running a robot in Gazebo

Needs ROS 2 Jazzy with Gazebo Harmonic. The body profile is turned into a Gazebo model with
wheels driven through real joints (differential, mecanum or front-steered, picked from the
profile's motion), the world into SDF, and the real Argo binary drives it through its ROS 2
driver while a local hub plans over mutual TLS.

```sh
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=python:build/python:$PYTHONPATH
python3 - <<'EOF'
from argus import profile
from argus.sim import world
from argus.gazebo.run import run_gazebo_episode

doc = profile.load("schemas/examples/planar_differential.yaml")
ep = run_gazebo_episode(doc, world.wall_with_gap(), (0.0, 0.0, 0.0, 0.0), (5.0, 2.0, 0.0), 90.0,
                        gui=True)
print(ep.mission_state, "collided:", ep.collided)
print("run log:", ep.log_dir)
EOF
```

Each run writes a log: the world and model SDF, the profile, the true trajectory, every
command Argo sent, the robot's own odometry, a summary of every ROS 2 topic (rate, stamp lag,
gaps) and Argo's events. Pass `record_bag=True` to also record every topic to an mcap bag.

## Running the hub

The hub keeps everything in its home directory, `~/.local/share/argus` unless you pass
`--home DIR` or set `ARGUS_HOME`:

```
profiles/        body profiles (*.yaml)
robots.yaml      robot-1: {profile: my-rover, site: warehouse}
pki/             the hub's certificate authority and certificate
runs/            run logs
run/app.sock     where the argus command reaches the hub (only you can open it)
```

Set it up once, giving every name or address robots will use to reach the hub, then start it:

```sh
export PYTHONPATH=python:build/python
python3 -m argus hub init --host hub.example.lan --host 192.168.1.20
python3 -m argus hub start --intake 20Mbit
```

The hub runs until Ctrl-C. From another terminal on the same machine (or over SSH):

```sh
python3 -m argus status                    # the hub, and how many robots are connected
python3 -m argus robots                    # each robot's pose, link and mission
python3 -m argus goal robot-1 3 1.5        # send a robot to a point in its site frame
python3 -m argus cancel robot-1
python3 -m argus watch                     # robots connecting, link and mission changes
```

`--intake` is the bandwidth set aside for map evidence from all robots together. The hub
shares it fairly between robots and lowers it while applying evidence takes too much of its
time; evidence over a robot's share is dropped, while heartbeats, telemetry and events are
always processed, so a busy hub degrades its map rather than its links.

## Certificates

The hub and every robot authenticate each other with certificates from the hub's own
authority. A robot's certificate name must match its Argo `node_id`. `hub init` makes the
authority and the hub's certificate; issue each robot's from the same directory:

```sh
python3 -m argus certs robot ~/.local/share/argus/pki robot-1
```

Keep `ca.key` off the robots; each robot needs only `ca.pem` and its own certificate and key.
`certs ca` and `certs hub` make an authority and a hub certificate in any directory, if you
manage certificates yourself.

On each robot, `tools/build_argo.sh` builds the `argo` binary (it lists any missing apt
packages), and `tools/argo.service` runs it under systemd.

## License

MIT. See `LICENSE`.
