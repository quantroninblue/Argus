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
network link, and Argus validates robot designs in Gazebo; nothing has run on real hardware
yet. To try it, follow [the tutorial](docs/tutorial.md); every command is in
[docs/cli.md](docs/cli.md).

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
  and the profile's failsafe when it is lost. Without the hub Argo only stops and parks
  (stop; hold, or land where it is, for a drone); finding a way anywhere is the hub's job.
- **Simulator** (`python/argus/sim/`): box worlds, ray-cast range sensors and a model of each
  control interface, plus a generator of random valid robot bodies. Collisions, limits and
  coverage are judged from simulator truth, not from Argo's own estimate.
- **Design validation in Gazebo** (`python/argus/hub/validation.py`): a robot design (its
  URDF, meshes, optionally its own Gazebo model) runs through a set of worlds with the real
  Argo, each run is recorded for playback and diagnosed by rules that say whether the design,
  Argus or the world is at fault.

What it does today, tested on robots modelled on common platforms (`tests/bodies/`: TurtleBot3
and TurtleBot4 class, Jackal and Husky class skid-steer bases, a 1:10 scale car, a mecanum base
and an X500 class quadcopter), which must reach their goals, and on randomly generated bodies
of every drive type, which must stay safe:

- Reaches waypoints that need a detour, on ground robots and multirotors.
- Never commands speeds, accelerations or turns the body cannot do.
- Never moves faster in a direction than its sensors can see far enough to stop for, and
  never moves where no sensor covers, unless the profile allows a creep speed there.
- Tracks people and other things moving nearby, on the ground and in the air, and stays far
  enough away to be at rest before anything walking toward it could reach it. It slows down
  near people and keeps out of the way they are heading; a drone well above someone's head
  is not held up by them. When someone stops in the way, the robot waits a few seconds, then
  goes around if they stay.
- Sees drops in the floor (a stair down, a dock edge) with sensors that look at the floor
  ahead, and never drives over the edge. A body whose sensors never see the floor cannot.
- A drone with no sensor that sees straight up must declare how high it may fly
  (`assumptions.max_flight_height_m`); it never climbs above that.
- Stops when sensor data goes stale or odometry fails, and refuses to move while the profile
  has unreviewed fields.
- Refuses sensor data that keeps arriving but cannot be right (a driver resending the same
  frame while the robot moves, a blinded sensor, timestamps that do not move forward), and
  then moves only where its other sensors can see. Slows down when odometry disagrees with
  what the range sensors show of the robot's own motion. A sensor that also gets no return
  from dark or see-through surfaces can declare `no_return: unknown`, so such rays never
  count as open space.
- Reaches goals through the hub in worlds that need backtracking out of a dead end, a detour
  that first leads away from the goal, or a new route when a gap is blocked mid-run.
- When the link drops for longer than the profile allows, stops and parks as the profile
  says, and carries on with the mission once the link is back.

## Roadmap

1. Done: Argo running in a headless simulator: local map, local planner, safety supervisor,
   command adapters.
2. Done: hub loop: sessions, site map, global planning and exploration with backtracking,
   tested under network loss.
3. Done: setup tool for importing and calibrating a robot, the on-robot runtime with its ROS 2
   driver, and validation of robot designs in Gazebo.
4. Next, for the first release: named places and routes, the hub correcting odometry drift
   against its map, Argo stopping and reporting when its way is blocked so the hub can
   re-route, and models on the hub (local or remotely hosted): explaining failed runs,
   reviewing designs and goals given in words.
5. Then: labelling what robots see, first runs on real hardware (Vicharak Axon), a graphical
   interface for the hub, and publishing on PyPI.

## Known issues

- A body moving fast can stop just outside its safety margin at the end of a wall and find no
  motion it considers clear (seen with the Jackal class in Gazebo). Being fixed: Argo will stop
  and report, and the hub will route it on.
- A body with no rear sensors can wedge itself in a tight spot when an obstacle appears behind
  it on a narrow way: it will not reverse into space it cannot see. It stays safe; the same fix
  applies.
- The car-like body in Gazebo fails now and then (1 run in 8): its odometry drifts and it loops
  near the goal. The hub's drift correction is the fix.
- The hub corrects odometry drift against surfaces it already knows, in x, y and yaw, and closes
  loops when a robot comes back to somewhere it has been. Heavy drift can still beat it: in
  simulation it held up to about 0.005 rad/s and not at 0.0075 rad/s. A closure lines a robot up
  with the map, so a first pass that went badly wrong stays wrong. It cannot find a robot started
  away from the site's origin: start robots there. A corridor corrects across its width, not along
  its length. On clean odometry it can sit up to one map cell (0.1 m) off the truth, so a robot
  may stop that much outside a place's tolerance.
- Loop closure has not been run in Gazebo yet, and its search can hold up the hub for a moment
  while it runs. Named places are not moved by a closure.

## Building

Tested on Ubuntu 24.04.

```sh
sudo apt install cmake g++ libeigen3-dev libyaml-cpp-dev libprotobuf-dev protobuf-compiler \
    libgrpc++-dev protobuf-compiler-grpc libgtest-dev pybind11-dev python3-dev python3-yaml \
    python3-jsonschema python3-protobuf python3-grpcio python3-numpy python3-pytest python3-jwt \
    python3-cryptography openssl
tools/check.sh
```

`tools/check.sh` builds everything into `build/` and runs the C++ and Python tests (a few
minutes; some tests run over a real local network link in wall time).

The `argus` command runs from the checkout through `tools/argus`. Put it on your PATH once:

```sh
mkdir -p ~/.local/bin && ln -s "$PWD/tools/argus" ~/.local/bin/argus
```

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
argus setup template > answers.yaml   # then fill it in
argus setup import robot.urdf -a answers.yaml -o profile.yaml
argus setup report profile.yaml
```

Import lists anything missing instead of guessing it, and prints a capability report: stopping
distances, the bearings and heights the sensors cover, and which values are still declared
rather than measured. `package://` mesh paths resolve through a sourced ROS 2 environment or
`--package NAME=DIRECTORY`.

On the robot, with ROS 2 sourced and the robot's drivers running, `setup check` confirms every
declared topic exists with the right type and publishes at its declared rate. It also checks
that each range sensor is mounted where the profile says, so run it with the robot on a flat
floor: a sensor that sees the floor must see it flat and at floor height, and two sensors that
see the same walls must place them together. A camera pitched or rolled about 2 degrees from
its declared mount, or turned about 4 degrees, fails the check. With `--motion`
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
argus setup check profile.yaml --motion
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
argus setup calibrate profile.yaml -o profile.calibrated.yaml
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

Worlds can hold moving obstacles too: boxes the size of a person (or anything else) that
walk a path at a set speed, once or back and forth, and never give way. Whether a world has
any is up to you; leave them out for an empty or still world.

```python
from argus.sim.movers import Mover, crossing

w = world.open_room()
w.add_mover(crossing((3.0, 0.0), 0.0, 2.5, start_s=3.0))          # someone crossing 3 m ahead
w.add_mover(Mover(path=[(6.5, 0.0), (-6.0, 0.0)], speed_mps=0.5))  # someone walking toward it
ep = run_episode(doc, w, (0.0, 0.0, 0.0, 0.0), goal, 30.0)
print(ep.collided, ep.contacts)
```

A contact counts against the robot only if it was driving or turning into the mover: someone
walking into a robot that has already stopped is recorded in `ep.contacts` but is not a
collision. The same movers run in Gazebo, where the robot's sensors see them.

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

Set it up once, giving every name or address robots will use to reach the hub, then start it
(from a terminal with ROS 2 sourced if you will validate designs in Gazebo):

```sh
argus hub init --host hub.example.lan --host 192.168.1.20
argus hub start --intake 20Mbit
```

The hub runs until Ctrl-C. From another terminal on the same machine (or over SSH):

```sh
argus status                    # the hub, and how many robots are connected
argus robots                    # each robot's pose, link and mission
argus goal robot-1 3 1.5        # send a robot to a point in its site frame
argus cancel robot-1
argus watch                     # robots connecting, link and mission changes
```

`--intake` is the bandwidth set aside for map evidence from all robots together. The hub
shares it fairly between robots and lowers it while applying evidence takes too much of its
time; evidence over a robot's share is dropped, while heartbeats, telemetry and events are
always processed, so a busy hub degrades its map rather than its links.

## Places and routes

Each site keeps named places and routes in the hub's home (`sites/<site>/`). Teach a place by
driving a robot there, or give its coordinates; a route is a list of places with an optional
dwell at each:

```sh
argus place here robot-1 dock                 # where robot-1 is now, in its site
argus place add warehouse shelf-3 6.5 2.0
argus route add warehouse rounds shelf-3:30 dock --mode loop --cycles 4
argus go robot-1 shelf-3
argus patrol robot-1 rounds
argus place list
argus route list warehouse
```

A route runs `once`, in a `loop` or `back_and_forth`, and `--on-unreachable skip` (the default)
moves on to the next stop when one cannot be reached, while `stop` ends the errand there. A
plain `argus goal` or `argus cancel` ends a robot's errand. `argus watch` shows each leg as it
starts, arrives, dwells or is skipped.

## Adding a robot design

A design is your robot as you have it: its URDF or xacro, the meshes it uses, optionally
your own Gazebo model of it (an SDF or xacro with its plugins), and an answers file for what
a URDF cannot say (speed and braking limits, the command topic, the failsafe; start one with
`argus setup template`). With the hub running:

```sh
argus design add robot.urdf.xacro -a answers.yaml --gazebo-model robot.sdf.xacro
argus design list
argus design show my-robot
```

`design add` expands xacro, copies every mesh the files reference into the design, and sends
it to the hub, which keeps each version (named by a hash of its files) and checks it before
anything may run it. The checks report problems rather than fixing them:

- errors stop the design from running: a moving link with no mass or with impossible inertia,
  a missing collision mesh, a file Gazebo rejects, or answers that do not make a valid profile;
- warnings do not: a missing visual mesh (the robot will look incomplete), anything Gazebo
  warns about;
- notes explain what Gazebo will do, such as merging a massless link into the link it is
  fixed to.

`design show` lists the findings and the capability report of the profile built from the
design. `package://` mesh paths resolve through a sourced ROS 2 environment or
`--package NAME=DIRECTORY`.

## Validating a design in Gazebo

With ROS 2 Jazzy and Gazebo Harmonic installed and sourced where the hub runs, the hub drives a
stored design through a set of Gazebo worlds, one run at a time, and diagnoses every run:

```sh
argus worlds                                   # what a job can use
argus validate my-robot --worlds wall_with_gap,clutter --contents objects,movers --view
argus jobs report JOB
```

Each run puts your robot, as your files describe it, into one world with a start and a goal,
sends it there through the real Argo binary and a hub of its own, and keeps the run's log.
Nothing from a validation run reaches the maps of your real sites. A world holds what you ask
for: `objects` (the world as shipped), `movers` (the same with people walking across the
robot's way) or `empty` (the bare room).

Gazebo runs without a window. To watch, add `--view` (or run `argus jobs view JOB` at any
time, before the first run or in the middle of one): a Gazebo window opens on each run as it
starts and closes when it ends. Closing the window yourself never stops a run. Ctrl-C stops
following; the job carries on, and `argus jobs watch JOB` picks it up again.

Every finding in a report says who is at fault:

- **design**: your robot. Motors that cannot reach the commanded speed, odometry that
  disagrees with where the robot really went, a sensor slower or later than declared, or a
  sensor your Gazebo model puts somewhere other than your URDF says.
- **argus**: Argus itself. Argo late on its control period, Argo driving into something,
  Argo stalling with a way open, messages lost between Gazebo and Argo. Each one is a bug in
  Argus; please report it with the run's log.
- **world**: the goal cannot be reached by this body in this world.
- **inconclusive**: the run failed and no rule explains it, or it may follow from a design
  finding; read the log.

```sh
argus runs show JOB 3       # one run: findings, a timeline of what Argo reported, the log files
argus runs replay JOB 3     # play the run back in Gazebo, with play, pause and seek
```

Every run is recorded, so any finished run, failed or not, can be watched again. The hub
keeps the newest 10 jobs; `argus jobs keep JOB` keeps one for good, and `argus jobs delete
JOB` frees its space. Jobs live in `jobs/` in the hub's home.

## Connecting a model

Models are optional. Without one, the hub gives its rule-based findings. With one, it can explain a
failed run in words, review a design, and turn an instruction into a trip to a named place. A model's
answer is advice: it cannot change a profile or a limit, and Argo never waits on one.

```sh
argus models connect openai-compatible http://localhost:11434/v1 --name local   # Ollama, llama.cpp
argus models connect anthropic --key-prompt      # an API key, kept in the hub's memory only
argus models connect chatgpt                     # sign in to your own ChatGPT plan in your browser
argus models connect claude-code                 # runs your installed Claude Code under your own sign-in
argus models use reasoning local/llama3.2
argus runs explain JOB 3 --ask "why did it stop at the gap?"
```

Anything from Gemini, OpenRouter or another OpenAI-compatible service connects the same way as the
first line, with `--key-prompt`. Keys and tokens are never written to disk, so a restarted hub asks
for them again. [docs/data.md](docs/data.md) lists what each command sends where. All commands are in
[docs/cli.md](docs/cli.md).

## Certificates

The hub and every robot authenticate each other with certificates from the hub's own
authority. A robot's certificate name must match its Argo `node_id`. `hub init` makes the
authority and the hub's certificate; issue each robot's from the same directory:

```sh
argus certs robot ~/.local/share/argus/pki robot-1
```

Keep `ca.key` off the robots; each robot needs only `ca.pem` and its own certificate and key.
`certs ca` and `certs hub` make an authority and a hub certificate in any directory, if you
manage certificates yourself.

On each robot, `tools/build_argo.sh` builds the `argo` binary (it lists any missing apt
packages), and `tools/argo.service` runs it under systemd.

## License

MIT. See `LICENSE`.
