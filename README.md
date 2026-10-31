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

What it does today, tested on generated bodies of every drive type:

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
3. Perception and language models on the hub, each with a smaller fallback.
4. Setup tool for importing and calibrating a real robot, first hardware runs on the Axon.

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

## License

MIT. See `LICENSE`.
