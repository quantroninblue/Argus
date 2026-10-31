# Argus

Argus is navigation software meant to work on robots it was not written for. You describe a
robot once (its shape, how it moves, how it is commanded, what sensors it has and where they
are mounted), and Argus works out what that body can do and plans for it. Ground robots and
multirotors are both in scope.

The system has two parts:

- A **hub** that runs in the cloud. It holds the site map, plans routes and exploration, and
  runs the heavier perception and reasoning models.
- A **node** that runs on the robot (Vicharak Axon first, Raspberry Pi and Jetson later). It
  turns the hub's waypoints into motion, keeps the robot safe on its own, and has a defined
  fallback for every body if the connection to the hub drops.

The hub only ever sends waypoints. Motor commands are produced on the robot, and the node has
the final say on safety.

## Status

Early development. Nothing here drives a robot yet.

What exists today:

- **Body profile format** (`schemas/`): a JSON Schema and example profiles for a differential
  base, an omnidirectional base and a multirotor.
- **Capability library** (`cpp/capabilities/`): reads a profile and derives motion limits,
  stopping distances, the collision footprint, sensor coverage in every direction (including
  blind spots caused by the robot's own body and obstacle heights no sensor can see),
  localization sources and available inference backends. Usable from C++ and Python.
- **Hub-node protocol** (`proto/`): protobuf messages for sessions, telemetry, events, mission
  plans, map updates and stream control.
- **Profile generator** (`python/argus/sim/`): random valid robot bodies used to test that
  nothing in the core depends on one particular robot.

## Roadmap

1. Node running in a headless simulator: local map, local planner, safety supervisor, command
   adapters.
2. Hub loop: sessions, site map, global planning and exploration with backtracking, tested
   under network loss.
3. Perception and language models on the hub, each with a smaller fallback.
4. Setup tool for importing and calibrating a real robot, first hardware runs on the Axon.

## Building

Tested on Ubuntu 24.04.

```sh
sudo apt install cmake g++ libeigen3-dev libyaml-cpp-dev libprotobuf-dev protobuf-compiler \
    libgtest-dev pybind11-dev python3-dev python3-yaml python3-jsonschema python3-protobuf \
    python3-pytest
tools/check.sh
```

`tools/check.sh` builds everything into `build/` and runs the C++ and Python tests.

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

## License

MIT. See `LICENSE`.
