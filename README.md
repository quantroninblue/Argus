# Argus

Argus is navigation software for robots it was not written for. Describe your robot once (its
shape, how it moves, how it is commanded, what sensors it has and where they sit), and Argus
works out what that body can do and drives it safely. Ground robots and multirotors are both
supported.

Argus has two parts:

- **The hub**, which you run on a computer or server. It holds the map of each site, plans
  routes and explores, and can use AI models if you connect them.
- **Argo**, which runs on each robot. It turns the hub's waypoints into motion and keeps the
  robot safe on its own. If the connection to the hub drops, it stops or lands as you told it to.

The hub only ever sends waypoints. Motor commands are made on the robot, and Argo has the final
say on safety.

> **Argus is early software.** It drives simulated robots end to end and validates robot designs
> in Gazebo. It has not run on real hardware yet. It is not a certified safety system, and any
> real robot still needs its own certified safety hardware.

## What you can do with it

- **Check a robot design before you build it.** Argus drives your URDF through a set of Gazebo
  worlds with people walking through them, then tells you what went wrong and whether the design,
  Argus or the world was at fault.
- **Send robots to named places.** Teach places and routes for a site, and send robots on patrols
  with stops and waits. If a way is blocked, the hub finds another.
- **Keep track of where robots are.** The hub corrects odometry drift against its map and
  recognises places a robot has been before.
- **Work around people.** Robots slow down near people, keep out of their way, and stop in time
  for anything walking toward them. They avoid drops in the floor they can see.
- **Ask in plain words.** Connect a model and it can explain why a run failed, review a design, or
  turn "take the loading dock" into a trip you confirm first. All of this is optional, and models
  never control a robot or change a safety limit.

## Install

You need Ubuntu 24.04. To validate robots in Gazebo you also need ROS 2 Jazzy desktop from the ROS
apt repository, and about 5 GB of disk space.

```sh
sudo apt install cmake g++ libeigen3-dev libyaml-cpp-dev libprotobuf-dev protobuf-compiler \
    libgrpc++-dev protobuf-compiler-grpc pybind11-dev python3-dev python3-venv python3-yaml \
    python3-jsonschema python3-protobuf python3-grpcio python3-numpy python3-jwt \
    python3-cryptography openssl
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-nav2-minimal-tb3-sim   # for Gazebo

git clone https://github.com/quantroninblue/Argus.git argus
cd argus
tools/build.sh
mkdir -p ~/.local/bin && ln -s "$PWD/tools/argus" ~/.local/bin/argus
```

The build takes several minutes. If `argus` is not found afterwards, log out and back in.

Then add the full-screen console. It lives in a small environment inside the checkout and needs
nothing else on your system. Without it, `argus` opens a plain prompt that takes the same commands.

```sh
python3 -m venv --system-site-packages .venv
.venv/bin/pip install "textual>=6.6,<7"
```

## Start

```sh
source /opt/ros/jazzy/setup.bash   # if you will validate robots in Gazebo
argus
```

This opens the Argus console. Type `/hub start` to start the hub, which sets itself up the first
time, then `/help` to see what you can do. Commands start with `/`, and Tab completes them. Anything
else you type is an instruction for a robot.

```
/status                       the hub, and how many robots are connected
/robots                       each robot, its link and mission
/place add warehouse dock 0 0
/go robot-1 dock
/validate my-robot            drive a design through Gazebo worlds
/quit                         leave; the hub keeps running
```

Every command also works from your shell, for scripts: `/robots` is `argus robots`.

## Tutorial

**[Validate your first robot](docs/tutorial.md)** takes about an hour, most of it waiting for runs.
It uses the TurtleBot3 that comes with ROS 2, then shows you how to do the same with your own robot.

## Guides

- [Using Argus](docs/guide.md): describing your robot, running the hub, places and routes,
  validating designs, connecting models, and certificates for real robots.
- [All commands](docs/cli.md)
- [What leaves your machine](docs/data.md) when you connect a model

## Connecting a model

Models are optional. Without one, Argus gives its rule-based findings. In the console:

```
/models connect openai-compatible http://localhost:11434/v1 --name local
/models connect anthropic --key-prompt
/models connect chatgpt
/models connect claude-code
/models use reasoning local/llama3.2
```

These are a local server such as Ollama, an API key, your own ChatGPT plan, and your own installed
Claude Code. Keys and tokens are kept in the hub's memory only, so a restarted hub asks for them
again. A local server keeps everything on your machine.

## Known limits

- Nothing has run on a real robot yet.
- Start robots at the site's origin: the hub cannot yet find a robot that starts somewhere else.
- Heavy odometry drift can beat the hub's correction. If a robot's first pass through a site goes
  badly wrong, its map stays wrong. Drift correction and place recognition have only been run in
  simulation so far, not in Gazebo.
- A robot may stop up to 0.1 m outside a place's tolerance.
- The hub can pause for a moment while it recognises a place it has seen before.
- A body moving fast can stop just outside its safety margin at the end of a wall and find no
  motion it considers clear. It stays safe.
- A body with no rear sensors can get stuck in a tight spot, because it will not reverse into
  space it cannot see. It stays safe.
- A car-like robot in Gazebo fails about 1 run in 8, looping near the goal.

## Coming next

First runs on real hardware (Vicharak Axon), labelling what robots see, a graphical interface, and
installing from PyPI.

## Feedback

Bug reports are welcome through GitHub issues. If a report says Argus was at fault, please attach
the run's log. Argus is not taking code contributions at the moment.

## License

MIT. See [LICENSE](LICENSE).
