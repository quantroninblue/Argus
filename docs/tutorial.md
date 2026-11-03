# Validate your robot in Gazebo

This walks through validating a robot design with Argus, using the TurtleBot3 Waffle that
ships with ROS 2 Jazzy, then your own robot. You will build Argus, start the hub, add the
robot as a design, drive it through Gazebo worlds, and read what the runs found.

Every command after the build is `argus ...`. Allow about an hour, most of it waiting for the
build and the runs.

## What you need

- Ubuntu 24.04 with a desktop session (Gazebo opens windows).
- ROS 2 Jazzy desktop, installed from the ROS apt repository.
- About 5 GB of disk space for the build, and more for run recordings if you keep many jobs.

## 1. Build Argus

```sh
sudo apt install cmake g++ libeigen3-dev libyaml-cpp-dev libprotobuf-dev protobuf-compiler \
    libgrpc++-dev protobuf-compiler-grpc libgtest-dev pybind11-dev python3-dev python3-yaml \
    python3-jsonschema python3-protobuf python3-grpcio python3-numpy python3-pytest openssl
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-nav2-minimal-tb3-sim

git clone https://github.com/quantroninblue/Argus.git argus
cd argus
source /opt/ros/jazzy/setup.bash
tools/check.sh
```

`tools/check.sh` builds the hub's modules and Argo, with its ROS 2 driver, into `build/`, then
runs the tests. It takes several minutes. When it finishes, put the `argus` command on your
PATH:

```sh
mkdir -p ~/.local/bin && ln -s "$PWD/tools/argus" ~/.local/bin/argus
argus --help
```

If `argus` is not found, log out and back in (Ubuntu adds `~/.local/bin` to PATH once it
exists).

## 2. Start the hub

The hub keeps everything in `~/.local/share/argus`. Set it up once:

```sh
argus hub init
```

Then start it in a terminal of its own, with ROS 2 sourced, and leave it running:

```sh
source /opt/ros/jazzy/setup.bash
argus hub start
```

In a second terminal, check that it answers:

```sh
argus status
```

## 3. Add the TurtleBot3 as a design

A design is a robot as you have it: its URDF (or xacro), the meshes it uses, optionally your
own Gazebo model of it, and an answers file with what a URDF cannot say: speed and braking
limits, the command topic, what to do when the link drops, which sensors it has. The answers
for the Waffle are in `tests/designs/`.

Start with the URDF alone:

```sh
TB3=/opt/ros/jazzy/share/nav2_minimal_tb3_sim
argus design add $TB3/urdf/turtlebot3_waffle.urdf -a tests/designs/turtlebot3_waffle_answers.yaml --name waffle
```

Argus copies the URDF and every mesh it names into the hub, checks it, and stores it as a
version. It reports, among others, five warnings: the URDF names visual meshes that the
package does not ship. Warnings never stop a design from running; here the robot will simply
look incomplete. Errors do stop it, and say what to fix.

```sh
argus design show waffle
```

shows the checks and the capability report Argus derived from the design: how fast it can
stop, which directions and heights its sensors cover, and where it is blind.

## 4. Validate it

```sh
argus validate waffle --worlds wall_with_gap,clutter --view
```

The hub runs the robot in each world in turn, from a start to a goal, through the same Argo
that would run on the real robot. `--view` opens a Gazebo window on each run as it starts;
watch the robot find its way round the wall to the goal behind it. Each run takes a minute or
two. Closing the window does not stop the run, and Ctrl-C only stops following: the job carries
on, and `argus jobs view JOB` brings the windows back.

A job's runs print as they finish, then a summary:

```
run 1 of 2: wall_with_gap, objects, repeat 1: reached in 31 s
run 2 of 2: clutter, objects, repeat 1: reached in 58 s
job 20261103-161204-a1b2 done: 2 of 2 reached; no Argus findings
```

Worlds can also hold people walking across the robot's way. Ask for them with `--contents
movers`, or `--contents objects,movers` for both. `argus worlds` lists every world.

## 5. Read a report

```sh
argus jobs report 20261103-161204
```

The report lists each run with its findings. Every finding says who is at fault:

- **design**: the robot as designed. Motors that cannot reach the speed Argo asked for,
  odometry that disagrees with where the robot really went, a sensor slower or later than
  declared, a sensor the Gazebo model puts somewhere other than the URDF says.
- **argus**: Argus itself. Each one is a bug in Argus; please report it with the run's log.
- **world**: the goal cannot be reached by this body in this world.
- **inconclusive**: no rule explains a failure, or it may follow from a design finding.

## 6. A design with a fault in it

The TurtleBot3 package also ships its own Gazebo model of the Waffle, with its own plugins and
sensors. Add the design again with it:

```sh
argus design add $TB3/urdf/turtlebot3_waffle.urdf -a tests/designs/turtlebot3_waffle_answers.yaml \
    --gazebo-model $TB3/urdf/gz_waffle.sdf.xacro --name waffle-gz
argus design show waffle-gz
```

This time there is a warning about the depth camera: the Gazebo model puts it about 10 cm
higher (and 6 cm forward and right) than the URDF does. Argo places everything a sensor sees
by the mount the design declares, so with this model it sees the floor 10 cm too low. Run it
and see what that does:

```sh
argus validate waffle-gz --worlds wall_with_gap --view
```

The robot turns on the spot and does not get far: everything ahead looks like the edge of a
drop. The report puts it down to the design (`sensor_mount`), and marks Argo's stall as
inconclusive, since it follows from what the misplaced camera showed. The fix belongs in the
design: make the URDF and the Gazebo model agree, add the design again (it becomes a new
version), and validate the new version.

## 7. Look at a run again

```sh
argus runs show 20261103-1620 1
argus runs replay 20261103-1620 1
```

`runs show` prints the run's findings, a timeline of what Argo reported (when it stopped,
which safety checks were holding it back) and the files in its log. `runs replay` plays the
run back in Gazebo from its recording, with play, pause and a timeline to drag: use the times
in the timeline to find the moment that matters. Any finished run can be replayed, whether it
reached its goal or not.

The hub keeps the newest 10 jobs. `argus jobs keep JOB` keeps one for good; `argus jobs
delete JOB` frees its space; `argus jobs list` shows them all with their size.

## 8. Your own robot

```sh
argus setup template > answers.yaml
```

Fill in the answers file: the limits from your motor controller's settings or datasheet, the
command topic and type your robot listens on, its controller timeout, and each sensor with the
topic it publishes on and the frame it is mounted at. Then:

```sh
argus design add my_robot.urdf.xacro -a answers.yaml --package my_robot_description=~/ws/src/my_robot_description
argus design show my-robot
argus validate my-robot --contents objects,movers --view
```

`--package NAME=DIRECTORY` tells Argus where `package://NAME/...` meshes live when the package
is not in your sourced ROS 2 environment. Add `--gazebo-model FILE` if you have your own Gazebo
model with your own plugins; without one, Argus converts the URDF and adds a differential drive
and the sensors you declared, and lists everything it added in each run's log.

When a run finds a design fault, change the design, add it again and validate the new version:
`argus design list` shows every version, and `argus validate my-robot@VERSION` runs an older
one.

## Running without a desktop

Over SSH, leave out `--view`: runs never need a window. Watch progress with `argus jobs watch
JOB`, and replay runs later from a desktop session on the hub's machine.

The full list of commands is in [cli.md](cli.md).
