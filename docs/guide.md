# Using Argus

The [README](../README.md) gets you installed and the [tutorial](tutorial.md) takes you through a
first validation. This guide covers each part in more depth. Every command is listed in
[cli.md](cli.md). The commands here can be typed into the Argus console too, with `/` in place of
`argus`: `argus jobs report JOB` is `/jobs report JOB`.

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
for them again. [data.md](data.md) lists what each command sends where. All commands are in
[cli.md](cli.md).

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
