# The argus command

Every command runs on the hub's machine (over SSH from elsewhere). Commands that talk to the
hub take `--home DIR` when the hub's home is not `~/.local/share/argus` (or `ARGUS_HOME`).
`--no-color` anywhere turns colour off, as do `NO_COLOR` and piped output. `argus COMMAND
--help` gives every option.

A job or a design version can be named by the start of its id, as long as only one matches.

## The hub

| Command | What it does |
|---|---|
| `argus hub init --host hub.lan --host 192.168.1.20` | Make the hub's home: directories, its certificate authority and certificate, valid for each name or address robots use to reach it |
| `argus hub start --intake 20Mbit` | Run the hub until Ctrl-C. `--listen HOST:PORT` for robots (default 0.0.0.0:50051); `--intake` is the bandwidth for map evidence from all robots |
| `argus status` | The running hub: version, where robots connect, how many are connected |
| `argus watch` | Hub events as they happen (robots, links, missions, validation jobs), until Ctrl-C |

## Robots

| Command | What it does |
|---|---|
| `argus robots` | Every registered robot: connected or not, pose, link, mission, evidence rate |
| `argus goal robot-1 3 1.5` | Send a robot to a point in its site frame (`Z` after `Y` for drones; `--tolerance` in metres) |
| `argus cancel robot-1` | Stop a robot's mission |
| `argus certs robot ~/.local/share/argus/pki robot-1` | A robot's certificate; its name must match Argo's `node_id` |
| `argus certs ca DIR`, `argus certs hub DIR --host H` | An authority and a hub certificate in any directory, if you manage certificates yourself |

## Places and routes

| Command | What it does |
|---|---|
| `argus place here robot-1 dock` | A place named `dock` where robot-1 is now, in its site (`--tolerance`, `--replace`) |
| `argus place add warehouse shelf-3 6.5 2.0` | A place by its coordinates in the site frame (`--heading` for a place to face a way at) |
| `argus place list`, `argus place remove warehouse shelf-3` | Every site's places; remove one no route uses |
| `argus route add warehouse rounds shelf-3:30 dock --mode loop --cycles 4` | A route: places in order, each with a dwell in seconds; `once`, `loop` or `back_and_forth`; `--on-unreachable skip` (default) or `stop` |
| `argus route list`, `argus route remove warehouse rounds` | Every site's routes; remove one |
| `argus go robot-1 shelf-3` | Send a robot to a named place in its site |
| `argus patrol robot-1 rounds` | Run a robot along a route; `argus goal` or `argus cancel` ends it |

## Describing a robot

| Command | What it does |
|---|---|
| `argus setup template > answers.yaml` | An answers file to fill in: what a URDF cannot say |
| `argus setup import robot.urdf -a answers.yaml -o profile.yaml` | A body profile from a URDF (or xacro) and the answers, with its capability report |
| `argus setup report profile.yaml` | The capability report of a profile |
| `argus setup check profile.yaml --motion` | On the robot: every topic, type and rate, sensor mounts, and the controller timeout (moves the robot; `--wheels-up` if lifted) |
| `argus setup calibrate profile.yaml -o calibrated.yaml` | Stepped motion tests; measured limits written back where they are worse than declared |

## Designs and validation

| Command | What it does |
|---|---|
| `argus design add robot.urdf.xacro -a answers.yaml` | Store a design on the hub and check it. `--gazebo-model FILE` for your own Gazebo model, `--package NAME=DIR` for `package://` meshes, `--name` to name it |
| `argus design list` | Every design and its versions |
| `argus design show my-robot` | A design's findings and capability report (`my-robot@VERSION` for an older version) |
| `argus worlds` | The worlds a validation job can use, and what they may hold |
| `argus validate my-robot --worlds wall_with_gap,clutter --contents objects,movers --view` | Drive a design through Gazebo worlds and diagnose each run. `--repeats N` runs each world N times; `--detach` starts the job and returns |
| `argus jobs list` | Every job: how many runs reached their goal, how many have Argus findings, its size |
| `argus jobs watch JOB` | Follow a job's runs until it ends (`--view` to open Gazebo on each run) |
| `argus jobs view JOB` | The same as `jobs watch JOB --view` |
| `argus jobs report JOB` | Each run's outcome and findings |
| `argus jobs cancel JOB` | Stop a job; the run in progress stops and keeps its log |
| `argus jobs keep JOB`, `argus jobs unkeep JOB` | Keep a job for good, or let it be deleted to make room (the newest 10 are kept) |
| `argus jobs delete JOB` | Delete a finished job and its logs |
| `argus runs show JOB 3` | One run: findings, a timeline of what Argo reported, the files in its log |
| `argus runs replay JOB 3` | Play a finished run back in Gazebo |

## On the robot

Argo is not part of the argus command: it runs on the robot (`argo --config argo.yaml`), or in
setup mode for `setup check` and `setup calibrate` (`argo --config argo.yaml --setup`). Build it
with `tools/build_argo.sh`; `tools/argo.service` runs it under systemd.
