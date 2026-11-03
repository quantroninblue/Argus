"""One validation run, in a process of its own: `python3 -m argus.gazebo.validate_run SPEC`.

The hub starts this for each run of a validation job (argus.hub.validation). The run has its
own private hub, certificates and site map (run_gazebo_episode), so nothing simulated reaches
the operator's sites, and rclpy, Gazebo's environment and any crash stay out of the hub's
process.

SPEC is a JSON file:
    profile      the body profile (YAML path)
    design_dir   the stored design version to run, or null for the model built from the profile
    world        a name from argus.hub.worlds.WORLDS
    contents     objects, movers or empty
    work         a directory for the run's files; removed at the end
    log_dir      where the run log ends up (must not exist yet), with Gazebo's state
                 recording in replay/ for playback
    live         written while Gazebo runs: what a Gazebo window needs to attach to it
                 (GZ_PARTITION, GZ_IP, GZ_SIM_RESOURCE_PATH); the hub removes it after the run
    result       where to write the outcome as JSON

Gazebo runs without a window. The argus command opens one in the user's desktop session when
asked (argus.hub.viewer), before or during the run, and closing it never touches the run.

Exit status 0 when the run happened (whatever its outcome), 1 when it could not.
"""

from __future__ import annotations

import json
import shutil
import sys
import traceback
from pathlib import Path


def main(argv: list[str]) -> int:
    if len(argv) != 1:
        print(__doc__, file=sys.stderr)
        return 2
    spec = json.loads(Path(argv[0]).read_text())
    work, log_dir, result = Path(spec["work"]), Path(spec["log_dir"]), Path(spec["result"])
    live = Path(spec["live"]) if spec.get("live") else None

    def started(env: dict) -> None:
        if live is not None:
            tmp = live.with_suffix(".tmp")
            tmp.write_text(json.dumps(env))
            tmp.replace(live)

    out: dict = {"ok": False}
    try:
        from argus import profile as profiles
        from argus.hub import worlds

        from .run import run_gazebo_episode

        doc = profiles.load(spec["profile"])
        kind = worlds.body_kind(doc)
        ws = worlds.WORLDS[spec["world"]]
        world = ws.world(kind, spec["contents"])
        duration = ws.duration_s(worlds.top_speed(doc))
        ep = run_gazebo_episode(doc, world, ws.start_pose(kind), ws.goal_point(kind), duration, workdir=str(work),
                                design_dir=spec.get("design_dir"), record_state=True, on_started=started)
        out = {"ok": True, "goal_reached": ep.goal_reached, "collided": ep.collided, "mission_state": ep.mission_state,
               "time_s": ep.time_s, "duration_s": duration, "reached_at_s": ep.reached_at_s}
    except Exception as e:  # reported to the hub as the run's error, with the traceback in its log
        traceback.print_exc()
        out = {"ok": False, "error": f"{type(e).__name__}: {e}"}
    finally:
        if (work / "log").is_dir():
            log_dir.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(work / "log"), str(log_dir))
        shutil.rmtree(work, ignore_errors=True)
        result.write_text(json.dumps(out))
    return 0 if out["ok"] else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
