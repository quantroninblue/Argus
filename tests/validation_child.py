"""Stands in for argus.gazebo.validate_run in tests/test_validation.py: writes a run log a
real run would write, without Gazebo. FAKE_RUN in the environment picks what happens:
"reach" (default), "collide", "crash" (exits without a log), "hang" (waits to be stopped)."""

import json
import os
import signal
import sys
import time
from pathlib import Path


def main() -> int:
    spec = json.loads(Path(sys.argv[1]).read_text())
    mode = os.environ.get("FAKE_RUN", "reach")
    result = Path(spec["result"])
    Path(spec["live"]).write_text(json.dumps({"GZ_PARTITION": "fake-run", "GZ_IP": "127.0.0.1",
                                              "GZ_SIM_RESOURCE_PATH": "/designs/bot/packages"}))
    if mode == "crash":
        print("Traceback: gz sim exited early")
        return 1
    if mode == "hang":
        def stop(*_):
            result.write_text(json.dumps({"ok": False}))
            sys.exit(1)

        signal.signal(signal.SIGTERM, stop)
        Path(spec["log_dir"] + ".started").touch()
        time.sleep(60)
        return 1
    log = Path(spec["log_dir"])
    log.mkdir(parents=True)
    reached = mode == "reach"
    (log / "run.json").write_text(json.dumps({
        "outcome": {"goal_reached": reached, "collided": mode == "collide", "mission_state": "reached" if reached else
                    "failed", "collisions": [] if reached else [{"t_s": 3.0}], "simulated_s": 12.5},
        "topics": {}, "world": {"name": spec["world"]}}))
    (log / "argo.log").write_text("")
    result.write_text(json.dumps({"ok": True, "goal_reached": reached}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
