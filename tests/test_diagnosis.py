"""Rule-based diagnosis of run logs (hub plan H5), on hand-made logs."""

import csv
import json

import yaml

from argus import profile as profiles
from argus.hub.diagnosis import ARGUS, DESIGN, INCONCLUSIVE, WORLD, diagnose, job_report


def _write_csv(path, header, rows):
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def _log(tmp_path, name="run", *, reached=True, collided=False, speed=0.5, achieved=0.5, odom_scale=1.0,
         argo="", topics=None, own_design=True):
    """A run that drives straight along x for 10 s at `achieved` m/s while Argo commands `speed`."""
    d = tmp_path / name
    d.mkdir()
    doc = profiles.load(profiles.EXAMPLES_DIR / "planar_differential.yaml")
    (d / "profile.yaml").write_text(yaml.safe_dump(doc))
    ts = [i * 0.05 for i in range(201)]
    _write_csv(d / "trajectory.csv", ["t_s", "x_m", "y_m", "z_m", "yaw_rad"], [[t, achieved * t, 0, 0, 0] for t in ts])
    _write_csv(d / "commands.csv", ["t_s", "vx_mps", "vy_mps", "vz_mps", "yaw_rate_radps"],
               [[t, speed, 0, 0, 0] for t in ts])
    _write_csv(d / "odometry.csv", ["t_s", "x_m", "y_m", "z_m", "yaw_rad", "vx_mps", "vy_mps", "yaw_rate_radps"],
               [[t, odom_scale * achieved * t, 0, 0, 0, achieved, 0, 0] for t in ts[::2]])
    run = {"outcome": {"goal_reached": reached, "collided": collided, "collisions": ["box_3"] if collided else [],
                       "mission_state": "reached" if reached else "failed"},
           "topics": topics or {}, "argo": {"events": {}, "flag_changes": {}},
           "body": {"drivetrain": {"kind": "differential", **({"from": "the design"} if own_design else {})}}}
    (d / "run.json").write_text(json.dumps(run))
    (d / "argo.log").write_text(argo)
    return d


def _rules(diag):
    return {(f.category, f.rule) for f in diag.findings}


def test_a_clean_run_has_no_findings(tmp_path):
    d = diagnose(_log(tmp_path))
    assert d.outcome == "reached" and d.findings == []


def test_a_body_slower_than_commanded_is_the_designs_fault(tmp_path):
    d = diagnose(_log(tmp_path, speed=0.8, achieved=0.4))
    assert (DESIGN, "commands_not_achieved") in _rules(d)
    f = next(f for f in d.findings if f.rule == "commands_not_achieved")
    assert f.time_s[0] >= 1.0 and f.values["worst_share"] < 0.6  # only once the command was steady


def test_odometry_that_overcounts_distance_is_the_designs_fault(tmp_path):
    d = diagnose(_log(tmp_path, odom_scale=1.1))
    f = next(f for f in d.findings if f.rule == "odometry_scale")
    assert f.category == DESIGN and "10% more" in f.sentence


def test_a_slow_sensor_and_dropped_messages_are_told_apart(tmp_path):
    topics = {"/scan": {"rate_hz": 9.0, "declared_rate_hz": 15, "first_s": 0.0, "last_s": 10.0,
                        "longest_gap_s": 0.2},
              "/camera/depth/image_raw": {"rate_hz": 15.0, "declared_rate_hz": 15, "first_s": 0.0, "last_s": 10.0,
                                          "longest_gap_s": 1.2, "stamp_lag_mean_s": 0.25}}
    rules = _rules(diagnose(_log(tmp_path, topics=topics)))
    assert (DESIGN, "sensor_rate") in rules  # the scanner itself is slow
    assert (ARGUS, "message_gap") in rules  # the camera keeps its rate, yet a second went missing
    assert (DESIGN, "stamp_lag") in rules


def test_argo_overrunning_its_period_is_argus_at_fault(tmp_path):
    argo = "12.000 start period_s=0.020000 hub=none\n40.000 stop ticks=1000 overruns=40 worst_ms=55.0\n"
    d = diagnose(_log(tmp_path, argo=argo))
    f = next(f for f in d.findings if f.rule == "control_overruns")
    assert f.category == ARGUS and f.values["overruns"] == 40


def test_a_collision_is_always_an_argus_finding(tmp_path):
    d = diagnose(_log(tmp_path, reached=False, collided=True))
    assert d.outcome == "collided" and (ARGUS, "collision") in _rules(d)


def test_why_a_goal_was_not_reached_comes_from_argos_blocked_events(tmp_path):
    dead = '20.500 event blocked waypoint=goal cause=dead_end_observed detail=""\n'
    stalled = '20.500 event blocked waypoint=goal cause=controller_no_progress detail="no progress"\n'
    assert _rules(diagnose(_log(tmp_path, "a", reached=False, argo=dead))) == {(WORLD, "dead_end_observed")}
    assert _rules(diagnose(_log(tmp_path, "b", reached=False, argo=stalled))) == {(ARGUS, "controller_no_progress")}
    unexplained = diagnose(_log(tmp_path, "c", reached=False))
    assert _rules(unexplained) == {(INCONCLUSIVE, "unexplained")}


def test_a_job_report_counts_outcomes_and_lists_runs_that_blame_argus(tmp_path):
    runs = [diagnose(_log(tmp_path, "ok")), diagnose(_log(tmp_path, "hit", reached=False, collided=True)),
            diagnose(_log(tmp_path, "slow", speed=0.8, achieved=0.4))]
    report = job_report(runs)
    assert report["runs"] == 3
    assert report["outcomes"] == {"reached": 2, "collided": 1}
    assert report["argus_runs"] == ["hit"]
    assert report["categories"][DESIGN] == 1


def test_turning_on_the_spot_is_not_counted_as_distance(tmp_path):
    # Truth jitters by millimetres while the robot turns in place; summed at 50 Hz that once
    # read as odometry 12% short, blaming a design whose odometry was right.
    d = _log(tmp_path)
    rows = [[t, 0.5 * t, 0, 0, 0] for t in [i * 0.02 for i in range(501)]]
    rows += [[10.0 + i * 0.02, 5.0 + 0.002 * (-1) ** i, 0.002 * (-1) ** (i // 2), 0, 0] for i in range(1, 3001)]
    _write_csv(d / "trajectory.csv", ["t_s", "x_m", "y_m", "z_m", "yaw_rad"], rows)
    _write_csv(d / "odometry.csv", ["t_s", "x_m", "y_m", "z_m", "yaw_rad", "vx_mps", "vy_mps", "yaw_rate_radps"],
               [[t, min(0.5 * t, 5.0), 0, 0, 0, 0, 0, 0] for t in [i * 0.05 for i in range(1401)]])
    assert (DESIGN, "odometry_scale") not in _rules(diagnose(d))


def test_a_misplaced_sensor_is_the_designs_fault_and_what_follows_is_not_argus(tmp_path):
    stalled = '20.500 event blocked waypoint=goal cause=controller_no_progress detail="no progress"\n'
    d = _log(tmp_path, reached=False, argo=stalled)
    run = json.loads((d / "run.json").read_text())
    run["mount_findings"] = ["depth: the Gazebo model's sensor cam is not where the profile says: it sits 10 cm higher"]
    (d / "run.json").write_text(json.dumps(run))
    diag = diagnose(d)
    assert _rules(diag) == {(DESIGN, "sensor_mount"), (INCONCLUSIVE, "controller_no_progress")}
    assert ARGUS not in diag.blames


def test_arriving_where_the_robot_is_not_is_argus_at_fault(tmp_path):
    d = _log(tmp_path, reached=False)
    run = json.loads((d / "run.json").read_text())
    run["outcome"].update(mission_state="reached", final_pose=[1.0, 0.0, 0.0])
    run["goal"] = [6.0, 0.0, 0.0]
    (d / "run.json").write_text(json.dumps(run))
    diag = diagnose(d)
    f = next(f for f in diag.findings if f.rule == "false_arrival")
    assert f.category == ARGUS and "5.00 m from it" in f.sentence
    assert (INCONCLUSIVE, "unexplained") not in _rules(diag)


def test_on_argus_s_own_model_of_a_profile_a_slow_drive_is_argus_at_fault(tmp_path):
    # Reference bodies have no design: Gazebo runs the model Argus builds from the profile.
    diag = diagnose(_log(tmp_path, speed=0.5, achieved=0.2, own_design=False))
    f = next(f for f in diag.findings if f.rule == "commands_not_achieved")
    assert f.category == ARGUS and f.sentence.startswith("In the Gazebo model Argus built")
