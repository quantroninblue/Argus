import copy
import math

import pytest

from argus import capabilities
from argus import profile as profiles
from argus.sim.profile_gen import generate_profiles

EXAMPLES = sorted(profiles.EXAMPLES_DIR.glob("*.yaml"))
GENERATED = generate_profiles(100, seed=0)


@pytest.mark.parametrize("path", EXAMPLES, ids=lambda p: p.stem)
def test_examples_are_valid(path):
    doc = profiles.load(path)
    assert profiles.schema_problems(doc) == []
    assert profiles.native_problems(doc) == []


@pytest.mark.parametrize("doc", GENERATED, ids=lambda d: d["id"])
def test_generated_profiles_validate_and_derive(doc):
    assert profiles.problems(doc) == []
    caps = profiles.derive(doc)
    motion = doc["motion"]

    assert caps.profile_id == doc["id"]
    assert caps.motion.space == motion["space"]
    assert caps.motion.can_strafe == motion["can_strafe"]
    assert caps.motion.can_reverse == motion["can_reverse"]
    assert caps.motion.can_hover == (motion["space"] == "aerial")

    for name in "xyz":
        axis = getattr(caps.motion, name)
        assert axis.min_mps <= 0.0 <= axis.max_mps
        if axis.enabled:
            top = max(abs(axis.min_mps), axis.max_mps)
            distances = [caps.motion.stopping_distance_m(name, top * f) for f in (0.25, 0.5, 1.0)]
            assert distances == sorted(distances) and distances[0] > 0.0

    bins = caps.sensing.bins
    assert len(bins) == (72 if motion["space"] == "planar" else 13 * 72)
    for b in bins:
        assert b.edge_distance_m > 0.0
        if b.seen:
            assert b.sensors
            assert 0.0 <= b.first_seen_m <= b.max_seen_m <= caps.sensing.evaluation_range_m + 1e-9
            assert b.first_seen_m <= b.full_height_from_m
            if math.isfinite(b.full_height_from_m):
                assert 0.0 <= b.full_height_reach_m <= b.max_seen_m + 1e-9
            else:
                assert b.full_height_reach_m == 0.0
        else:
            assert b.sensors == [] and math.isinf(b.first_seen_m) and b.full_height_reach_m == 0.0

    range_kinds = {"range_2d", "range_3d", "depth"}
    if not any(s["kind"] in range_kinds for s in doc["sensors"]):
        assert caps.sensing.seen_fraction == 0.0

    drafts = [k for k, v in doc.get("provenance", {}).items() if v == "llm_draft"]
    assert caps.safety_ready == (not drafts)
    assert sorted(caps.unreviewed_fields) == sorted(drafts)


def test_generator_covers_the_spread_we_need():
    spaces = {d["motion"]["space"] for d in GENERATED}
    assert spaces == {"planar", "aerial"}
    assert any(d["motion"]["min_turn_radius_m"] > 0 for d in GENERATED if d["motion"]["space"] == "planar")
    fractions = [profiles.derive(d).sensing.seen_fraction for d in GENERATED[:40]]
    assert min(fractions) == 0.0
    assert max(fractions) == 1.0
    assert any(0.0 < f < 1.0 for f in fractions)
    assert any(not profiles.derive(d).safety_ready for d in GENERATED[:40])


def test_generator_is_deterministic():
    again = generate_profiles(100, seed=0)
    assert [profiles.profile_hash(d) for d in again] == [profiles.profile_hash(d) for d in GENERATED]
    assert generate_profiles(5, seed=1) != GENERATED[:5]


def _base():
    return profiles.load(profiles.EXAMPLES_DIR / "planar_differential.yaml")


def _set(path, value):
    def edit(doc):
        node = doc
        for key in path[:-1]:
            node = node[key]
        node[path[-1]] = value
    return edit


def _delete(path):
    def edit(doc):
        node = doc
        for key in path[:-1]:
            node = node[key]
        del node[path[-1]]
    return edit


# (name, edit, schema should also reject it)
MUTATIONS = [
    ("missing motion", _delete(["motion"]), True),
    ("unknown top-level field", _set(["colour"], "red"), True),
    ("rate as text", _set(["control", "rate_hz"], "fast"), True),
    ("negative rate", _set(["control", "rate_hz"], -5), True),
    ("bad motion space", _set(["motion", "space"], "underwater"), True),
    ("bad sensor kind", _set(["sensors", 0, "kind"], "sonar"), True),
    ("box with two sides", _set(["geometry", "shapes", 0, "size_m"], [0.5, 0.5]), True),
    ("range sensor without fov", _delete(["sensors", 0, "fov_deg"]), True),
    ("creep without speed", _set(["failsafe", "uncovered_motion"], "creep"), True),
    ("bad id", _set(["id"], "Has Spaces"), True),
    ("unknown provenance value", _set(["provenance", "control.latency_s"], "guessed"), True),
    ("sensor frame missing", _set(["sensors", 0, "frame"], "nowhere"), False),
    ("duplicate sensor id", _set(["sensors", 1, "id"], "front_lidar"), False),
    ("sideways speed without strafing", _set(["motion", "limits", "y"],
     {"velocity_mps": [-0.5, 0.5], "accel_mps2": 1.0, "decel_mps2": 1.0}), False),
    ("vertical speed on a planar body", _set(["motion", "limits", "z"],
     {"velocity_mps": [-0.5, 0.5], "accel_mps2": 1.0, "decel_mps2": 1.0}), False),
    ("velocity range without zero", _set(["motion", "limits", "x", "velocity_mps"], [0.2, 1.0]), False),
    ("reverse speed when it cannot reverse", _set(["motion", "can_reverse"], False), False),
    ("planar body told to land", _set(["failsafe", "on_link_lost"], "land"), False),
    ("provenance for a missing field", _set(["provenance", "motion.top_speed"], "declared"), False),
    ("planar body without ground height", _delete(["geometry", "ground_z_m"]), False),
    ("moving axis without braking", _delete(["motion", "limits", "x", "decel_mps2"]), False),
]


@pytest.mark.parametrize("name,edit,schema_rejects", MUTATIONS, ids=[m[0] for m in MUTATIONS])
def test_invalid_profiles_are_rejected(name, edit, schema_rejects):
    doc = copy.deepcopy(_base())
    edit(doc)
    assert profiles.native_problems(doc), f"capability library accepted: {name}"
    if schema_rejects:
        assert profiles.schema_problems(doc), f"schema accepted: {name}"


def test_derive_raises_value_error_on_bad_profile():
    doc = _base()
    doc["sensors"][0]["frame"] = "nowhere"
    with pytest.raises(ValueError, match="not defined in geometry.frames"):
        profiles.derive(doc)


def test_profile_hash_ignores_key_order():
    doc = _base()
    reordered = dict(reversed(list(doc.items())))
    assert profiles.profile_hash(doc) == profiles.profile_hash(reordered)
    changed = copy.deepcopy(doc)
    changed["control"]["latency_s"] = 0.09
    assert profiles.profile_hash(doc) != profiles.profile_hash(changed)
    assert profiles.profile_hash(doc).startswith("sha256:")


def test_bindings_match_formula():
    caps = capabilities.derive_from_file(str(profiles.EXAMPLES_DIR / "planar_differential.yaml"))
    assert caps.motion.stopping_distance_m("x", 1.0) == pytest.approx(0.08 + 1.0 / 2.4)
    with pytest.raises(ValueError):
        caps.motion.stopping_distance_m("w", 1.0)
    assert caps.control["interface"] == "velocity"
    assert caps.failsafe["on_link_lost"] == "stop"
    assert caps.compute.accelerator == "rk3588_npu"
    assert len(caps.collision.footprint) >= 4


def _paths(node, prefix=()):
    yield prefix
    if isinstance(node, dict):
        for key, value in node.items():
            yield from _paths(value, prefix + (key,))
    elif isinstance(node, list):
        for i, value in enumerate(node):
            yield from _paths(value, prefix + (i,))


@pytest.mark.parametrize("example", EXAMPLES, ids=lambda p: p.stem)
def test_loader_reports_instead_of_crashing(example):
    base = profiles.load(example)
    for path in _paths(base):
        if not path:
            continue
        for junk in (None, "junk", [], {}, -1, [1, "a"], {"x": {"y": 1}}):
            doc = copy.deepcopy(base)
            node = doc
            for key in path[:-1]:
                node = node[key]
            node[path[-1]] = junk
            assert isinstance(capabilities.validate_profile_text(profiles.dump(doc)), list)
    for text in ("", "[]", "42", "argus_profile: 1\nsensors: {}", "geometry:\n  frames: [1, 2]"):
        assert capabilities.validate_profile_text(text)
