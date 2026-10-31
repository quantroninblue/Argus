from google.protobuf.descriptor import FieldDescriptor

from argus.protocol.v1 import (
    common_pb2,
    hub_messages_pb2,
    link_pb2,
    node_messages_pb2,
    session_pb2,
)

MODULES = [common_pb2, session_pb2, node_messages_pb2, hub_messages_pb2, link_pb2]

# Words that would mean the hub is reaching past waypoints into motor control.
FORBIDDEN_WORDS = ("velocity", "twist", "torque", "thrust", "setpoint", "cmd", "effort", "throttle")


def _message_types(descriptor, seen=None):
    seen = set() if seen is None else seen
    if descriptor.full_name in seen:
        return seen
    seen.add(descriptor.full_name)
    for field in descriptor.fields:
        if field.type == FieldDescriptor.TYPE_MESSAGE:
            _message_types(field.message_type, seen)
    return seen


def _fields(descriptor, seen=None):
    seen = set() if seen is None else seen
    if descriptor.full_name in seen:
        return
    seen.add(descriptor.full_name)
    for field in descriptor.fields:
        yield descriptor.full_name, field
        if field.type == FieldDescriptor.TYPE_MESSAGE:
            yield from _fields(field.message_type, seen)


def test_mission_plan_round_trip():
    out = link_pb2.HubMessage(seq=4)
    plan = out.mission_plan
    plan.generation = 12
    plan.expires.unix_nanos = 1_700_000_000_000_000_000
    wp = plan.waypoints.add(id="wp-1", speed_cap_mps=0.5, speculative=True)
    wp.pose.position.x = 3.0
    back = link_pb2.HubMessage.FromString(out.SerializeToString())
    assert back.WhichOneof("body") == "mission_plan"
    assert back.mission_plan.generation == 12
    assert back.mission_plan.waypoints[0].pose.position.x == 3.0


def test_hub_can_only_send_the_agreed_messages():
    body = link_pb2.HubMessage.DESCRIPTOR.oneofs_by_name["body"]
    assert {f.name for f in body.fields} == {
        "session_accept", "session_reject", "heartbeat", "mission_plan", "plan_cancel",
        "map_correction", "map_delta", "stream_request", "mode_request",
    }


def test_hub_messages_carry_no_motor_commands():
    reachable = _message_types(link_pb2.HubMessage.DESCRIPTOR)
    assert "argus.protocol.v1.MeasuredVelocity" not in reachable
    for owner, field in _fields(link_pb2.HubMessage.DESCRIPTOR):
        for word in FORBIDDEN_WORDS:
            assert word not in field.name, f"{owner}.{field.name}"


def test_every_enum_starts_unspecified():
    for module in MODULES:
        for enum in module.DESCRIPTOR.enum_types_by_name.values():
            assert enum.values[0].number == 0
            assert enum.values[0].name.endswith("_UNSPECIFIED"), enum.name


def test_failure_causes_match_the_architecture():
    names = {v.name for v in common_pb2.FailureCause.DESCRIPTOR.values}
    assert names == {
        "FAILURE_CAUSE_UNSPECIFIED", "FAILURE_CAUSE_DEAD_END_OBSERVED",
        "FAILURE_CAUSE_SEARCHED_NO_GOAL", "FAILURE_CAUSE_TEMPORARY_OBSTRUCTION",
        "FAILURE_CAUSE_CONTROLLER_NO_PROGRESS", "FAILURE_CAUSE_LOCALIZATION_INVALID",
        "FAILURE_CAUSE_SENSOR_INVALID", "FAILURE_CAUSE_COMPUTE_DEADLINE_MISS",
        "FAILURE_CAUSE_LINK_LOST",
    }
