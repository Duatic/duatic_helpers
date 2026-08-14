from types import SimpleNamespace

from duatic_controller_msgs.msg import DriveState, DriveStateCollection
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from duatic_monitoring.estop_error_logger_node import (
    SEVERITY_NAMES,
    BufferedTopic,
    EStopErrorLoggerNode,
)

has_state_changed = EStopErrorLoggerNode._has_state_changed
is_latched = EStopErrorLoggerNode._is_latched
compatible_reliability = EStopErrorLoggerNode._compatible_reliability

SEC = 1_000_000_000
NO_BUDGET = 1 << 40


def publisher(durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE):
    """Build a stand-in for the TopicEndpointInfo returned by the ROS graph."""
    qos = SimpleNamespace(durability=durability, reliability=reliability)
    return SimpleNamespace(qos_profile=qos)


def buffered_topic(is_latched: bool = False, **kwargs) -> BufferedTopic:
    return BufferedTopic(type_name="std_msgs/msg/String", subscription=None, is_latched=is_latched)


def collection(*positions: float) -> DriveStateCollection:
    """Build a DriveStateCollection with one drive per given joint position."""
    msg = DriveStateCollection()
    for position in positions:
        state = DriveState()
        state.joint_position = position
        msg.states.append(state)
    return msg


def test_identical_readings_are_not_a_change():
    assert not has_state_changed(collection(1.0, 2.0), collection(1.0, 2.0))


def test_changed_joint_position_is_a_change():
    assert has_state_changed(collection(1.0, 2.0), collection(1.0, 2.5))


def test_smallest_possible_jitter_is_a_change():
    # A live drive is detected purely by the last bits of its readings moving.
    assert has_state_changed(collection(1.0), collection(1.0 + 1e-15))


def test_differing_drive_count_is_a_change():
    assert has_state_changed(collection(1.0), collection(1.0, 2.0))


def test_trim_evicts_messages_older_than_the_window():
    topic = buffered_topic()
    for age_sec in (20, 10, 1):
        topic.append(100 * SEC - age_sec * SEC, b"payload")

    topic.trim(cutoff_ns=100 * SEC - 15 * SEC, budget_bytes=NO_BUDGET)

    assert [t for t, _ in topic.messages] == [90 * SEC, 99 * SEC]
    assert topic.buffered_bytes == 2 * len(b"payload")


def test_trim_of_a_silent_topic_empties_it():
    # The regression that stretched the dumped bag to minutes: a topic that published at
    # startup and then went quiet kept its stale messages, because eviction only ever ran
    # from its own callback. A clock-driven trim has to drain it even with nothing arriving.
    topic = buffered_topic()
    topic.append(0, b"transition_event")

    topic.trim(cutoff_ns=141 * SEC, budget_bytes=NO_BUDGET)

    assert not topic.messages
    assert topic.buffered_bytes == 0


def test_trim_keeps_latched_messages_however_old():
    topic = buffered_topic(is_latched=True)
    topic.append(0, b"<robot/>")  # /robot_description, latched at startup

    topic.trim(cutoff_ns=141 * SEC, budget_bytes=NO_BUDGET)

    assert [t for t, _ in topic.messages] == [0]


def test_trim_still_holds_latched_topics_to_the_byte_budget():
    topic = buffered_topic(is_latched=True)
    topic.append(0, b"stale")
    topic.append(1 * SEC, b"fresh")

    topic.trim(cutoff_ns=0, budget_bytes=len(b"fresh"))

    assert [t for t, _ in topic.messages] == [1 * SEC]


def test_topic_is_latched_only_when_every_publisher_is_transient_local():
    transient_local = publisher(durability=DurabilityPolicy.TRANSIENT_LOCAL)
    volatile = publisher(durability=DurabilityPolicy.VOLATILE)

    assert is_latched([transient_local])
    assert is_latched([transient_local, transient_local])
    assert not is_latched([volatile])
    # A transient-local subscription would not match the volatile publisher at all, so the
    # retained sample is not worth losing that publisher's live data over.
    assert not is_latched([transient_local, volatile])


def test_reliability_drops_to_best_effort_unless_every_publisher_is_reliable():
    reliable = publisher(reliability=ReliabilityPolicy.RELIABLE)
    best_effort = publisher(reliability=ReliabilityPolicy.BEST_EFFORT)

    assert compatible_reliability([reliable]) == ReliabilityPolicy.RELIABLE
    # A reliable subscription does not match a best-effort publisher.
    assert compatible_reliability([reliable, best_effort]) == ReliabilityPolicy.BEST_EFFORT
    assert compatible_reliability([best_effort]) == ReliabilityPolicy.BEST_EFFORT


def test_severity_names_use_ros2_log_levels():
    # Regression guard: ROS 1 numbered these 1/2/4/8/16, ROS 2 uses 10/20/30/40/50. Getting
    # this wrong silently renders every dumped log line as "LVL20" instead of "INFO".
    assert SEVERITY_NAMES[Log.DEBUG] == "DEBUG"
    assert SEVERITY_NAMES[Log.INFO] == "INFO"
    assert SEVERITY_NAMES[Log.WARN] == "WARN"
    assert SEVERITY_NAMES[Log.ERROR] == "ERROR"
    assert SEVERITY_NAMES[Log.FATAL] == "FATAL"
