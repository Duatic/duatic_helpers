from duatic_controller_msgs.msg import DriveState, DriveStateCollection

from duatic_monitoring.estop_monitor_node import (
    FreezeDetector,
    has_state_changed,
    is_new_event,
)

SEC = 1_000_000_000


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


def test_reading_that_stays_nan_is_not_a_change():
    assert not has_state_changed(collection(float("nan")), collection(float("nan")))


def test_reading_that_becomes_nan_is_a_change():
    assert has_state_changed(collection(1.0), collection(float("nan")))


def test_topic_that_never_moved_is_not_frozen():
    detector = FreezeDetector(timeout_ns=5 * SEC)
    detector.update(collection(1.0), 0)
    detector.update(collection(1.0), 60 * SEC)
    assert not detector.is_frozen(60 * SEC)


def test_live_topic_freezes_after_the_timeout():
    detector = FreezeDetector(timeout_ns=5 * SEC)
    detector.update(collection(1.0), 0)
    detector.update(collection(2.0), 1 * SEC)
    detector.update(collection(2.0), 5 * SEC)
    assert not detector.is_frozen(6 * SEC)
    detector.update(collection(2.0), 7 * SEC)
    assert detector.is_frozen(7 * SEC)
    assert not detector.is_silent(7 * SEC)


def test_live_topic_that_goes_silent_is_frozen():
    detector = FreezeDetector(timeout_ns=5 * SEC)
    detector.update(collection(1.0), 0)
    detector.update(collection(2.0), 1 * SEC)
    assert detector.is_frozen(7 * SEC)
    assert detector.is_silent(7 * SEC)


def test_disarmed_topic_freezes_again_only_after_moving():
    detector = FreezeDetector(timeout_ns=5 * SEC)
    detector.update(collection(1.0), 0)
    detector.update(collection(2.0), 1 * SEC)
    detector.disarm()
    assert not detector.is_frozen(20 * SEC)
    detector.update(collection(3.0), 21 * SEC)
    assert detector.is_frozen(27 * SEC)


def test_first_stopped_topic_is_a_new_event():
    assert is_new_event([3 * SEC], last_trigger_ns=None)


def test_topic_that_stopped_before_the_last_trigger_joins_its_event():
    assert not is_new_event([4 * SEC], last_trigger_ns=9 * SEC)


def test_topic_that_moved_after_the_last_trigger_is_a_new_event():
    assert is_new_event([4 * SEC, 10 * SEC], last_trigger_ns=9 * SEC)
