import fcntl

from duatic_controller_msgs.msg import DriveState, DriveStateCollection

from duatic_monitoring.failure_snapshot_trigger_node import (
    EVENTS_FILE,
    LOCK_FILE,
    FreezeDetector,
    has_state_changed,
    is_new_event,
    prune_sessions,
    prune_snapshots,
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


def session(root, name: str, with_event: bool = True):
    path = root / name
    path.mkdir()
    if with_event:
        (path / EVENTS_FILE).write_text("event\n")
    return path


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


def test_prune_removes_sessions_without_an_event(tmp_path):
    current = session(tmp_path, "2026-01-03_00-00-00-000000", with_event=False)
    empty = session(tmp_path, "2026-01-01_00-00-00-000000", with_event=False)
    kept = session(tmp_path, "2026-01-02_00-00-00-000000")

    assert prune_sessions(tmp_path, current, max_sessions=10) == [empty]
    assert kept.exists() and current.exists() and not empty.exists()


def test_prune_keeps_the_newest_sessions_counting_the_current_one(tmp_path):
    oldest = session(tmp_path, "2026-01-01_00-00-00-000000")
    middle = session(tmp_path, "2026-01-02_00-00-00-000000")
    newest = session(tmp_path, "2026-01-03_00-00-00-000000")
    current = session(tmp_path, "2026-01-04_00-00-00-000000", with_event=False)

    assert prune_sessions(tmp_path, current, max_sessions=3) == [oldest]
    assert middle.exists() and newest.exists() and current.exists()


def test_prune_leaves_other_directories_alone(tmp_path):
    current = session(tmp_path, "2026-01-02_00-00-00-000000", with_event=False)
    unrelated = tmp_path / "maps"
    unrelated.mkdir()

    assert prune_sessions(tmp_path, current, max_sessions=1) == []
    assert unrelated.exists()


def test_prune_leaves_a_session_another_node_holds(tmp_path):
    current = session(tmp_path, "2026-01-02_00-00-00-000000", with_event=False)
    live = session(tmp_path, "2026-01-01_00-00-00-000000", with_event=False)
    with open(live / LOCK_FILE, "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        assert prune_sessions(tmp_path, current, max_sessions=10) == []
    assert live.exists()

    assert prune_sessions(tmp_path, current, max_sessions=10) == [live]


def test_prune_snapshots_keeps_the_newest_and_the_open_file(tmp_path):
    for index in (0, 1, 2, 9, 10):
        (tmp_path / f"snapshot_{index}.mcap").touch()

    removed = prune_snapshots(tmp_path, max_snapshots=2)

    assert [f.name for f in removed] == ["snapshot_0.mcap", "snapshot_1.mcap"]
    assert sorted(f.name for f in tmp_path.iterdir()) == [
        "snapshot_10.mcap",
        "snapshot_2.mcap",
        "snapshot_9.mcap",
    ]
