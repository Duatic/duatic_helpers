import fcntl

from duatic_monitoring.failure_snapshot_node import EVENTS_FILE, LOCK_FILE, SNAPSHOT_BAG, prune

MB = 1_000_000


def session(root, name: str, snapshot_sizes: tuple[int, ...] = ()):
    """Create a session directory whose snapshot bag holds files of the given sizes.

    A session with snapshots also has an event and the empty file the recorder leaves last.
    """
    path = root / name
    (path / SNAPSHOT_BAG).mkdir(parents=True)
    if snapshot_sizes:
        (path / EVENTS_FILE).write_text("event\n")
        for index, size in enumerate((*snapshot_sizes, 0)):
            (path / SNAPSHOT_BAG / f"snapshot_{index}.mcap").write_bytes(b"x" * size)
    return path


def test_prune_removes_the_oldest_snapshots_over_the_budget(tmp_path):
    old = session(tmp_path, "2026-01-01_00-00-00-000000", (3 * MB, 3 * MB))
    new = session(tmp_path, "2026-01-02_00-00-00-000000", (3 * MB,))
    current = session(tmp_path, "2026-01-03_00-00-00-000000")

    snapshots, sessions = prune(tmp_path, current, max_bytes=7 * MB)

    assert snapshots == [old / SNAPSHOT_BAG / "snapshot_0.mcap"]
    assert sessions == []
    assert (old / SNAPSHOT_BAG / "snapshot_1.mcap").exists()
    assert (new / SNAPSHOT_BAG / "snapshot_0.mcap").exists()


def test_prune_reaches_into_the_current_session_but_not_its_open_file(tmp_path):
    current = session(tmp_path, "2026-01-01_00-00-00-000000", (3 * MB, 3 * MB))

    snapshots, sessions = prune(tmp_path, current, max_bytes=4 * MB)

    assert snapshots == [current / SNAPSHOT_BAG / "snapshot_0.mcap"]
    assert sessions == []
    assert sorted(f.name for f in (current / SNAPSHOT_BAG).iterdir()) == [
        "snapshot_1.mcap",
        "snapshot_2.mcap",
    ]


def test_prune_removes_a_finished_session_once_it_holds_no_snapshot(tmp_path):
    emptied = session(tmp_path, "2026-01-01_00-00-00-000000", (3 * MB,))
    no_event = session(tmp_path, "2026-01-02_00-00-00-000000")
    kept = session(tmp_path, "2026-01-03_00-00-00-000000", (3 * MB,))
    current = session(tmp_path, "2026-01-04_00-00-00-000000")

    snapshots, sessions = prune(tmp_path, current, max_bytes=4 * MB)

    assert snapshots == [emptied / SNAPSHOT_BAG / "snapshot_0.mcap"]
    assert sessions == [emptied, no_event]
    assert kept.exists() and current.exists()


def test_prune_leaves_other_directories_alone(tmp_path):
    current = session(tmp_path, "2026-01-02_00-00-00-000000")
    unrelated = tmp_path / "maps"
    unrelated.mkdir()

    assert prune(tmp_path, current, max_bytes=0) == ([], [])
    assert unrelated.exists()


def test_prune_leaves_a_session_another_node_holds(tmp_path):
    current = session(tmp_path, "2026-01-02_00-00-00-000000")
    live = session(tmp_path, "2026-01-01_00-00-00-000000")
    with open(live / LOCK_FILE, "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        assert prune(tmp_path, current, max_bytes=0) == ([], [])
    assert live.exists()

    assert prune(tmp_path, current, max_bytes=0) == ([], [live])
