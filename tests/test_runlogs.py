import pytest

from argus.hub.runlogs import RunLogStore


def _log(tmp_path, name="log"):
    d = tmp_path / name
    d.mkdir()
    (d / "run.json").write_text('{"outcome": {}}')
    (d / "bag").mkdir()
    (d / "bag" / "bag_0.mcap").write_bytes(b"\x89MCAP0\r\n")
    return d


def test_reading_a_run_deletes_it_unless_kept(tmp_path):
    store = RunLogStore(tmp_path / "store")
    a = store.add(_log(tmp_path, "a"))
    b = store.add(_log(tmp_path, "b"), keep=True)
    assert set(store.ids()) == {a, b}

    got = store.consume(a)
    assert got["run.json"] == '{"outcome": {}}'
    assert got["bag"].startswith("<")
    assert store.ids() == [b]

    store.consume(b)
    assert store.ids() == [b] and store.is_kept(b)
    store.keep(b, False)
    assert "KEEP" not in store.read(b)
    store.consume(b)
    assert store.ids() == []


def test_run_ids_cannot_leave_the_store(tmp_path):
    store = RunLogStore(tmp_path / "store")
    for bad in ("..", "../x", "a/b", ""):
        with pytest.raises(ValueError):
            store.path(bad)
    with pytest.raises(KeyError):
        store.path("missing")
