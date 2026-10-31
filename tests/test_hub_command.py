import asyncio
import shutil

from argus import certs
from argus import profile as profiles
from argus.__main__ import main
from argus.hub.run import serve


def _hub_dir(tmp_path):
    d = tmp_path / "hub"
    (d / "profiles").mkdir(parents=True)
    (d / "pki").mkdir()
    shutil.copy(profiles.EXAMPLES_DIR / "planar_differential.yaml", d / "profiles" / "diff.yaml")
    (d / "robots.yaml").write_text("rover-1: {profile: example-planar-differential, site: lab}\n")
    certs.make_ca(d / "pki")
    certs.make_hub(d / "pki", ["localhost", "127.0.0.1"])
    return d


def test_hub_console_sends_goals_and_reports_status(tmp_path, capsys):
    d = _hub_dir(tmp_path)

    async def lines():
        for line in ["status\n", "goal rover-1 3 1.5\n", "goal nobody 1 1\n", "goal rover-1 x y\n",
                     "status\n", "cancel rover-1\n", "status\n", "quit\n", "status\n"]:
            yield line

    asyncio.run(serve(d, "127.0.0.1:0", lines()))
    out = capsys.readouterr().out
    assert "hub listening on 127.0.0.1:" in out and "1 robot(s) registered" in out
    assert "rover-1: not connected" in out
    assert "rover-1: goal 3 1.5 0, tolerance 0.4 m" in out
    assert "no robot 'nobody'" in out
    assert "could not convert" in out  # bad numbers get the usage, not a crash
    assert "rover-1: cancelled" in out
    assert out.count("rover-1: not connected") == 3  # nothing after quit


def test_hub_refuses_an_incomplete_directory(tmp_path, capsys):
    (tmp_path / "profiles").mkdir()
    assert main(["hub", str(tmp_path)]) == 1
    err = capsys.readouterr().err
    assert "robots.yaml" in err and "pki/hub.key" in err
