import subprocess

from argus.__main__ import main


def _subject(pem):
    out = subprocess.run(["openssl", "x509", "-in", str(pem), "-noout", "-subject", "-ext", "subjectAltName"],
                         capture_output=True, text=True, check=True).stdout
    return out


def test_certs_command_makes_a_ca_hub_and_robot(tmp_path):
    d = tmp_path / "pki"
    assert main(["certs", "ca", str(d)]) == 0
    assert main(["certs", "hub", str(d), "--host", "hub.local", "--host", "10.0.0.5"]) == 0
    assert main(["certs", "robot", str(d), "robot-7"]) == 0
    hub = _subject(d / "hub.pem")
    assert "DNS:hub.local" in hub and "IP Address:10.0.0.5" in hub
    assert "CN = robot-7" in _subject(d / "robot-7.pem")
    for key in d.glob("*.key"):
        assert key.stat().st_mode & 0o077 == 0, key
    subprocess.run(["openssl", "verify", "-CAfile", str(d / "ca.pem"), str(d / "robot-7.pem")], check=True,
                   capture_output=True)


def test_certs_command_never_overwrites_a_key(tmp_path):
    assert main(["certs", "robot", str(tmp_path), "r1"]) == 1  # no CA yet
    assert main(["certs", "ca", str(tmp_path)]) == 0
    before = (tmp_path / "ca.key").read_bytes()
    assert main(["certs", "ca", str(tmp_path)]) == 1
    assert (tmp_path / "ca.key").read_bytes() == before
