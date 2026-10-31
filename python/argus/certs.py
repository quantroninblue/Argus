"""Certificates for Argus with openssl: a CA that signs the hub and every robot.

A robot's certificate carries its node id as the common name; the hub checks it against the
Hello. Keys are P-256.
"""

from __future__ import annotations

import subprocess
from pathlib import Path


def _run(*args, cwd) -> None:
    subprocess.run(["openssl", *args], cwd=cwd, check=True, capture_output=True)


def _key(name: str, cwd) -> None:
    _run("ecparam", "-name", "prime256v1", "-genkey", "-noout", "-out", f"{name}.key", cwd=cwd)


def _signed(name: str, cn: str, ext: str, ca: str, cwd, days: int) -> None:
    _key(name, cwd)
    _run("req", "-new", "-key", f"{name}.key", "-subj", f"/CN={cn}", "-out", f"{name}.csr", cwd=cwd)
    (Path(cwd) / f"{name}.ext").write_text(ext)
    _run("x509", "-req", "-in", f"{name}.csr", "-CA", f"{ca}.pem", "-CAkey", f"{ca}.key", "-CAcreateserial",
         "-days", str(days), "-extfile", f"{name}.ext", "-out", f"{name}.pem", cwd=cwd)


def make_ca(directory, name: str = "ca", days: int = 3650) -> None:
    _key(name, directory)
    _run("req", "-x509", "-new", "-key", f"{name}.key", "-subj", f"/CN=argus-{name}", "-days", str(days),
         "-out", f"{name}.pem", cwd=directory)


def make_hub(directory, hosts: list[str], ca: str = "ca", days: int = 825) -> None:
    """Hub certificate valid for `hosts` (names or IP addresses)."""
    alt = ",".join(("IP:" if h.replace(".", "").isdigit() else "DNS:") + h for h in hosts)
    _signed("hub", "argus-hub", f"subjectAltName={alt}\nextendedKeyUsage=serverAuth\n", ca, directory, days)


def make_node(directory, node_id: str, ca: str = "ca", days: int = 825) -> None:
    _signed(node_id, node_id, "extendedKeyUsage=clientAuth\n", ca, directory, days)
