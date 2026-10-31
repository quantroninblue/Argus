"""Throwaway certificates for tests: a CA, the hub (valid for localhost) and some nodes."""

import subprocess
from pathlib import Path


def _run(*args, cwd):
    subprocess.run(["openssl", *args], cwd=cwd, check=True, capture_output=True)


def _key(name, cwd):
    _run("ecparam", "-name", "prime256v1", "-genkey", "-noout", "-out", f"{name}.key", cwd=cwd)


def _signed(name, cn, ext, ca, cwd):
    _key(name, cwd)
    _run("req", "-new", "-key", f"{name}.key", "-subj", f"/CN={cn}", "-out", f"{name}.csr", cwd=cwd)
    (Path(cwd) / f"{name}.ext").write_text(ext)
    _run("x509", "-req", "-in", f"{name}.csr", "-CA", f"{ca}.pem", "-CAkey", f"{ca}.key", "-CAcreateserial",
         "-days", "1", "-extfile", f"{name}.ext", "-out", f"{name}.pem", cwd=cwd)


def make_ca(directory, name="ca"):
    _key(name, directory)
    _run("req", "-x509", "-new", "-key", f"{name}.key", "-subj", f"/CN=argus-test-{name}", "-days", "1",
         "-out", f"{name}.pem", cwd=directory)


def make_hub(directory, ca="ca"):
    _signed("hub", "argus-hub", "subjectAltName=DNS:localhost,IP:127.0.0.1\nextendedKeyUsage=serverAuth\n",
            ca, directory)


def make_node(directory, node_id, ca="ca", name=None):
    _signed(name or node_id, node_id, "extendedKeyUsage=clientAuth\n", ca, directory)
