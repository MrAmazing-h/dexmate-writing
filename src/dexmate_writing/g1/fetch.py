"""Download pinned official G1/Inspire meshes; verify cached files as well."""

import argparse
import hashlib
import json
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from importlib.resources import files
from pathlib import Path


def fetch(row, output):
    dest = output / row["path"]
    if dest.exists():
        if hashlib.sha256(dest.read_bytes()).hexdigest() != row["sha256"]:
            raise ValueError(f"Cached asset differs; refusing overwrite: {dest}")
        return
    dest.parent.mkdir(parents=True, exist_ok=True)
    temp = dest.with_suffix(dest.suffix + ".partial")
    urllib.request.urlretrieve(row["url"], temp)
    if hashlib.sha256(temp.read_bytes()).hexdigest() != row["sha256"]:
        raise ValueError(f"Downloaded asset checksum mismatch: {dest.name}")
    temp.replace(dest)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, default=Path("external/g1"))
    a = p.parse_args()
    manifest = json.loads(files(__package__).joinpath("data/assets.json").read_text())
    with ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(lambda row: fetch(row, a.output), manifest["files"]))
    print(f"Verified {len(manifest['files'])} public meshes: {a.output}")


if __name__ == "__main__":
    main()
