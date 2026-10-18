"""Package only a completed, tracking-checked G1 record and its video."""

import argparse
import hashlib
import json
import zipfile
from pathlib import Path

import numpy as np

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("record", type=Path)
p.add_argument("destination", type=Path)
a = p.parse_args()
checks = json.loads((a.record / "writing_checks.json").read_text())
if not checks["writing_tracking_pass"] or checks["smoke_truncated"]:
    raise SystemExit("Refusing to package failed or truncated writing as a full demonstration")
with np.load(a.record / "rollout.npz", allow_pickle=False) as z:
    assert z["qpos"].shape[0] == len(z["time_s"])
    assert np.isfinite(z["qpos"]).all()
    assert np.allclose(np.diff(z["time_s"]), 1 / float(z["fps"]))
    assert hashlib.sha256((a.record / "scene.xml").read_bytes()).hexdigest() == str(
        z["scene_sha256"]
    )
paths = [
    "scene.xml",
    "layout.json",
    "reference.npz",
    "rollout.npz",
    "metrics.json",
    "writing_checks.json",
    "videos/g1_contact_writing.mp4",
    "videos/final.jpg",
]
for name in paths:
    if not (a.record / name).is_file() or (a.record / name).stat().st_size == 0:
        raise SystemExit(f"Missing or empty artifact: {name}")
readme = """# WriteUrRobo — G1 recorded contact-writing demo

This archive contains one complete simulated writing run and its original-speed video.
It is not a hardware command file or a demonstration of unsupported standing balance.
The base is fixed, robot links are gravity compensated, and the pen is rigidly attached.
The nib is compliant; displayed ink comes from actual nib–paper contacts.

Install WriteUrRobo v0.3.0, fetch the pinned public G1 meshes, then replay:

    g1-fetch-writing-model --output external/g1
    MUJOCO_GL=egl g1-render-writing --output g1_record --model-dir external/g1

The XML's original relative mesh path is retained so its recorded hash stays valid.
--model-dir changes only the in-memory search path, not the recorded robot model.
See https://github.com/MrAmazing-h/WriteUrRobo/blob/main/docs/G1_WRITING.md
for assumptions, controller settings, units, frame conventions and reproduction.

rollout.npz stores actual qpos/qvel at 100 Hz; time_s begins at 0.01 s (post-step).
joint_names and qpos_addresses/qvel_addresses specify ordering. The passive nib
slide is included. Metres, radians, seconds, newtons; right-handed Z-up world.
reference.npz is the planned command, NOT the actual motion shown in the video.
writing_checks.json covers tracking/contact only. Enabled self-contacts and force
peaks remain documented in metrics.json; no hardware safety claim is made.
"""
a.destination.parent.mkdir(parents=True, exist_ok=True)
with zipfile.ZipFile(a.destination, "x", compression=zipfile.ZIP_DEFLATED) as archive:
    hashes = []
    for name in paths:
        data = (a.record / name).read_bytes()
        archive.writestr("g1_record/" + name, data)
        hashes.append(hashlib.sha256(data).hexdigest() + "  g1_record/" + name)
    archive.writestr("README.md", readme)
    archive.writestr("SHA256SUMS", "\n".join(hashes) + "\n")
with zipfile.ZipFile(a.destination) as archive:
    assert archive.testzip() is None
print(a.destination, hashlib.sha256(a.destination.read_bytes()).hexdigest())
