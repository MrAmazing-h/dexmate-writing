import json
import os
import xml.etree.ElementTree as ET
from importlib.resources import files
from pathlib import Path

import mujoco
import numpy as np
import pytest

from dexmate_writing.g1.fetch import fetch
from dexmate_writing.g1.render import ink_segments
from dexmate_writing.g1.write import ARM, make_scene, path, solve


def test_pinned_mesh_manifest_covers_model():
    root = files("dexmate_writing.g1")
    manifest = json.loads(root.joinpath("data/assets.json").read_text())
    xml = ET.fromstring(root.joinpath("data/g1_inspire.xml").read_text())
    paths = {r["path"] for r in manifest["files"]}
    assert len(paths) == len(manifest["files"])
    for mesh in xml.findall("asset/mesh"):
        assert "meshes/" + mesh.get("file") in paths
    for row in manifest["files"]:
        assert manifest["unitree_revision"] in row["url"]
        assert len(row["sha256"]) == 64
    joints = {j.get("name") for j in xml.findall(".//joint")}
    assert set(ARM) <= joints


def test_fetch_refuses_modified_cache(tmp_path):
    (tmp_path / "mesh.stl").write_bytes(b"user modified")
    with pytest.raises(ValueError, match="refusing overwrite"):
        fetch({"path": "mesh.stl", "sha256": "0" * 64}, tmp_path)
    assert (tmp_path / "mesh.stl").read_bytes() == b"user modified"


def test_path_lifts_and_continuity():
    strokes = [
        np.array([[0.35, -0.2, 0.751], [0.35, -0.22, 0.751]]),
        np.array([[0.36, -0.2, 0.751], [0.36, -0.22, 0.751]]),
    ]
    points, down, ids = path(strokes)
    assert np.allclose(points[down, 2], 0.751)
    assert points[~down, 2].max() == pytest.approx(0.769)
    assert np.linalg.norm(np.diff(points, axis=0), axis=1).max() < 0.001
    assert set(ids) == {0, 1}
    assert not down[0] and not down[-1]


def test_render_does_not_connect_separate_contacts():
    ink = np.array(
        [
            [0, 0, 0.1, 0.1, 0.75],
            [1, 0, 0.101, 0.1, 0.75],
            [20, 0, 0.102, 0.1, 0.75],
            [21, 1, 0.103, 0.1, 0.75],
        ]
    )
    segments = ink_segments(ink)
    assert len(segments) == 1
    assert segments[0][0] == 1
    assert len(ink_segments(np.empty((0, 5)))) == 0


@pytest.mark.skipif(not os.environ.get("G1_MODEL_DIR"), reason="Pinned meshes not fetched")
def test_actual_scene_and_centre_ik(tmp_path):
    m, q, orientation = make_scene(Path(os.environ["G1_MODEL_DIR"]).resolve(), tmp_path)
    assert m.nq == m.nv == 54  # 29 body + 24 finger hinges + passive nib
    assert m.nu == 41 and m.neq == 12
    assert not np.any(m.jnt_type == mujoco.mjtJoint.mjJNT_FREE)
    assert np.allclose(m.opt.gravity, [0, 0, -9.81])
    assert m.geom("nib").contype and m.geom("paper").contype
    d = mujoco.MjData(m)
    q, error = solve(m, d, q, np.array([0.35, -0.20, 0.769]), orientation, iterations=300)
    assert error < 0.002
    assert np.isfinite(q).all()
