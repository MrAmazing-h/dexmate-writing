import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

from dexmate_writing import handwriting as hw
from dexmate_writing.model_contract import validate_model


def test_packaged_strokes_and_paper_bounds():
    strokes = hw.paths()
    assert len(strokes) == 53
    assert set(c for c, _, _ in strokes) == set(range(9))
    points = np.concatenate([p for _, _, p in strokes])
    assert np.isfinite(points).all()
    assert np.all((points[:, 0] > 0.28) & (points[:, 0] < 0.46))
    assert np.all((points[:, 1] > -0.355) & (points[:, 1] < 0.145))
    np.testing.assert_allclose(points[:, 2], 0.9033)


def test_minimum_jerk_blend_is_monotonic_with_endpoint_easing():
    t = np.linspace(0, 1, 1001)
    x = hw.quintic(t)
    assert x[0] == 0 and x[-1] == 1
    assert (np.diff(x) >= 0).all()
    assert (x[1] - x[0]) < 1e-7 and (x[-1] - x[-2]) < 1e-7


def test_incompatible_model_rejected():
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body><freejoint/><geom size=".1" mass="1"/></body></worldbody></mujoco>'
    )
    with pytest.raises(ValueError, match="Incompatible"):
        validate_model(model)


def test_recorded_motion_tip_gate_and_timing():
    repo = Path(__file__).resolve().parents[1]
    z = np.load(repo / "examples/writing_reference.npz", allow_pickle=False)
    assert z["qpos"].shape == (3654, 55)
    assert float(z["fps"]) == 60
    err = np.linalg.norm(z["pen_tip_actual"] - z["pen_tip_target"], axis=1)
    assert err[z["pen_down"]].max() < 0.001
    expected = json.loads((repo / "results/recorded_evaluation.json").read_text())
    assert abs(err.max() * 1000 - expected["tip_error_max_mm"]) < 1e-9
