import string
import zipfile

import numpy as np
import pytest

from dexmate_writing.contact.fetch_station import extract
from dexmate_writing.contact.text_layout import layout


@pytest.mark.parametrize(
    "text", ["Yuxiong Lab", "DexMate 2026!\nHello, Lab.", "longword" * 4, "Hello\r\n123\tworld"]
)
def test_layout_respects_station_workspace(text, tmp_path):
    result = layout(text, tmp_path)
    points = np.concatenate([s["points"] for s in result["strokes"]])
    assert np.isfinite(points).all()
    assert points[:, 0].min() >= 0.345 - 1e-9
    assert points[:, 0].max() <= 0.415 + 1e-9
    assert points[:, 1].min() >= -0.300 - 1e-9
    assert points[:, 1].max() <= -0.020 + 1e-9


@pytest.mark.parametrize("text", ["", "  \n ", "你好", "hello\x00", "A" * 501])
def test_invalid_input_is_rejected(text, tmp_path):
    with pytest.raises(ValueError):
        layout(text, tmp_path)


def test_printable_glyphs_have_finite_paths(tmp_path):
    # Coverage of the layout vocabulary is not a physical execution test.
    for index, ch in enumerate(string.ascii_letters + string.digits + string.punctuation):
        result = layout(ch, tmp_path / str(index))
        assert result["strokes"], repr(ch)
        assert all(np.isfinite(s["points"]).all() for s in result["strokes"])


def test_download_archive_cannot_escape_destination(tmp_path):
    archive = tmp_path / "bad.zip"
    with zipfile.ZipFile(archive, "w") as z:
        z.writestr("vega_1p_contact_station/../../escape", "no")
    with pytest.raises(ValueError):
        extract(archive, tmp_path / "output")
    assert not (tmp_path / "escape").exists()
