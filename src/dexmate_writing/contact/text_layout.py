"""Single-line Hershey ASCII strokes, word wrapping and bounded page fitting."""

import json
import re
from pathlib import Path

import numpy as np
from HersheyFonts import HersheyFonts
from PIL import Image, ImageDraw


def layout(text, out, max_cap_mm=26):
    text = text.replace("\r\n", "\n").replace("\r", "\n").expandtabs(4)
    if not text.strip():
        raise ValueError("Text must contain visible characters.")
    bad = sorted(set(c for c in text if c != "\n" and not 32 <= ord(c) <= 126))
    if bad:
        raise ValueError(f"Only printable ASCII and line breaks are supported: {bad}")
    if len(text) > 500:
        raise ValueError("Too much text for this single-page writing station.")
    if not 10 <= max_cap_mm <= 30:
        raise ValueError("Maximum capital height must be 10 to 30 mm.")
    font = HersheyFonts()
    font.load_default_font("futural")
    width = 0.28
    height = 0.070
    cap = 21.0
    chosen = None

    def measure(s):
        return sum(g.char_width for g in font.glyphs_for_text(s))

    for mm in np.linspace(max_cap_mm, 10, 65):
        scale = mm / 1000 / cap
        limit = width / scale
        lines = []
        for paragraph in text.split("\n"):
            line = ""
            for token in re.findall(r"\S+| +", paragraph):
                if token.isspace():
                    if line:
                        line += " "
                    continue
                if measure(line + token) <= limit:
                    line += token
                    continue
                if line.strip():
                    lines.append(line.rstrip())
                    line = ""
                for ch in token:
                    if measure(line + ch) > limit:
                        lines.append(line)
                        line = ""
                    line += ch
            lines.append(line.rstrip())
        paths = []
        for li, line in enumerate(lines):
            for stroke in font.strokes_for_text(line):
                p = np.asarray(stroke, float)
                p[:, 1] += li * 36
                if len(p) > 1:
                    paths.append((li, p))
        allpts = np.concatenate([p for _, p in paths])
        lower = allpts.min(0)
        upper = allpts.max(0)
        if (upper[1] - lower[1]) * scale <= height and (upper[0] - lower[0]) * scale <= width:
            chosen = mm, scale, lines, paths, lower, upper
            break
    if chosen is None:
        raise ValueError(
            "Text does not fit at readable size (minimum 10 mm capitals); shorten it or use separate pages."
        )
    mm, scale, lines, paths, lower, upper = chosen
    center = (lower[1] + upper[1]) / 2
    strokes = []
    for li, p in paths:
        # Screen right = world -Y; screen down = world -X.
        world = np.c_[
            0.380 - (p[:, 1] - center) * scale, -0.020 - p[:, 0] * scale, np.full(len(p), 0.9033)
        ]
        # Remove duplicate consecutive samples to make arc interpolation well-defined.
        world = world[np.r_[True, np.linalg.norm(np.diff(world, axis=0), axis=1) > 1e-12]]
        if len(world) > 1:
            strokes.append({"line": li, "points": world.tolist()})
    report = {
        "text": text,
        "font": "Hershey futural",
        "capital_height_mm": float(mm),
        "lines": lines,
        "strokes": strokes,
        "workspace_world_xy_m": {"x": [0.345, 0.415], "y": [-0.300, -0.020]},
        "axis_mapping": "page right = world -Y; page down = world -X",
        "note": "Layout is a reference, not proof of successful physical contact.",
    }
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "layout.json").write_text(json.dumps(report, indent=2))
    im = Image.new("RGB", (1120, 320), "white")
    draw = ImageDraw.Draw(im)
    for s in strokes:
        p = np.asarray(s["points"])
        uv = np.c_[20 + (-0.020 - p[:, 1]) * 3800, 20 + (0.415 - p[:, 0]) * 3800]
        draw.line([tuple(v) for v in uv], fill="black", width=2)
    im.save(out / "layout_preview.png")
    return report
