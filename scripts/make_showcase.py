"""Compose independent original-speed DexMate/G1 recordings; label end holds."""

import argparse
import json
import math
from pathlib import Path

import imageio.v2 as iio
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps


def font(size):
    return ImageFont.truetype("DejaVuSans.ttf", size)


p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--dexmate", type=Path, required=True)
p.add_argument("--g1", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=False)
readers = [iio.get_reader(a.dexmate), iio.get_reader(a.g1)]
counts = [r.count_frames() for r in readers]
rates = [float(r.get_meta_data()["fps"]) for r in readers]
durations = [n / fps for n, fps in zip(counts, rates)]
nframes = math.ceil((max(durations) + 2) * 25)
writer = iio.get_writer(
    a.output / "writeurrobo_showcase.mp4",
    fps=25,
    codec="libx264",
    quality=8,
    macro_block_size=1,
    ffmpeg_params=["-movflags", "+faststart"],
)
gif = []
last_indices = [-1, -1]
images = [None, None]
for index in range(nframes):
    t = index / 25
    im = Image.new("RGB", (1920, 928), "#0c1525")
    draw = ImageDraw.Draw(im)
    draw.text((30, 18), "WriteUrRobo", font=font(34), fill="white")
    draw.text(
        (330, 28),
        "FROM TEXT TO TOUCH / TWO INDEPENDENT PHYSICS DEMOS",
        font=font(21),
        fill="#6bdcc8",
    )
    for k, (r, fps, count, duration) in enumerate(zip(readers, rates, counts, durations)):
        frame = min(int(t * fps), count - 1)
        if frame != last_indices[k]:
            images[k] = Image.fromarray(r.get_data(frame))
            last_indices[k] = frame
        picture = ImageOps.contain(images[k], (960, 720))
        im.paste(picture, (k * 960 + (960 - picture.width) // 2, 128 + (720 - picture.height) // 2))
        title = "DEXMATE / Vega-1P" if k == 0 else "G1 / Inspire DFQ"
        clock = f"{min(t, duration):.1f} / {duration:.1f} s"
        hold = "  [COMPLETE — FINAL FRAME HELD]" if t >= duration else "  [1x]"
        draw.text((k * 960 + 30, 88), title + "   " + clock + hold, font=font(23), fill="white")
    draw.text(
        (30, 872),
        "Fixed bases + gravity compensation + attached pens. Actual contact ink; not a matched benchmark or hardware validation.",
        font=font(22),
        fill="#b9ccdd",
    )
    writer.append_data(np.asarray(im))
    if index % 10 == 0:
        small = im.resize((960, 464), Image.Resampling.LANCZOS)
        sd = ImageDraw.Draw(small)
        sd.rectangle((770, 0, 960, 25), fill="#193a47")
        sd.text((785, 4), "4x showcase preview", font=font(14), fill="white")
        gif.append(small)
    if index % 500 == 0:
        print("COMPOSITE", index, nframes, flush=True)
writer.close()
im.save(a.output / "showcase_final.jpg", quality=93)
gif[0].save(
    a.output / "showcase.gif",
    save_all=True,
    append_images=gif[1:],
    duration=100,
    loop=0,
    optimize=False,
)
for r in readers:
    r.close()
with iio.get_reader(a.output / "writeurrobo_showcase.mp4") as r:
    decoded = sum(1 for _ in r)
assert decoded == nframes
(a.output / "validation.json").write_text(
    json.dumps(
        {
            "input_frames": counts,
            "input_fps": rates,
            "input_durations_s": durations,
            "output_fps": 25,
            "output_frames_decoded": decoded,
            "gif_speed": "4x",
            "gif_frames": len(gif),
            "timing": "Original-speed independent demonstrations; finished panels explicitly hold their last frame. Not synchronized glyphs or a matched benchmark.",
        },
        indent=2,
    )
)
