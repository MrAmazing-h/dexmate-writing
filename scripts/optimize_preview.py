"""Make a smaller, globally paletted copy of the labelled 4x showcase GIF."""

import argparse
import json
import subprocess
from pathlib import Path

import imageio_ffmpeg
from PIL import Image

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("source", type=Path)
p.add_argument("destination", type=Path)
a = p.parse_args()
subprocess.run(
    [
        imageio_ffmpeg.get_ffmpeg_exe(),
        "-n",
        "-i",
        str(a.source),
        "-filter_complex",
        "fps=10,scale=800:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle",
        "-loop",
        "0",
        str(a.destination),
    ],
    check=True,
)
with Image.open(a.destination) as im:
    count, duration = im.n_frames, 0
    for i in range(count):
        im.seek(i)
        im.load()
        duration += im.info.get("duration", 0)
    report = {
        "frames_decoded": count,
        "duration_ms": duration,
        "size": list(im.size),
        "bytes": a.destination.stat().st_size,
        "speed": "4x",
    }
    print(json.dumps(report))
    a.destination.with_suffix(".validation.json").write_text(json.dumps(report, indent=2))
