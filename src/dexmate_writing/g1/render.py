"""Replay recorded G1 physics; draw only measured nib-paper contact positions."""

import argparse
import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path

import imageio.v2 as iio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def font(size):
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size)
    except OSError:
        return ImageFont.load_default(size=size)


def ink_segments(ink):
    """Do not connect lifts, missing-contact gaps, or different strokes."""
    segments = []
    for p, q in zip(ink, ink[1:]):
        if q[0] - p[0] <= 2 and p[1] == q[1] and np.linalg.norm(q[2:4] - p[2:4]) < 0.004:
            a, b = p[2:5].copy(), q[2:5].copy()
            a[2] = b[2] = 0.7503
            segments.append((int(q[0]), a, b))
    return segments


def render(out, preview=False, model_dir=None):
    scene = out / "scene.xml"
    z = np.load(out / "rollout.npz", allow_pickle=False)
    if hashlib.sha256(scene.read_bytes()).hexdigest() != str(z["scene_sha256"]):
        raise ValueError("Recorded scene hash differs; refusing mismatched model replay")
    if model_dir is None:
        m = mujoco.MjModel.from_xml_path(str(scene))
    else:
        root = ET.fromstring(scene.read_text())
        root.find("compiler").set("meshdir", str((model_dir / "meshes").resolve()))
        m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    d = mujoco.MjData(m)
    q = z["qpos"]
    segments = ink_segments(z["ink"])
    dest = out / "videos"
    dest.mkdir(exist_ok=True)
    r = mujoco.Renderer(m, height=640, width=640, max_geom=len(segments) + 3000)
    opt = mujoco.MjvOption()
    # This model uses group 1 for visual meshes; group 3 is collision geometry.
    opt.geomgroup[3] = 0
    # Hide collision proxies only in the renderer, never in the physics model.
    for g in range(m.ngeom):
        if m.geom_bodyid[g] > 0 and (m.geom_contype[g] or m.geom_conaffinity[g]):
            if m.geom(g).name not in ("nib", "pen_barrel"):
                m.geom_group[g] = 3
    cameras = []
    for lookat, distance, azimuth, elevation in (
        ([0.05, -0.08, 0.8], 2.2, 140, -18),
        ([0.35, -0.20, 0.80], 0.48, 135, -30),
    ):
        cam = mujoco.MjvCamera()
        cam.lookat[:] = lookat
        cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
        cameras.append(cam)
    frames = [0, len(q) // 2, len(q) - 1] if preview else range(0, len(q), 4)
    writer = (
        None
        if preview
        else iio.get_writer(
            dest / "g1_contact_writing.mp4",
            fps=float(z["fps"]) / 4,
            codec="libx264",
            quality=8,
            macro_block_size=1,
            ffmpeg_params=["-movflags", "+faststart"],
        )
    )
    for frame in frames:
        d.qpos[:] = q[frame]
        mujoco.mj_forward(m, d)
        panels = []
        for cam in cameras:
            r.update_scene(d, camera=cam, scene_option=opt)
            r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            for stop, a, b in segments:
                if stop > frame:
                    break
                g = r.scene.geoms[r.scene.ngeom]
                mujoco.mjv_initGeom(
                    g,
                    mujoco.mjtGeom.mjGEOM_CAPSULE,
                    np.zeros(3),
                    np.zeros(3),
                    np.eye(3).ravel(),
                    np.array([0.03, 0.08, 0.15, 1]),
                )
                mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.00035, a, b)
                g.category = mujoco.mjtCatBit.mjCAT_DECOR
                r.scene.ngeom += 1
            panels.append(Image.fromarray(r.render()))
        im = Image.new("RGB", (1280, 720), "#0c1525")
        im.paste(panels[0], (0, 80))
        im.paste(panels[1].resize((320, 320)), (800, 80))
        draw = ImageDraw.Draw(im)
        draw.rounded_rectangle((660, 420, 1260, 700), radius=16, fill="#f1f3ed")
        draw.text((680, 432), "ACTUAL CONTACT TRACE / XY projection", font=font(15), fill="#415970")

        def project(p):
            return (960 - (p[1] + 0.20) * 2200, 560 - (p[0] - 0.35) * 2200)

        for stop, a, b in segments:
            if stop > frame:
                break
            draw.line((project(a), project(b)), fill="#183d55", width=2)
        for row in z["ink"]:
            if row[0] > frame:
                break
            x, y = project(row[2:5])
            draw.ellipse((x - 1, y - 1, x + 1, y + 1), fill="#183d55")
        draw.text((24, 12), "WriteUrRobo / G1 + Inspire", font=font(25), fill="white")
        draw.text(
            (24, 48),
            "Fixed base | Gravity compensation | Attached pen fixture",
            font=font(17),
            fill="#b2c6dc",
        )
        draw.text((680, 12), "Measured-contact ink", font=font(25), fill="#61daca")
        draw.text(
            (680, 48),
            f"Physics replay / 1x / t = {z['time_s'][frame]:.2f} s",
            font=font(17),
            fill="#b2c6dc",
        )
        if preview:
            im.save(dest / f"preview_{frame:05d}.jpg")
        else:
            writer.append_data(np.asarray(im))
        if frame >= len(q) - 4:
            im.save(dest / "final.jpg")
        if frame % 1000 == 0:
            print("RENDER", frame, len(q), flush=True)
    if writer:
        writer.close()
    r.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--preview", action="store_true")
    p.add_argument("--model-dir", type=Path, help="Relocate verified meshes for downloaded records")
    a = p.parse_args()
    render(a.output, a.preview, a.model_dir)


if __name__ == "__main__":
    main()
