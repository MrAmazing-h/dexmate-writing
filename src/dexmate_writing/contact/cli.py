"""Printable ASCII -> layout -> reference -> contact rollout -> evaluation."""

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

from .text_layout import layout


def acceptance(output):
    """Geometric simulation gates, not a hardware or grip-force certificate."""
    m = json.loads((output / "metrics.json").read_text())
    c = json.loads((output / "substep_collision_audit.json").read_text())
    with np.load(output / "rollout.npz", allow_pickle=False) as z:
        finite = all(np.isfinite(z[key]).all() for key in ("qpos", "stats", "target"))
    checks = {
        "finite_recorded_states": bool(finite),
        "ink_coverage_at_least_99pct": m["ink_path_within_1mm_fraction"] >= 0.99,
        "mean_xy_error_below_1mm": m["pen_down_xy_tracking_error_mm"]["mean"] < 1,
        "contact_coverage_at_least_98pct": m["pen_down_contact_coverage"] >= 0.98,
        "no_enabled_collision_deeper_than_1mm": not c["pairs_exceeding_1mm"],
        "reference_unintended_collision_below_1mm": max(
            m["reference_unintended_penetration_by_pair_mm"].values(), default=0
        )
        <= 1,
    }
    result = {
        "passed": all(checks.values()),
        "checks": checks,
        "scope": "Simulation geometry only; fixed pen, fixed base, inherited collision exclusions. "
        "No contact-force safety or friction-only grasp acceptance criterion.",
    }
    (output / "acceptance.json").write_text(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--text")
    group.add_argument("--text-file", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--station-dir", type=Path, default=Path("external/vega_1p_contact_station")
    )
    parser.add_argument("--max-cap-mm", type=float, default=26)
    parser.add_argument("--layout-only", action="store_true")
    parser.add_argument("--no-render", action="store_true")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        parser.error("Output must be a new or empty directory; existing results are preserved.")
    station = args.station_dir.resolve()
    required = [
        "model/contact_scene.xml",
        "calibrated_qpos.npy",
        "controller.json",
        "reversed_pen.txt",
    ]
    if not args.layout_only and any(not (station / name).is_file() for name in required):
        parser.error(
            "Missing contact station. Run dexmate-fetch-contact-station first, "
            "or provide --station-dir with the calibrated release asset."
        )
    text = args.text if args.text is not None else args.text_file.read_text(encoding="utf-8")
    try:
        result = layout(text, output, args.max_cap_mm)
    except ValueError as error:
        parser.error(str(error))
    print("LAYOUT", result["lines"], result["capital_height_mm"], flush=True)
    if args.layout_only:
        return
    # Keep each run light; the separately downloaded asset directory must stay in place.
    (output / "model").symlink_to(
        os.path.relpath(station / "model", output), target_is_directory=True
    )
    for name in required[1:]:
        shutil.copy2(station / name, output / name)
    (output / "videos").mkdir()
    (output / "tmp").mkdir()
    env = os.environ.copy()
    env.update(TMPDIR=str(output / "tmp"), PYTHONDONTWRITEBYTECODE="1")
    env.setdefault("MUJOCO_GL", "egl")
    commands = [
        ("contact_writing", "plan"),
        ("smooth_reference",),
        ("contact_writing", "simulate"),
        ("evaluate_contact",),
    ]
    if not args.no_render:
        commands.append(("render_contact",))
    for module, *arguments in commands:
        log_path = output / ("_".join([module, *arguments]) + ".log")
        print("RUN", module, *arguments, "log:", log_path, flush=True)
        with log_path.open("w") as log:
            subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "dexmate_writing.contact." + module,
                    *arguments,
                    "--output",
                    str(output),
                ],
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                check=True,
            )
    result = acceptance(output)
    print("ACCEPTANCE", result, flush=True)
    if not result["passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
