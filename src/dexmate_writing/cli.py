"""Command line interface for the fixed writing demonstration."""

import argparse
from pathlib import Path

from . import handwriting as hw
from .model_contract import validate_model
from .validation import validate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["build", "solve", "validate", "preview", "render", "all"])
    parser.add_argument("--model-dir", type=Path, default=Path("external/vega_1p_f5d6"))
    parser.add_argument("--output", type=Path, default=Path("outputs/writing"))
    args = parser.parse_args()
    hw.ROOT = args.output.resolve()
    hw.SOURCE = args.model_dir.resolve()
    if args.mode in ("build", "all"):
        if hw.ROOT.exists() and any(hw.ROOT.iterdir()):
            parser.error(
                "Use a new output directory for build/all; existing results are preserved."
            )
        (hw.ROOT / "model").mkdir(parents=True, exist_ok=True)
        (hw.ROOT / "videos").mkdir(exist_ok=True)
        hw.build()
    validate_model(hw.model()[0])
    if args.mode in ("solve", "all"):
        hw.trajectory()
    if args.mode in ("validate", "all"):
        validate(hw.ROOT)
    if args.mode == "preview":
        hw.render(True)
    if args.mode in ("render", "all"):
        hw.render(False)


if __name__ == "__main__":
    main()
