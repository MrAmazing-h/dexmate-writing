# Third-party notices

## Robot assets

Dexmate robot descriptions and meshes: Copyright 2025 Dexmate Inc., [official dexmate-urdf](https://github.com/dexmate-ai/dexmate-urdf), Apache License 2.0. The separately downloadable research conversion retains the full upstream LICENSE and NOTICE, lists modifications, and includes mesh/conversion hashes. Changes include MuJoCo conversion, local mesh paths, a floating root, grounded placement, collision proxies/exclusions and mimic handling. This is not an official Dexmate release or endorsement. The repository's MIT license does not replace the robot asset license.

## Numerical and rendering dependencies

MuJoCo (Apache 2.0), NumPy/SciPy (BSD), OSQP where used (Apache 2.0), Pillow (HPND), ImageIO (BSD) and imageio-ffmpeg retain their respective upstream licenses. They are installed as dependencies rather than vendored. FFmpeg binary terms depend on the distributed build.

## Stroke data

The eight unmodified JSON files in `src/dexmate_writing/data/strokes/` are a subset of [Hanzi Writer Data 2.0.1](https://github.com/chanind/hanzi-writer-data), maintained by David Chanin. The data derives from Make Me a Hanzi and fonts by Arphic Technology Co., Ltd. Copyright (C) 1999 Arphic Technology Co., Ltd. The full Arphic Public License is distributed beside those files as `ARPHICPL.TXT`. These data are not relicensed under MIT. The repeated character in the demonstration reuses the same unmodified file.

## Demonstration media

The v0.1.0 media depict kinematic output. The v0.2.0 media depict simulated contact-writing rollouts with a fixed pen attachment. Neither validates hardware writing or friction-only grasping. Upstream data and asset rights remain applicable.

## ASCII stroke data

ASCII layout uses [Hershey-Fonts 2.1.0](https://github.com/apshu/HersheyFonts) by apshu, installed as a dependency under its MIT license. Glyphs derive from A. V. Hershey's vector fonts; upstream data notices remain applicable. No robot or Chinese font asset is relicensed by this dependency.

## Calibrated contact station

The v0.2.0 station is a separate research conversion of the same Vega-1P/F5D6 assets. Its archive retains the upstream LICENSE and NOTICE and records station changes in MODIFICATIONS.txt: fixed base, gravity compensation, actuators, inherited collision exclusions/proxies, table/paper and attached compliant-nib pen. It is not an official Dexmate controller.
