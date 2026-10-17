"""Download the calibrated contact-writing station with its asset licenses."""

import argparse
import hashlib
import tempfile
import urllib.request
import zipfile
from pathlib import Path

URL = "https://github.com/MrAmazing-h/dexmate-writing/releases/download/v0.2.0/vega_1p_contact_station.zip"
SHA256 = "71c51752e81259c2e329656f7371696e1d09f663839f34f763b9073dae3b7a9f"
DIRECTORY = "vega_1p_contact_station"


def extract(archive, output):
    with zipfile.ZipFile(archive) as z:
        for info in z.infolist():
            name = Path(info.filename)
            if (
                name.is_absolute()
                or ".." in name.parts
                or not name.parts
                or name.parts[0] != DIRECTORY
            ):
                raise ValueError("Unexpected contact-station archive member")
        z.extractall(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("external"))
    args = parser.parse_args()
    if (args.output / DIRECTORY).exists():
        parser.error("Station already exists; use a fresh output directory")
    with tempfile.TemporaryDirectory() as tmp:
        archive = Path(tmp) / "station.zip"
        urllib.request.urlretrieve(URL, archive)
        if hashlib.sha256(archive.read_bytes()).hexdigest() != SHA256:
            raise RuntimeError("Contact-station archive checksum mismatch")
        extract(archive, args.output)
    print((args.output / DIRECTORY).resolve())


if __name__ == "__main__":
    main()
