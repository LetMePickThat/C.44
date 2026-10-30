"""Star background of the scenario tool's 3D views (user, 29 October 2026): converts the downloaded sources into the
files of assets/sky that gui/sky.py reads.

  stars                Yale Bright Star Catalogue, 5th revised edition (Hoffleit & Warren 1991, CDS V/50), 9 110
                       stars to about magnitude 6.5, J2000; the NASA HEASARC copy (heasarc_bsc5p.tdat.gz,
                       https://heasarc.gsfc.nasa.gov/FTP/heasarc/dbase/tdat_files/), public
  constellation lines  d3-celestial (Olaf Frohn, data/constellations.lines.json, BSD 3-clause licence, kept beside
                       the output), the 88 IAU constellations, RA (-180 .. 180 deg) and Dec, J2000

Outputs: assets/sky/stars.csv (hr, ra_deg, dec_deg, vmag, bv; bv empty when the catalogue has none), assets/sky/
constellation_lines.json ({id: [[[ra_deg, dec_deg], ...], ...]}, plus _source), assets/sky/LICENSE_d3-celestial.txt.

Run: python tools/gen_sky.py <heasarc_bsc5p.tdat[.gz]> <constellations.lines.json> <d3-celestial LICENSE>
"""
from __future__ import annotations

import csv
import gzip
import json
import os
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "assets", "sky")


def read_bsc5(path: str):
    """(hr, ra_deg, dec_deg, vmag, bv or None) of every star with a position and a magnitude."""
    opener = gzip.open if path.endswith(".gz") else open
    with opener(path, "rt", encoding="latin-1") as fh:
        text = fh.read().splitlines()
    cols = next(ln.split("=", 1)[1].split() for ln in text if ln.startswith("line[1]"))
    start = text.index("<DATA>") + 1
    rows = []
    for ln in text[start:]:
        if ln.startswith("<END>") or not ln.strip():
            break
        f = dict(zip(cols, ln.split("|")))
        try:
            ra, dec, vmag = float(f["ra"]), float(f["dec"]), float(f["vmag"])
        except (KeyError, ValueError):
            continue                                   # a few entries (novae, removed stars) have no position
        bv = f.get("bv_color", "").strip()
        rows.append((int(f["hr"]), ra, dec, vmag, float(bv) if bv else None))
    return sorted(rows)


def read_lines(path: str) -> dict:
    with open(path, encoding="utf-8") as fh:
        data = json.load(fh)
    return {ft["id"]: ft["geometry"]["coordinates"] for ft in data["features"]}


def main(bsc: str, lines: str, licence: str) -> None:
    os.makedirs(OUT, exist_ok=True)
    stars = read_bsc5(bsc)
    with open(os.path.join(OUT, "stars.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["hr", "ra_deg", "dec_deg", "vmag", "bv"])
        for hr, ra, dec, vmag, bv in stars:
            w.writerow([hr, f"{ra:.5f}", f"{dec:.5f}", f"{vmag:.2f}", "" if bv is None else f"{bv:.2f}"])
    con = read_lines(lines)
    con = {"_source": "d3-celestial, (c) 2015 Olaf Frohn, BSD 3-clause licence (LICENSE_d3-celestial.txt)", **con}
    with open(os.path.join(OUT, "constellation_lines.json"), "w", encoding="utf-8") as fh:
        json.dump(con, fh, separators=(",", ":"))
    shutil.copyfile(licence, os.path.join(OUT, "LICENSE_d3-celestial.txt"))
    print(f"{len(stars)} stars, {len(con) - 1} constellations -> {OUT}")


if __name__ == "__main__":
    main(*sys.argv[1:4])
