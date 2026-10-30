#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GEN_APP_ICON -- the icon of the Mission Scenario Editor (user, 30 October 2026), in the colours of its three 3D views:
a planet (Global, blue), its orbit (Orbital, orange) and on it a manoeuvre node (Ship, green), from which the edited
trajectory leaves, dashed. The small sizes drop the dashes and thicken the lines, to stay readable.
Writes assets/icons/mission_scenario_editor.ico (16 to 256 px) and .png (256 px); drawn by Pillow, 4x supersampled.
Run again whenever the icon changes:
    python tools/gen_app_icon.py
"""
from __future__ import annotations

import math
import os
from typing import List, Tuple

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "assets", "icons")
NAME = "mission_scenario_editor"
SIZES = (16, 24, 32, 48, 64, 128, 256)
SS = 4                                  # supersampling
BG, EDGE = "#0f1a2e", "#34466a"
BLUE, BLUE_DARK = "#3f90e6", "#1d5a9e"
ORANGE, GREEN, WHITE = "#f0932f", "#46d07f", "#f2f5fa"
PLANET = (114.0, 156.0, 54.0)           # centre x, y, radius, on a 256 px canvas
ORBIT = (96.0, 38.0, math.radians(-24.0))   # semi-axes, tilt
NODE_DEG = 345.0                        # the node's place on the orbit


def _orbit_pt(deg: float) -> Tuple[float, float]:
    a, b, tilt = ORBIT
    x, y = a * math.cos(math.radians(deg)), b * math.sin(math.radians(deg))
    return PLANET[0] + x * math.cos(tilt) - y * math.sin(tilt), PLANET[1] + x * math.sin(tilt) + y * math.cos(tilt)


def _line(d: ImageDraw.ImageDraw, pts: List[Tuple[float, float]], k: float, color: str, width: float) -> None:
    pts_px = [(x * k, y * k) for x, y in pts]
    d.line(pts_px, fill=color, width=round(width * k), joint="curve")
    r = 0.5 * width * k
    for x, y in (pts_px[0], pts_px[-1]):
        d.ellipse((x - r, y - r, x + r, y + r), fill=color)


def draw(size: int) -> Image.Image:
    px = size * SS
    k = px / 256.0
    small = size < 48
    im = Image.new("RGBA", (px, px), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    d.rounded_rectangle((4 * k, 4 * k, 252 * k, 252 * k), radius=46 * k, fill=BG, outline=EDGE, width=round(5 * k))
    width = 20.0 if small else 12.0
    _line(d, [_orbit_pt(a) for a in range(180, 361, 4)], k, ORANGE, width)        # behind the planet
    cx, cy, r = PLANET
    disc = ((cx - r) * k, (cy - r) * k, (cx + r) * k, (cy + r) * k)
    planet = Image.new("RGBA", (px, px), BLUE_DARK if not small else BLUE)
    if not small:                                                                  # lit from the upper right
        lx, ly, lr = cx + 0.22 * r, cy - 0.24 * r, 1.0 * r
        ImageDraw.Draw(planet).ellipse(((lx - lr) * k, (ly - lr) * k, (lx + lr) * k, (ly + lr) * k), fill=BLUE)
    mask = Image.new("L", (px, px), 0)
    ImageDraw.Draw(mask).ellipse(disc, fill=255)
    im.paste(planet, (0, 0), mask)
    _line(d, [_orbit_pt(a) for a in range(0, 181, 4)], k, ORANGE, width)          # in front of it
    nx, ny = _orbit_pt(NODE_DEG)
    # the edited trajectory: from the node toward the upper right corner
    p0, p1, p2 = (nx, ny), (nx + 2.0, ny - 42.0), (224.0, 34.0)
    path = [((1 - t) ** 2 * p0[0] + 2 * t * (1 - t) * p1[0] + t * t * p2[0],
             (1 - t) ** 2 * p0[1] + 2 * t * (1 - t) * p1[1] + t * t * p2[1]) for t in (i / 40.0 for i in range(41))]
    if small:
        _line(d, path[10:], k, WHITE, 14.0)
    else:
        for i in range(14, 40, 8):
            _line(d, path[i:i + 4], k, WHITE, 9.0)
    s = 34.0 if small else 27.0                                                   # the node: a diamond
    d.polygon([((nx + x) * k, (ny + y) * k) for x, y in ((0, -s), (s, 0), (0, s), (-s, 0))],
              fill=GREEN, outline=BG, width=round(5 * k))
    return im.resize((size, size), Image.LANCZOS)


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    frames = {s: draw(s) for s in SIZES}
    frames[256].save(os.path.join(OUT, f"{NAME}.png"))
    frames[256].save(os.path.join(OUT, f"{NAME}.ico"), sizes=[(s, s) for s in SIZES],
                     append_images=[frames[s] for s in SIZES[:-1]])


if __name__ == "__main__":
    main()
