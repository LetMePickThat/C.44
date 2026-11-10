#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GEN_APP_ICON -- the icons of the two tools, same frame and colours, drawn by Pillow, 4x supersampled:
  - Scenario Editor (user, 30 October 2026), in the colours of its three 3D views: a planet (Global, blue), its orbit
    (Orbital, orange) and on it a manoeuvre node (Ship, green), from which the edited trajectory leaves, dashed;
  - Radar Mission Preparation (user, 10 November 2026): an ultra-simplified ship seen from its side, with watch
    volumes leaving its radar panels (see draw_prep).
The small sizes drop the details and thicken the lines, to stay readable.
Writes assets/icons/<name>.ico (16 to 256 px) and .png (256 px).
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


# Radar Mission Preparation: the ship seen from its side (+Y, the radiator wings in the X-Z plane seen flat), nose to
# the right, from the 3D model's proportions (149.5 m long, hull radius about 8.6 m aft ending in a truncated aerospike, 6 m at the radar station, the
# station 15.4 m behind the nose tip), the lengths drawn at SHIP_PX per metre and the hull radii exaggerated
# (RADIUS_GAIN) to stay readable; the radiator wings in their steps (RADIATORS, the model's outline, |x| from 9.7 m);
# the radar panels at the station, tilted 25 deg toward the nose: F1 (top, +X) and F3 (bottom, -X) in profile, each
# with its watch volume; F2 (+Y) seen from the front on the hull. Colours of the faces: core/geometry.FACE_COLORS.
HULL = "#e2e0da"
F1, F2, F3 = "#2f7fd0", "#e08a1e", "#2fa35a"
RADIATOR = "#9aa3ad"
RADIATORS = ((-116.0, -106.0, 21.2), (-106.0, -98.0, 29.3), (-98.0, -76.0, 37.3), (-76.0, -66.0, 30.4),
             (-66.0, -60.0, 16.0))         # (z from, z to, |x| max), m
RADIATOR_X0 = 9.7
AXIS_Y = 128.0                          # the ship's axis on the canvas
NOSE_X = 170.0                          # the nose tip
SHIP_PX = 0.98                          # px per metre along the ship
RADIUS_GAIN = 1.7
PROFILE = ((-128.0, 8.5), (-22.0, 8.6), (0.0, 6.0), (8.0, 4.4), (14.0, 2.8), (15.4, 0.0))   # hull (z m, r m)
SPIKE = ((-128.0, 8.5), (-129.5, 6.1), (-130.5, 5.0), (-131.5, 4.0), (-132.5, 3.5), (-134.0, 2.6))  # aerospike, truncated
SPIKE_COLOR = "#a9acb2"
SPIKE_GAIN = 2.5                        # the spike's length exaggerated, like the radii (6 m of 149.5)
TILT_DEG = 25.0
VOLUME_LEN, VOLUME_HALF_DEG = 96.0, 18.0
ROLL_DEG = 45.0                         # the whole drawing inclined (user, 10 November 2026)


def _ship_pt(z_m: float, r_m: float) -> Tuple[float, float]:
    return NOSE_X + (z_m - 15.4) * SHIP_PX, AXIS_Y - r_m * SHIP_PX * RADIUS_GAIN


def draw_prep(size: int) -> Image.Image:
    px = size * SS
    k = px / 256.0
    small = size < 48
    base = Image.new("RGBA", (px, px), (0, 0, 0, 0))
    frame = (4 * k, 4 * k, 252 * k, 252 * k)
    ImageDraw.Draw(base).rounded_rectangle(frame, radius=46 * k, fill=BG, outline=EDGE, width=round(5 * k))
    clip = Image.new("L", (px, px), 0)                                            # the volumes stay in the frame
    ImageDraw.Draw(clip).rounded_rectangle(frame, radius=46 * k, fill=255)
    im = Image.new("RGBA", (px, px), (0, 0, 0, 0))                                # the ship and its volumes, level
    gain = 1.6 if small else 1.0                                                  # thicker hull at small sizes
    ax, ay = _ship_pt(0.0, 0.0)
    r_station = 6.0 * SHIP_PX * RADIUS_GAIN * gain
    for side, colour in ((1.0, F1), (-1.0, F3)):                                  # the two volumes in profile
        a = math.radians(-90.0 * side + TILT_DEG * side)                          # boresight, toward the nose
        ox, oy = ax, ay - side * r_station
        arc = [a + math.radians(VOLUME_HALF_DEG) * (2 * i / 24.0 - 1) for i in range(25)]
        wedge = [(ox, oy)] + [(ox + VOLUME_LEN * math.cos(t), oy + VOLUME_LEN * math.sin(t)) for t in arc]
        layer = Image.new("RGBA", (px, px), (0, 0, 0, 0))
        rgb = tuple(int(colour[i:i + 2], 16) for i in (1, 3, 5))
        ld = ImageDraw.Draw(layer)
        ld.polygon([(x * k, y * k) for x, y in wedge], fill=rgb + (150 if small else 105,))
        _line(ld, wedge + [(ox, oy)], k, colour, 12.0 if small else 6.0)
        if not small:                                                             # a range ring in the volume
            _line(ld, [(ox + 0.62 * VOLUME_LEN * math.cos(t), oy + 0.62 * VOLUME_LEN * math.sin(t)) for t in arc],
                  k, colour, 4.0)
        im.alpha_composite(layer)
    d = ImageDraw.Draw(im)
    r_hull = 8.6 * SHIP_PX * RADIUS_GAIN * gain                                   # the radiator wings, both sides
    for z0, z1, x1 in RADIATORS:
        xa, xb = _ship_pt(z0, 0.0)[0], _ship_pt(z1, 0.0)[0]
        out = r_hull + (x1 - RADIATOR_X0) * SHIP_PX
        for sgn in (1.0, -1.0):
            ya, yb = AXIS_Y - sgn * 0.5 * r_hull, AXIS_Y - sgn * out
            d.rectangle((xa * k, min(ya, yb) * k, xb * k, max(ya, yb) * k), fill=RADIATOR,
                        outline=None if small else BG, width=round(2 * k))
    top = [(_ship_pt(z, 0)[0], AXIS_Y - (AXIS_Y - _ship_pt(z, r)[1]) * gain) for z, r in PROFILE]
    hull = top + [(x, 2 * AXIS_Y - y) for x, y in reversed(top)]
    d.polygon([(x * k, y * k) for x, y in hull], fill=HULL)
    spike = [(_ship_pt(-128.0 + (z + 128.0) * SPIKE_GAIN, 0)[0], AXIS_Y - (AXIS_Y - _ship_pt(z, r)[1]) * gain)
             for z, r in SPIKE]   # the engine:
    d.polygon([(x * k, y * k) for x, y in spike + [(x, 2 * AXIS_Y - y) for x, y in reversed(spike)]],  # an aerospike
              fill=SPIKE_COLOR)                                                   # (truncated), not a bell
    half = 2.3 * SHIP_PX * RADIUS_GAIN * (1.8 if small else 1.0)                  # the panels (4.6 m), tilted
    for side, colour in ((1.0, F1), (-1.0, F3)):
        cx, cy = ax, ay - side * r_station
        t = math.radians(TILT_DEG)
        dx, dy = half * math.cos(t), half * math.sin(t) * side
        _line(d, [(cx - dx, cy - dy), (cx + dx, cy + dy)], k, colour, 13.0 if small else 7.0)
    if not small:                                                                 # F2, seen from the front
        r = 6.0
        d.polygon([((ax + r * math.cos(math.radians(22.5 + 45 * i))) * k,
                    (ay + r * math.sin(math.radians(22.5 + 45 * i))) * k) for i in range(8)], fill=F2)
    im = im.rotate(ROLL_DEG, resample=Image.BICUBIC)                              # inclined, nose up to the right
    x0, y0, x1, y1 = im.getchannel("A").getbbox()                                 # then centred in the frame
    art = Image.new("RGBA", (px, px), (0, 0, 0, 0))
    art.alpha_composite(im, (round((px - (x1 - x0)) / 2 - x0), round((px - (y1 - y0)) / 2 - y0)))
    base.paste(art, (0, 0), Image.composite(art.getchannel("A"), Image.new("L", (px, px), 0), clip))
    return base.resize((size, size), Image.LANCZOS)


ICONS = (("scenario_editor", draw), ("radar_mission_preparation", draw_prep))


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    for name, fn in ICONS:
        frames = {s: fn(s) for s in SIZES}
        frames[256].save(os.path.join(OUT, f"{name}.png"))
        frames[256].save(os.path.join(OUT, f"{name}.ico"), sizes=[(s, s) for s in SIZES],
                         append_images=[frames[s] for s in SIZES[:-1]])


if __name__ == "__main__":
    main()
