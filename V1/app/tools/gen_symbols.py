#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GEN_SYMBOLS -- the symbols of the simulator's views (proposal of 29 October 2026, inspired by MIL-STD-2525 / APP-6,
adapted to 3D on a dark background): the FRAME gives the identity (shape and colour), the GLYPH inside the class.

  identities  friendly  circle, cyan          hostile  diamond, red        suspect  diamond dashed, orange
              neutral   square, green         unknown  quatrefoil, yellow  pending  quatrefoil dashed, yellow
              OWNSHIP   double circle, friendly blue (pending: the default identity of every class, user 29 October 2026)
  classes     ST satellite (body + two panels)   GENERIC dot   KKV filled arrowhead   DECOY hollow arrowhead
              SHIP pointed hull                  DEBRIS cross

  multitrack  several objects of one identity and class close together on screen (user, 29 October 2026): the
              frame's outline doubled (a second one inside, MULTI_INNER of it), the glyph shrunk as much
Writes into assets/symbols/: <identity>_<CLASS>.svg and .png (64 x 64, transparent), multi_<identity>_<CLASS>.svg /
.png, frame_<identity>.svg / .png, ownship.svg / .png and legend.png (every symbol on one sheet). The SVG files are the source; the PNG are drawn with
the same geometry by Pillow (4x supersampled). Run again whenever the symbology changes:
    python tools/gen_symbols.py
"""
from __future__ import annotations

import math
import os
from typing import List, Tuple

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "assets", "symbols")
SIZE = 64                      # px of the PNG (the SVG viewBox is the same)
SS = 4                         # supersampling of the PNG
STROKE = 3.0                   # frame outline width (px at SIZE)
GLYPH_COLOR = "#F2F2F2"
FILL_ALPHA = 0.28              # frame fill: the identity colour at this opacity
MULTI_INNER = 0.80             # multitrack: the inner outline and the glyph at this x the frame, about the centre

IDENTITIES = {                 # name: (shape, colour, dashed)
    "friendly": ("circle", "#4FC3F7", False),
    "hostile": ("diamond", "#FF4D4D", False),
    "suspect": ("diamond", "#FF9A3C", True),
    "neutral": ("square", "#5CD65C", False),
    "unknown": ("quatrefoil", "#FFE24D", False),
    "pending": ("quatrefoil", "#FFE24D", True),
}
OWNSHIP_COLOR = IDENTITIES["friendly"][1]         # OWNSHIP is friendly (user, 29 October 2026): its blue
CLASSES = ("ST", "GENERIC", "KKV", "DECOY", "SHIP", "DEBRIS")
Pt = Tuple[float, float]


# ----------------------------------------------------------------------------- geometry (64 x 64, y down)
def _circle(cx: float, cy: float, r: float, n: int = 96) -> List[Pt]:
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def _quatrefoil(n: int = 240) -> List[Pt]:
    """Outline of the union of four circles (lobes) around the centre: along each ray, the farthest crossing."""
    c, r = 12.5, 14.5
    centres = [(c, 0.0), (-c, 0.0), (0.0, c), (0.0, -c)]
    pts = []
    for k in range(n):
        a = 2 * math.pi * k / n
        ux, uy = math.cos(a), math.sin(a)
        far = 0.0
        for cx, cy in centres:
            b = ux * cx + uy * cy
            disc = b * b - (cx * cx + cy * cy - r * r)
            if disc >= 0.0:
                far = max(far, b + math.sqrt(disc))
        pts.append((32 + far * ux, 32 + far * uy))
    return pts


def frame_outline(shape: str) -> List[Pt]:
    if shape == "circle":
        return _circle(32, 32, 26)
    if shape == "diamond":
        return [(32, 3), (61, 32), (32, 61), (3, 32)]
    if shape == "square":
        return [(8, 8), (56, 8), (56, 56), (8, 56)]
    return _quatrefoil()


def _shrink(pts: List[Pt], k: float) -> List[Pt]:
    return [(32 + k * (x - 32), 32 + k * (y - 32)) for x, y in pts]


def glyph(cls: str):
    """[(kind, points, filled)] of a class: kind 'poly' (closed) or 'line' (open)."""
    if cls == "ST":
        return [("poly", [(28, 24), (36, 24), (36, 40), (28, 40)], True),
                ("poly", [(13, 28), (25, 28), (25, 36), (13, 36)], False),
                ("poly", [(39, 28), (51, 28), (51, 36), (39, 36)], False),
                ("line", [(25, 32), (28, 32)], False), ("line", [(36, 32), (39, 32)], False)]
    if cls == "GENERIC":
        return [("poly", _circle(32, 32, 6, 40), True)]
    if cls == "KKV":
        return [("poly", [(32, 13), (43, 45), (32, 38), (21, 45)], True)]
    if cls == "DECOY":
        return [("poly", [(32, 13), (43, 45), (32, 38), (21, 45)], False)]
    if cls == "SHIP":
        return [("poly", [(32, 12), (39, 24), (39, 46), (32, 51), (25, 46), (25, 24)], True)]
    return [("line", [(22, 22), (42, 42)], False), ("line", [(42, 22), (22, 42)], False)]


def _dashes(pts: List[Pt], on: float = 6.0, off: float = 4.0) -> List[List[Pt]]:
    """The dashes along a closed outline."""
    ring = pts + [pts[0]]
    out, cur, left, drawing = [], [ring[0]], on, True
    for (x0, y0), (x1, y1) in zip(ring[:-1], ring[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        t = 0.0
        while seg - t > 1e-9:
            step = min(left, seg - t)
            t += step
            p = (x0 + (x1 - x0) * t / seg, y0 + (y1 - y0) * t / seg)
            if drawing:
                cur.append(p)
            left -= step
            if left <= 1e-9:
                if drawing:
                    out.append(cur)
                drawing = not drawing
                left = on if drawing else off
                cur = [p]
    if drawing and len(cur) > 1:
        out.append(cur)
    return out


# ----------------------------------------------------------------------------- SVG
def _path(pts: List[Pt], close: bool = True) -> str:
    d = "M " + " L ".join(f"{x:.2f} {y:.2f}" for x, y in pts)
    return d + (" Z" if close else "")


def svg(identity: str, cls: str, multi: bool = False) -> str:
    parts = []
    gk = MULTI_INNER if multi else 1.0
    if identity == "ownship":
        for r in (27, 21):
            parts.append(f'<circle cx="32" cy="32" r="{r}" fill="none" stroke="{OWNSHIP_COLOR}" stroke-width="{STROKE}"/>')
    else:
        shape, col, dashed = IDENTITIES[identity]
        dash = ' stroke-dasharray="6 4"' if dashed else ""
        parts.append(f'<path d="{_path(frame_outline(shape))}" fill="{col}" fill-opacity="{FILL_ALPHA}" '
                     f'stroke="{col}" stroke-width="{STROKE}" stroke-linejoin="round"{dash}/>')
        if multi:                                      # the doubled outline of a multitrack
            parts.append(f'<path d="{_path(_shrink(frame_outline(shape), MULTI_INNER))}" fill="none" '
                         f'stroke="{col}" stroke-width="{STROKE}" stroke-linejoin="round"{dash}/>')
    if cls:
        for kind, pts, filled in glyph(cls):
            pts = _shrink(pts, gk)
            if kind == "line":
                parts.append(f'<path d="{_path(pts, False)}" fill="none" stroke="{GLYPH_COLOR}" stroke-width="3.5" '
                             f'stroke-linecap="round"/>')
            else:
                parts.append(f'<path d="{_path(pts)}" fill="{GLYPH_COLOR if filled else "none"}" '
                             f'stroke="{GLYPH_COLOR}" stroke-width="2.5" stroke-linejoin="round"/>')
    title = f"{identity} {cls}{' multitrack' if multi else ''}".strip()
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{SIZE}" height="{SIZE}" viewBox="0 0 64 64">'
            f"<title>{title}</title>" + "".join(parts) + "</svg>\n")


# ----------------------------------------------------------------------------- PNG
def _rgba(hex_col: str, a: float = 1.0) -> Tuple[int, int, int, int]:
    h = hex_col.lstrip("#")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16), int(round(255 * a))


def png(identity: str, cls: str, multi: bool = False) -> Image.Image:
    k = SIZE * SS / 64.0
    img = Image.new("RGBA", (SIZE * SS, SIZE * SS), (0, 0, 0, 0))
    dr = ImageDraw.Draw(img)
    sc = lambda pts: [(x * k, y * k) for x, y in pts]        # noqa: E731
    w = int(round(STROKE * k))
    if identity == "ownship":
        for r in (27, 21):
            dr.line(sc(_circle(32, 32, r) + [_circle(32, 32, r)[0]]), fill=_rgba(OWNSHIP_COLOR), width=w, joint="curve")
    else:
        shape, col, dashed = IDENTITIES[identity]
        out = frame_outline(shape)
        layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
        ImageDraw.Draw(layer).polygon(sc(out), fill=_rgba(col, FILL_ALPHA))
        img = Image.alpha_composite(img, layer)
        dr = ImageDraw.Draw(img)
        for o in [out] + ([_shrink(out, MULTI_INNER)] if multi else []):     # multitrack: doubled outline
            for piece in (_dashes(o) if dashed else [o + [o[0]]]):
                dr.line(sc(piece), fill=_rgba(col), width=w, joint="curve")
    if cls:
        g = _rgba(GLYPH_COLOR)
        for kind, pts, filled in glyph(cls):
            pts = _shrink(pts, MULTI_INNER if multi else 1.0)
            if kind == "line":
                dr.line(sc(pts), fill=g, width=int(round(3.5 * k)))
            elif filled:
                dr.polygon(sc(pts), fill=g, outline=g)
            else:
                dr.line(sc(pts + [pts[0]]), fill=g, width=int(round(2.5 * k)), joint="curve")
    return img.resize((SIZE, SIZE), Image.LANCZOS)


# ----------------------------------------------------------------------------- files
def _write(name: str, identity: str, cls: str, multi: bool = False) -> None:
    with open(os.path.join(OUT, name + ".svg"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write(svg(identity, cls, multi))
    png(identity, cls, multi).save(os.path.join(OUT, name + ".png"))


def legend() -> Image.Image:
    """Every symbol on one sheet, on the views' dark background: a row per identity, a column per class."""
    cell, left, top = 84, 110, 40
    ids = list(IDENTITIES) + ["ownship"]
    sheet = Image.new("RGBA", (left + cell * len(CLASSES) + cell, top + cell * len(ids)), _rgba("#0b1020"))
    dr = ImageDraw.Draw(sheet)
    font = ImageFont.load_default()
    for j, cls in enumerate(["(frame)"] + list(CLASSES)):
        dr.text((left + j * cell + 10, 12), cls, fill=_rgba("#E6E6E6"), font=font)
    for i, ident in enumerate(ids):
        y = top + i * cell
        dr.text((10, y + 28), ident, fill=_rgba("#E6E6E6"), font=font)
        for j, cls in enumerate([""] + list(CLASSES)):
            if ident == "ownship" and j > 0:
                continue
            sheet.alpha_composite(png(ident, cls), (left + j * cell + 10, y + 6))
    return sheet


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    n = 0
    for ident in IDENTITIES:
        _write(f"frame_{ident}", ident, "")
        n += 1
        for cls in CLASSES:
            _write(f"{ident}_{cls}", ident, cls)
            _write(f"multi_{ident}_{cls}", ident, cls, multi=True)
            n += 2
    _write("ownship", "ownship", "")
    legend().save(os.path.join(OUT, "legend.png"))
    print(f"{n + 1} symbols (SVG + PNG) and legend.png in {os.path.normpath(OUT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
