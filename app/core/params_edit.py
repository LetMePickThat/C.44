#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PARAMS_EDIT -- edits of radar_system.toml from the tool (step 5.5, user 13 November 2026), without a TOML writer:
only the value of an entry's own line is rewritten, so every comment, alignment and the order of the file are kept.

  find_lines(text)                 {key path: line index} of every entry a line holds (a section's direct keys)
  editable(text, key)              the entry can be rewritten (its line found; a sub-key of an inline table cannot)
  parse_value(text, old)           the value typed by the operator (TOML syntax), checked against the old value's type
  apply(text, edits, today)        the file text with the new values; a system entry (value / unit / status / source /
                                   date) also gets status "D" (derived / proposed, to confirm) and the date of the edit
  check(text, edits)               the edited text loaded by the parameter loader: every value edited as asked, every
                                   other value unchanged (ParamError otherwise)

No Qt dependency (gui/params_window.py shows it). The values are read once per process: an edit applies after the
tool is started again.
"""
from __future__ import annotations

import math
import os
import re
import tempfile
import tomllib
from typing import Any, Dict, Optional

from . import params as pm

_HEADER = re.compile(r"^\s*\[([^\[\]]+)\]\s*(#.*)?$")
_KEY = re.compile(r'^(\s*)([A-Za-z0-9_\-]+|"[^"]*")(\s*=\s*)(.*)$')
_SYSTEM = re.compile(r"^(\{\s*value\s*=\s*)(.+?)(\s*,\s*unit\s*=.*)$")


def _unquote(name: str) -> str:
    return name[1:-1] if name.startswith('"') else name


def _split_comment(rest: str):
    """(value text, the spaces and comment after it) of what follows "key = " on a line (a # inside a string kept)."""
    quote = None
    i = 0
    while i < len(rest):
        c = rest[i]
        if quote:
            if c == "\\" and quote == '"':
                i += 2
                continue
            if c == quote:
                quote = None
        elif c in ('"', "'"):
            quote = c
        elif c == "#":
            j = i
            while j > 0 and rest[j - 1] in " \t":
                j -= 1
            return rest[:j], rest[j:]
        i += 1
    stripped = rest.rstrip()
    return stripped, rest[len(stripped):]


def find_lines(text: str) -> Dict[str, int]:
    """{key path: line index}: the entries a line of the file holds (system entries and bare tool values)."""
    out: Dict[str, int] = {}
    section = ""
    for i, line in enumerate(text.split("\n")):
        m = _HEADER.match(line)
        if m:
            section = ".".join(_unquote(x.strip()) for x in m.group(1).split("."))
            continue
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        k = _KEY.match(line)
        if k:
            name = _unquote(k.group(2))
            out[f"{section}.{name}" if section else name] = i
    return out


def editable(text: str, key: str) -> bool:
    return key in find_lines(text)


def toml_literal(v: Any) -> str:
    """A Python value as a TOML literal (bool, int, float, string, array)."""
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        if math.isnan(v):
            return "nan"
        if math.isinf(v):
            return "inf" if v > 0 else "-inf"
        s = repr(v)
        return s if any(c in s for c in ".eEn") else s + ".0"
    if isinstance(v, str):
        return '"' + v.replace("\\", "\\\\").replace('"', '\\"') + '"'
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(toml_literal(x) for x in v) + "]"
    if isinstance(v, dict):
        return "{ " + ", ".join(f"{k if re.fullmatch(r'[A-Za-z0-9_-]+', str(k)) else toml_literal(str(k))} = "
                                f"{toml_literal(x)}" for k, x in v.items()) + " }"
    raise TypeError(f"no TOML literal for {type(v).__name__}")


def parse_value(typed: str, old: Any) -> Any:
    """The operator's text read as a TOML value, of the old value's type (an int accepted for a float)."""
    try:
        v = tomllib.loads("x = " + typed.strip())["x"]
    except tomllib.TOMLDecodeError as exc:
        raise ValueError(f"not a TOML value ({exc}); strings need quotes, arrays brackets") from None
    if isinstance(old, bool) or isinstance(v, bool):
        if type(v) is not type(old):
            raise ValueError(f"expected {'true / false' if isinstance(old, bool) else type(old).__name__}")
        return v
    if isinstance(old, float) and isinstance(v, int):
        return float(v)
    if isinstance(old, (int, float)) and not isinstance(v, (int, float)):
        raise ValueError("expected a number")
    if isinstance(old, int) and isinstance(v, float):
        raise ValueError("expected an integer")
    if isinstance(old, str) and not isinstance(v, str):
        raise ValueError('expected a string (in quotes, "...")')
    if isinstance(old, list) and not isinstance(v, list):
        raise ValueError("expected an array [ ... ]")
    if isinstance(old, dict) and not isinstance(v, dict):
        raise ValueError("expected an inline table { key = value, ... }")
    return v


def apply(text: str, edits: Dict[str, Any], today: str) -> str:
    """The file text with each edited entry's value replaced on its own line (system entries: status D, date today)."""
    nl = "\r\n" if "\r\n" in text else "\n"
    lines = text.replace("\r\n", "\n").split("\n")
    where = find_lines("\n".join(lines))
    for key, value in edits.items():
        if key not in where:
            raise pm.ParamError(f"'{key}' cannot be edited here (no line of its own in the file)")
        i = where[key]
        k = _KEY.match(lines[i])
        rest = k.group(4)
        sysm = _SYSTEM.match(rest)
        if sysm:
            body = sysm.group(3)
            body = re.sub(r'(status\s*=\s*)"[^"]*"', r'\1"D"', body, count=1)
            body = re.sub(r'(date\s*=\s*)"[^"]*"', lambda m: m.group(1) + f'"{today}"', body, count=1)
            rest = sysm.group(1) + toml_literal(value) + body
        else:
            _val, tail = _split_comment(rest)
            rest = toml_literal(value) + tail
        lines[i] = k.group(1) + k.group(2) + k.group(3) + rest
    return nl.join(lines)


def check(text: str, edits: Dict[str, Any], before: Optional[pm.Params] = None) -> pm.Params:
    """The edited text through the parameter loader: the values asked, every other one unchanged."""
    fd, tmp = tempfile.mkstemp(suffix=".toml")
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as fh:
            fh.write(text)
        after = pm.load(tmp)
    finally:
        os.remove(tmp)
    ref = {p.path: p.value for p in (before if before is not None else pm.P)}
    got = {p.path: p.value for p in after}
    if set(ref) != set(got):
        raise pm.ParamError(f"the entries changed: {sorted(set(ref) ^ set(got))[:5]}")
    for k, v in got.items():
        want = edits.get(k, ref[k])
        if v != want and not (isinstance(v, float) and isinstance(want, float) and math.isnan(v) and math.isnan(want)):
            raise pm.ParamError(f"'{k}' reads {v!r} after the edit, expected {want!r}")
    return after
