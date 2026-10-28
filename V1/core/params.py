#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PARAMS -- loader of radar_system.toml, the single source of every radar-system value and tool setting.

Every module reads its values here (P.get("band.S.p_pk_trm_w")) instead of holding its own constants, so a change of the
radar architecture is one edit in the file. The file is read once per process, at the first import; worker processes
of the optimiser read the same file (RADAR_PARAMS is inherited from the parent environment).

  P = load()                   the file named by RADAR_PARAMS, else radar_system.toml at the repository root
  P.get("detection.pfa")       a value (KeyError naming the file if missing)
  P.meta("detection.pfa")      the whole entry: Param(value, unit, status, source, date)
  P.section("band.S.losses_db") the values of one section, in file order

Entry of a system section: name = { value = ..., unit = "...", status = "V" | "D" | "?", source = "...", date = "..." }
(all five keys required). [tool.*] sections may give a bare value.
"""
from __future__ import annotations

import os
import tomllib
from dataclasses import dataclass
from typing import Any, Dict, Iterator, Optional, Tuple

PARAMS_ENV = "RADAR_PARAMS"
DEFAULT_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "radar_system.toml")
STATUSES = ("V", "D", "?")
_ENTRY_KEYS = ("value", "unit", "status", "source", "date")


class ParamError(ValueError):
    pass


@dataclass(frozen=True)
class Param:
    path: str
    value: Any
    unit: str = ""
    status: str = ""
    source: str = ""
    date: str = ""


class Params:
    def __init__(self, entries: Dict[str, Param], path: str):
        self._entries = entries
        self.path = path
        self.read_keys: set = set()     # keys the code has read (the test bench flags entries nothing reads)

    def get(self, key: str) -> Any:
        return self.meta(key).value

    def meta(self, key: str) -> Param:
        try:
            p = self._entries[key]
        except KeyError:
            raise KeyError(f"parameter '{key}' missing from {self.path}") from None
        self.read_keys.add(key)
        return p

    def section(self, prefix: str) -> Dict[str, Any]:
        """Values directly under one section, in file order (e.g. "band.S.losses_db" -> {"DSP": 2.0, ...})."""
        n = len(prefix) + 1
        keys = [k for k in self._entries if k.startswith(prefix + ".") and "." not in k[n:]]
        if not keys:
            raise KeyError(f"section '{prefix}' missing from {self.path}")
        self.read_keys.update(keys)
        return {k[n:]: self._entries[k].value for k in keys}

    def __iter__(self) -> Iterator[Param]:
        return iter(self._entries.values())


def _walk(node: dict, prefix: str, path: str, out: Dict[str, Param]) -> None:
    tool = prefix == "tool" or prefix.startswith("tool.")
    for name, item in node.items():
        key = f"{prefix}.{name}" if prefix else name
        if isinstance(item, dict) and "value" in item:
            extra = set(item) - set(_ENTRY_KEYS)
            missing = [k for k in _ENTRY_KEYS if k not in item] if not tool else []
            if extra or missing:
                raise ParamError(f"{path}: entry '{key}' "
                                 + (f"has unknown keys {sorted(extra)}" if extra else f"misses {missing}"))
            if "status" in item and item["status"] not in STATUSES:
                raise ParamError(f"{path}: entry '{key}' has status '{item['status']}' (expected V, D or ?)")
            out[key] = Param(key, item["value"], item.get("unit", ""), item.get("status", ""),
                             item.get("source", ""), item.get("date", ""))
        elif isinstance(item, dict):
            _walk(item, key, path, out)
        elif tool:
            out[key] = Param(key, item)
        else:
            raise ParamError(f"{path}: '{key}' is a bare value in a system section (needs value/unit/status/source/date)")


def load(path: Optional[str] = None) -> Params:
    path = path or os.environ.get(PARAMS_ENV) or DEFAULT_PATH
    try:
        with open(path, "rb") as fh:
            data = tomllib.load(fh)
    except FileNotFoundError:
        raise ParamError(f"radar parameter file not found: {path}") from None
    except tomllib.TOMLDecodeError as exc:
        raise ParamError(f"{path}: {exc}") from None
    entries: Dict[str, Param] = {}
    _walk(data, "", path, entries)
    return Params(entries, path)


# Values a derived file (core/cost_table.json) does NOT depend on: GUI, CLI and mission-prep settings
_FINGERPRINT_SKIP = ("meta.", "tool.cli.", "tool.display.", "tool.mission_defaults.", "tool.redundancy.", "tool.confirmation.",
                     "tool.frames.", "tool.robustness.", "sim.")


def fingerprint(params: "Params") -> str:
    """Hash of every value a derived file depends on (system values + optimiser and cost-table settings). A derived
    file stores it when built; a different fingerprint means the parameter file changed since: rebuild it."""
    import hashlib
    import json
    vals = {p.path: p.value for p in params if not p.path.startswith(_FINGERPRINT_SKIP)}
    return hashlib.sha256(json.dumps(vals, sort_keys=True).encode("utf-8")).hexdigest()[:16]


def summary(params: "Params") -> Tuple[int, Dict[str, int]]:
    """Number of system entries and their count per status (tool settings excluded)."""
    counts = {s: 0 for s in STATUSES}
    n = 0
    for p in params:
        if p.status:
            counts[p.status] += 1
            n += 1
    return n, counts


P = load()
