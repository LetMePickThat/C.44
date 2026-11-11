#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BACKGROUND -- what the fast estimate is still computing outside the UI thread (user, 11 November 2026: any estimated
recalculation shows on the ESTIMATE badge, so that the operator sees it): the cost profiles (core/cost_profile, their
own count), the reliefs (core/redundancy.relief_cached) and the redundancy check of the mission preparation. Each
tracker counts its jobs and estimates the running one's share from the mean duration of the last ones.
"""
from __future__ import annotations

import threading
import time
from typing import Dict, List, Optional, Tuple


class Tracker:
    """Jobs of one kind running in the background: how many, and the share done of the oldest (an estimate)."""

    def __init__(self, label: str, default_s: float):
        self.label = label
        self._lock = threading.Lock()
        self._running: Dict[str, float] = {}
        self._durations: List[float] = []
        self._default_s = default_s

    def start(self, key: str) -> None:
        with self._lock:
            self._running.setdefault(key, time.monotonic())

    def done(self, key: str) -> None:
        with self._lock:
            t0 = self._running.pop(key, None)
            if t0 is not None:
                self._durations.append(time.monotonic() - t0)
                del self._durations[:-8]

    def pending(self) -> int:
        with self._lock:
            return len(self._running)

    def progress(self) -> Optional[Tuple[int, float]]:
        """(jobs running, share done of the oldest: its elapsed time over the mean of the last ones, kept below 1)."""
        with self._lock:
            if not self._running:
                return None
            mean = sum(self._durations) / len(self._durations) if self._durations else self._default_s
            elapsed = time.monotonic() - min(self._running.values())
            return len(self._running), min(0.95, elapsed / max(mean, 1.0))


RELIEF = Tracker("relief", 60.0)
REDUNDANCY = Tracker("redundancy check", 60.0)


def status() -> List[dict]:
    """What is being computed: [{"label", "detail", "fraction"}] (empty: nothing)."""
    from . import cost_profile
    out = []
    pr = cost_profile.progress()
    if pr is not None:
        done, total, frac = pr
        out.append({"label": "cost profiles", "detail": f"{done}/{total}", "fraction": frac})
    for tr in (RELIEF, REDUNDANCY):
        p = tr.progress()
        if p is not None:
            out.append({"label": tr.label, "detail": f"x{p[0]}" if p[0] > 1 else "", "fraction": p[1]})
    return out


def pending() -> int:
    """Background jobs of every kind (the views keep polling while any runs)."""
    from . import cost_profile
    return cost_profile.pending() + REDUNDANCY.pending()        # (cost_profile.pending counts the reliefs too)
