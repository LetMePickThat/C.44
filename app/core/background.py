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

    def cancel(self, key: str) -> None:
        """A job no longer awaited (its inputs replaced): forgotten, its duration not counted."""
        with self._lock:
            self._running.pop(key, None)

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


RELIEF = Tracker("relief", 300.0)            # (counted in the window from the estimate's "relief_pending": the
                                             # relief runs in a process of the estimate's worker, unseen from here)
REDUNDANCY = Tracker("redundancy check", 60.0)
ESTIMATE = Tracker("estimate", 2.0)          # the fast estimate in its worker process (cost_estimate, 12 Nov 2026)
_GEN = [0]


def bump() -> None:
    """A background result the views show changed (they are redrawn: core.cost_profile.generation)."""
    _GEN[0] += 1


def generation() -> int:
    return _GEN[0]


def status() -> List[dict]:
    """What is being computed: [{"label", "detail", "fraction"}] (empty: nothing)."""
    from . import cost_profile
    out = []
    pr = cost_profile.progress()
    if pr is not None:
        done, total, frac = pr
        out.append({"label": "cost profiles", "detail": f"{done}/{total}", "fraction": frac})
    for tr in (ESTIMATE, RELIEF, REDUNDANCY):
        p = tr.progress()
        if p is not None:
            out.append({"label": tr.label, "detail": f"x{p[0]}" if p[0] > 1 else "", "fraction": p[1]})
    return out


def pending() -> int:
    """Background jobs of every kind (the views keep polling while any runs)."""
    from . import cost_profile
    return cost_profile.pending() + REDUNDANCY.pending()        # (cost_profile.pending counts the reliefs and the
                                                                # estimates too)


# ----------------------------------------------------------------------------- pause during an exact calculation
# (user, 13 November 2026: "when an exact calculation is asked, pause the approximate calculation of the instant in
# progress") the fast estimate gives its processors to the exact run: nothing new is started (live_allocation_async
# gives the last result), and the worker processes of the estimate (the live estimate, the cost and instant profiles,
# the reliefs, the redundancy check, with the processes they started) are suspended where they are, then resumed when
# the exact run ends (done, failed or cancelled); what they were computing goes on from there.
_PAUSE = {"on": False, "procs": []}
_lock = threading.Lock()


def _estimate_pids() -> List[int]:
    from . import cost_estimate, cost_profile, redundancy
    pids = []
    for ex in (getattr(cost_estimate, "_A_POOL", None), getattr(cost_profile, "_pool", None),
               getattr(cost_profile, "_ipool", None), getattr(redundancy, "_R_POOL", None),
               getattr(redundancy, "_F_POOL", None)):
        procs = getattr(ex, "_processes", None) or {}
        pids.extend(int(p) for p in procs)
    return pids


def pause_estimate(on: bool) -> None:
    """Suspend (on) or resume (off) the fast estimate (above); idempotent."""
    with _lock:
        if bool(on) == _PAUSE["on"]:
            return
        _PAUSE["on"] = bool(on)
    try:
        import psutil
    except ImportError:                              # (no psutil: nothing new is started, the running jobs go on)
        psutil = None
    if on:
        procs = []
        if psutil is not None:
            for pid in _estimate_pids():
                try:
                    p = psutil.Process(pid)
                    tree = [p] + p.children(recursive=True)
                except psutil.Error:
                    continue
                for q in tree:
                    try:
                        q.suspend()
                        procs.append(q)
                    except psutil.Error:
                        pass
        _PAUSE["procs"] = procs
    else:
        for q in reversed(_PAUSE["procs"]):
            try:
                q.resume()
            except Exception:  # noqa: BLE001 -- a process gone meanwhile
                pass
        _PAUSE["procs"] = []
        bump()                                       # (the views ask again: the estimate goes on)


def estimate_paused() -> bool:
    return bool(_PAUSE["on"])


def _resume_at_exit() -> None:
    if _PAUSE["on"]:
        pause_estimate(False)                        # (never leave a suspended worker behind)


import atexit as _atexit                              # noqa: E402
_atexit.register(_resume_at_exit)
