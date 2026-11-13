#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MISSION REPORT (step 5.5, point C, framed with the user on 13 November 2026, C1-C6): one self-contained HTML page of
an exact result, to keep or pass on (opens anywhere, prints to PDF from a browser, no dependency), in English as the
tool. Sections: (1) identity (file, model version, instant and UTC, navigation, computed, outdated); (2) verdict (state,
limiting resource, margin reuse); (3) constraints in alert or failed with their advice and verifications; (4) every
constraint by branch (Volumes, F1-F4, Ship) with the figures of the state played with the margin reuse, folded;
(5) per volume: asked / held, revisit asked and real, waveforms, faces, confirmation, tracks; (6) per face: loads at the
revisits asked and with the reuse; (7) redundancies and overlaps; (8) Monte-Carlo when it was run; (9) the calculation's
warnings; then the 3D view as shown (C3), the profile along the flight plan when there is one (C6), and the parameter
trace (C5: model version and parameter fingerprint; the whole parameter file as an annex on request). No physics: every
figure is read from the result.
"""
from __future__ import annotations

import base64
import html
import math
from datetime import datetime
from typing import Dict, List, Optional

from . import margin
from . import validation

STATE_COLORS = {"OK": "#2f8a4f", "ALERT": "#c77a12", "DEGRADED": "#c8551e", "NOK": "#c0392b", "INCOMPLETE": "#6b7280"}
CSS = """
body{font-family:Segoe UI,Arial,sans-serif;margin:24px auto;max-width:1200px;color:#1f2328;background:#fff;font-size:13px}
h1{font-size:22px;margin:0 0 4px}h2{font-size:17px;margin:26px 0 8px;border-bottom:1px solid #d0d7de;padding-bottom:4px}
h3{font-size:14px;margin:14px 0 6px}table{border-collapse:collapse;width:100%;margin:6px 0 10px}
th,td{border:1px solid #d0d7de;padding:3px 6px;text-align:left;vertical-align:top}th{background:#f6f8fa}
td.n{text-align:right;font-variant-numeric:tabular-nums}.muted{color:#6b7280}.badge{display:inline-block;padding:2px 8px;
border-radius:4px;color:#fff;font-weight:600}details{margin:4px 0}summary{cursor:pointer;font-weight:600}
.adv{color:#8a5a00}img{max-width:100%;border:1px solid #d0d7de}code{font-size:12px}
@media print{details{display:block}details>summary{display:none}}
"""


def _e(x) -> str:
    return html.escape("" if x is None else str(x))


def _num(x, fmt: str = ".4g") -> str:
    if isinstance(x, bool) or x is None:
        return "-" if x is None else str(x)
    if isinstance(x, (int, float)):
        return "-" if not math.isfinite(float(x)) else format(float(x), fmt)
    return str(x)


def _pct(x) -> str:
    return "-" if x is None else f"{100 * float(x):.1f}"


def _state(s: str) -> str:
    return f"<span class='badge' style='background:{STATE_COLORS.get(s, '#6b7280')}'>{_e(s)}</span>"


def _table(head: List[str], rows: List[List[str]], numeric: Optional[List[int]] = None) -> str:
    numeric = numeric or []
    out = ["<table><tr>" + "".join(f"<th>{_e(h)}</th>" for h in head) + "</tr>"]
    for r in rows:
        out.append("<tr>" + "".join(f"<td{' class=n' if j in numeric else ''}>{c}</td>" for j, c in enumerate(r))
                   + "</tr>")
    out.append("</table>")
    return "".join(out)


def _row_cells(r: dict, obj: bool = True) -> List[str]:
    wr = r.get("with_reuse") or {}
    unit = f" {r['unit']}" if r.get("unit") and r.get("value") is not None else ""
    unit_r = f" {wr['unit']}" if wr.get("unit") and wr.get("value") is not None else ""
    ci = r.get("ci")
    return ([_e(r.get("object"))] if obj else []) + [
        _e(r.get("quantity")), _state(r.get("state", "")), _e(r.get("source", "model")), _e(r.get("priority") or "-"),
        _pct(r.get("load")), _pct(wr.get("load")) if wr else "-", _e(_num(r.get("value")) + unit),
        _e(_num(wr.get("value")) + unit_r) if wr else "-", _e(_num(r.get("limit"))), _e(_num(r.get("analytic"))),
        _e(f"[{_num(ci[0])}, {_num(ci[1])}]") if ci else "-", _e(r.get("note", ""))]


def _advice_line(a: dict) -> str:
    change = validation.changes_text(a, most=len(a["changes"])) if a.get("changes") else "-"
    ver = a.get("verified")
    v = ("" if ver is None else (" &mdash; <b>verified: holds</b>" if ver.get("holds") else
                                 " &mdash; <b>verified: does not hold</b>"))
    extra = "; ".join(x for x in (a.get("law"), "approximate" if a.get("approximate") else "", a.get("note"))
                      if x)
    return (f"<li class='adv'>advice {a['n']} ({_e(a.get('kind'))}{' on ' + _e(a['item']) if a.get('item') else ''}):"
            f" <b>{_e(change)}</b>{(' &mdash; ' + _e(extra)) if extra else ''}{v}</li>")


def _identity(mission: dict, result: dict, meta: dict) -> str:
    g = result.get("global") or {}
    nav = g.get("nav") or {}
    rows = [["File", _e(meta.get("file") or "-")],
            ["Model version", _e(result.get("_model_version") or meta.get("model_version") or "-")],
            ["Instant", _e(f"t = {meta.get('t_s')} s" if meta.get("t_s") is not None else "-")
             + (f" &nbsp; {_e(meta['utc'])}" if meta.get("utc") else "")],
            ["Computed", _e(result.get("_computed_local") or "-")],
            ["State of the result", "<b style='color:#c0392b'>OUTDATED: the mission was edited since</b>"
             if meta.get("outdated") else "current"],
            ["Scan zone", _e(f"eps {g.get('eps_min_deg', '-')} .. {g.get('eps_max_deg', '-')} deg")],
            ["Realloc mode / load ceiling", _e(f"{margin.realloc_mode(mission)} / "
                                               f"{_pct((result.get('verdict') or {}).get('load_ceiling'))} %")],
            ["Navigation", _e(", ".join(f"{k}: {v}" for k, v in nav.items() if not isinstance(v, (list, dict)))
                              or "ship frame")],
            ["Report written", _e(datetime.now().strftime("%Y-%m-%d %H:%M"))]]
    return "<h2>1. Identity</h2>" + _table(["", ""], rows)


def _verdict(vd: dict, result: dict) -> str:
    lines = validation.summary_lines(vd) if vd else []
    out = [f"<h2>2. Verdict &nbsp; {_state(vd.get('state', '-'))}</h2>"]
    lim = vd.get("limiting")
    c = vd.get("counts") or {}
    out.append(f"<p>{c.get('NOK', 0)} NOK, {c.get('DEGRADED', 0)} degraded, {c.get('ALERT', 0)} alert, "
               f"{c.get('OK', 0)} OK (load ceiling {_pct(vd.get('load_ceiling'))} %)."
               + (f" Limiting resource at the revisits asked: <b>{_e(lim['object'])} {_e(lim['resource'])}</b> at "
                  f"{_pct(lim['load'])} %." if lim else "") + "</p>")
    wr = vd.get("with_reuse")
    if wr and wr.get("sectors"):
        ls = wr.get("limiting") or {}
        out.append("<p>Margin reuse: " + ", ".join(
            f"{_e(s)} {_num(v.get('rr_target_s'), 'g')} &rarr; {_num(v.get('rr_real_s'), '.3g')} s "
            f"(x{_num(v.get('g'), '.3g')}, {_e(v.get('limited_by'))})" for s, v in wr["sectors"].items())
            + (f"; most loaded then: <b>{_e(ls.get('object'))} {_e(ls.get('resource'))}</b> {_pct(ls.get('load'))} %"
               if ls else "") + ". The verdict stays at the revisits asked.</p>")
    pk = result.get("peak")
    if pk:
        out.append("<p>Peak of targets: " + ("absorbed" if pk.get("status") == "ABSORBED" else "<b>NOT absorbed</b>")
                   + (", P4 shed while it lasts: " + _e(", ".join(sorted(pk["shed"]))) if pk.get("shed") else "")
                   + ".</p>")
    if vd.get("tight"):
        out.append("<p>At the limit: " + _e("; ".join(vd["tight"])) + "</p>")
    if lines:
        out.append("<details><summary>Verdict as text</summary><pre>" + _e("\n".join(lines)) + "</pre></details>")
    return "".join(out)


def _bad(vd: dict, tree: List[dict]) -> str:
    head = ["object", "constraint", "state", "source", "priority", "load %", "load % (reuse)", "value",
            "value (reuse)", "limit", "analytic", "MC interval", "note"]
    out = ["<h2>3. Constraints in alert or failed, with their advice</h2>"]
    rows = [(r, al) for b in tree for g in b["groups"] for r, al in g["rows"] if r["state"] != "OK"]
    if not rows:
        return out[0] + "<p>None: every constraint is OK at the revisits asked.</p>"
    for r, al in rows:
        out.append(_table(head, [_row_cells(r)], [5, 6]))
        if al:
            out.append("<ul>" + "".join(_advice_line(a) for a in al) + "</ul>")
    return "".join(out)


def _all(tree: List[dict]) -> str:
    head = ["constraint", "state", "source", "priority", "load %", "load % (reuse)", "value", "value (reuse)", "limit",
            "analytic", "MC interval", "note"]
    out = ["<h2>4. Every constraint</h2><p class=muted>Load %: at the revisits asked (the verdict); (reuse): the "
           "state played with the margin reuse.</p>"]
    for b in tree:
        n_bad = sum(1 for g in b["groups"] for r, _a in g["rows"] if r["state"] != "OK")
        out.append(f"<details{' open' if n_bad else ''}><summary>{_e(b['branch'])} &nbsp; {_state(b['state'])}"
                   f" <span class=muted>({n_bad} in alert or failed)</span></summary>")
        for g in b["groups"]:
            if g["sector"] is not None:
                out.append(f"<h3>{_e(g['sector'])} &nbsp; {_state(g['state'])}</h3>")
            out.append(_table(head, [_row_cells(r, obj=False) if str(r.get("object")) in (str(g["sector"]), b["branch"])
                                     else _row_cells(dict(r, quantity=f"{r.get('object')} {r.get('quantity')}"),
                                                     obj=False) for r, _a in g["rows"]], [4, 5]))
        out.append("</details>")
    return "".join(out)


def _volumes(mission: dict, result: dict) -> str:
    secs = {s.get("id"): s for s in mission.get("sectors", [])}
    reuse = (result.get("margin_reuse") or {}).get("sectors", {})
    trk = {s["id"]: s for s in (result.get("tracking") or {}).get("sectors", [])}
    rows = []
    for r in result.get("sectors", []):
        sid = r.get("id")
        sec = secs.get(sid, {})
        rg = r.get("range_km") if isinstance(r.get("range_km"), dict) else {}
        wf = r.get("wf") or {}
        mr = reuse.get(sid) or {}
        t = trk.get(sid) or {}
        n_tr = sum(float(b.get("n_targets", 0) or 0) for b in t.get("at", []))
        faces = ", ".join(f"F{int(k) + 1}" for k in sorted(r.get("faces", {}), key=int))
        rows.append([_e(sid), _state(r.get("status", "-")) if r.get("status") in STATE_COLORS else _e(r.get("status")),
                     _e(_num(sec.get("range_km"), "g")), _e(_num(rg.get("at_worst_scan"), ".0f")),
                     _e(_num(rg.get("at_best_scan"), ".0f")), _e(_num(sec.get("rr_target_s"), "g")),
                     _e(_num(r.get("rr_est_mission_s"), ".3g")),
                     _e(_num(mr.get("rr_real_s"), ".3g") if mr else "-"),
                     _e(f"{_num(wf.get('freq_ghz'), 'g')} GHz, {len(r.get('wf_set') or []) or 1} WF"
                        if wf else "-"), _e(faces or "-"), _e(t.get("confirm_mode") or "-"),
                     _e(f"{n_tr:g}" if t.get("at") else "-"),
                     _e("; ".join(r.get("flags", [])) or r.get("note", "") or "")])
    return ("<h2>5. Volumes</h2>" + _table(["id", "status", "range asked (km)", "worst range (km)", "best range (km)",
                                             "revisit asked (s)", "revisit in mission (s)", "revisit with reuse (s)",
                                             "waveforms", "faces", "confirmation", "tracks", "flags"], rows,
                                            [2, 3, 4, 5, 6, 7]))


def _faces(result: dict) -> str:
    trk = (result.get("tracking") or {}).get("faces", {})
    pa = (result.get("power_allocation") or {}).get("faces", {})
    mr = result.get("margin_reuse") or {}
    pa_r = (mr.get("power_allocation") or {}).get("faces", {})
    mr_f = mr.get("faces") or {}
    rows = []
    for k in range(4):
        f, a, ar = trk.get(str(k), {}), pa.get(str(k), {}), pa_r.get(str(k), {})
        rr = (mr_f.get(str(k)) or {}).get("resources_real") or {}
        rows.append([f"F{k + 1}", _e(_num(a.get("alloc_MW"), ".3f")), _e(_num(ar.get("alloc_MW"), ".3f") if ar else "-"),
                     _e(_num(a.get("always_on_MW"), ".3f")), _pct(f.get("total_s_time")),
                     _pct(rr.get("S time")) if rr else "-", _pct(f.get("total_ku_time")),
                     _pct(rr.get("Ku time")) if rr else "-", _pct(f.get("load")), _e(f.get("binding", "-")),
                     _e(f.get("status", "-"))])
    return ("<h2>6. Faces</h2>" + _table(["face", "power drawn (MW)", "with reuse (MW)", "always-on (MW)", "S time %",
                                           "with reuse %", "Ku time %", "with reuse %", "energy load %", "binding",
                                           "status"], rows, [1, 2, 3, 4, 5, 6, 7, 8]))


def _redundancy(vd: dict) -> str:
    rows = []
    for d in vd.get("redundancy", []):
        rows.append(["redundancy", _e(d.get("sector")), _e(", ".join(d.get("by", []))),
                     _e(f"{100 * float(d.get('fraction', 0)):.0f} % held"), _e(validation.redundancy_text(d))])
    for o in vd.get("overlap", []):
        if o.get("takeable", 0) <= 0:
            continue
        rows.append(["overlap", _e(o.get("face")), _e(o.get("neighbour")),
                     _e(f"{100 * float(o['takeable']):.0f} % takeable"), _e(validation.overlap_text(o))])
    return "<h2>7. Redundancies and overlaps</h2>" + (_table(["kind", "on", "by / neighbour", "share", "detail"], rows)
                                                       if rows else "<p>None.</p>")


def _montecarlo(vd: dict) -> str:
    m = vd.get("montecarlo")
    if not m:
        return "<h2>8. Monte-Carlo</h2><p class=muted>Not run for this result.</p>"
    c = m.get("load_ceiling") or {}
    return (f"<h2>8. Monte-Carlo &nbsp; {_state(m.get('state', '-'))}</h2>"
            + (f"<p>Load ceiling proposed {100 * float(c['proposed']):.1f} % of the transmit time (not applied).</p>"
               if c.get("proposed") is not None else "")
            + "<p class=muted>Its rows are in the constraints above (source MC).</p>")


def _warnings(result: dict) -> str:
    w = list(result.get("warnings") or [])
    return "<h2>9. Warnings of the calculation</h2>" + ("<ul>" + "".join(f"<li>{_e(x)}</li>" for x in w) + "</ul>"
                                                       if w else "<p>None.</p>")


def _image(png: Optional[bytes]) -> str:
    if not png:
        return ""
    return ("<h2>3D view</h2><p class=muted>As shown in the Results tab at the export.</p>"
            f"<img alt='3D view' src='data:image/png;base64,{base64.b64encode(png).decode('ascii')}'>")


def _profile(prof: Optional[dict]) -> str:
    if not prof or not prof.get("t_s"):
        return ""
    out = [f"<h2>Profile along the flight plan</h2><p>{len(prof['t_s'])} instants over "
           f"{_num(prof.get('duration_s'), '.0f')} s (fast estimate, load ceiling {_pct(prof.get('load_ceiling'))} %)"
           + (" &mdash; <b>partial (stopped)</b>" if prof.get("cancelled") else "") + ".</p>"]
    rows = []
    for sid, a in (prof.get("availability") or {}).items():
        rows.append([_e(sid), _pct(a.get("held")), _pct(a.get("degraded")), _pct(a.get("lost")),
                     _e(_num(a.get("longest_outage_s"), ".3g")), _e(_num(a.get("outage_over_rr"), ".3g")),
                     _pct(a.get("covered_mean")), _pct(a.get("covered_min"))])
    out.append(_table(["volume", "held %", "degraded %", "lost %", "longest outage (s)", "outage / revisit",
                       "covered mean %", "covered min %"], rows, [1, 2, 3, 4, 5, 6, 7]))
    ev = prof.get("events") or []
    if ev:
        out.append("<details><summary>Events (" + str(len(ev)) + ")</summary>" + _table(
            ["t (s)", "volume", "change", "flagged for an exact run"],
            [[_e(_num(e.get("t_s"), ".1f")), _e(e.get("object")), _e(e.get("what")), "yes" if e.get("flagged") else ""]
             for e in ev], [0]) + "</details>")
    return "".join(out)


def _params(annex: bool) -> str:
    from . import params as pm
    from . import radar_model as rm
    P = pm.P
    n, counts = pm.summary(P)
    out = [f"<h2>Parameters</h2><p>Model version <b>{_e(rm.MODEL_VERSION)}</b>; parameter file "
           f"<code>{_e(P.path)}</code>, fingerprint <code>{_e(pm.fingerprint(P))}</code>, values stamp "
           f"<code>{_e(pm.values_stamp(P))}</code>; {n} system entries: "
           + ", ".join(f"{v} {k}" for k, v in counts.items()) + " (V locked, D derived / to confirm, ? to settle).</p>"]
    if annex:
        rows = [[f"<code>{_e(p.path)}</code>", _e(p.value if not isinstance(p.value, float) else _num(p.value, ".6g")),
                 _e(p.unit), _e(p.status), _e(p.source), _e(p.date)] for p in P]
        out.append("<details><summary>Every parameter (" + str(len(rows)) + ")</summary>"
                   + _table(["key", "value", "unit", "status", "source", "date"], rows) + "</details>")
    return "".join(out)


def build_html(mission: dict, result: dict, meta: Optional[dict] = None, image_png: Optional[bytes] = None,
               profile: Optional[dict] = None, params_annex: bool = False) -> str:
    """The report page (module docstring). meta: {"file", "t_s", "utc", "outdated"}."""
    meta = meta or {}
    vd = result.get("verdict") or {}
    ids = [r.get("id") for r in result.get("sectors", [])] + [s.get("id") for s in mission.get("sectors", [])]
    tree = validation.verdict_tree(vd, ids) if vd else []
    title = f"Mission report — {meta.get('file') or 'mission'}"
    body = [f"<h1>{_e(title)}</h1><p class=muted>Radar mission preparation, exact calculation. Every figure is a "
            f"model output of a fictional system (test values are not radar performance).</p>",
            _identity(mission, result, meta), _verdict(vd, result) if vd else "<h2>2. Verdict</h2><p>None.</p>",
            _bad(vd, tree), _all(tree), _volumes(mission, result), _faces(result), _redundancy(vd), _montecarlo(vd),
            _warnings(result), _image(image_png), _profile(profile), _params(params_annex)]
    return ("<!doctype html><html lang='en'><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width, initial-scale=1'>"
            f"<title>{_e(title)}</title><style>{CSS}</style></head><body>" + "".join(body) + "</body></html>")
