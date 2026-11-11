#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CONTROLS -- the commands of the window where they act (B14, user, 29 October 2026, replaces the ribbon B13): small
tool buttons, a "Layers" drop-down of display toggles, and panels drawn over a 3D view (anchored to one of its
corners, native windows so they stay above the OpenGL one).
"""
from __future__ import annotations

from typing import Callable, Dict, List, Optional, Sequence, Tuple

from PyQt5.QtCore import QEvent, QObject, QRect, QSize, Qt
from PyQt5.QtGui import QColor, QFont, QFontMetrics, QIcon, QPainter, QPalette, QPen
from PyQt5.QtWidgets import (QAbstractScrollArea, QAbstractSpinBox, QApplication, QComboBox, QCheckBox, QFrame, QHBoxLayout, QHeaderView, QLabel, QMenu, QMessageBox, QSlider,
                             QStyle, QStyleOptionSlider, QTableWidget, QTableWidgetItem, QToolButton, QToolTip,
                             QWidget, QWidgetAction)

OVERLAY_GAP = 8                                   # px between an overlay and the view's edges
OVERLAY_STYLES = {                                # (background, edge, text) of an overlay on a dark / light view
    "dark": ("#161d33", "#3a4566", "#e6e6e6"),
    "light": ("#ffffff", "#c8c8c8", "#202020"),
}


def std_icon(widget: QWidget, name: Optional[str]):
    """A standard Qt icon by its QStyle name (e.g. "SP_DialogSaveButton"), or None."""
    if not name:
        return None
    return widget.style().standardIcon(getattr(QStyle, name))


# the attributions of the textures of the Earth, the Moon and the Sun (assets/textures), shown by File > About
TEXTURE_ATTRIBUTIONS = ("Creative Commons Attribution 4.0 International", "NASA")
MODEL_ATTRIBUTIONS = (                            # the ship's model, the moons' shape models (user, 29 October 2026)
    "Ship model: Theo Bouvier (https://www.artstation.com/theo_bouvier)",
    "Phobos, Deimos: NASA/JPL-Caltech",
    "Thomas, P., Joseph, J., and Ansty, T., Saturn Small Moon Shape Models V1.0. "
    "urn:nasa:pds:saturn_satellite_shape_models::1.0. NASA Planetary Data System, 2018",
    "Stooke, P., Small Body Shape Models. EAR-A-5-DDR-STOOKE-SHAPE-MODELS-V2.0. NASA Planetary Data System, 2002",
    "Schenk, P. (2010) New Moons – First Global Topographic Maps of (Saturn's) Icy Moons")


def file_title(path, modified: bool) -> str:
    """The document's name as both tools show it (title bar, status bar; pass V3, user 10 November 2026): its file
    name, "(not saved)" before a first save, " *" when it has unsaved changes."""
    import os
    return (os.path.basename(path) if path else "(not saved)") + (" *" if modified else "")


def confirm_discard(parent: QWidget, tool: str, modified: bool, save: Optional[Callable[[], object]] = None,
                    is_modified: Optional[Callable[[], bool]] = None) -> bool:
    """Before New, Open, an import or closing the window (pass V3): True when the action may go on. User, 11 November
    2026: "Save the changes?" (Save / Don't save / Cancel) instead of "Discard the unsaved changes?" (Yes / No); Save
    saves first (the action goes on only once saved: a save dialog cancelled stops it)."""
    if not modified:
        return True
    buttons = (QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel) if save is not None else         (QMessageBox.Discard | QMessageBox.Cancel)
    box = QMessageBox(QMessageBox.Question, tool, f"Save the changes to the {tool.lower()}?", buttons, parent)
    box.setDefaultButton(QMessageBox.Save if save is not None else QMessageBox.Cancel)
    if save is not None:
        box.button(QMessageBox.Save).setText("Save")
    box.button(QMessageBox.Discard).setText("Don't save")
    answer = box.exec_()
    if answer == QMessageBox.Discard:
        return True
    if answer == QMessageBox.Save and save is not None:
        save()
        return not (is_modified() if is_modified is not None else False)
    return False


class _WheelScrollsPage(QObject):
    """The wheel never changes a value (user, 10 November 2026: scrolling the mission preparation's long columns kept
    changing the spin box or list under the pointer): over a spin box, a list or a slider it scrolls the page that
    holds it, as anywhere else; the values change by the keyboard, the arrows or the mouse."""

    def eventFilter(self, obj, event) -> bool:
        if event.type() != QEvent.Wheel or not isinstance(obj, (QAbstractSpinBox, QComboBox, QSlider)):
            return False
        w = obj.parentWidget()
        while w is not None and not isinstance(w, QAbstractScrollArea):
            w = w.parentWidget()
        if w is not None:
            QApplication.sendEvent(w.viewport(), event)   # (the page scrolls)
        return True                                       # (the value stays)


def wheel_scrolls_page(app: QApplication) -> QObject:
    """Installs _WheelScrollsPage on the application; keep the returned object alive."""
    f = _WheelScrollsPage(app)
    app.installEventFilter(f)
    return f


def show_about(parent: QWidget, tool: str) -> None:
    """File > About (user, 29 October 2026): a box in the middle of the window with the attributions."""
    box = QMessageBox(parent)
    box.setWindowTitle("About")
    box.setTextFormat(Qt.RichText)
    box.setText(f"<b>{tool}</b>")
    from gui.sky import ATTRIBUTIONS as SKY
    box.setInformativeText("<p style='white-space:pre'>Textures of the Earth, the Moon and the Sun:<br>"
                           + "<br>".join(TEXTURE_ATTRIBUTIONS) + "<br><br>Sky:<br>" + "<br>".join(SKY)
                           + "<br><br>3D models:<br>" + "<br>".join(MODEL_ATTRIBUTIONS)
                           + "</p>")                                             # lines kept whole
    box.setStandardButtons(QMessageBox.Ok)
    box.exec_()


def tool_button(text: str, slot: Optional[Callable] = None, icon: Optional[str] = None, tip: str = "",
                icon_px: int = 16) -> QToolButton:
    """A command: its icon beside its text (text only without an icon)."""
    b = QToolButton()
    b.setText(text)
    b.setAutoRaise(True)
    ic = std_icon(b, icon)
    if ic is not None:
        b.setIcon(ic)
        b.setIconSize(QSize(icon_px, icon_px))
    b.setToolButtonStyle(Qt.ToolButtonTextBesideIcon if ic is not None else Qt.ToolButtonTextOnly)
    if tip:
        b.setToolTip(tip)
    if slot is not None:
        b.clicked.connect(lambda _c=False: slot())
    return b


def labelled(text: str, w: QWidget) -> QWidget:
    """A label and its control on one row."""
    box = QWidget()
    h = QHBoxLayout(box)
    h.setContentsMargins(0, 0, 0, 0)
    h.setSpacing(4)
    h.addWidget(QLabel(text))
    h.addWidget(w)
    return box


def menu_button(text: str, menu: QMenu, icon: Optional[str] = None, tip: str = "", arrow: bool = True
                ) -> QToolButton:
    """A button that opens its menu at once; without its arrow (arrow False) for a one-sign button ("+", drawn
    larger) or a menu-bar-like word ("File")."""
    b = tool_button(text, None, icon, tip)
    b.setMenu(menu)
    b.setPopupMode(QToolButton.InstantPopup)
    if not arrow:
        size = "font-size: 15pt; " if len(text) == 1 else ""
        b.setStyleSheet(f"QToolButton {{{size}padding: 0 6px}} QToolButton::menu-indicator {{image: none}}")
    return b


def layers_menu(parent: QWidget, sections: Sequence[Tuple[Optional[str], Sequence[object]]]
                ) -> Tuple[QMenu, Dict[object, QWidgetAction]]:
    """The "Layers" menu: sections (title or None, items); an item is a widget (a check box, a labelled row: kept
    open while it is used) or (text, slot) for a command. Returns the menu and the widget -> action map (to show or
    hide an item)."""
    menu = QMenu(parent)
    acts: Dict[object, QWidgetAction] = {}
    for title, items in sections:
        if title:
            menu.addSection(title)
        elif not menu.isEmpty():
            menu.addSeparator()
        for it in items:
            if isinstance(it, tuple):
                text, slot = it
                menu.addAction(text, slot)
                continue
            act = QWidgetAction(menu)
            box = QWidget()
            h = QHBoxLayout(box)
            h.setContentsMargins(12, 2, 12, 2)
            h.addWidget(it)
            act.setDefaultWidget(box)
            menu.addAction(act)
            acts[it] = act
    return menu, acts


class Toggle(QCheckBox):
    """An on / off switch (user, 29 October 2026): a check box drawn as a sliding switch, its text on the right
    (in `color` if given), and `off_text` on its left for a two-way choice (e.g. "Global" / "Ship"); two_way: such a
    choice, its other side named by the caller."""

    TRACK_W, TRACK_H, GAP = 30, 16, 6
    ON_COLOR, OFF_COLOR = QColor("#2f7de1"), QColor("#8a8f99")

    def __init__(self, text: str, checked: bool = False, color: Optional[str] = None, off_text: str = "",
                 tip: str = "", two_way: bool = False):
        super().__init__(text)
        self.off_text = off_text
        self.two_way = two_way or bool(off_text)
        self._color = QColor(color) if color else None
        self._on_color = self.ON_COLOR
        self.setChecked(checked)
        self.setCursor(Qt.PointingHandCursor)
        if tip:
            self.setToolTip(tip)

    def set_on_color(self, color: str) -> None:
        """The track's colour when on (a two-way choice: on both sides)."""
        self._on_color = QColor(color)
        self.update()

    def _off_w(self) -> int:
        return self.fontMetrics().horizontalAdvance(self.off_text) + self.GAP if self.off_text else 0

    def sizeHint(self) -> QSize:
        fm = self.fontMetrics()
        return QSize(self._off_w() + self.TRACK_W + self.GAP + fm.horizontalAdvance(self.text()) + 2,
                     max(self.TRACK_H, fm.height()) + 4)

    def minimumSizeHint(self) -> QSize:
        return self.sizeHint()

    def hitButton(self, pos) -> bool:
        return self.rect().contains(pos)

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        on, h = self.isChecked(), self.height()
        text = self._color or self.palette().color(QPalette.WindowText)
        if not self.isEnabled():
            text = QColor(text)
            text.setAlpha(110)
        x0 = self._off_w()
        if self.off_text:
            p.setPen(text)
            p.drawText(QRect(0, 0, x0, h), Qt.AlignVCenter | Qt.AlignLeft, self.off_text)
        # a two-way choice: both sides are a choice, one colour
        track = QColor(self._on_color if on or self.two_way else self.OFF_COLOR)
        if not self.isEnabled():
            track.setAlpha(90)
        y0 = (h - self.TRACK_H) // 2
        p.setPen(Qt.NoPen)
        p.setBrush(track)
        p.drawRoundedRect(QRect(x0, y0, self.TRACK_W, self.TRACK_H), self.TRACK_H / 2, self.TRACK_H / 2)
        d = self.TRACK_H - 4
        p.setBrush(QColor("#ffffff"))
        p.drawEllipse(QRect(x0 + (self.TRACK_W - d - 2 if on else 2), y0 + 2, d, d))
        p.setPen(text)
        p.drawText(QRect(x0 + self.TRACK_W + self.GAP, 0, self.width(), h), Qt.AlignVCenter | Qt.AlignLeft,
                   self.text())


class InfoTable(QTableWidget):
    """The Info box (user, 29 October 2026): one piece of information per row, its label and its value in two
    columns, in the window's font; no wrapping, no horizontal scroll. Filled as text: one line per row, label and
    value separated by a tab (setPlainText / toPlainText, as the plain text box it replaces)."""

    PLACEHOLDER_COLOR = "#8a8f99"

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(0, 2, parent)
        self._text, self._placeholder = "", ""
        self.horizontalHeader().hide()
        self.verticalHeader().hide()
        self.setShowGrid(False)
        self.setWordWrap(False)
        self.setEditTriggers(QTableWidget.NoEditTriggers)
        self.setSelectionMode(QTableWidget.NoSelection)
        self.setFocusPolicy(Qt.NoFocus)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.horizontalHeader().setStretchLastSection(True)
        row = self.fontMetrics().height() + 4                     # rows as tight as a text line
        self.verticalHeader().setMinimumSectionSize(row)
        self.verticalHeader().setDefaultSectionSize(row)
        self.verticalHeader().setSectionResizeMode(QHeaderView.Fixed)
        self.setStyleSheet("QTableWidget::item {padding: 0 4px}")

    def setPlaceholderText(self, text: str) -> None:
        self._placeholder = text
        if not self._text:
            self.setPlainText("")

    def toPlainText(self) -> str:
        return self._text

    def setPlainText(self, text: str) -> None:
        self._text = text
        rows = [ln.split("\t", 1) for ln in text.splitlines()] if text else [["", self._placeholder]]
        self.setRowCount(len(rows))
        for i, r in enumerate(rows):
            for j, t in enumerate((r + [""])[:2]):
                it = QTableWidgetItem(t)
                if not text:
                    it.setForeground(QColor(self.PLACEHOLDER_COLOR))
                self.setItem(i, j, it)

    def set_icon(self, path: Optional[str], px: int = 32) -> None:
        """An icon (the object's symbol) beside the value of the first row; None: no icon."""
        if self.rowCount() == 0 or self.item(0, 1) is None:
            return
        if path:
            self.setIconSize(QSize(px, px))
            self.item(0, 1).setIcon(QIcon(path))
            self.setRowHeight(0, max(px + 4, self.verticalHeader().defaultSectionSize()))
        else:
            self.item(0, 1).setIcon(QIcon())
            self.setRowHeight(0, self.verticalHeader().defaultSectionSize())

    def needed_width(self, lines: Sequence[str]) -> int:
        """px: the box's width that shows these lines (label TAB value) whole, with its scroll bar."""
        fm = self.fontMetrics()
        pairs = [(ln.split("\t", 1) + [""])[:2] for ln in lines]
        lab = max([fm.horizontalAdvance(a) for a, _ in pairs] + [0])
        val = max([fm.horizontalAdvance(b) for _, b in pairs] + [0])
        pad = 2 * self.style().pixelMetric(QStyle.PM_FocusFrameHMargin) + 12      # cell paddings
        return lab + val + 2 * pad + 2 * self.frameWidth() + self.verticalScrollBar().sizeHint().width() + 4


class MarkedSlider(QSlider):
    """A horizontal slider with marks along its groove (user, 29 October 2026: the events on the playback bar): a
    balloon gives the detail of the marks under the mouse, a click on a mark goes to it. Marks: (value, text); the
    marks closer than MARK_PX on screen are one."""

    MARK_PX = 4                                       # px: the marks this close are one, and the mouse's reach
    MARK_COLOR = QColor("#e8a33d")
    MAX_LINES = 25                                    # lines of a balloon at most

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(Qt.Horizontal, parent)
        self._marks: List[Tuple[int, str]] = []
        self.setMouseTracking(True)

    def set_marks(self, marks: Sequence[Tuple[int, str]]) -> None:
        self._marks = sorted((int(v), t) for v, t in marks)
        self.update()

    def _groove(self) -> Tuple[int, int]:
        """x of the minimum and the maximum on screen (the handle's centre at both ends)."""
        opt = QStyleOptionSlider()
        self.initStyleOption(opt)
        groove = self.style().subControlRect(QStyle.CC_Slider, opt, QStyle.SC_SliderGroove, self)
        handle = self.style().subControlRect(QStyle.CC_Slider, opt, QStyle.SC_SliderHandle, self)
        return groove.left() + handle.width() // 2, groove.right() - handle.width() // 2

    def _x(self, v: int) -> float:
        x0, x1 = self._groove()
        span = max(self.maximum() - self.minimum(), 1)
        return x0 + (v - self.minimum()) / span * (x1 - x0)

    def _clusters(self) -> List[Tuple[float, int, List[str]]]:
        """(x, first value, texts) of the marks, those within MARK_PX merged."""
        out: List[Tuple[float, int, List[str]]] = []
        for v, t in self._marks:
            x = self._x(v)
            if out and x - out[-1][0] <= self.MARK_PX:
                out[-1][2].append(t)
            else:
                out.append((x, v, [t]))
        return out

    def _near(self, x: float):
        best = None
        for c in self._clusters():
            if abs(c[0] - x) <= self.MARK_PX and (best is None or abs(c[0] - x) < abs(best[0] - x)):
                best = c
        return best

    def paintEvent(self, event) -> None:
        super().paintEvent(event)
        if not self._marks:
            return
        p = QPainter(self)
        p.setPen(QPen(self.MARK_COLOR, 2))
        h = self.height()
        for x, _v, _t in self._clusters():
            p.drawLine(int(round(x)), 1, int(round(x)), h - 2)

    def mouseMoveEvent(self, event) -> None:
        # no balloon while a button is held (the handle dragged over the marks, user 29 October 2026)
        c = self._near(event.pos().x()) if event.buttons() == Qt.NoButton and not self.isSliderDown() else None
        if c is None:
            QToolTip.hideText()
        else:
            lines = c[2][:self.MAX_LINES]
            if len(c[2]) > self.MAX_LINES:
                lines.append(f"... and {len(c[2]) - self.MAX_LINES} more")
            QToolTip.showText(event.globalPos(), "\n".join(lines), self)
        super().mouseMoveEvent(event)

    def mousePressEvent(self, event) -> None:
        c = self._near(event.pos().x()) if event.button() == Qt.LeftButton else None
        if c is None:
            super().mousePressEvent(event)
        else:
            self.setValue(c[1])


class Overlay(QFrame):
    """A panel over a 3D view, in one of its corners ("top_right", "top_left", "bottom_right", "bottom_left"),
    following the view's size; host: the widget the view lies in (the panel's parent)."""

    def __init__(self, host: QWidget, view: QWidget, corner: str = "top_right", theme: str = "dark"):
        super().__init__(host)
        self.view, self.corner = view, corner
        bg, edge, text = OVERLAY_STYLES[theme]
        self.setObjectName("overlay")
        self.setAttribute(Qt.WA_NativeWindow)              # stays above the native OpenGL window
        self.setStyleSheet(f"#overlay {{background: {bg}; border: 1px solid {edge}; border-radius: 4px}}"
                           f" #overlay QLabel, #overlay QCheckBox, #overlay QToolButton {{color: {text}}}")
        self.row = QHBoxLayout(self)
        self.row.setContentsMargins(6, 3, 6, 3)
        self.row.setSpacing(6)
        self._filter = _Follow(self)
        view.installEventFilter(self._filter)

    def add(self, w: QWidget) -> QWidget:
        self.row.addWidget(w)
        return w

    def place(self) -> None:
        self.adjustSize()
        v = self.view
        right, bottom = "right" in self.corner, "bottom" in self.corner
        p = v.mapTo(self.parentWidget(), v.rect().topLeft())
        x = p.x() + (v.width() - self.width() - OVERLAY_GAP if right else OVERLAY_GAP)
        y = p.y() + (v.height() - self.height() - OVERLAY_GAP if bottom else OVERLAY_GAP)
        self.move(x, y)
        self.raise_()


class _Follow(QObject):
    """Places its overlay again when the view is resized, moved or shown."""

    def __init__(self, overlay: Overlay):
        super().__init__(overlay)
        self.overlay = overlay

    def eventFilter(self, obj, event) -> bool:
        if event.type() in (QEvent.Resize, QEvent.Move, QEvent.Show):
            self.overlay.place()
        return False


# ----------------------------------------------------------------------------- U3: where the figures come from
SOURCE_STYLES = {                       # (user, 11 November 2026: estimates and exact results told apart at a glance)
    "estimate": ("ESTIMATE", "#6b7280"),                     # fast estimate, not recalibrated
    "recalibrated": ("ESTIMATE \u00b7 recalibrated", "#2b579a"),   # fast estimate corrected by exact runs
    "estimating": ("ESTIMATE", "#b7791f"),      # being computed: the detail says what (scene_view.BADGE_ACTIVITY)
    "exact": ("EXACT", "#2f8a4f"),                            # an exact calculation
    "outdated": ("EXACT · OUTDATED", "#a8432f"),           # an exact result, the mission edited since
}


BADGE_WIDEST = "ESTIMATE \u00b7 computing \u00b7 cost profiles 10/10"   # (its width: the badge never resizes)


def source_badge(kind: str = "estimate", detail: str = "", tip: str = "") -> QLabel:
    """A small coloured badge saying where the figures next to it come from (U3); set_source() changes it. Its width
    is fixed (user, 11 November 2026: the face bars beside it moved with its text); a longer text is cut ("...") and
    given whole in the tooltip."""
    lb = QLabel()
    f = QFont(lb.font())
    f.setBold(True)
    w = QFontMetrics(f).horizontalAdvance(f" {BADGE_WIDEST} ") + 12
    lb.setFixedWidth(w)
    lb.setProperty("elide_w", w - 12)
    lb.setAlignment(Qt.AlignCenter)
    set_source(lb, kind, detail, tip)
    return lb


class busy:
    """`with busy(parent, "Opening ..."):` a small window saying what is being done, painted BEFORE the work starts,
    and the wait cursor (user, 11 November 2026: the window froze with nothing said while a file was loaded)."""

    def __init__(self, parent, text: str):
        self.parent, self.text, self.dlg = parent, text, None

    def __enter__(self):
        from PyQt5.QtWidgets import QProgressDialog
        self.dlg = QProgressDialog(self.text, None, 0, 0, self.parent)
        self.dlg.setWindowTitle("Please wait")
        self.dlg.setWindowModality(Qt.WindowModal)
        self.dlg.setMinimumDuration(0)
        self.dlg.setCancelButton(None)
        self.dlg.show()
        QApplication.setOverrideCursor(Qt.WaitCursor)
        for _ in range(3):                             # (shown and painted before the window freezes)
            QApplication.processEvents()
        return self

    def __exit__(self, *exc):
        QApplication.restoreOverrideCursor()
        if self.dlg is not None:
            self.dlg.close()
            self.dlg.deleteLater()
        return False


def set_source(lb: QLabel, kind: str, detail: str = "", tip: str = "", progress: Optional[float] = None) -> None:
    """progress (0-1): the badge is its own progress bar (user, 11 November 2026): the base colour up to the share
    done, a darker shade of it beyond."""
    text, colour = SOURCE_STYLES.get(kind, SOURCE_STYLES["estimate"])
    full = f" {text}{' \u00b7 ' + detail if detail else ''} "
    ew = lb.property("elide_w")
    if ew:                                             # (fixed width: cut, the whole text in the tooltip)
        f = QFont(lb.font())
        f.setBold(True)
        shown = QFontMetrics(f).elidedText(full, Qt.ElideRight, int(ew))
        lb.setText(shown)
        if shown != full:
            tip = (full.strip() + ("\n" + tip if tip else ""))
    else:
        lb.setText(full)
    bg = colour
    if progress is not None:
        f = min(max(float(progress), 0.0), 1.0)
        dark = QColor(colour).darker(190).name()
        if f <= 0.0:
            bg = dark
        elif f < 1.0:
            bg = (f"qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 {colour}, stop:{f:.4f} {colour}, "
                  f"stop:{min(f + 0.001, 1.0):.4f} {dark}, stop:1 {dark})")
    lb.setStyleSheet(f"background: {bg}; color: white; font-weight: bold; border-radius: 3px; padding: 1px 4px;")
    lb.setToolTip(tip)

