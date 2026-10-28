#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WIDGETS -- small Qt widgets shared across tabs.
"""
from __future__ import annotations

from PyQt5.QtCore import QLocale, QPoint, QRect, QSize, Qt
from PyQt5.QtWidgets import QDoubleSpinBox, QLayout


class DecimalSpinBox(QDoubleSpinBox):
    """QDoubleSpinBox that accepts '.' and ',' interchangeably as the decimal separator, regardless of the
    system locale -- on a French (',') locale the numeric keypad's '.' key is otherwise rejected as invalid
    input by the default, locale-aware validator."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setLocale(QLocale(QLocale.C))

    def validate(self, text: str, pos: int):
        return super().validate(text.replace(",", "."), pos)

    def valueFromText(self, text: str) -> float:
        return super().valueFromText(text.replace(",", "."))


class FlowLayout(QLayout):
    """A layout that lays its widgets left to right and wraps them to the next line when the width runs out (the
    tool bars of the 3D views: their minimum width is their widest item, not the sum, so the window fits a small
    screen, user 29 October 2026). After Qt's flow layout example."""

    def __init__(self, parent=None, margin: int = 0, spacing: int = 6):
        super().__init__(parent)
        self.setContentsMargins(margin, margin, margin, margin)
        self.setSpacing(spacing)
        self._items = []

    def addItem(self, item) -> None:
        self._items.append(item)

    def insertWidget(self, index: int, widget) -> None:
        """Put a widget at index (0: first) in the flow."""
        from PyQt5.QtWidgets import QWidgetItem
        self.addChildWidget(widget)
        self._items.insert(index, QWidgetItem(widget))
        self.invalidate()

    def addStretch(self, _stretch: int = 0) -> None:
        """Kept for the QHBoxLayout call sites: a flow has no stretch."""

    def addSpacing(self, _size: int) -> None:
        """Kept for the QHBoxLayout call sites: the flow's spacing separates the items."""

    def count(self) -> int:
        return len(self._items)

    def itemAt(self, i):
        return self._items[i] if 0 <= i < len(self._items) else None

    def takeAt(self, i):
        return self._items.pop(i) if 0 <= i < len(self._items) else None

    def expandingDirections(self):
        return Qt.Orientations(0)

    def hasHeightForWidth(self) -> bool:
        return True

    def heightForWidth(self, width: int) -> int:
        return self._lay(QRect(0, 0, width, 0), True)

    def setGeometry(self, rect) -> None:
        super().setGeometry(rect)
        self._lay(rect, False)

    def sizeHint(self):
        return self.minimumSize()

    def minimumSize(self):
        size = QSize()
        for it in self._items:
            size = size.expandedTo(it.minimumSize())
        m = self.contentsMargins()
        return size + QSize(m.left() + m.right(), m.top() + m.bottom())

    def _lay(self, rect, test: bool) -> int:
        m = self.contentsMargins()
        r = rect.adjusted(m.left(), m.top(), -m.right(), -m.bottom())
        x, y, line = r.x(), r.y(), 0
        sp = self.spacing()
        for it in self._items:
            hint = it.sizeHint()
            nx = x + hint.width() + sp
            if nx - sp > r.right() and line > 0:
                x, y = r.x(), y + line + sp
                nx, line = x + hint.width() + sp, 0
            if not test:
                it.setGeometry(QRect(QPoint(x, y), hint))
            x, line = nx, max(line, hint.height())
        return y + line - rect.y() + m.bottom()
