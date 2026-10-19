#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WIDGETS -- small Qt widgets shared across tabs.
"""
from __future__ import annotations

from PyQt5.QtCore import QLocale
from PyQt5.QtWidgets import QDoubleSpinBox


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
