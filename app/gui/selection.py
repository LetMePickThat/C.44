#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SHARED SELECTION of the mission tool (step 5.5, point E1, user 13 November 2026, Q1-Q5): the rule of the Scenario
Editor (8 November 2026) "a click on an element selects it in every list where it is" in the mission preparation, the
Results and the Profile. One selection for the whole window (Q3): a watch volume (kind "sector", its id) or a face
(kind "face", "0".."3"), or nothing (Q1: the volumes and the faces). Every view and list that shows it follows
`changed` and sets it on a click; setting the same selection again emits nothing (no echo loops).
"""
from __future__ import annotations

from typing import Optional, Tuple

from PyQt5.QtCore import QObject, pyqtSignal

SECTOR, FACE = "sector", "face"


class Selection(QObject):
    changed = pyqtSignal(str, str)                 # (kind "sector" | "face" | "", id)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.kind: str = ""
        self.id: str = ""

    def set(self, kind: Optional[str], ident=None) -> None:
        k, i = (kind or ""), ("" if ident is None else str(ident))
        if not k:
            i = ""
        if (k, i) == (self.kind, self.id):
            return
        self.kind, self.id = k, i
        self.changed.emit(k, i)

    def get(self) -> Tuple[str, str]:
        return self.kind, self.id

    def sector(self) -> Optional[str]:
        return self.id if self.kind == SECTOR else None

    def face(self) -> Optional[int]:
        return int(self.id) if self.kind == FACE and self.id.isdigit() else None
