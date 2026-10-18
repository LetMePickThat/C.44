#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VIEW_CONTROLS -- display bar shared by the mission-prep and results 3D views: one check box per face to show/hide its
responsibility zone, and buttons to show/hide every mission watch sector at once (each sector also has its own check
box in the window's sector list/table).
"""
from __future__ import annotations

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QCheckBox, QHBoxLayout, QLabel, QPushButton, QWidget

import geometry as geo


class DisplayBar(QWidget):
    faces_changed = pyqtSignal()
    show_all_sectors = pyqtSignal()
    hide_all_sectors = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        h = QHBoxLayout(self)
        h.setContentsMargins(4, 2, 4, 2)
        h.addWidget(QLabel("Responsibility zones:"))
        self.cb_faces = []
        for k in range(4):
            cb = QCheckBox(geo.FACE_NAMES[k])
            cb.setChecked(True)
            cb.setStyleSheet(f"color: {geo.FACE_COLORS[k]}; font-weight: bold;")
            cb.toggled.connect(self.faces_changed)
            h.addWidget(cb)
            self.cb_faces.append(cb)
        h.addSpacing(16)
        h.addWidget(QLabel("Mission volumes:"))
        b_all, b_none = QPushButton("Show all"), QPushButton("Hide all")
        b_all.clicked.connect(self.show_all_sectors)
        b_none.clicked.connect(self.hide_all_sectors)
        h.addWidget(b_all)
        h.addWidget(b_none)
        h.addStretch(1)

    def face_visible(self, k: int) -> bool:
        return self.cb_faces[k].isChecked()
