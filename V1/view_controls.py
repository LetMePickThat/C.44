#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VIEW_CONTROLS -- display bar shared by the mission-prep and results 3D views: one check box per face to show/hide its
responsibility zone, buttons to show/hide every mission watch sector at once (each sector also has its own check
box in the window's sector list/table), and a "Scale 1:1" box drawing the watch volumes at their true size in km.
"""
from __future__ import annotations

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QCheckBox, QHBoxLayout, QLabel, QPushButton, QWidget

import geometry as geo


class DisplayBar(QWidget):
    faces_changed = pyqtSignal()
    show_all_sectors = pyqtSignal()
    hide_all_sectors = pyqtSignal()
    scale_changed = pyqtSignal(bool)

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
        h.addSpacing(16)
        self.cb_true_scale = QCheckBox("Scale 1:1")
        self.cb_true_scale.setToolTip("Draw each watch volume at its true size (1 unit = 1 km), from the radar out to "
                                      "its required range, and zoom to fit them all. Untick to return to the "
                                      "default view of the unit sphere.")
        self.cb_true_scale.toggled.connect(self.scale_changed)
        h.addWidget(self.cb_true_scale)
        h.addStretch(1)

    def face_visible(self, k: int) -> bool:
        return self.cb_faces[k].isChecked()

    def true_scale(self) -> bool:
        return self.cb_true_scale.isChecked()
