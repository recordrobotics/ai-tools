from __future__ import annotations

from pathlib import Path
from typing import Callable, List

from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QLabel,
    QScrollArea,
    QGridLayout,
    QPushButton,
    QFrame,
)

# thumbnail generation handled by Qt/QPixmap; Pillow helper not required here


class ImageCard(QFrame):
    def __init__(self, path: Path, annotated: bool, on_open: Callable[[Path], None]):
        super().__init__()
        self.path = path
        self.on_open = on_open
        self.setFrameShape(QFrame.StyledPanel)
        self.setObjectName("imageCard")
        layout = QVBoxLayout(self)
        self.thumb = QLabel(alignment=Qt.AlignCenter)
        self.thumb.setFixedSize(QSize(240, 160))
        layout.addWidget(self.thumb)
        name = QLabel(f"{path.stem}")
        ext = QLabel(f"{path.suffix.lower().lstrip('.')}")
        status = QLabel("Annotated" if annotated else "Not annotated")
        layout.addWidget(name)
        layout.addWidget(ext)
        layout.addWidget(status)
        btn = QPushButton("Open")
        btn.clicked.connect(self._open)
        layout.addWidget(btn)
        # Use Qt QPixmap loading for robustness; scale into thumbnail box
        qpix = QPixmap(str(path))
        if qpix.isNull():
            # fallback to small blank pixmap
            qpix = QPixmap(self.thumb.size())
            qpix.fill(Qt.transparent)
        self.thumb.setPixmap(qpix.scaled(self.thumb.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def _open(self) -> None:
        self.on_open(self.path)


class GridView(QWidget):
    def __init__(self, image_paths: List[Path], annotated_set: set, on_open: Callable[[Path], None]):
        super().__init__()
        layout = QVBoxLayout(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        container = QWidget()
        grid = QGridLayout(container)
        grid.setSpacing(12)
        cols = 3
        for i, p in enumerate(image_paths):
            card = ImageCard(p, p in annotated_set, on_open)
            r = i // cols
            c = i % cols
            grid.addWidget(card, r, c)
        scroll.setWidget(container)
        layout.addWidget(scroll)
