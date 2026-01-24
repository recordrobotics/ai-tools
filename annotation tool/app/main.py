from __future__ import annotations

import sys
from pathlib import Path
from typing import List

from PySide6.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QPushButton,
    QFileDialog,
    QLabel,
)

from app.utils.images import list_images
from app.io.annotations import load_annotations
from app.io.labels import load_labels, save_labels
from app.ui.grid_view import GridView
from app.ui.image_viewer import ImageViewer


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Image Annotation Tool")
        self.resize(1000, 700)
        layout = QVBoxLayout(self)

        self.btn_open = QPushButton("Choose folder")
        self.btn_open.clicked.connect(self.choose_folder)
        layout.addWidget(self.btn_open)
        self.info = QLabel("No folder selected")
        layout.addWidget(self.info)

        self.grid_container = QWidget()
        layout.addWidget(self.grid_container, 1)

        self.image_paths: List[Path] = []
        self.current_folder: Path | None = None
        # keep references to any open viewer windows so they are not garbage-collected
        self._viewers: List[object] = []

    def choose_folder(self) -> None:
        dlg = QFileDialog(self, "Select image folder")
        dlg.setFileMode(QFileDialog.Directory)
        if dlg.exec():
            folder = Path(dlg.selectedFiles()[0])
            self.load_folder(folder)

    def load_folder(self, folder: Path) -> None:
        self.current_folder = folder
        imgs = list_images(folder)
        # load folder-level class labels (list of Label)
        self.labels = load_labels(folder)
        annotated = set()
        for p in imgs:
            anns = load_annotations(p)
            if anns:
                annotated.add(p)
        self.image_paths = imgs
        self.info.setText(f"Folder: {folder} ({len(imgs)} images)")
        # build grid view
        grid = GridView(imgs, annotated, self.open_image)
        layout = self.layout()
        # replace grid container
        layout.removeWidget(self.grid_container)
        self.grid_container.deleteLater()
        self.grid_container = grid
        layout.addWidget(self.grid_container, 1)

    def open_image(self, path: Path) -> None:
        try:
            index = self.image_paths.index(path)
        except ValueError:
            index = 0
        viewer = ImageViewer(
            self.image_paths,
            index,
            self._on_image_updated,
            labels=list(self.labels),
            folder=self.current_folder,
            on_labels_changed=self._on_labels_changed,
        )
        # keep a strong reference so Qt doesn't destroy the window immediately
        self._viewers.append(viewer)
        # remove from list when destroyed to avoid memory leak
        try:
            viewer.destroyed.connect(
                lambda _, v=viewer: (
                    self._viewers.remove(v) if v in self._viewers else None
                )
            )
        except Exception:
            pass
        viewer.show()

    def _on_image_updated(self, path: Path) -> None:
        # Called when an image's annotations were updated; could refresh grid status
        # For simplicity, reload grid to update annotation badges
        if self.current_folder:
            self.load_folder(self.current_folder)

    def _on_labels_changed(self, labels: List) -> None:
        # save labels and refresh UI
        if not self.current_folder:
            return
        save_labels(self.current_folder, labels)
        # reload folder to reflect any label changes
        self.load_folder(self.current_folder)


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
