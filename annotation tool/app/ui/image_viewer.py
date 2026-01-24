from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Callable

from PySide6.QtCore import Qt, QRectF, QPointF
from PySide6.QtGui import QPixmap, QPen, QColor, QPainter, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QLabel,
    QColorDialog,
    QGraphicsView,
    QGraphicsScene,
    QGraphicsRectItem,
    QInputDialog,
)

from app.io.annotations import Annotation, load_annotations, save_annotations
from app.io.labels import save_labels, remove_label_and_reindex, Label
from app.utils.colors import color_for_index


@dataclass
class BoxItem:
    rect_item: QGraphicsRectItem
    annotation: Annotation


class ImageViewer(QWidget):
    def __init__(
        self,
        image_paths: List[Path],
        index: int,
        on_update: Callable[[Path], None],
        labels: List[Label] | None = None,
        folder: Path | None = None,
        on_labels_changed: Callable[[List[Label]], None] | None = None,
    ):
        super().__init__()
        self.image_paths = image_paths
        self.index = index
        self.on_update = on_update
        self.labels = list(labels or [Label("class_0", None)])
        self.folder = folder
        self.on_labels_changed = on_labels_changed
        self.current_class_index = 0
        self.setWindowTitle("Image Viewer")
        self.resize(1200, 800)

        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)
        # enable antialiasing and smooth pixmap transform for nicer rendering
        self.view.setRenderHints(
            self.view.renderHints()
            | QPainter.Antialiasing
            | QPainter.SmoothPixmapTransform
        )
        # always show the whole image (no scrollbars) and allow fitting on resize
        self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.view.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.view.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)

        # Labels (class palette) and annotations list
        self.labels_widget = QListWidget()
        self._refresh_labels_widget()
        self.labels_widget.setCurrentRow(self.current_class_index)
        self.labels_widget.currentRowChanged.connect(self._on_label_selected)

        add_lbl_btn = QPushButton("Add class")
        add_lbl_btn.clicked.connect(self.add_label)
        rename_lbl_btn = QPushButton("Rename class")
        rename_lbl_btn.clicked.connect(self.rename_label)
        remove_lbl_btn = QPushButton("Remove class")
        remove_lbl_btn.clicked.connect(self.remove_label)
        edit_color_btn = QPushButton("Edit color")
        edit_color_btn.clicked.connect(self.edit_label_color)

        self.list_widget = QListWidget()

        left_layout = QVBoxLayout()
        left_layout.addWidget(QLabel("Classes"))
        left_layout.addWidget(self.labels_widget)
        left_layout.addWidget(add_lbl_btn)
        left_layout.addWidget(rename_lbl_btn)
        left_layout.addWidget(edit_color_btn)
        left_layout.addWidget(remove_lbl_btn)
        left_layout.addWidget(QLabel("Annotations"))
        left_layout.addWidget(self.list_widget)
        btn_del = QPushButton("Delete selected")
        btn_del.clicked.connect(self.delete_selected)
        left_layout.addWidget(btn_del)
        btn_edit = QPushButton("Edit class")
        btn_edit.clicked.connect(self.edit_selected_class)
        left_layout.addWidget(btn_edit)

        nav_prev = QPushButton("← Prev")
        nav_prev.clicked.connect(self.prev_image)
        nav_next = QPushButton("Next →")
        nav_next.clicked.connect(self.next_image)
        left_layout.addWidget(nav_prev)
        left_layout.addWidget(nav_next)

        # Keyboard shortcuts: Enter/Return trigger Next image
        try:
            self._shortcut_next_return = QShortcut(QKeySequence(Qt.Key_Return), self)
            self._shortcut_next_return.setAutoRepeat(False)
            self._shortcut_next_return.activated.connect(self.next_image)
            self._shortcut_next_enter = QShortcut(QKeySequence(Qt.Key_Enter), self)
            self._shortcut_next_enter.setAutoRepeat(False)
            self._shortcut_next_enter.activated.connect(self.next_image)
        except Exception:
            pass

        container = QHBoxLayout(self)
        sidebar = QWidget()
        sidebar.setLayout(left_layout)
        sidebar.setFixedWidth(280)
        container.addWidget(sidebar)
        container.addWidget(self.view, 1)

        self._dragging = False
        self._start = QPointF()
        self._current_rect: QGraphicsRectItem | None = None
        self.box_items: List[BoxItem] = []

        self._load_current_image()

        # connect mouse events
        self.view.viewport().installEventFilter(self)

    def _load_current_image(self) -> None:
        self.scene.clear()
        self.box_items.clear()
        self.list_widget.clear()
        path = self.image_paths[self.index]
        pix = QPixmap(str(path))
        self.pix_item = self.scene.addPixmap(pix)
        self.scene.setSceneRect(QRectF(pix.rect()))
        # fit the image into the view (maintain aspect ratio)
        self.view.fitInView(self.pix_item, Qt.KeepAspectRatio)
        # load annotations
        anns = load_annotations(path)
        for a in anns:
            rect = self._yolo_to_rect(a, pix.width(), pix.height())
            pen = QPen(self._qcolor_for_index(a.class_id), 2)
            grect = self.scene.addRect(rect, pen)
            self.box_items.append(BoxItem(grect, a))
            label = (
                self.labels[a.class_id].name
                if 0 <= a.class_id < len(self.labels)
                else str(a.class_id)
            )
            self.list_widget.addItem(
                f"{a.class_id}:{label} {a.x_center:.3f},{a.y_center:.3f} {a.width:.3f}x{a.height:.3f}"
            )

    def _yolo_to_rect(self, a: Annotation, w: int, h: int):
        x_center = a.x_center * w
        y_center = a.y_center * h
        bw = a.width * w
        bh = a.height * h
        x = x_center - bw / 2
        y = y_center - bh / 2
        return QRectF(x, y, bw, bh)

    def _rect_to_yolo(self, rect: QRectF, w: int, h: int) -> Annotation:
        x = rect.x()
        y = rect.y()
        bw = rect.width()
        bh = rect.height()
        # clamp to image bounds so normalized coords are in [0,1]
        # allow rectangles drawn partially outside to be clipped
        x0 = max(0.0, min(x, w))
        y0 = max(0.0, min(y, h))
        bw = max(0.0, min(bw, w - x0))
        bh = max(0.0, min(bh, h - y0))
        x_c = (x0 + bw / 2) / w if w > 0 else 0.0
        y_c = (y0 + bh / 2) / h if h > 0 else 0.0
        w_n = bw / w if w > 0 else 0.0
        h_n = bh / h if h > 0 else 0.0
        # clamp final normalized values to [0,1]
        x_c = max(0.0, min(1.0, x_c))
        y_c = max(0.0, min(1.0, y_c))
        w_n = max(0.0, min(1.0, w_n))
        h_n = max(0.0, min(1.0, h_n))
        return Annotation(self.current_class_index, x_c, y_c, w_n, h_n)

    def resizeEvent(self, event) -> None:  # type: ignore[override]
        # keep the image fitted to the view when the window is resized
        super().resizeEvent(event)
        if getattr(self, "pix_item", None) is not None:
            try:
                self.view.fitInView(self.pix_item, Qt.KeepAspectRatio)
            except Exception:
                pass

    def eventFilter(self, source, event) -> bool:  # type: ignore[override]
        from PySide6.QtCore import QEvent

        if source is self.view.viewport():
            if (
                event.type() == QEvent.MouseButtonPress
                and event.button() == Qt.LeftButton
            ):
                pos = self.view.mapToScene(event.pos())
                # clamp start to image bounds so drawing cannot begin outside
                if getattr(self, "pix_item", None) is not None:
                    img_w = float(self.pix_item.pixmap().width())
                    img_h = float(self.pix_item.pixmap().height())
                    x = max(0.0, min(pos.x(), img_w))
                    y = max(0.0, min(pos.y(), img_h))
                    pos = QPointF(x, y)
                self._start = pos
                self._dragging = True
                # preview pen uses currently selected class color
                pen = QPen(self._qcolor_for_index(self.current_class_index), 2)
                self._current_rect = self.scene.addRect(QRectF(pos, pos), pen)
                return True
            if (
                event.type() == QEvent.MouseMove
                and self._dragging
                and self._current_rect
            ):
                pos = self.view.mapToScene(event.pos())
                # clamp position to image bounds so rectangle cannot grow beyond edges
                if getattr(self, "pix_item", None) is not None:
                    img_w = float(self.pix_item.pixmap().width())
                    img_h = float(self.pix_item.pixmap().height())
                    x = max(0.0, min(pos.x(), img_w))
                    y = max(0.0, min(pos.y(), img_h))
                    pos = QPointF(x, y)
                r = QRectF(self._start, pos).normalized()
                # further clamp rectangle to the scene/image rect
                r = r.intersected(self.scene.sceneRect())
                self._current_rect.setRect(r)
                return True
            if (
                event.type() == QEvent.MouseButtonRelease
                and event.button() == Qt.LeftButton
                and self._dragging
            ):
                self._dragging = False
                if not self._current_rect:
                    return True
                rect = self._current_rect.rect()
                # clamp final rect to image bounds
                rect = rect.intersected(self.scene.sceneRect())
                img_w = int(self.pix_item.pixmap().width())
                img_h = int(self.pix_item.pixmap().height())
                ann = self._rect_to_yolo(rect, img_w, img_h)
                # add to lists
                pen = QPen(self._qcolor_for_index(ann.class_id), 2)
                self._current_rect.setPen(pen)
                self.box_items.append(BoxItem(self._current_rect, ann))
                lbl = (
                    self.labels[ann.class_id].name
                    if 0 <= ann.class_id < len(self.labels)
                    else str(ann.class_id)
                )
                self.list_widget.addItem(
                    f"{ann.class_id}:{lbl} {ann.x_center:.3f},{ann.y_center:.3f} {ann.width:.3f}x{ann.height:.3f}"
                )
                # persist
                save_annotations(
                    self.image_paths[self.index], [b.annotation for b in self.box_items]
                )
                self._current_rect = None
                self.on_update(self.image_paths[self.index])
                return True
        return super().eventFilter(source, event)

    def delete_selected(self) -> None:
        row = self.list_widget.currentRow()
        if row < 0:
            return
        item = self.box_items.pop(row)
        self.scene.removeItem(item.rect_item)
        self.list_widget.takeItem(row)
        save_annotations(
            self.image_paths[self.index], [b.annotation for b in self.box_items]
        )
        self.on_update(self.image_paths[self.index])

    def edit_selected_class(self) -> None:
        row = self.list_widget.currentRow()
        if row < 0:
            return
        # pick from existing labels
        items = [f"{i}:{lbl.name}" for i, lbl in enumerate(self.labels)]
        current_id = self.box_items[row].annotation.class_id
        cur_label = (
            f"{current_id}:{self.labels[current_id].name}"
            if 0 <= current_id < len(self.labels)
            else str(current_id)
        )
        item, ok = QInputDialog.getItem(
            self,
            "Select class",
            "Class:",
            items,
            current=items.index(cur_label) if cur_label in items else 0,
            editable=False,
        )
        if ok and item:
            new_id = int(item.split(":", 1)[0])
            self.box_items[row].annotation.class_id = new_id
            lbl = self.labels[new_id] if 0 <= new_id < len(self.labels) else str(new_id)
            self.list_widget.item(row).setText(
                f"{new_id}:{lbl} {self.box_items[row].annotation.x_center:.3f},{self.box_items[row].annotation.y_center:.3f} {self.box_items[row].annotation.width:.3f}x{self.box_items[row].annotation.height:.3f}"
            )
            save_annotations(
                self.image_paths[self.index], [b.annotation for b in self.box_items]
            )
            self.on_update(self.image_paths[self.index])

    def _refresh_labels_widget(self) -> None:
        """Rebuild the labels widget with colored swatches. Preserves selection if possible."""
        sel = self.labels_widget.currentRow() if self.labels_widget else 0
        self.labels_widget.clear()
        from PySide6.QtWidgets import QWidget, QHBoxLayout, QLabel

        for i, lbl in enumerate(self.labels):
            item = QListWidgetItem()
            w = QWidget()
            h = QHBoxLayout(w)
            sw = QLabel()
            sw.setFixedSize(16, 16)
            color = (
                lbl.color if getattr(lbl, "color", None) else color_for_index(i).name()
            )
            sw.setStyleSheet(f"background-color: {color}; border: 1px solid #333;")
            text = QLabel(lbl.name)
            h.setContentsMargins(4, 2, 4, 2)
            h.addWidget(sw)
            h.addWidget(text)
            self.labels_widget.addItem(item)
            self.labels_widget.setItemWidget(item, w)
        # restore selection
        if 0 <= sel < self.labels_widget.count():
            self.labels_widget.setCurrentRow(sel)
        else:
            self.labels_widget.setCurrentRow(0)

    def _on_label_selected(self, row: int) -> None:
        if row < 0:
            return
        self.current_class_index = row

    def _qcolor_for_index(self, index: int) -> QColor:
        """Return a QColor for a label index: use stored color if present, else fallback."""
        if 0 <= index < len(self.labels):
            col = getattr(self.labels[index], "color", None)
            if col:
                try:
                    return QColor(col)
                except Exception:
                    pass
        return color_for_index(index)

    def edit_label_color(self) -> None:
        row = self.labels_widget.currentRow()
        if row < 0:
            return
        cur = self.labels[row]
        initial = (
            QColor(cur.color)
            if getattr(cur, "color", None)
            else self._qcolor_for_index(row)
        )
        col = QColorDialog.getColor(initial, self, f"Choose color for {cur.name}")
        if col.isValid():
            hexcol = col.name()
            self.labels[row].color = hexcol
            # persist
            if self.folder:
                save_labels(self.folder, self.labels)
            # refresh swatches
            self._refresh_labels_widget()
            if self.on_labels_changed:
                self.on_labels_changed(self.labels)

    # Label (class) management
    def add_label(self) -> None:
        text, ok = QInputDialog.getText(self, "Add class", "Class name:")
        if ok and text:
            # append Label with no explicit color (will be auto-filled)
            self.labels.append(Label(text, None))
            # refresh widget so we show color swatch
            self._refresh_labels_widget()
            if self.on_labels_changed:
                self.on_labels_changed(self.labels)

    def rename_label(self) -> None:
        row = self.labels_widget.currentRow()
        if row < 0:
            return
        cur = self.labels[row]
        text, ok = QInputDialog.getText(
            self, "Rename class", "New name:", text=cur.name
        )
        if ok and text:
            self.labels[row].name = text
            self._refresh_labels_widget()
            # save labels
            if self.folder:
                save_labels(self.folder, self.labels)
            if self.on_labels_changed:
                self.on_labels_changed(self.labels)

    def remove_label(self) -> None:
        row = self.labels_widget.currentRow()
        if row < 0:
            return
        if not self.folder:
            return
        # remove label and reindex annotations across folder
        new_labels = remove_label_and_reindex(self.folder, row)
        self.labels = new_labels
        # refresh labels widget
        self._refresh_labels_widget()
        # notify host to refresh UI
        if self.on_labels_changed:
            self.on_labels_changed(self.labels)
        # reload current image to reflect any annotation changes
        self._load_current_image()

    def prev_image(self) -> None:
        if self.index > 0:
            self.index -= 1
            self._load_current_image()

    def next_image(self) -> None:
        if self.index < len(self.image_paths) - 1:
            self.index += 1
            self._load_current_image()
