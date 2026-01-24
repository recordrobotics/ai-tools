from __future__ import annotations

from pathlib import Path
from typing import List, Tuple
from dataclasses import dataclass

from app.utils.images import list_images
from app.io.annotations import load_annotations, save_annotations


LABELS_FILENAME = "labels.txt"


@dataclass
class Label:
    name: str
    color: str | None = None  # hex like #RRGGBB


def labels_path(folder: Path) -> Path:
    return folder / LABELS_FILENAME


def _ensure_color(hex_color: str | None, index: int) -> str:
    """Return a hex color string; if hex_color is None, generate a default based on index."""
    if hex_color:
        return hex_color
    # simple deterministic color pick using golden angle
    hue = int((index * 137.5) % 360)
    return f"#{((hue * 1234567) & 0xFFFFFF):06x}"


def load_labels(folder: Path) -> List[Label]:
    p = labels_path(folder)
    if not p.exists():
        return [Label("class_0", None)]
    raw = [l.strip() for l in p.read_text(encoding="utf-8").splitlines() if l.strip()]
    labels: List[Label] = []
    for i, line in enumerate(raw):
        if "|" in line:
            name, col = line.split("|", 1)
            labels.append(Label(name, col if col else None))
        else:
            labels.append(Label(line, None))
    # ensure colors exist (so UI can render); do not overwrite existing color strings
    for i, lbl in enumerate(labels):
        if not lbl.color:
            lbl.color = _ensure_color(None, i)
    return labels


def save_labels(folder: Path, labels: List[Label]) -> None:
    p = labels_path(folder)
    lines: List[str] = []
    for lbl in labels:
        if lbl.color:
            lines.append(f"{lbl.name}|{lbl.color}")
        else:
            lines.append(lbl.name)
    p.write_text("\n".join(lines), encoding="utf-8")


def remove_label_and_reindex(folder: Path, index: int) -> List[Label]:
    """Remove label at index and reindex annotations across the folder.

    Any annotation with class_id == index will be removed. Any with class_id > index
    will be decremented by 1. Returns the new labels list.
    """
    labels = load_labels(folder)
    if index < 0 or index >= len(labels):
        return labels
    labels.pop(index)
    # reindex annotations for all images
    for img in list_images(folder):
        anns = load_annotations(img)
        changed = False
        new_anns = []
        for a in anns:
            if a.class_id == index:
                changed = True
                continue
            if a.class_id > index:
                a.class_id -= 1
                changed = True
            new_anns.append(a)
        if changed:
            save_annotations(img, new_anns)
    save_labels(folder, labels)
    return labels
