from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List


@dataclass
class Annotation:
    class_id: int
    x_center: float  # normalized [0,1]
    y_center: float
    width: float
    height: float


def annotation_txt_path(image_path: Path) -> Path:
    return image_path.with_suffix(".txt")


def load_annotations(image_path: Path) -> List[Annotation]:
    txt = annotation_txt_path(image_path)
    anns: List[Annotation] = []
    if not txt.exists():
        return anns
    for line in txt.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 5:
            continue
        try:
            class_id = int(parts[0])
            vals = [float(x) for x in parts[1:5]]
            anns.append(Annotation(class_id, *vals))
        except Exception:
            # skip malformed line
            continue
    return anns


def save_annotations(image_path: Path, anns: List[Annotation]) -> None:
    txt = annotation_txt_path(image_path)
    lines: List[str] = []
    for a in anns:
        lines.append(
            f"{a.class_id} {a.x_center:.6f} {a.y_center:.6f} {a.width:.6f} {a.height:.6f}"
        )
    txt.write_text("\n".join(lines), encoding="utf-8")
