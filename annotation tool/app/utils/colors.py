from __future__ import annotations

from PySide6.QtGui import QColor


def color_for_index(index: int) -> QColor:
    """Return a visually distinct QColor for a given index.

    Uses a golden-angle spaced hue to distribute colors.
    """
    # Golden angle in degrees ~137.5
    hue = int((index * 137.5) % 360)
    sat = 200
    val = 220
    return QColor.fromHsv(hue, sat, val)
