from __future__ import annotations

from pathlib import Path
from typing import List
from PIL import Image


SUPPORTED_EXT = {".jpg", ".jpeg", ".png", ".bmp", ".gif", ".tiff", ".webp"}


def list_images(folder: Path) -> List[Path]:
    if not folder.exists():
        return []
    files = [p for p in folder.iterdir() if p.suffix.lower() in SUPPORTED_EXT and p.is_file()]
    files.sort()
    return files


def make_thumbnail(image_path: Path, size=(320, 240)) -> Image.Image:
    img = Image.open(image_path)
    img.thumbnail(size, Image.LANCZOS)
    return img.convert("RGBA")
