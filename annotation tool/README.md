# Image Annotation Tool

This is a image annotation GUI app implemented in Python using PySide6.

Features:

- Choose a folder containing images
- Grid view of images with thumbnail, name, extension, and annotation status
- Click a card to open a full image viewer
- Left sidebar lists annotations for the current image (edit/delete)
- Click-and-drag on the image to create a bounding box
- Navigate previous/next images from viewer
- Annotations read/written in YOLO format (class_id x_center y_center width height, normalized) alongside images (.txt)

Quick start

1. Create a Python venv and install requirements:

```pwsh
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r "requirements.txt"
```

or on macOS and Linux

```bash
python -m venv .venv
. .venv/bin/activate
pip install -r "requirements.txt"
```

2. Run the app:

```pwsh
python -m app.main
```

Notes

- This tool uses the standard YOLO single-line format per object: `class_id x_center y_center width height` with coordinates normalized to [0,1].
- Default class id used when creating new boxes is 0. You can edit class ids from the left sidebar.

Project structure
- `app/` - package containing app logic
- `requirements.txt` - runtime requirements
- `README.md` - this file
