import traceback
import sys
import importlib

try:
    m = importlib.import_module('PySide6.QtWidgets')
    print('PySide6.QtWidgets imported OK')
    print('module file:', getattr(m, '__file__', None))
except Exception:
    traceback.print_exc()
    sys.exit(1)
