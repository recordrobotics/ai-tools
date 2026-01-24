import os,sys,traceback,pathlib
p = pathlib.Path(r"C:\Users\dbark\miniconda3\Lib\site-packages\PySide6")
print('adding dll dir', p)
try:
    os.add_dll_directory(str(p))
except Exception as e:
    print('add_dll_directory failed:', e)
try:
    from PySide6 import QtWidgets
    print('PySide6.QtWidgets imported OK after adding', p)
    print('module file:', getattr(QtWidgets, '__file__', None))
except Exception:
    traceback.print_exc()
    sys.exit(1)
