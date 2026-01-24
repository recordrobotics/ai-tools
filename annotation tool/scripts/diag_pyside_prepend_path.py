import os,sys,traceback,pathlib
p = str(pathlib.Path(r"C:/Users/dbark/miniconda3/Lib/site-packages/PySide6"))
print('prepended to PATH:', p)
os.environ['PATH'] = p + os.pathsep + os.environ.get('PATH','')
try:
    from PySide6 import QtWidgets
    print('PySide6.QtWidgets imported OK after prepending', p)
    print('module file:', getattr(QtWidgets, '__file__', None))
except Exception:
    traceback.print_exc()
    sys.exit(1)
