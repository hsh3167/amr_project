"""FreeCADCmd executes Python scripts; GUI users may run ImportAMR.FCMacro."""
from pathlib import Path
macro=Path(__file__).with_name('ImportAMR.FCMacro')
exec(compile(macro.read_text(encoding='utf-8'),str(macro),'exec'),{'__file__':str(macro),'__name__':'__main__'})
