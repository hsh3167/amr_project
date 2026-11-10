"""Compatibility entry; build123d is the default CAD engine.
Frozen CadQuery source: amr_layout_cadquery.py.
"""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from cad.__main__ import main
if __name__=='__main__':
    if len(sys.argv)==1:sys.argv.append('build')
    main()
