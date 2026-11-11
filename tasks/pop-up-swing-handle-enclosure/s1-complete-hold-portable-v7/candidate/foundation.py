"""Explicit foundation entry; actual physics only in an allocated root-owned worker."""
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from run import main
if __name__=="__main__":raise SystemExit(main('foundation'))
