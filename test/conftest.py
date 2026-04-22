import importlib.util
from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if importlib.util.find_spec("go2w_real") is None and str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
