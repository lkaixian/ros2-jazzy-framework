"""Moved to src/arm_control; this keeps `python3 tools/arm/path_check.py` and `import path_check` working."""
import sys

from arm_control import path_check as _m

if __name__ == "__main__":
    getattr(_m, "main", lambda: None)()
else:
    sys.modules[__name__] = _m
