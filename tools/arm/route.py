"""Moved to src/arm_control; this keeps `python3 tools/arm/route.py` and `import route` working."""
import sys

from arm_control import route as _m

if __name__ == "__main__":
    getattr(_m, "main", lambda: None)()
else:
    sys.modules[__name__] = _m
