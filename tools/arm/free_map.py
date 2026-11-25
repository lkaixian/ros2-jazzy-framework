"""Moved to src/arm_control; this keeps `python3 tools/arm/free_map.py` and `import free_map` working."""
import sys

from arm_control import free_map as _m

if __name__ == "__main__":
    getattr(_m, "main", lambda: None)()
else:
    sys.modules[__name__] = _m
