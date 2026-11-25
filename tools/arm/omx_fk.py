"""Moved to src/arm_control; this keeps `python3 tools/arm/omx_fk.py` and `import omx_fk` working."""
import sys

from arm_control import omx_fk as _m

if __name__ == "__main__":
    getattr(_m, "main", lambda: None)()
else:
    sys.modules[__name__] = _m
