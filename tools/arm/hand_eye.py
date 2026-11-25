"""Moved to src/arm_control; this keeps `python3 tools/arm/hand_eye.py` and `import hand_eye` working."""
import sys

from arm_control import hand_eye as _m

if __name__ == "__main__":
    getattr(_m, "main", lambda: None)()
else:
    sys.modules[__name__] = _m
