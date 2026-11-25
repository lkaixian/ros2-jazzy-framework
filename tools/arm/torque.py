"""Moved to src/arm_control; this keeps `import torque` working."""
import sys

from arm_control import torque as _m

if __name__ == "__main__":
    getattr(_m, "main", lambda: None)()
else:
    sys.modules[__name__] = _m
