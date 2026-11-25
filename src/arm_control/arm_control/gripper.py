"""Gripper calibration maths (no bus, no ROS). Hand-measured 2026-11-25, see param/arm_gripper.yaml."""
M_PER_TICK, M_OFFSET = 1.7627e-5, -0.0361          # driver joint units (m) = ticks * M_PER_TICK + M_OFFSET
CLOSED_TICKS, OPEN_TICKS, GAP_CLOSED_MM, GAP_OPEN_MM = 590, -930, 20.0, 72.0
SAFE_OPEN, SAFE_CLOSE = OPEN_TICKS + 80, CLOSED_TICKS + 30   # stop short of the open stop; a little past the soft closed end so it presses


def to_m(raw):
    return raw * M_PER_TICK + M_OFFSET


def gap_mm(raw):
    """Estimated distance between the finger pads for a servo position (+-3 mm)."""
    return GAP_CLOSED_MM + (CLOSED_TICKS - raw) * (GAP_OPEN_MM - GAP_CLOSED_MM) / (CLOSED_TICKS - OPEN_TICKS)


def ticks_for_gap(gap):
    return round(CLOSED_TICKS - (gap - GAP_CLOSED_MM) * (CLOSED_TICKS - OPEN_TICKS) / (GAP_OPEN_MM - GAP_CLOSED_MM))


def joint_m_for_gap(gap):
    """GripperCommand position (driver joint metres) for a finger gap in mm, kept inside the safe ticks."""
    return to_m(max(min(ticks_for_gap(gap), SAFE_CLOSE), SAFE_OPEN))
