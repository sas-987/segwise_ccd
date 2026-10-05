import math

class Pose2D:
    def __init__(self, x: float, y: float, theta: float = 0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def __repr__(self):
        return f"Pose2D(x={self.x:.3f}, y={self.y:.3f}, theta={math.degrees(self.theta):.1f}°)"


def wrap_to_pi(angle: float) -> float:
    """Wraps an angle into [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def update_orientations_2d(p_prev: Pose2D, p_curr: Pose2D, p_next: Pose2D) -> Pose2D:
    """
    Local Kinematic Feasibility Adjustment (Section III-E).
    Only updates and returns p_curr to avoid altering already-certified poses.
    """
    # 1. Displacement vectors
    dx1 = p_curr.x - p_prev.x
    dy1 = p_curr.y - p_prev.y
    dx2 = p_next.x - p_curr.x
    dy2 = p_next.y - p_curr.y

    # 2. Chord / travel directions
    alpha1 = math.atan2(dy1, dx1)
    alpha2 = math.atan2(dy2, dx2)

    # 3. Smooth circular arc bisector angle for the middle pose
    sin_sum = math.sin(alpha1) + math.sin(alpha2)
    cos_sum = math.cos(alpha1) + math.cos(alpha2)

    if abs(sin_sum) > 1e-9 or abs(cos_sum) > 1e-9:
        p_curr.theta = math.atan2(sin_sum, cos_sum)
    else:
        p_curr.theta = alpha1

    # Keep p_prev and p_next untouched so existing waypoints stay stable!
    return p_curr