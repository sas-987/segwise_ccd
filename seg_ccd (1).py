"""
Segment-wise Continuous Collision Detection (CCD) with Orientation Smoothing
=============================================================================

A Python implementation of Algorithm 1 "Segment-wise CCD" with Local Kinematic
Feasibility Adjustment (Orientation Smoothing): given a discrete trajectory of
collision-free poses, recursively bisect segments whose conservative motion
bound L(p_i, p_{i+1}) cannot be proven collision-free from pointwise clearances
alone, until every sub-segment is certified collision-free.
"""

from __future__ import annotations

import heapq
import itertools
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Basic geometric types
# ---------------------------------------------------------------------------

@dataclass
class Pose:
    """A single SE(3) pose: 3D position + unit quaternion orientation (w, x, y, z)."""

    position: np.ndarray
    orientation: np.ndarray

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=float).reshape(3)
        q = np.asarray(self.orientation, dtype=float).reshape(4)
        n = np.linalg.norm(q)
        self.orientation = q / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])

    def __repr__(self):
        p = np.array2string(self.position, precision=3)
        q = np.array2string(self.orientation, precision=3)
        return f"Pose(pos={p}, quat={q})"


def quat_angle(q1: np.ndarray, q2: np.ndarray) -> float:
    """Angle (rad) of the rotation that takes orientation q1 to q2."""
    dot = float(np.clip(abs(np.dot(q1, q2)), -1.0, 1.0))
    return 2.0 * np.arccos(dot)


def slerp(q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between two unit quaternions."""
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    dot = float(np.dot(q1, q2))
    if dot < 0.0:            # take the shorter arc
        q2 = -q2
        dot = -dot
    dot = np.clip(dot, -1.0, 1.0)
    if dot > 0.9995:         # nearly identical: fall back to linear + renormalize
        out = q1 + t * (q2 - q1)
        return out / np.linalg.norm(out)
    theta0 = np.arccos(dot)
    theta = theta0 * t
    q_perp = q2 - q1 * dot
    q_perp = q_perp / np.linalg.norm(q_perp)
    return q1 * np.cos(theta) + q_perp * np.sin(theta)


def bisect_pose(p_i: Pose, p_j: Pose) -> Pose:
    """Bisect(S_i): geometric midpoint pose (line 10)."""
    mid_pos = 0.5 * (p_i.position + p_j.position)
    mid_quat = slerp(p_i.orientation, p_j.orientation, 0.5)
    return Pose(mid_pos, mid_quat)


def default_motion_bound(p_i: Pose, p_j: Pose, radius: float) -> float:
    """
    L(p_i, p_j) -- 'Eq. 2': a conservative upper bound on how far any point
    of a rigid body of bounding radius `r` can travel while the body moves
    from pose p_i to p_j (translation + worst-case rotational sweep):

        L = ||t_j - t_i|| + r * angle(q_i, q_j)
    """
    dt = float(np.linalg.norm(p_j.position - p_i.position))
    dtheta = quat_angle(p_i.orientation, p_j.orientation)
    return dt + radius * dtheta


# ---------------------------------------------------------------------------
# Orientation Smoothing / Local Kinematic Feasibility Adjustment (Section III-E)
# ---------------------------------------------------------------------------

def yaw_to_quaternion(yaw: float) -> np.ndarray:
    """Converts a planar yaw angle (radians) to a unit quaternion [w, 0, 0, z]."""
    half_yaw = 0.5 * yaw
    return np.array([np.cos(half_yaw), 0.0, 0.0, np.sin(half_yaw)], dtype=float)


def update_orientations_se3(p_prev: Pose, p_curr: Pose, p_next: Pose) -> Pose:
    """
    Local Kinematic Feasibility Adjustment (Paper Line 13 & Section III-E).
    
    Fits consecutive waypoints to a common circular arc of constant curvature,
    ensuring non-holonomic mobile robots can physically execute the turn.
    """
    # 1. Extract 2D coordinates from 3D positions
    x_prev, y_prev = p_prev.position[0], p_prev.position[1]
    x_curr, y_curr = p_curr.position[0], p_curr.position[1]
    x_next, y_next = p_next.position[0], p_next.position[1]

    # 2. Compute chord angles (incoming and outgoing directions)
    alpha1 = np.arctan2(y_curr - y_prev, x_curr - x_prev)
    alpha2 = np.arctan2(y_next - y_curr, x_next - x_curr)

    # 3. Vector averaging (smooth arc bisector angle)
    sin_sum = np.sin(alpha1) + np.sin(alpha2)
    cos_sum = np.cos(alpha1) + np.cos(alpha2)

    if abs(sin_sum) > 1e-9 or abs(cos_sum) > 1e-9:
        theta_mid = np.arctan2(sin_sum, cos_sum)
    else:
        theta_mid = alpha1

    # 4. Update midpoint orientation as a valid unit quaternion
    p_curr.orientation = yaw_to_quaternion(theta_mid)
    return p_curr


# ---------------------------------------------------------------------------
# Linked-list node: keeps the pose sequence in order without list shifts
# ---------------------------------------------------------------------------

class _Node:
    __slots__ = ("pose", "next", "dt_to_next")

    def __init__(self, pose: Pose):
        self.pose = pose
        self.next: Optional["_Node"] = None
        self.dt_to_next: Optional[float] = None


# ---------------------------------------------------------------------------
# Main algorithm
# ---------------------------------------------------------------------------

class SegmentWiseCCD:
    """
    Implementation of Algorithm 1: Segment-wise CCD.
    """

    def __init__(
        self,
        clearance_fn: Callable[[Pose], float],
        radius: float,
        motion_bound_fn: Callable[[Pose, Pose, float], float] = default_motion_bound,
        correction_fn: Optional[Callable[[Pose], Pose]] = None,
        orientation_update_fn: Optional[Callable[[Pose, Pose, Pose], Pose]] = None,
        min_segment_length: Optional[float] = None,
        size_margin: float = 1.0,
        max_depth: int = 40,
        min_dt: float = 1e-9,
    ):
        self.d = clearance_fn
        self.r = radius
        self.L = motion_bound_fn
        self.correction_fn = correction_fn or self._default_pose_correction
        # Uses update_orientations_se3 as default if none is supplied
        self.orientation_update_fn = orientation_update_fn or update_orientations_se3
        self.min_segment_length = (
            min_segment_length if min_segment_length is not None else size_margin * radius
        )
        self.max_depth = max_depth
        self.min_dt = min_dt
        self._log: List[str] = []

    def _default_pose_correction(self, pose: Pose, eps: float = 1e-3) -> Pose:
        """Nudges the midpoint along the clearance gradient until clearance is r/2."""
        d0 = self.d(pose)
        grad = np.zeros(3)
        for k in range(3):
            perturbed = pose.position.copy()
            perturbed[k] += eps
            d1 = self.d(Pose(perturbed, pose.orientation))
            grad[k] = (d1 - d0) / eps
        norm = np.linalg.norm(grad)
        if norm < 1e-9:
            return pose
        step = max(self.r / 2.0 - d0, 0.0)
        new_pos = pose.position + (grad / norm) * step
        return Pose(new_pos, pose.orientation)

    def run(self, chi: List[Pose], tau: List[float]) -> Tuple[List[Pose], List[float]]:
        """Execute Algorithm 1 on an input trajectory."""
        if len(tau) != len(chi) - 1:
            raise ValueError("len(tau) must equal len(chi) - 1")

        # Build initial linked list
        nodes = [_Node(p) for p in chi]
        for k in range(len(nodes) - 1):
            nodes[k].next = nodes[k + 1]
            nodes[k].dt_to_next = tau[k]
        head = nodes[0]

        heap: list = []
        counter = itertools.count()

        def push_segment(node_i: _Node, node_j: _Node, dt: float, depth: int):
            L_i = self.L(node_i.pose, node_j.pose, self.r)
            heapq.heappush(heap, (-L_i, next(counter), node_i, node_j, dt, depth))

        for k in range(len(nodes) - 1):
            push_segment(nodes[k], nodes[k + 1], nodes[k].dt_to_next, depth=0)

        while heap:
            neg_L, _, node_i, node_j, dt_i, depth = heapq.heappop(heap)

            if node_i.next is not node_j:
                continue

            L = -neg_L
            d_i = self.d(node_i.pose)
            d_j = self.d(node_j.pose)

            # Lines 7-8: certified safe
            if L < d_i + d_j:
                continue

            segment_len = float(np.linalg.norm(node_j.pose.position - node_i.pose.position))

            # Primary Guard: Physical robot footprint
            if segment_len < self.min_segment_length:
                self._log.append(
                    f"WARNING: likely physically untraversable gap -- segment length "
                    f"({segment_len:.6g}) fell below the robot footprint guard "
                    f"({self.min_segment_length:.6g}) while still uncertified."
                )
                continue

            # Fallback Guard: Numerical limits
            if depth >= self.max_depth or dt_i <= self.min_dt:
                self._log.append(
                    f"WARNING: segment could not be certified collision-free after {depth} bisections."
                )
                continue

            # Line 10: Bisect
            p_mid = bisect_pose(node_i.pose, node_j.pose)

            # Lines 11-12: Pose Correction
            if self.d(p_mid) <= self.r / 2.0:
                p_mid = self.correction_fn(p_mid)

            # Line 13: Orientation Smoothing (Circular Arc Fit)
            p_mid = self.orientation_update_fn(node_i.pose, p_mid, node_j.pose)

            # Line 14: Recompute motion bounds with the newly smoothed orientation
            L1 = self.L(node_i.pose, p_mid, self.r)
            L2 = self.L(p_mid, node_j.pose, self.r)
            denom = max(L1 + L2, 1e-12)

            # Line 15: Split dt proportionally
            dt1 = dt_i * L1 / denom
            dt2 = dt_i * L2 / denom

            # Lines 18-19: Splice midpoint into linked list (O(1))
            node_mid = _Node(p_mid)
            node_mid.next = node_j
            node_mid.dt_to_next = dt2
            node_i.next = node_mid
            node_i.dt_to_next = dt1

            # Lines 16-17: Push new subsegments back into heap
            push_segment(node_i, node_mid, dt1, depth + 1)
            push_segment(node_mid, node_j, dt2, depth + 1)

        # Flatten final chain into (chi*, tau*)
        chi_star: List[Pose] = []
        tau_star: List[float] = []
        node = head
        while node is not None:
            chi_star.append(node.pose)
            if node.next is not None:
                tau_star.append(node.dt_to_next)
            node = node.next

        return chi_star, tau_star

    @property
    def log(self) -> List[str]:
        return self._log


# ---------------------------------------------------------------------------
# Example usage
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    obstacle_center = np.array([1.0, 0.0, 0.0])
    obstacle_radius = 0.4

    def my_clearance(pose: Pose) -> float:
        """Distance from pose to nearest obstacle surface."""
        return float(np.linalg.norm(pose.position - obstacle_center) - obstacle_radius)

    identity_quat = np.array([1.0, 0.0, 0.0, 0.0])
    chi0 = [
        Pose([-1.0, 0.05, 0.0], identity_quat),
        Pose([3.0, 0.05, 0.0], identity_quat),
    ]
    tau0 = [4.0]

    robot_radius = 0.05

    # Initializing SegmentWiseCCD with orientation smoothing wired in
    ccd = SegmentWiseCCD(
        clearance_fn=my_clearance,
        radius=robot_radius,
        orientation_update_fn=update_orientations_se3
    )

    chi_star, tau_star = ccd.run(chi0, tau0)

    print("=== Refined Trajectory with Orientation Smoothing ===")
    print(f"Waypoints generated: {len(chi_star)}")
    for i, p in enumerate(chi_star):
        print(f"  p{i}: pos={p.position}, quat={p.orientation}, clearance={my_clearance(p):.4f}")