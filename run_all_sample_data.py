"""
Runner script to evaluate Segment-wise CCD (seg_ccd (1).py) and Orientation Smoothing
on the synthetic dataset from https://github.com/MahienSanklechaa/sample-data
"""

import os
import sys
import json
import time
import importlib.util
import numpy as np
import pandas as pd

# Add sample-data directory to python path for mock_sdf import
SAMPLE_DATA_DIR = os.path.join(os.path.dirname(__file__), "sample-data")
sys.path.append(SAMPLE_DATA_DIR)
from mock_sdf import GridSDF

# Load seg_ccd (1).py dynamically
spec = importlib.util.spec_from_file_location("seg_ccd", os.path.join(os.path.dirname(__file__), "seg_ccd (1).py"))
seg_ccd = importlib.util.module_from_spec(spec)
spec.loader.exec_module(seg_ccd)

Pose = seg_ccd.Pose
SegmentWiseCCD = seg_ccd.SegmentWiseCCD
yaw_to_quaternion = seg_ccd.yaw_to_quaternion
update_orientations_se3 = seg_ccd.update_orientations_se3
default_motion_bound = seg_ccd.default_motion_bound


def quaternion_to_yaw(q: np.ndarray) -> float:
    """Extracts yaw (in radians [-pi, pi]) from unit quaternion [w, x, y, z]."""
    w, x, y, z = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


def evaluate_scenarios(robot_radius: float = 0.15):
    manifest_path = os.path.join(SAMPLE_DATA_DIR, "manifest.json")
    with open(manifest_path, "r") as f:
        manifest = json.load(f)

    print("=" * 80)
    print(f"RUNNING SEGMENT-WISE CCD ON BENCHMARK SCENARIOS (Robot Radius: {robot_radius}m)")
    print("=" * 80)

    results = []

    for name, desc in manifest["scenarios"]:
        map_path = os.path.join(SAMPLE_DATA_DIR, "maps", f"{name}_sdf.npz")
        traj_path = os.path.join(SAMPLE_DATA_DIR, "trajectories", f"{name}_trajectory.csv")

        if not os.path.exists(map_path) or not os.path.exists(traj_path):
            print(f"[-] Missing files for scenario: {name}")
            continue

        sdf = GridSDF.from_npz(map_path)
        df = pd.read_csv(traj_path)

        chi_init = [
            Pose(
                position=np.array([row.x, row.y, 0.0]),
                orientation=yaw_to_quaternion(row.theta),
            )
            for _, row in df.iterrows()
        ]
        tau_init = [df.iloc[i + 1].t - df.iloc[i].t for i in range(len(df) - 1)]

        # Custom SDF gradient pose correction for high precision
        def sdf_correction(pose: Pose) -> Pose:
            x, y = pose.position[0], pose.position[1]
            d0 = sdf.distance(x, y)
            grad2d = sdf.gradient(x, y)
            step = max(robot_radius / 2.0 - d0, 0.0)
            new_pos = np.array([x + grad2d[0] * step, y + grad2d[1] * step, 0.0])
            return Pose(new_pos, pose.orientation)

        clearance_fn = lambda p: sdf.distance(p.position[0], p.position[1])

        ccd = SegmentWiseCCD(
            clearance_fn=clearance_fn,
            radius=robot_radius,
            correction_fn=sdf_correction,
            orientation_update_fn=update_orientations_se3,
        )

        t_start = time.perf_counter()
        chi_star, tau_star = ccd.run(chi_init, tau_init)
        elapsed_ms = (time.perf_counter() - t_start) * 1000.0

        # Safety & certification validation
        min_clearance_init = min(clearance_fn(p) for p in chi_init)
        min_clearance_refined = min(clearance_fn(p) for p in chi_star)

        # Check certification inequality L < d_i + d_j on all final segments
        all_certified = True
        max_violation = 0.0
        for i in range(len(chi_star) - 1):
            p_a, p_b = chi_star[i], chi_star[i + 1]
            L = default_motion_bound(p_a, p_b, robot_radius)
            d_sum = clearance_fn(p_a) + clearance_fn(p_b)
            if L >= d_sum:
                all_certified = False
                max_violation = max(max_violation, L - d_sum)

        print(f"\nScenario: {name}")
        print(f"  Description:         {desc}")
        print(f"  Initial Waypoints:   {len(chi_init)} (Duration: {sum(tau_init):.2f}s)")
        print(f"  Refined Waypoints:   {len(chi_star)} (Duration: {sum(tau_star):.2f}s)")
        print(f"  Runtime:             {elapsed_ms:.2f} ms")
        print(f"  Min Clearance:       Initial = {min_clearance_init:+.4f}m  -->  Refined = {min_clearance_refined:+.4f}m")
        print(f"  All Segments Safe:   {'YES [Certified]' if all_certified else f'NO [Max violation {max_violation:.4f}m]'}")
        if ccd.log:
            print(f"  Warnings ({len(ccd.log)}):")
            for w in ccd.log:
                print(f"    * {w}")

        results.append({
            "scenario": name,
            "desc": desc,
            "init_pts": len(chi_init),
            "refined_pts": len(chi_star),
            "all_certified": all_certified,
            "min_clearance_init": min_clearance_init,
            "min_clearance_refined": min_clearance_refined,
            "runtime_ms": elapsed_ms,
            "warnings": len(ccd.log),
        })

    return results


def evaluate_orientation_cases():
    print("\n" + "=" * 80)
    print("RUNNING ORIENTATION SMOOTHING CASES")
    print("=" * 80)

    cases = ["straight", "ninety_degree_turn", "zigzag", "angle_wrap"]
    for case in cases:
        csv_file = os.path.join(SAMPLE_DATA_DIR, "trajectories", f"orientation_{case}.csv")
        if not os.path.exists(csv_file):
            continue

        df = pd.read_csv(csv_file)
        print(f"\nCase: orientation_{case}")
        print(f"  Original waypoints: {len(df)}")
        for i, row in df.iterrows():
            print(f"    p{i}: x={row.x:6.3f}, y={row.y:6.3f}, theta={row.theta:7.3f} rad ({np.degrees(row.theta):6.1f} deg)")

        # Test bisection with update_orientations_se3
        poses = [
            Pose(np.array([r.x, r.y, 0.0]), yaw_to_quaternion(r.theta))
            for _, r in df.iterrows()
        ]

        print("  Testing arc tangent orientation on synthetic midpoints:")
        for i in range(len(poses) - 1):
            p_i, p_j = poses[i], poses[i + 1]
            p_mid = Pose(0.5 * (p_i.position + p_j.position), p_i.orientation)
            # Find next point for curvature context if available
            p_next = poses[i + 2] if i + 2 < len(poses) else Pose(2.0 * p_j.position - p_i.position, p_j.orientation)
            p_smoothed = update_orientations_se3(p_i, p_mid, p_next)
            yaw = quaternion_to_yaw(p_smoothed.orientation)
            print(f"    Midpoint between p{i} and p{i+1}: pos=({p_smoothed.position[0]:.2f}, {p_smoothed.position[1]:.2f}) -> smoothed theta={yaw:7.3f} rad ({np.degrees(yaw):6.1f} deg)")


if __name__ == "__main__":
    evaluate_scenarios()
    evaluate_orientation_cases()
