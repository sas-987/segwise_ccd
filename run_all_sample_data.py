"""
Comprehensive evaluation script running all CSV and NPZ files from
https://github.com/MahienSanklechaa/sample-data on Segment-wise CCD (seg_ccd (1).py).
"""

import os
import sys
import glob
import time
import importlib.util
import numpy as np
import pandas as pd

SAMPLE_DATA_DIR = os.path.join(os.path.dirname(__file__), "sample-data")
sys.path.append(SAMPLE_DATA_DIR)
from mock_sdf import GridSDF

# Load seg_ccd (1).py
spec = importlib.util.spec_from_file_location("seg_ccd", os.path.join(os.path.dirname(__file__), "seg_ccd (1).py"))
seg_ccd = importlib.util.module_from_spec(spec)
spec.loader.exec_module(seg_ccd)

Pose = seg_ccd.Pose
SegmentWiseCCD = seg_ccd.SegmentWiseCCD
yaw_to_quaternion = seg_ccd.yaw_to_quaternion
update_orientations_se3 = seg_ccd.update_orientations_se3
default_motion_bound = seg_ccd.default_motion_bound
slerp = seg_ccd.slerp


def quat_to_yaw(q: np.ndarray) -> float:
    w, x, y, z = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


def run_all():
    npz_files = sorted(glob.glob(os.path.join(SAMPLE_DATA_DIR, "maps", "*.npz")))
    csv_files = sorted(glob.glob(os.path.join(SAMPLE_DATA_DIR, "trajectories", "*.csv")))

    print("=" * 85)
    print("INVENTORY OF DATASET FILES:")
    print(f"Total NPZ Map files:        {len(npz_files)}")
    for f in npz_files:
        print(f"  - {os.path.basename(f)}")
    print(f"Total CSV Trajectory files: {len(csv_files)}")
    for f in csv_files:
        print(f"  - {os.path.basename(f)}")
    print("=" * 85)

    robot_radius = 0.15

    # 1. Run the 6 Scenario Trajectories paired with their respective 6 NPZ SDF Maps
    print("\n" + "=" * 85)
    print("PART 1: RUNNING 6 SCENARIO TRAJECTORIES WITH MATCHING NPZ MAPS")
    print("=" * 85)

    scenario_results = []
    scenario_pairs = [
        ("01_open_space_trajectory.csv", "01_open_space_sdf.npz", "Open space (control case)"),
        ("02_endpoints_safe_middle_collision_trajectory.csv", "02_endpoints_safe_middle_collision_sdf.npz", "Safe endpoints, middle intersects obstacle"),
        ("03_narrow_passage_trajectory.csv", "03_narrow_passage_sdf.npz", "Biased path through narrow gap"),
        ("04_long_diagonal_trajectory.csv", "04_long_diagonal_sdf.npz", "Long segment through obstacle / recursive bisection"),
        ("05_multiple_obstacles_trajectory.csv", "05_multiple_obstacles_sdf.npz", "Multiple obstacles and risky zones"),
        ("06_safe_corridor_trajectory.csv", "06_safe_corridor_sdf.npz", "Safe corridor (control case)"),
    ]

    for traj_name, map_name, desc in scenario_pairs:
        traj_path = os.path.join(SAMPLE_DATA_DIR, "trajectories", traj_name)
        map_path = os.path.join(SAMPLE_DATA_DIR, "maps", map_name)

        sdf = GridSDF.from_npz(map_path)
        df = pd.read_csv(traj_path)

        chi = [Pose([r.x, r.y, 0.0], yaw_to_quaternion(r.theta)) for _, r in df.iterrows()]
        tau = [df.iloc[i + 1].t - df.iloc[i].t for i in range(len(df) - 1)]

        clearance_fn = lambda p: sdf.distance(p.position[0], p.position[1])

        def sdf_correction(p: Pose) -> Pose:
            d0 = clearance_fn(p)
            gx, gy = sdf.gradient(p.position[0], p.position[1])
            step = max(robot_radius / 2.0 - d0, 0.0)
            return Pose(p.position + np.array([gx * step, gy * step, 0.0]), p.orientation)

        ccd = SegmentWiseCCD(
            clearance_fn=clearance_fn,
            radius=robot_radius,
            correction_fn=sdf_correction,
            orientation_update_fn=update_orientations_se3,
        )

        t0 = time.perf_counter()
        chi_star, tau_star = ccd.run(chi, tau)
        elapsed = (time.perf_counter() - t0) * 1000.0

        min_d_init = min(clearance_fn(p) for p in chi)
        min_d_ref = min(clearance_fn(p) for p in chi_star)

        all_certified = True
        max_viol = 0.0
        for i in range(len(chi_star) - 1):
            L = default_motion_bound(chi_star[i], chi_star[i + 1], robot_radius)
            d_sum = clearance_fn(chi_star[i]) + clearance_fn(chi_star[i + 1])
            if L >= d_sum:
                all_certified = False
                max_viol = max(max_viol, L - d_sum)

        status_str = "CERTIFIED SAFE" if all_certified else f"FLAGGED UNTRAVERSABLE (viol: {max_viol:.3f}m)"
        print(f"\n[Scenario] {traj_name} + {map_name}")
        print(f"  Description:         {desc}")
        print(f"  Waypoints:           {len(chi)} -> {len(chi_star)} (Refinement Factor: {len(chi_star)/len(chi):.1f}x)")
        print(f"  Min Clearance:       {min_d_init:+.3f}m -> {min_d_ref:+.3f}m")
        print(f"  Status:              {status_str}")
        print(f"  Runtime:             {elapsed:.2f} ms")
        if ccd.log:
            print(f"  Logs/Warnings ({len(ccd.log)}):")
            for line in ccd.log:
                print(f"    - {line}")

    # 2. Run the 4 Orientation Trajectories through SegmentWiseCCD
    print("\n" + "=" * 85)
    print("PART 2: RUNNING 4 ORIENTATION TRAJECTORIES THROUGH SEGMENTWISECCD")
    print("=" * 85)

    sdf_open = GridSDF.from_npz(os.path.join(SAMPLE_DATA_DIR, "maps", "01_open_space_sdf.npz"))
    clearance_open = lambda p: sdf_open.distance(p.position[0], p.position[1])

    orientation_trajs = [
        ("orientation_straight.csv", "Straight line heading"),
        ("orientation_ninety_degree_turn.csv", "Sharp 90-degree right angle turn"),
        ("orientation_zigzag.csv", "Alternating heading zigzag"),
        ("orientation_angle_wrap.csv", "Angle wrap around +/- pi branch cut"),
    ]

    for traj_name, desc in orientation_trajs:
        traj_path = os.path.join(SAMPLE_DATA_DIR, "trajectories", traj_name)
        df = pd.read_csv(traj_path)

        chi = [Pose([r.x, r.y, 0.0], yaw_to_quaternion(r.theta)) for _, r in df.iterrows()]
        tau = [df.iloc[i + 1].t - df.iloc[i].t for i in range(len(df) - 1)]

        # Run with open space SDF
        ccd_open = SegmentWiseCCD(
            clearance_fn=clearance_open,
            radius=robot_radius,
            orientation_update_fn=update_orientations_se3,
        )
        chi_open, tau_open = ccd_open.run(chi, tau)

        # Run with forced subdivision (clearance buffer 0.35m) to inspect orientation on all newly bisected poses
        ccd_subdiv = SegmentWiseCCD(
            clearance_fn=lambda p: 0.35,
            radius=robot_radius,
            orientation_update_fn=update_orientations_se3,
        )
        chi_subdiv, tau_subdiv = ccd_subdiv.run(chi, tau)

        print(f"\n[Orientation Test] {traj_name}: {desc}")
        print(f"  Initial points:      {len(chi)}")
        print(f"  With Open SDF:       {len(chi_open)} points (safe)")
        print(f"  With Bisection:      {len(chi_subdiv)} points")
        print("  Pose sequence after bisection & orientation smoothing:")
        for idx, p in enumerate(chi_subdiv):
            yaw = quat_to_yaw(p.orientation)
            print(f"    Pose {idx:2d}: pos=({p.position[0]:.2f}, {p.position[1]:.2f}) | yaw={yaw:+6.3f} rad ({np.degrees(yaw):+6.1f} deg)")


if __name__ == "__main__":
    run_all()
