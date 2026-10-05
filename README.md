# Segment-Wise Continuous Collision Detection (CCD) & Orientation Smoothing

Python implementation of **Algorithm 1 (Segment-wise CCD)** and **Section III-E (Local Kinematic Feasibility / Orientation Smoothing)** based on the research paper:
> *"Adaptive Trajectory Refinement for Optimization-based Local Planning in Narrow Passages"*

---

## Features

- **Continuous Collision Detection (CCD)**: Guarantees spatial safety between discrete waypoints via conservative motion displacement bound $L(p_i, p_j) < d(p_i) + d(p_j)$.
- **SE(3) Pose Representation**: 3D positions with unit quaternion orientations.
- **Adaptive Priority-Queue Bisection**: Uses a max-heap linked-list structure for efficient $O(1)$ segment insertion.
- **Gradient-Based Pose Correction**: Nudges penetrating or tight waypoints away from obstacles along $\nabla d$ when clearance $d(p) \le r/2$.
- **Orientation Smoothing (Kinematic Feasibility)**: Adjusts midpoint headings using circular arc bisector vector averaging to ensure traversability for wheeled mobile robots.
- **Physical Footprint & Numerical Guards**: Prevents infinite recursion in impassable corridors by enforcing robot footprint limits ($L_{\text{min}} \le r$).

---

## Repository Structure

```
├── seg_ccd (1).py                         # Main SegmentWiseCCD algorithm implementation
├── orientation_smoothing.py               # 2D & SE(3) orientation smoothing primitives
├── run_all_sample_data.py                 # Automated benchmark test suite
├── OUTPUT_DOCUMENTATION.md                # Comprehensive output results & analysis
├── SegmentWiseCCD_Code_Documentation (1).docx # Original architectural documentation
├── sample-data/                           # Synthetic benchmark maps and trajectories
│   ├── maps/                              # 6 SDF maps (.npz) and occupancy grids (.npy)
│   ├── trajectories/                      # 6 scenario CSVs + 4 orientation test CSVs
│   ├── mock_sdf.py                        # GridSDF bilinear interpolation oracle
│   └── manifest.json                      # Dataset specification
└── LICENSE                                # MIT License
```

---

## Quickstart & Benchmark Execution

To run all 6 benchmark maps and 4 orientation test cases:

```bash
python run_all_sample_data.py
```

For full numerical metrics, case-by-case analysis, and verification results, see **[OUTPUT_DOCUMENTATION.md](OUTPUT_DOCUMENTATION.md)**.
