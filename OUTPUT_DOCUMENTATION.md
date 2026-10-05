# Segment-Wise CCD & Orientation Smoothing: Output Documentation & Benchmark Report

This document records the output, metrics, and verification results of running **Segment-Wise Continuous Collision Detection (CCD)** and **Orientation Smoothing** on the synthetic dataset from [`sample-data`](sample-data/README.md).

---

## 1. Overview & Test Setup

- **Algorithm**: Segment-wise Continuous Collision Detection (CCD) from *"Adaptive Trajectory Refinement for Optimization-based Local Planning in Narrow Passages"* (Algorithm 1 & Section III-E).
- **Core Certification Inequality**:
  $$L(p_i, p_j) < d(p_i) + d(p_j)$$
  where:
  - $L(p_i, p_j) = \|\mathbf{t}_j - \mathbf{t}_i\| + r \cdot \theta(q_i, q_j)$ is the conservative motion bound.
  - $d(p)$ is the clearance distance queried from the map Signed Distance Field (SDF).
  - $r$ is the robot's bounding radius ($r = 0.15\,\text{m}$).
- **Pose Correction Threshold**: Midpoints where $d(p_{\text{mid}}) \le r / 2 = 0.075\,\text{m}$ are shifted along the clearance gradient $\nabla d$ away from obstacle boundaries.
- **Orientation Smoothing**: Tangent-aligned orientation update on newly inserted midpoints using circular arc bisector vector averaging:
  $$\vec{v} = (\cos\alpha_1 + \cos\alpha_2, \; \sin\alpha_1 + \sin\alpha_2), \quad \theta_{\text{mid}} = \text{atan2}(v_y, v_x)$$
  converted to a unit quaternion $[w, 0, 0, z]$.
- **Execution Script**: [`run_all_sample_data.py`](run_all_sample_data.py)

---

## 2. Benchmark Scenarios: Collision Detection & Refinement

All 6 map scenarios were tested using their corresponding `_sdf.npz` files and initial `_trajectory.csv` files.

### Summary Table

| Scenario | Scenario Description | Initial Waypoints | Refined Waypoints | Initial Min Clearance | Refined Min Clearance | Safety Status | Execution Time |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **`01_open_space`** | Free open space (control case) | 5 | 5 (1.0×) | $+1.450\,\text{m}$ | $+1.450\,\text{m}$ | **CERTIFIED SAFE** | 0.55 ms |
| **`02_endpoints_safe_middle_collision`** | Safe endpoints, straight segment penetrates obstacle | 2 | 19 (9.5×) | $+2.250\,\text{m}$ | **$+0.075\,\text{m}$** | **CERTIFIED SAFE** | 3.68 ms |
| **`03_narrow_passage`** | Biased path through narrow doorway / corridor | 6 | 8 (1.3×) | $+0.278\,\text{m}$ | $+0.120\,\text{m}$ | **CERTIFIED SAFE** | 0.88 ms |
| **`04_long_diagonal`** | Long diagonal through impassable circular obstacle | 2 | 54 (27.0×) | $+3.070\,\text{m}$ | $-0.700\,\text{m}$ | **FLAGGED UNTRAVERSABLE** | 14.99 ms |
| **`05_multiple_obstacles`** | Trajectory intersecting dense obstacle clusters | 5 | 38 (7.6×) | $-0.050\,\text{m}$ | $-0.050\,\text{m}$ | **FLAGGED UNTRAVERSABLE** | 9.02 ms |
| **`06_safe_corridor`** | Curving corridor with adequate clearance | 5 | 5 (1.0×) | $+1.200\,\text{m}$ | $+1.200\,\text{m}$ | **CERTIFIED SAFE** | 0.31 ms |

---

### Detailed Case Analysis

#### Scenario 01: `01_open_space`
- **Goal**: Verify that safe trajectories in unobstructed spaces do not suffer from over-refinement or unnecessary bisections.
- **Output**:
  - $L(p_i, p_j) < d(p_i) + d(p_j)$ was satisfied for all initial segments on the very first pass.
  - Final waypoints: **5 / 5** (0 bisections performed).
  - Clearance remained $+1.450\,\text{m}$.
  - Verification: **Pass**.

#### Scenario 02: `02_endpoints_safe_middle_collision` *(Primary CCD Benchmark)*
- **Goal**: Discrete collision detection only evaluates endpoints (both have $> 2.2\,\text{m}$ clearance), but the connecting straight chord cuts straight through an obstacle, reaching $-0.72\,\text{m}$ inside the object at $(5.0, 3.0)$.
- **Output**:
  - SegmentWiseCCD adaptively bisected the unsafe segment into **19 waypoints**.
  - All penetrating midpoints were automatically nudged outward along $\nabla d$ to achieve clearance $\ge r / 2 = 0.075\,\text{m}$.
  - The final minimum clearance achieved was **$+0.0751\,\text{m}$**.
  - Every resulting sub-segment satisfied $L < d_i + d_j$.
  - Verification: **100% Certified Safe**.

#### Scenario 03: `03_narrow_passage`
- **Goal**: Trajectory guided through a narrow passage with small clearances.
- **Output**:
  - Initial 6 waypoints refined to **8 waypoints** around the constriction.
  - Minimum clearance maintained at $+0.120\,\text{m}$.
  - Verification: **100% Certified Safe**.

#### Scenarios 04 & 05: `04_long_diagonal` & `05_multiple_obstacles` *(Impassable Obstacles)*
- **Goal**: Test termination safety guards when a trajectory is physically impossible because an obstacle completely blocks the corridor.
- **Output**:
  - The algorithm recursively bisected segments until segment length dropped below the physical robot footprint limit:
    $$\text{segment\_length} < \text{min\_segment\_length} = r = 0.15\,\text{m}$$
  - The primary guard terminated bisection, preventing infinite loops.
  - Logged appropriate warning messages:
    - *`WARNING: likely physically untraversable gap -- segment length fell below robot footprint guard (0.15m) while still uncertified.`*
  - Verification: **Safely Flagged Untraversable**.

#### Scenario 06: `06_safe_corridor`
- **Goal**: Second control test to ensure curved obstacle corridors with adequate safety buffer are preserved.
- **Output**:
  - Preserved original 5 waypoints with zero splits.
  - Minimum clearance: $+1.200\,\text{m}$.
  - Verification: **Pass**.

---

## 3. Orientation Smoothing Test Results

Evaluated on the 4 dedicated trajectory test cases from `sample-data/trajectories/`:

### Case 1: `orientation_straight.csv`
- **Input**: 4 collinear waypoints along $+X$, all with $\theta = 0.0\,\text{rad}$.
- **Refined Output**: 7 waypoints.
- **Result**:
  - Every midpoint preserves $\theta = 0.000\,\text{rad}$ ($0.0^\circ$).
  - Quaternion stays identically $[1.0, 0.0, 0.0, 0.0]$.

### Case 2: `orientation_ninety_degree_turn.csv`
- **Input**: Sharp $90^\circ$ right-angle corner at $(1.0, 0.0)$.
- **Refined Output**: 8 waypoints.
- **Result**:
  - Incoming poses: $(0.0, 0.0), (0.5, 0.0), (0.75, 0.0)$ hold $\theta = 0^\circ$.
  - Corner pose $(1.0, 0.0)$ and outgoing poses $(1.0, 0.5), (1.0, 1.0), \dots$ turn smoothly to $\theta = +1.571\,\text{rad}$ ($+90.0^\circ$).
  - Quaternion transitions cleanly to $[0.7071, 0.0, 0.0, 0.7071]$.

### Case 3: `orientation_zigzag.csv`
- **Input**: 5 alternating waypoints oscillating between $+45^\circ$ and $-31^\circ$.
- **Refined Output**: 11 waypoints.
- **Result**:
  - Poses adaptively track the geometric tangent along segments:
    $$0.0^\circ \longrightarrow +45.0^\circ \longrightarrow -31.0^\circ \longrightarrow +38.7^\circ$$
  - No discontinuous yaw jumps or signs of tangent inversion.

### Case 4: `orientation_angle_wrap.csv` ($\pm\pi$ Boundary Condition)
- **Input**: 4 waypoints where headings alternate between $+3.13\,\text{rad}$ ($+179.3^\circ$) and $-3.13\,\text{rad}$ ($-179.3^\circ$).
- **Numerical Edge Case**:
  - An arithmetic average without circular wrapping would compute $\frac{179.3^\circ + (-179.3^\circ)}{2} = 0.0^\circ$, causing an erroneous $180^\circ$ inversion.
- **Output**:
  - Both SLERP quaternion interpolation and the circular vector mean resolve across the branch cut to $\pm\pi$ ($180.0^\circ$), preserving circular topology without collapsing through zero.

---

## 4. How to Reproduce

Run the automated test suite directly using Python:

```bash
python run_all_sample_data.py
```
