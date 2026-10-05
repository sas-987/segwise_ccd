# ATR synthetic test data

Purpose: test Segment-wise CCD and Orientation Smoothing before the real BARN SDF implementation is ready.

Grid: 200x120 at 0.05 m/cell = 10.0m x 6.0m.

## Important
The `*_sdf.npz` and `mock_sdf.py` files are temporary test fixtures. They are NOT the final SDF implementation. They only expose the same `distance(x,y)` and `gradient(x,y)` interface that CCD needs.

## CCD cases
- `02_endpoints_safe_middle_collision`: most important case; endpoints are safe but the connecting segment crosses the obstacle.
- `03_narrow_passage`: biased trajectory through a narrow passage.
- `04_long_diagonal`: long segment / recursive subdivision.
- `05_multiple_obstacles`: several risky regions.
- `01_open_space` and `06_safe_corridor`: control cases; should not be over-refined.

## Orientation cases
`orientation_straight.csv`, `orientation_ninety_degree_turn.csv`, `orientation_zigzag.csv`, and `orientation_angle_wrap.csv`.
The angle-wrap case specifically checks that +pi/-pi headings are handled circularly rather than averaged through zero.

## Suggested use
Load a CSV into your existing `Trajectory` representation and a matching `*_sdf.npz` with `GridSDF.from_npz()`. Pass the mock SDF to your CCD function. When your real SDF implementation is complete, replace the mock with the real SDF and reuse the exact same trajectories as regression tests.
