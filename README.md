# 4-DOF Robot Kinematics, Dynamics & Trajectory Planning

MATLAB coursework project exploring a four-revolute-joint (RRRR) manipulator, symbolic robot dynamics, differential kinematics, and piecewise cubic joint trajectory planning.

## Contents

- `assignment04.m` — original assignment computations, including Jacobian-based velocity analysis, symbolic Euler–Lagrange modelling, and cubic trajectory generation.
- `visualizeRRRRobot.m` — 3D link and coordinate-frame plotting helper. A misleading comment was corrected; no algorithmic changes were made.
- `assignment_report.pdf` — submitted assignment report.
- `assignment_worksheet.mw` — original worksheet, preserved for reference (may require compatible software).

## Highlights

1. Four-DOF homogeneous transforms and 3D robot visualization.
2. Jacobian evaluation, manipulability calculation, forward differential kinematics and pseudoinverse velocity solution.
3. Symbolic Lagrangian equations of motion.
4. Two cubic trajectory segments: 20° → 80° (0–5 s) and 80° → 25° (5–10 s), with position, velocity and acceleration plots.

## Running

1. Open MATLAB and set the current folder to `MATLAB/`.
2. Run `assignment04.m` section by section; the symbolic section may be computationally expensive.
3. To visualize separately, call `visualizeRRRRobot(pi/2, 0, 0, pi/6)`.

**Requirements:** MATLAB with Symbolic Math Toolbox for the symbolic modelling sections; MATLAB graphics for the visualization. The trajectory section also uses symbolic display operations.

## Important limitations

- **Not independently executed or validated in MATLAB.** Review numerical outputs and equations before using them in research or design.
- The visualization helper has been lightly cleaned, but has not been runtime-tested.
- The supplied Jacobian has a zero fourth column and the corresponding manipulability expression `sqrt(det(J*J.'))` is zero for a 6×4 Jacobian (rank at most four). This quantity should **not** be interpreted as a useful nonzero manipulability index. A task-specific reduced Jacobian or suitable singular-value measure is needed for meaningful interpretation.
- The original assignment's dynamics expressions and frame conventions have not been independently verified; they are retained as submitted rather than silently replaced.
- The two cubic segments impose zero velocity at the intermediate waypoint, so the joint stops at the 5-second boundary.

## Academic context

Originally prepared as an undergraduate robotics assignment at the University of Ruhuna. This repository is a learning and portfolio artifact, not production robot-control software.
