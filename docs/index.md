# Principles of Inertial Estimation, VIO, Factor Graphs and SLAM

A working reference: equations, pseudocode, architecture. Written for someone who already knows the vocabulary and wants the derivational spine and the implementation-level detail.

**Contents**

- [Chapter 0 — Lie Group Primer](chapter-0.md)
- [Chapter 1 — IMU Preintegration](chapter-1.md)
- [Chapter 2 — EKF/INS: PX4 EKF2 and ArduPilot EKF3](chapter-2.md)
- [Chapter 3 — Visual-Inertial Odometry: ORB-SLAM3 and cuVSLAM](chapter-3.md)
- [Chapter 4 — GTSAM and Factor Graphs](chapter-4.md)
- [Chapter 5 — SLAM as a Whole](chapter-5.md)
- [References](references.md)

**Notation.** $(\cdot)_W$ world/navigation frame (ENU or NED — stated per chapter), $(\cdot)_B$ body/IMU frame, $(\cdot)_C$ camera frame. $\mathbf{R}_{WB} \in SO(3)$ rotates body vectors into world. $\lfloor \mathbf{v} \rfloor_\times$ is the skew-symmetric matrix. $\tilde{(\cdot)}$ denotes a measurement, $\hat{(\cdot)}$ an estimate, $\bar{(\cdot)}$ a noise-free quantity or one evaluated at the linearization point, and $\delta(\cdot)$ an error.
