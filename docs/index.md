# Principles of Inertial Estimation, VIO, Factor Graphs and SLAM

A working reference: equations, pseudocode, architecture. Written for someone who already knows the vocabulary and wants the derivational spine and the implementation-level detail.

**Contents**

- [Chapter 0 — Lie Group Primer](chapter-0.md)
- [Chapter 1 — IMU Preintegration](chapter-1.md)
- [Chapter 2 — GTSAM and Factor Graphs](chapter-2.md)
- [Chapter 3 — Visual-Inertial Odometry: ORB-SLAM3 and cuVSLAM](chapter-3.md)
- [Chapter 4 — SLAM as a Whole](chapter-4.md)
- [Chapter 5 — EKF/INS: PX4 EKF2 and ArduPilot EKF3](chapter-5.md)
- [References](references.md)

## How the chapters chain

The chapters are ordered so that nothing is used before it is introduced. Each one hands a concrete object to the next:

```
  raw IMU            ┌────────────────────────────────────────────────┐
  200-1000 Hz ──────►│ Ch.1   IMU PREINTEGRATION                      │
                     │   N samples ──► ΔR, Δv, Δp, Σ, ∂Δ/∂b           │
                     └──────────────────────┬─────────────────────────┘
                                            │  ONE IMU factor per keyframe
                                            ▼
                     ┌────────────────────────────────────────────────┐
                     │ Ch.2   FACTOR GRAPH   (GTSAM, iSAM2)           │
                     │   the machinery that consumes factors:         │
                     │   variables, elimination, marginalization      │
                     └──────────────────────┬─────────────────────────┘
                                            │  + reprojection factors
  camera / lidar ───────────────────────────┤
                                            ▼
                     ┌────────────────────────────────────────────────┐
                     │ Ch.3   VISUAL-INERTIAL ODOMETRY                │
                     │   that graph over a sliding window             │
                     │   ORB-SLAM3 · cuVSLAM · VINS · MSCKF           │
                     └──────────────────────┬─────────────────────────┘
                                            │  odometry — smooth, drifts
                                            ▼
                     ┌────────────────────────────────────────────────┐
                     │ Ch.4   SLAM AS A WHOLE                         │
                     │   + place recognition, loop closure, mapping   │
                     └──────────────────────┬─────────────────────────┘
                                            │  pose / odom, map→odom
                                            ▼
                     ┌────────────────────────────────────────────────┐
                     │ Ch.5   EKF / INS  on the flight controller     │
                     │   PX4 EKF2 · ArduPilot EKF3 — fuses that       │
                     │   odometry with GPS, baro, mag → controller    │
                     └────────────────────────────────────────────────┘

  Ch.0 (Lie groups) underpins every box above — it is where the Jacobians
  and the perturbation conventions that all of them share are defined.
```

Two directions are worth naming explicitly, because they are what tie the optimization half of this document to the filtering half:

- **Downward** — an IMU factor built in Chapter 1 is consumed as `ImuFactor` in Chapter 2, appears as the inertial term of the VI bundle-adjustment objective in Chapter 3, and is one edge of the pose chain in Chapter 4.
- **Across** — Chapters 1–4 build an *optimization-based* estimator. Chapter 5 is the *filtering* one that runs on the autopilot, and in a real vehicle the two are chained: VIO/SLAM publishes odometry, and EKF2/EKF3 ingests it as an external-vision aiding source alongside GPS, baro and magnetometer. Same IMU, fused twice, for different reasons — which is exactly the architecture that Chapter 5 §5.9 argues about.

**Notation.** $(\cdot)_W$ world/navigation frame (ENU or NED — stated per chapter), $(\cdot)_B$ body/IMU frame, $(\cdot)_C$ camera frame. $\mathbf{R}_{WB} \in SO(3)$ rotates body vectors into world. $\lfloor \mathbf{v} \rfloor_\times$ is the skew-symmetric matrix. $\tilde{(\cdot)}$ denotes a measurement, $\hat{(\cdot)}$ an estimate, $\bar{(\cdot)}$ a noise-free quantity or one evaluated at the linearization point, and $\delta(\cdot)$ an error.
