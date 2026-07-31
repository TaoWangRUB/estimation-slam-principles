# Principles of Inertial Estimation, VIO, Factor Graphs and SLAM

A working reference: equations, pseudocode, architecture. Written for someone who already knows the vocabulary and wants the derivational spine and the implementation-level detail.

**Contents**

- [Chapter 0 — Lie Group Primer](chapter-0.md)
- [Chapter 1 — System Architecture: components, interfaces, data flow](chapter-1.md)
- [Chapter 2 — IMU Preintegration](chapter-2.md)
- [Chapter 3 — GTSAM and Factor Graphs](chapter-3.md)
- [Chapter 4 — Visual-Inertial Odometry: ORB-SLAM3 and cuVSLAM](chapter-4.md)
- [Chapter 5 — SLAM as a Whole](chapter-5.md)
- [Chapter 6 — EKF/INS: PX4 EKF2 and ArduPilot EKF3](chapter-6.md)
- [References](references.md)

## How to read this

**[Chapter 1](chapter-1.md) is the map.** It defines the components, the interfaces between them, and the type carried on every wire. Every chapter after it decomposes exactly one box, and declares at the top which interface it implements. If a chapter ever seems to appear from nowhere, go back to §1.1 and find its box.

```mermaid
flowchart LR
  C0["<b>Ch.0</b><br/>Lie groups"]
  C1["<b>Ch.1</b><br/>Architecture<br/><i>the contract</i>"]
  C2["<b>Ch.2</b><br/>IMU<br/>preintegration"]
  C3["<b>Ch.3</b><br/>Factor graphs<br/>GTSAM"]
  C4["<b>Ch.4</b><br/>VIO"]
  C5["<b>Ch.5</b><br/>SLAM"]
  C6["<b>Ch.6</b><br/>Autopilot EKF"]

  C1 --> C2 -->|PreintegratedImu| C3 -->|NavState| C4 -->|Keyframe| C5
  C5 -->|Odometry| C6
  C0 -.->|"Jacobians,<br/>conventions"| C2
  C0 -.-> C3
  C0 -.-> C4

  style C1 fill:#31456b,stroke:#8ab4f8,color:#fff
  style C6 fill:#6b3145,stroke:#f8a1b4,color:#fff
```

Two threads tie the halves of this document together:

- **Downward** — an IMU factor built in Chapter 2 is consumed as `ImuFactor` in Chapter 3, appears as the inertial term of the VI bundle-adjustment objective in Chapter 4, and is one edge of the pose chain in Chapter 5.
- **Across** — Chapters 2–5 build an *optimization-based* estimator. Chapter 6 is the *filtering* one on the autopilot, and in a real vehicle the two are chained: VIO/SLAM publishes odometry, and EKF2/EKF3 ingests it as an external-vision aiding source alongside GPS, baro and magnetometer. Same IMU, fused twice, for different reasons — see §1.1 and §6.9.

**Notation.** $(\cdot)_W$ world/navigation frame (ENU or NED — stated per chapter), $(\cdot)_B$ body/IMU frame, $(\cdot)_C$ camera frame. $\mathbf{R}_{WB} \in SO(3)$ rotates body vectors into world. $\lfloor \mathbf{v} \rfloor_\times$ is the skew-symmetric matrix. $\tilde{(\cdot)}$ denotes a measurement, $\hat{(\cdot)}$ an estimate, $\bar{(\cdot)}$ a noise-free quantity or one evaluated at the linearization point, and $\delta(\cdot)$ an error.
