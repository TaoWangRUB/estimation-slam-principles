# References

Sources this reference draws on, grouped by the chapter they matter most to. Where a chapter's equations were checked line-by-line against a source, that is noted.

## System architecture (Chapter 1)

The component split, rate domains and TF ownership of Chapter 1 are synthesized from the systems named throughout — ORB-SLAM3's three-thread structure, cuVSLAM's frontend/backend split, LIO-SAM's graph, and PX4's delayed-horizon design — rather than taken from any single source. **REP-105** ([ros.org/reps/rep-0105.html](https://www.ros.org/reps/rep-0105.html)) is the normative reference for the `map → odom → base_link` ownership rules of §1.6.

## IMU preintegration (Chapter 2)

**Qiu Xiaochen (邱笑晨), 《预积分总结与公式推导》, 2018-08-27.**
[github.com/PetWorm/IMU-Preintegration-Propogation-Doc](https://github.com/PetWorm/IMU-Preintegration-Propogation-Doc)

A 25-page line-by-line derivation of Forster's on-manifold preintegration, in Chinese. It is the most complete public working-through of the algebra that the papers compress into a few equations: every $\mathrm{Exp}$/$\mathrm{Log}$ manipulation is justified, every approximation is labelled with the identity that permits it, and the Adjoint property is proved rather than cited. **Chapter 2 of this reference was cross-checked against it in full** — sensor model, kinematics, the preintegration definitions, the $\mathbf{A}$/$\mathbf{B}$ noise-propagation matrices, the bias-update Jacobians, the residuals, and every analytic Jacobian. See [the cross-check note](#cross-check-notes) below.

Worth knowing: the document itself carries an erratum notice — it corrects a sign inside the bracket of $\mathbf{J}_r^{-1}$ that was wrong in an earlier, widely-circulated serialization of the same material.

**Forster, Carlone, Dellaert, Scaramuzza**, *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry*, IEEE T-RO 33(1), 2017. Also the earlier RSS 2015 paper, *IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation*, and its supplementary material. The source of the on-manifold formulation and of what GTSAM implements.

**Lupton & Sukkarieh**, *Visual-Inertial-Aided Navigation for High-Dynamic Motion in Built Environments Without Initial Conditions*, IEEE T-RO 28(1), 2012. The original preintegration idea, in Euler angles.

## Estimation and Lie theory background

- **Barfoot**, *State Estimation for Robotics*, Cambridge University Press. The standard reference for matrix Lie groups in estimation; the $SE_2(3)$ and Jacobian material in Chapter 0.
- **Gao Xiang (高翔)**, 《视觉SLAM十四讲》 (*Visual SLAM: From Theory to Practice*). Accessible treatment of $SO(3)$/$SE(3)$, perturbation models and BA.
- **Qin Yongyuan (秦永元)**, 《惯性导航》 (*Inertial Navigation*). Strapdown INS mechanization — the classical counterpart to the static-world simplifications of §2.1.
- **Solà, Deray, Atchuthan**, *A micro Lie theory for state estimation in robotics*, arXiv:1812.01537. The most readable derivation of right/left Jacobians and the $\oplus$/$\ominus$ conventions of §0.2.
- **Barrau & Bonnabel**, *The Invariant Extended Kalman Filter as a Stable Observer*, IEEE TAC 62(4), 2017. The group-affine property and InEKF result cited in §0.3.

## EKF / INS (Chapter 6)

- **PX4 ECL EKF2** — [docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html) and the `PX4-Autopilot/src/modules/ekf2` source tree. The SymForce `derivation.py` that generates the covariance prediction and measurement Jacobians is the single most instructive file.
- **ArduPilot EKF3** — [ardupilot.org/copter/docs/common-apm-navigation-extended-kalman-filter-overview.html](https://ardupilot.org/copter/docs/common-apm-navigation-extended-kalman-filter-overview.html); see `EK3_AFFINITY`, `EK3_ERR_THRESH` and the `EK3_SRCn_*` source-set parameters for the lane-switching and sensor-arbitration architecture of §6.7.
- **Solà**, *Quaternion kinematics for the error-state Kalman filter*, arXiv:1711.02508. The definitive treatment of the error-state formulation, injection and reset Jacobian of §6.2.

## Visual-inertial odometry (Chapter 4)

- **Campos, Elvira, Rodríguez, Montiel, Tardós**, *ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM*, IEEE T-RO 37(6), 2021.
- **cuVSLAM** — NVIDIA, *cuVSLAM: CUDA accelerated visual odometry and mapping*, arXiv:2506.04359; the source release at [github.com/nvidia-isaac/cuVSLAM](https://github.com/nvidia-isaac/cuVSLAM) (v17.0.0, 2026-07-21; public API in `libs/cuvslam/cuvslam2.h`, design notes in `DESIGN_CONCEPTS.md`) (NVIDIA Community License — prebuilt `libcuvslam.so` with open C++/Python bindings, up to 32 cameras, Jetson Orin/Thor); and the [Isaac ROS Visual SLAM](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/) documentation.
- **cuNLS** — [github.com/nvidia-isaac/cuNLS](https://github.com/nvidia-isaac/cuNLS). NVIDIA's CUDA-accelerated nonlinear least-squares solver for bundle adjustment, pose-graph optimization and ICP-style alignment; bundled into cuVSLAM. The GPU counterpart to the solvers of Chapter 3.
- **Mourikis & Roumeliotis**, *A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation*, ICRA 2007. The MSCKF null-space projection of §4.5.
- **Geneva, Eckenhoff, Lee, Yang, Huang**, *OpenVINS: A Research Platform for Visual-Inertial Estimation*, ICRA 2020. FEJ and observability-constrained consistency fixes.
- **Qin, Li, Shen**, *VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator*, IEEE T-RO 34(4), 2018, and **VINS-Fusion** ([github.com/HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)). Midpoint preintegration (`vins/src/factor/integration_base.h`), Ceres fixed-lag smoothing with two marginalization modes, `fastPredictIMU()` as output predictor, and online camera↔IMU extrinsic calibration (`initial/initial_ex_rotation`).

The source-level comparison in §4.6 was traced against ORB-SLAM3 `src/ImuTypes.cc`/`Tracking.cc`/`LocalMapping.cc`/`LoopClosing.cc`, cuVSLAM v15.0.0 `libs/{imu,pipelines,sof}`, and VINS-Fusion `vins/src/{factor,estimator,initial}`.

## Factor graphs (Chapter 3)

- **Dellaert & Kaess**, *Factor Graphs for Robot Perception*, Foundations and Trends in Robotics, 2017. The book-length treatment of everything in Chapter 3.
- **Kaess et al.**, *iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree*, IJRR 31(2), 2012.
- **GTSAM** — [gtsam.org](https://gtsam.org/). See also Dellaert, *Factor Graphs and GTSAM: A Hands-on Introduction* (technical report).
- **Yang, Antonante, Tzoumas, Carlone**, *Graduated Non-Convexity for Robust Spatial Perception*, RA-L 2020. The GNC recommended for loop closures in §3.6.

## SLAM at large (Chapter 5)

- **Cadena et al.**, *Past, Present, and Future of Simultaneous Localization and Mapping: Toward the Robust-Perception Age*, IEEE T-RO 32(6), 2016. The standard survey.
- **Thrun, Burgard, Fox**, *Probabilistic Robotics*, MIT Press. EKF-SLAM, FastSLAM, occupancy grids.
- **Grisetti, Stachniss, Burgard**, *Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters*, IEEE T-RO 23(1), 2007. GMapping's improved proposal and adaptive resampling.
- **Zhang, Kaess, Singh**, *On Degeneracy of Optimization-based State Estimation Problems*, ICRA 2016. The eigen-analysis and solution remapping of §5.6.
- **Labbé & Michaud**, *RTAB-Map as an open-source lidar and visual SLAM library*, JFR 36(2), 2019. The STM/WM/LTM memory management of §5.6.
- **Oleynikova et al.**, *Voxblox*, IROS 2017, and NVIDIA **nvblox**. TSDF/ESDF mapping for §5.5.
- **evo** — [github.com/MichaelGrupp/evo](https://github.com/MichaelGrupp/evo). ATE/RPE evaluation as described in §5.7.

---

## Cross-check notes

Chapter 2 was verified equation-by-equation against Qiu's derivation. Recording the outcome, since a reference is only useful if you know what it was actually used for.

**Confirmed identical** — the sensor and kinematic models; the zero-order-hold discretization; the three preintegration definitions of §2.4; the $\mathbf{A}$ and $\mathbf{B}$ matrices and the covariance recursion of §2.5 (Qiu writes $\Delta\tilde{\mathbf{R}}_{j,j-1}$ where §2.5 writes $\Delta\tilde{\mathbf{R}}_{k,k+1}^\top$ — the same matrix); the first-order bias-correction structure of §2.6; the residuals of §2.7; and all four analytic Jacobians originally quoted there.

**Independently verified as correct** — the $\mathbf{J}_r^{-1}$ expression in §0.1 carries the *corrected* bracket sign, matching Qiu's erratum rather than the earlier circulated version. The gravity-collapse identity in §2.2 ($\tfrac{n(n-1)}{2} + \tfrac{n}{2} = \tfrac{n^2}{2}$) agrees with Qiu's arithmetic-series form. The bias-Jacobian recursions in the §2.8 pseudocode were expanded by hand and shown to reproduce Qiu's closed forms exactly, **including the update ordering** — position before velocity before rotation.

**One error found and fixed** — §2.5 previously wrote the noise separation as

$$\Delta\tilde{\mathbf{R}}_{ij} = \Delta\bar{\mathbf{R}}_{ij}\,\mathrm{Exp}(-\delta\boldsymbol{\phi}_{ij}), \qquad \Delta\tilde{\mathbf{v}}_{ij} = \Delta\bar{\mathbf{v}}_{ij} + \delta\mathbf{v}_{ij}$$

mixing a *negative* rotational perturbation with *positive* additive ones. Forster's eq. (35)–(37) and Qiu both define these consistently — either measurement $=$ truth $\oplus$ noise for all three, or truth $=$ measurement $\ominus$ noise for all three. As written, $\delta\boldsymbol{\phi}$ carried the opposite sign to the convention the $\mathbf{A}$ recursion immediately below it assumes. Now corrected, with both equivalent forms stated explicitly.

**Material added from Qiu** — the commuting Adjoint form $\mathrm{Exp}(\boldsymbol{\phi})\mathbf{R} = \mathbf{R}\,\mathrm{Exp}(\mathbf{R}^\top\boldsymbol{\phi})$ (§0.1), the static world assumption (§2.1), closed forms of the five bias Jacobians and the ordering constraint they impose (§2.6), the complete Jacobian table including $\partial\mathbf{r}_{\Delta\mathbf{R}}/\partial\delta\mathbf{b}^g$, and the lifting-convention discussion explaining why $\mathbf{p}\leftarrow\mathbf{p}+\mathbf{R}\,\delta\mathbf{p}$ yields $-\mathbf{I}$ and $\mathbf{R}_i^\top\mathbf{R}_j$ rather than $-\mathbf{R}_i^\top$ and $\mathbf{R}_i^\top$ (§2.7).
