# Principles of Inertial Estimation, VIO, Factor Graphs & SLAM

A working reference — equations, pseudocode and architecture — for inertial
estimation, visual-inertial odometry, factor graphs and SLAM. Written for
someone who already knows the vocabulary and wants the derivational spine and
the implementation-level detail.

📖 **Read the wiki:** https://taowangrub.github.io/estimation-slam-principles/

## Contents

- **Ch. 0 — Lie Group Primer** — SO(3)/SE(3)/SE₂(3), perturbation conventions, uncertainty on manifolds
- **Ch. 1 — IMU Preintegration** — sensor model, kinematics, noise propagation, bias correction, residuals
- **Ch. 2 — EKF / INS** — PX4 EKF2 and ArduPilot EKF3: error-state, delayed fusion horizon, arbitration
- **Ch. 3 — Visual-Inertial Odometry** — ORB-SLAM3, cuVSLAM, MSCKF
- **Ch. 4 — GTSAM & Factor Graphs** — formulation, iSAM2, smart factors, a LIO-SAM-shaped graph
- **Ch. 5 — SLAM as a Whole** — taxonomy, the modern pipeline, data association, maps, evaluation
- **References** — sources per chapter, plus cross-check notes

Chapter 1 has been verified equation-by-equation against
[Qiu Xiaochen's 《预积分总结与公式推导》](https://github.com/PetWorm/IMU-Preintegration-Propogation-Doc),
a 25-page line-by-line derivation of Forster's on-manifold preintegration.

## Built with

[MkDocs](https://www.mkdocs.org/) + [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/),
with LaTeX rendered by MathJax (via `pymdownx.arithmatex`). Every push to `main`
rebuilds and deploys the site to GitHub Pages through the workflow in
[`.github/workflows/deploy.yml`](.github/workflows/deploy.yml).

## Run it locally

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
mkdocs serve            # live preview at http://127.0.0.1:8000
mkdocs build --strict   # produce the static site in ./site
```

## Editing

The prose lives in [`docs/`](docs/), one Markdown file per chapter. Adjust the
sidebar order in [`mkdocs.yml`](mkdocs.yml) under `nav:`. Math uses standard
`$…$` (inline) and `$$…$$` (display) LaTeX.
