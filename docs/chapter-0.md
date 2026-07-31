# Chapter 0 — Lie Group Primer

Every chapter below rests on this. Get it wrong and the Jacobians are wrong everywhere.

## 0.1 SO(3)

$$SO(3) = \{\mathbf{R} \in \mathbb{R}^{3\times3} : \mathbf{R}^\top\mathbf{R} = \mathbf{I},\ \det\mathbf{R} = 1\}$$

The Lie algebra $\mathfrak{so}(3)$ is the set of skew matrices, isomorphic to $\mathbb{R}^3$ via $\lfloor\cdot\rfloor_\times$.

**Exponential map (Rodrigues).** For $\boldsymbol{\phi} \in \mathbb{R}^3$, $\theta = \|\boldsymbol{\phi}\|$, $\mathbf{a} = \boldsymbol{\phi}/\theta$:

$$\mathrm{Exp}(\boldsymbol{\phi}) = \mathbf{I} + \sin\theta\,\lfloor\mathbf{a}\rfloor_\times + (1-\cos\theta)\lfloor\mathbf{a}\rfloor_\times^2$$

**Logarithm.**

$$\mathrm{Log}(\mathbf{R}) = \frac{\theta(\mathbf{R} - \mathbf{R}^\top)^\vee}{2\sin\theta}, \qquad \theta = \cos^{-1}\!\left(\frac{\mathrm{tr}(\mathbf{R})-1}{2}\right)$$

Both need Taylor fallbacks near $\theta \to 0$ (and $\theta \to \pi$ for Log). Every production implementation has this branch; if yours doesn't, it has a latent NaN.

**Adjoint.** $\mathbf{R}\,\mathrm{Exp}(\boldsymbol{\phi})\,\mathbf{R}^\top = \mathrm{Exp}(\mathbf{R}\boldsymbol{\phi})$, i.e. $\mathrm{Ad}_\mathbf{R} = \mathbf{R}$ for $SO(3)$. Used constantly to move perturbations from the right side to the left. The form you actually reach for when deriving is the commuting version:

$$\mathrm{Exp}(\boldsymbol{\phi})\,\mathbf{R} = \mathbf{R}\,\mathrm{Exp}(\mathbf{R}^\top\boldsymbol{\phi})$$

It says a rotation can be *pushed through* an $\mathrm{Exp}$ at the cost of rotating the tangent vector. Every telescoping-product step in Chapter 1 — the noise propagation of §1.5 and the gyro-bias Jacobian of §1.6 — is that identity applied repeatedly to migrate all the $\mathrm{Exp}$ factors to one side. It follows from the first form together with $(\mathbf{R}\boldsymbol{\phi})^\wedge = \mathbf{R}\,\boldsymbol{\phi}^\wedge\mathbf{R}^\top$.

**Right Jacobian** — the single most important object for on-manifold Jacobians:

$$\mathbf{J}_r(\boldsymbol{\phi}) = \mathbf{I} - \frac{1-\cos\theta}{\theta^2}\lfloor\boldsymbol{\phi}\rfloor_\times + \frac{\theta - \sin\theta}{\theta^3}\lfloor\boldsymbol{\phi}\rfloor_\times^2$$

$$\mathbf{J}_r^{-1}(\boldsymbol{\phi}) = \mathbf{I} + \tfrac{1}{2}\lfloor\boldsymbol{\phi}\rfloor_\times + \left(\frac{1}{\theta^2} - \frac{1+\cos\theta}{2\theta\sin\theta}\right)\lfloor\boldsymbol{\phi}\rfloor_\times^2$$

Its defining property (first-order BCH):

$$\mathrm{Exp}(\boldsymbol{\phi} + \delta\boldsymbol{\phi}) \approx \mathrm{Exp}(\boldsymbol{\phi})\,\mathrm{Exp}(\mathbf{J}_r(\boldsymbol{\phi})\,\delta\boldsymbol{\phi})$$

$$\mathrm{Log}(\mathrm{Exp}(\boldsymbol{\phi})\mathrm{Exp}(\delta\boldsymbol{\phi})) \approx \boldsymbol{\phi} + \mathbf{J}_r^{-1}(\boldsymbol{\phi})\,\delta\boldsymbol{\phi}$$

## 0.2 Perturbation conventions — pick one and never deviate

| Convention | Definition | Used by |
|---|---|---|
| **Right (local / body)** | $\mathbf{R} = \hat{\mathbf{R}}\,\mathrm{Exp}(\delta\boldsymbol{\phi})$ | GTSAM, Forster preintegration, most VIO |
| **Left (global / world)** | $\mathbf{R} = \mathrm{Exp}(\delta\boldsymbol{\phi})\,\hat{\mathbf{R}}$ | Some INS derivations, MSCKF variants |

They differ by the adjoint: $\delta\boldsymbol{\phi}_{\text{left}} = \mathbf{R}\,\delta\boldsymbol{\phi}_{\text{right}}$. **Mixing them is the single most common source of "my filter almost works" bugs.** A covariance expressed in the wrong tangent basis produces an estimator that converges slowly and behaves inconsistently under rotation, which is maddening to debug because it never fails outright.

## 0.3 SE(3) and $SE_2(3)$

$SE(3)$ stacks rotation and translation. Its exponential involves the left Jacobian $\mathbf{J}_l$ coupling the two blocks:

$$\mathrm{Exp}\!\begin{pmatrix}\boldsymbol{\rho}\\\boldsymbol{\phi}\end{pmatrix} = \begin{bmatrix}\mathrm{Exp}(\boldsymbol{\phi}) & \mathbf{J}_l(\boldsymbol{\phi})\boldsymbol{\rho}\\ \mathbf{0} & 1\end{bmatrix}$$

$SE_2(3)$ (the **extended pose** / "double direct isometry" group) bundles $(\mathbf{R}, \mathbf{p}, \mathbf{v})$ into a single Lie group. This is what GTSAM's `NavState` represents, and it is the correct group for inertial navigation because IMU kinematics are *group-affine* on $SE_2(3)$ — which is exactly the condition that makes the **Invariant EKF (InEKF)** work, giving state-independent error propagation and therefore linearization-error-free covariance prediction for the deterministic part. If someone asks "what's better than an ESKF," the answer is InEKF, and this is why.

## 0.4 Uncertainty on manifolds

A Gaussian on a manifold is defined in the tangent space at the mean:

$$\mathbf{R} = \hat{\mathbf{R}}\,\mathrm{Exp}(\boldsymbol{\epsilon}), \qquad \boldsymbol{\epsilon} \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma})$$

This is a *concentrated* Gaussian and is only meaningful for small $\boldsymbol{\Sigma}$ (rule of thumb: standard deviation well under ~30°). It is why a 4-element quaternion in a state vector with a 4×4 covariance block is wrong — the covariance is necessarily singular along the norm-constraint direction. Everything in Chapter 1 and 2 follows from taking this seriously.
