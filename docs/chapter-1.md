# Chapter 1 — IMU Preintegration

!!! note "Sources"
    The formulation here is Forster et al. (2015/2017), which is what GTSAM implements. The equations in this chapter have been cross-checked line-by-line against [Qiu Xiaochen's 《预积分总结与公式推导》](https://github.com/PetWorm/IMU-Preintegration-Propogation-Doc), a 25-page derivation that works through every algebraic step the papers compress. See [References → cross-check notes](references.md#cross-check-notes) for what that check confirmed and what it corrected.

## 1.1 Sensor model

World frame here is any inertial frame with gravity $\mathbf{g}$ (a vector, e.g. $[0,0,-9.81]^\top$ in ENU). This rests on a **static world assumption** that is worth stating out loud, because it is what separates this from classical strapdown INS: the Earth's rotation is neglected, $\mathbf{g}$ is taken constant in both magnitude and direction, and the world frame — normally the local-level frame at initialization — is treated as inertial. For MEMS-grade sensors over SLAM-scale distances and durations this is sound, since a MEMS gyro cannot observe Earth rate anyway. On a navigation-grade IMU, or over hundreds of kilometres, it is not, and you need the Coriolis and gravity-model terms that PX4 and ArduPilot carry in Chapter 2.

The IMU measures **specific force** — not acceleration — and angular rate in the body frame:

$$\tilde{\boldsymbol{\omega}}_t = \boldsymbol{\omega}_t + \mathbf{b}^g_t + \boldsymbol{\eta}^g_t$$

$$\tilde{\mathbf{a}}_t = \mathbf{R}_t^\top(\mathbf{a}_t - \mathbf{g}) + \mathbf{b}^a_t + \boldsymbol{\eta}^a_t$$

with biases modelled as Brownian motion (random walk):

$$\dot{\mathbf{b}}^g = \boldsymbol{\eta}^{bg}, \qquad \dot{\mathbf{b}}^a = \boldsymbol{\eta}^{ba}$$

Four noise parameters, all from **Allan variance**, not from the datasheet:

| Parameter | Symbol | Units |
|---|---|---|
| Gyro noise density | $\sigma_g$ | rad/s/$\sqrt{\text{Hz}}$ |
| Accel noise density | $\sigma_a$ | m/s²/$\sqrt{\text{Hz}}$ |
| Gyro bias random walk | $\sigma_{bg}$ | rad/s²/$\sqrt{\text{Hz}}$ |
| Accel bias random walk | $\sigma_{ba}$ | m/s³/$\sqrt{\text{Hz}}$ |

**Where these numbers come from.** Log a stationary IMU for several hours, compute the Allan deviation over averaging time $\tau$, and read the parameters off the slopes of a log-log plot:

$$\sigma_A^2(\tau) = \underbrace{\frac{N^2}{\tau}}_{\text{white noise}} + \underbrace{\frac{2\ln 2}{\pi}B^2}_{\text{bias instability}} + \underbrace{\frac{K^2\tau}{3}}_{\text{bias random walk}}$$

```
 σ_A(τ)
 [log]
   │
   │╲                                                          ╱
   │ ╲                                                        ╱
   │  ╲   slope −1/2                          slope +1/2     ╱
   │   ╲  WHITE NOISE                     BIAS RANDOM WALK  ╱
   │    ╲ N = σ_g , σ_a                    K = σ_bg , σ_ba ╱
   │     ╲                                                ╱
   │      ╲                                              ╱
   │       ╲                                            ╱
   │        ╲__                                      __╱
   │           ╲___                              ___╱
   │               ╲_____   slope 0        ____╱
   │                     ╲_______________╱
   │                        ▲ min = 0.664·B
   │                          BIAS INSTABILITY (flicker floor)
   │                          — NOT modelled by the ESKF —
   └────┬───────┬───────┬───────┬───────┬───────┬───────┬────► τ [log]
      0.01s   0.1s     1s      3s      10s    100s   1000s
                        ▲       ▲
                        │       └── read K here:  σ_A(3 s) = K
                        └────────── read N here:  σ_A(1 s) = N
```

How to read it physically: at **short $\tau$** you are averaging a handful of samples, so white noise dominates and averaging longer helps — the curve falls. At **long $\tau$** the bias has had time to wander, so averaging longer *hurts* — the curve rises. The minimum between them is the bias instability, the flicker floor, which a random-walk bias model does **not** represent. That mismatch is why $\sigma_{bg}$ read straight off the plot is usually optimistic in practice and often needs inflating by 2–10× in a real filter.

Discrete-time covariances scale as $\sigma^2/\Delta t$ for white noise and $\sigma^2 \Delta t$ for random walk:

```
  continuous density          discrete variance @ Δt         as Δt ↓ (faster IMU)
  ──────────────────          ──────────────────────         ────────────────────
  white noise   σ_g   ──────►      σ_g²  /  Δt          ──►   variance grows  ▲
                                                              (each sample averages
                                                               less noise away)

  random walk   σ_bg  ──────►      σ_bg² ·  Δt          ──►   variance shrinks ▼
                                                              (less time to drift)
```

Getting this inversion backwards is a classic bug — and its symptom is a filter that is beautifully tuned at one IMU rate and diverges at another.

## 1.2 Deriving the kinematics over $[i, j]$

Four steps: invert the sensor model, write the continuous-time ODEs, discretize, chain.

**Step 1 — invert the sensor model.** The IMU equations of §1.1 are written measurement-side. Solve each for the true quantity:

$$\boldsymbol{\omega}_t = \tilde{\boldsymbol{\omega}}_t - \mathbf{b}^g_t - \boldsymbol{\eta}^g_t$$

$$\tilde{\mathbf{a}}_t = \mathbf{R}_t^\top(\mathbf{a}_t - \mathbf{g}) + \mathbf{b}^a_t + \boldsymbol{\eta}^a_t \;\;\Longrightarrow\;\; \mathbf{a}_t = \mathbf{g} + \mathbf{R}_t(\tilde{\mathbf{a}}_t - \mathbf{b}^a_t - \boldsymbol{\eta}^a_t)$$

Read the second one physically: the accelerometer hands you a body-frame vector, $\mathbf{R}_t$ rotates it into the world, and adding $\mathbf{g}$ back undoes the $-\mathbf{g}$ the sensor applied. Gravity leaves the rotation and reappears as a standalone world-frame term — which is why $\mathbf{g}$ shows up unrotated in every equation from here on.

**Step 2 — continuous-time rigid-body kinematics.**

$$\dot{\mathbf{R}}_t = \mathbf{R}_t\lfloor\boldsymbol{\omega}_t\rfloor_\times, \qquad \dot{\mathbf{v}}_t = \mathbf{a}_t, \qquad \dot{\mathbf{p}}_t = \mathbf{v}_t$$

The first is where the convention of §0.2 bites. $\boldsymbol{\omega}$ is **body-resolved**, so the increment applies on the **right**:

$$\mathbf{R}_{t+dt} = \mathbf{R}_t\,\mathrm{Exp}(\boldsymbol{\omega}\,dt) \approx \mathbf{R}_t(\mathbf{I} + \lfloor\boldsymbol{\omega}\rfloor_\times dt) \;\Longrightarrow\; \frac{\mathbf{R}_{t+dt}-\mathbf{R}_t}{dt} = \mathbf{R}_t\lfloor\boldsymbol{\omega}\rfloor_\times$$

Had $\boldsymbol{\omega}$ been world-resolved you would get $\lfloor\boldsymbol{\omega}_W\rfloor_\times\mathbf{R}_t$ — left multiplication. The skew form is not a choice: differentiating the orthogonality constraint $\mathbf{R}^\top\mathbf{R}=\mathbf{I}$ gives $\dot{\mathbf{R}}^\top\mathbf{R} + \mathbf{R}^\top\dot{\mathbf{R}} = \mathbf{0}$, so $\mathbf{R}^\top\dot{\mathbf{R}}$ **must** be skew-symmetric, and every 3×3 skew matrix is $\lfloor\mathbf{v}\rfloor_\times$ for exactly one vector. That vector is the angular velocity. This is the same statement as $\mathfrak{so}(3) \cong \mathbb{R}^3$, and it is why a rotation carries 3 DoF rather than 9.

Substituting Step 1:

$$\dot{\mathbf{R}}_t = \mathbf{R}_t\lfloor\tilde{\boldsymbol{\omega}}_t - \mathbf{b}^g_t - \boldsymbol{\eta}^g_t\rfloor_\times, \qquad \dot{\mathbf{v}}_t = \mathbf{g} + \mathbf{R}_t(\tilde{\mathbf{a}}_t - \mathbf{b}^a_t - \boldsymbol{\eta}^a_t), \qquad \dot{\mathbf{p}}_t = \mathbf{v}_t$$

**Step 3 — discretize with zero-order hold.** Assume $\tilde{\boldsymbol{\omega}}$ and $\tilde{\mathbf{a}}$ constant over $[t,\,t+\Delta t]$.

*Rotation.* With constant $\boldsymbol{\omega}$, the ODE $\dot{\mathbf{R}}=\mathbf{R}\lfloor\boldsymbol{\omega}\rfloor_\times$ has the **exact** solution

$$\mathbf{R}_{t+\Delta t} = \mathbf{R}_t\,\mathrm{Exp}\big((\tilde{\boldsymbol{\omega}}_t - \mathbf{b}^g_t - \boldsymbol{\eta}^g_t)\Delta t\big)$$

That is where $\mathrm{Exp}$ comes from — not an approximation, but the matrix exponential solving a linear ODE on the group. It ceases to be exact only when $\boldsymbol{\omega}$ *rotates* within the interval, which is precisely what coning correction addresses.

*Velocity.* Integrate once, holding $\mathbf{R}_\tau \approx \mathbf{R}_t$:

$$\mathbf{v}_{t+\Delta t} = \mathbf{v}_t + \int_t^{t+\Delta t}\!\!\big[\mathbf{g} + \mathbf{R}_\tau(\cdot)\big]\,d\tau \approx \mathbf{v}_t + \mathbf{g}\Delta t + \mathbf{R}_t(\tilde{\mathbf{a}}_t - \mathbf{b}^a_t - \boldsymbol{\eta}^a_t)\Delta t$$

*Position.* Integrate twice; the double integral of a constant produces the $\tfrac{1}{2}(\cdot)\Delta t^2$ terms:

$$\mathbf{p}_{t+\Delta t} = \mathbf{p}_t + \mathbf{v}_t\Delta t + \tfrac{1}{2}\mathbf{g}\Delta t^2 + \tfrac{1}{2}\mathbf{R}_t(\tilde{\mathbf{a}}_t - \mathbf{b}^a_t - \boldsymbol{\eta}^a_t)\Delta t^2$$

The $\mathbf{R}_\tau\approx\mathbf{R}_t$ hold is the **only real approximation in the entire derivation**. Replacing it with $\tfrac{1}{2}(\mathbf{R}_t+\mathbf{R}_{t+\Delta t})$ is midpoint integration — same structure, better accuracy, free.

**Step 4 — chain from $i$ to $j$.** Apply the one-step maps repeatedly. Rotations **telescope** into a product; velocities and positions **sum**, with the gravity terms collecting because $\mathbf{g}$ is constant ($\sum_k\mathbf{g}\Delta t = \mathbf{g}\Delta t_{ij}$):

$$\mathbf{R}_j = \mathbf{R}_i \prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol{\omega}}_k - \mathbf{b}^g_k - \boldsymbol{\eta}^g_k)\Delta t\big)$$

$$\mathbf{v}_j = \mathbf{v}_i + \mathbf{g}\Delta t_{ij} + \sum_{k}\mathbf{R}_k(\tilde{\mathbf{a}}_k - \mathbf{b}^a_k - \boldsymbol{\eta}^a_k)\Delta t$$

$$\mathbf{p}_j = \mathbf{p}_i + \sum_k \left[\mathbf{v}_k\Delta t + \tfrac{1}{2}\mathbf{g}\Delta t^2 + \tfrac{1}{2}\mathbf{R}_k(\tilde{\mathbf{a}}_k - \mathbf{b}^a_k - \boldsymbol{\eta}^a_k)\Delta t^2\right]$$

Worth checking that the position gravity term collapses correctly. Substituting $\mathbf{v}_k = \mathbf{v}_i + \mathbf{g}(k-i)\Delta t + \dots$ with $n = j-i$:

$$\underbrace{\mathbf{g}\Delta t^2\frac{n(n-1)}{2}}_{\text{from } \sum_k \mathbf{v}_k\Delta t} + \underbrace{\mathbf{g}\Delta t^2\frac{n}{2}}_{\text{from } \sum_k \tfrac{1}{2}\mathbf{g}\Delta t^2} = \frac{\mathbf{g}\,n^2\Delta t^2}{2} = \tfrac{1}{2}\mathbf{g}\Delta t_{ij}^2 \;\;\checkmark$$

which is exactly the $\tfrac{1}{2}\mathbf{g}\Delta t_{ij}^2$ appearing on the left-hand side of the $\Delta\mathbf{p}_{ij}$ definition in §1.4.

**The telescoping is the whole point:**

```
              ┌──────────── ΔR_ij = R_iᵀ R_j ────────────┐
              │                                          │
   world ─── R_i ──[Exp]── R_i₊₁ ──[Exp]── R_i₊₂ ── … ── R_j
              ▲                                          ▲
              │                                          │
      states being optimized —        the bracketed product is not:
      they move every iteration       only IMU samples and the bias
```

**Assumptions made, in order of how much they matter:**

1. $\mathbf{R}_\tau \approx \mathbf{R}_k$ within each interval — the only genuine approximation. Fixed by midpoint or RK4.
2. $\tilde{\boldsymbol{\omega}}$ constant within each interval — exact for the rotation ODE, breaks under coning.
3. $\mathbf{b}_k \approx \mathbf{b}$ constant over the whole of $[i,j]$ — this is what justifies pulling the bias out of the sum, and it is exactly why the first-order bias-correction Jacobians of §1.6 exist: they patch the error when the bias estimate later moves.

## 1.3 The problem preintegration solves

Look at where $\mathbf{R}_k$ sits in the sums above. **Every term depends on the state at $i$.** In an optimizer, whenever $\mathbf{R}_i$ changes — every iteration — you must re-integrate all $N$ IMU samples between keyframes. At 200 Hz IMU and 10 Hz keyframes that is 20 samples × every variable × every iteration. Unacceptable.

## 1.4 Preintegrated measurements

Move the $i$-frame quantities to the left-hand side. Define **relative** increments that depend only on the IMU samples and the bias:

$$\boxed{\Delta\mathbf{R}_{ij} \triangleq \mathbf{R}_i^\top\mathbf{R}_j = \prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol{\omega}}_k - \mathbf{b}^g - \boldsymbol{\eta}^g_k)\Delta t\big)}$$

$$\boxed{\Delta\mathbf{v}_{ij} \triangleq \mathbf{R}_i^\top(\mathbf{v}_j - \mathbf{v}_i - \mathbf{g}\Delta t_{ij}) = \sum_{k}\Delta\mathbf{R}_{ik}(\tilde{\mathbf{a}}_k - \mathbf{b}^a - \boldsymbol{\eta}^a_k)\Delta t}$$

$$\boxed{\Delta\mathbf{p}_{ij} \triangleq \mathbf{R}_i^\top(\mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i\Delta t_{ij} - \tfrac{1}{2}\mathbf{g}\Delta t_{ij}^2) = \sum_k\left[\Delta\mathbf{v}_{ik}\Delta t + \tfrac{1}{2}\Delta\mathbf{R}_{ik}(\tilde{\mathbf{a}}_k - \mathbf{b}^a - \boldsymbol{\eta}^a_k)\Delta t^2\right]}$$

The right-hand sides contain **no** $\mathbf{R}_i, \mathbf{v}_i, \mathbf{p}_i$. Compute once, reuse across all optimizer iterations. That is the entire idea. (Lupton & Sukkarieh 2012 in Euler angles; Forster et al. 2015/2017 on-manifold, which is what GTSAM implements.)

## 1.5 Noise propagation

Separate the noise-free part $\Delta\bar{\mathbf{R}}$ from perturbation. Using the right-Jacobian BCH identity, the *measurement* equals the noise-free value perturbed by noise:

$$\Delta\tilde{\mathbf{R}}_{ij} = \Delta\bar{\mathbf{R}}_{ij}\,\mathrm{Exp}(\delta\boldsymbol{\phi}_{ij}), \quad \Delta\tilde{\mathbf{v}}_{ij} = \Delta\bar{\mathbf{v}}_{ij} + \delta\mathbf{v}_{ij}, \quad \Delta\tilde{\mathbf{p}}_{ij} = \Delta\bar{\mathbf{p}}_{ij} + \delta\mathbf{p}_{ij}$$

**Mind the direction of this definition.** All three must perturb the *same* way — measurement $=$ truth $\oplus$ noise. Forster (eq. 35–37) and Qiu's derivation write the algebraically identical inverse form, solving for the true value instead:

$$\Delta\mathbf{R}_{ij} = \Delta\tilde{\mathbf{R}}_{ij}\,\mathrm{Exp}(-\delta\boldsymbol{\phi}_{ij}), \quad \Delta\mathbf{v}_{ij} = \Delta\tilde{\mathbf{v}}_{ij} - \delta\mathbf{v}_{ij}, \quad \Delta\mathbf{p}_{ij} = \Delta\tilde{\mathbf{p}}_{ij} - \delta\mathbf{p}_{ij}$$

Writing $\mathrm{Exp}(-\delta\boldsymbol{\phi})$ alongside $+\,\delta\mathbf{v}$ — mixing the two forms in one line — silently flips the sign of $\delta\boldsymbol{\phi}$ relative to the $\mathbf{A}$ recursion below, and the resulting covariance is wrong in the rotation block only. It is a hard bug to see because $\boldsymbol{\Sigma}$ stays symmetric positive-definite and merely mis-weights.

The 9-dimensional noise vector $\boldsymbol{\eta}_{ij} = [\delta\boldsymbol{\phi}_{ij}, \delta\mathbf{v}_{ij}, \delta\mathbf{p}_{ij}]^\top$ propagates linearly:

$$\boldsymbol{\eta}_{ij} = \mathbf{A}_{j-1}\boldsymbol{\eta}_{ij-1} + \mathbf{B}_{j-1}\boldsymbol{\eta}^d_{j-1}$$

$$
\mathbf{A}_{k} = \begin{bmatrix}
\Delta\tilde{\mathbf{R}}_{k,k+1}^\top & \mathbf{0} & \mathbf{0}\\
-\Delta\tilde{\mathbf{R}}_{ik}\lfloor\tilde{\mathbf{a}}_k - \mathbf{b}^a\rfloor_\times\Delta t & \mathbf{I} & \mathbf{0}\\
-\tfrac{1}{2}\Delta\tilde{\mathbf{R}}_{ik}\lfloor\tilde{\mathbf{a}}_k - \mathbf{b}^a\rfloor_\times\Delta t^2 & \mathbf{I}\Delta t & \mathbf{I}
\end{bmatrix},
\quad
\mathbf{B}_k = \begin{bmatrix}
\mathbf{J}^k_r\Delta t & \mathbf{0}\\
\mathbf{0} & \Delta\tilde{\mathbf{R}}_{ik}\Delta t\\
\mathbf{0} & \tfrac{1}{2}\Delta\tilde{\mathbf{R}}_{ik}\Delta t^2
\end{bmatrix}
$$

$$\boldsymbol{\Sigma}_{ij} = \mathbf{A}_{j-1}\boldsymbol{\Sigma}_{ij-1}\mathbf{A}_{j-1}^\top + \mathbf{B}_{j-1}\boldsymbol{\Sigma}^\eta\mathbf{B}_{j-1}^\top$$

This 9×9 (or 15×15 in the "combined" variant that carries bias) covariance becomes the noise model of the factor. It grows without bound with $\Delta t_{ij}$, which is *correct* — it's why a long gap between keyframes automatically down-weights the IMU factor without any special-casing.

## 1.6 Bias correction — the second key idea

$\Delta\bar{\mathbf{R}}, \Delta\bar{\mathbf{v}}, \Delta\bar{\mathbf{p}}$ were computed at a linearization bias $\bar{\mathbf{b}}$. The optimizer will change the bias estimate. Rather than re-integrate, store first-order Jacobians during integration and apply a linear correction:

$$\Delta\bar{\mathbf{R}}_{ij}(\mathbf{b}^g) \approx \Delta\bar{\mathbf{R}}_{ij}(\bar{\mathbf{b}}^g)\,\mathrm{Exp}\!\left(\frac{\partial\Delta\bar{\mathbf{R}}_{ij}}{\partial\mathbf{b}^g}\delta\mathbf{b}^g\right)$$

$$\Delta\bar{\mathbf{v}}_{ij}(\mathbf{b}) \approx \Delta\bar{\mathbf{v}}_{ij}(\bar{\mathbf{b}}) + \frac{\partial\Delta\bar{\mathbf{v}}_{ij}}{\partial\mathbf{b}^g}\delta\mathbf{b}^g + \frac{\partial\Delta\bar{\mathbf{v}}_{ij}}{\partial\mathbf{b}^a}\delta\mathbf{b}^a$$

$$\Delta\bar{\mathbf{p}}_{ij}(\mathbf{b}) \approx \Delta\bar{\mathbf{p}}_{ij}(\bar{\mathbf{b}}) + \frac{\partial\Delta\bar{\mathbf{p}}_{ij}}{\partial\mathbf{b}^g}\delta\mathbf{b}^g + \frac{\partial\Delta\bar{\mathbf{p}}_{ij}}{\partial\mathbf{b}^a}\delta\mathbf{b}^a$$

These five Jacobians propagate incrementally alongside the mean and covariance (recursions in Forster §III-C, implemented in §1.8 below). In closed form they are:

$$\frac{\partial\Delta\bar{\mathbf{R}}_{ij}}{\partial\mathbf{b}^g} = -\sum_{k=i}^{j-1}\Delta\bar{\mathbf{R}}_{k+1,j}^\top\,\mathbf{J}_r^k\,\Delta t$$

$$\frac{\partial\Delta\bar{\mathbf{v}}_{ij}}{\partial\mathbf{b}^a} = -\sum_{k=i}^{j-1}\Delta\bar{\mathbf{R}}_{ik}\Delta t, \qquad \frac{\partial\Delta\bar{\mathbf{v}}_{ij}}{\partial\mathbf{b}^g} = -\sum_{k=i}^{j-1}\Delta\bar{\mathbf{R}}_{ik}\lfloor\tilde{\mathbf{a}}_k-\bar{\mathbf{b}}^a\rfloor_\times\frac{\partial\Delta\bar{\mathbf{R}}_{ik}}{\partial\mathbf{b}^g}\Delta t$$

$$\frac{\partial\Delta\bar{\mathbf{p}}_{ij}}{\partial\mathbf{b}^a} = \sum_{k=i}^{j-1}\left[\frac{\partial\Delta\bar{\mathbf{v}}_{ik}}{\partial\mathbf{b}^a}\Delta t - \tfrac{1}{2}\Delta\bar{\mathbf{R}}_{ik}\Delta t^2\right], \qquad \frac{\partial\Delta\bar{\mathbf{p}}_{ij}}{\partial\mathbf{b}^g} = \sum_{k=i}^{j-1}\left[\frac{\partial\Delta\bar{\mathbf{v}}_{ik}}{\partial\mathbf{b}^g}\Delta t - \tfrac{1}{2}\Delta\bar{\mathbf{R}}_{ik}\lfloor\tilde{\mathbf{a}}_k-\bar{\mathbf{b}}^a\rfloor_\times\frac{\partial\Delta\bar{\mathbf{R}}_{ik}}{\partial\mathbf{b}^g}\Delta t^2\right]$$

Note the nesting: $\partial\Delta\bar{\mathbf{p}}/\partial\mathbf{b}$ is defined in terms of $\partial\Delta\bar{\mathbf{v}}/\partial\mathbf{b}$, which is itself defined in terms of $\partial\Delta\bar{\mathbf{R}}/\partial\mathbf{b}^g$. **That dependency chain dictates the update order in code** — position Jacobians first, then velocity, then rotation, each consuming the *previous* step's value. Update $\partial\Delta\bar{\mathbf{R}}/\partial\mathbf{b}^g$ first and every downstream Jacobian is one step out of date, which produces a bias correction that is subtly wrong only for large $\|\delta\mathbf{b}\|$ — i.e. exactly when you need it. The pseudocode in §1.8 is written in this order deliberately.

**Re-integrate fully only when $\|\delta\mathbf{b}\|$ exceeds a threshold** (GTSAM exposes `biasAccOmegaInt` and repropagation control). This is the practical knob: too loose and the linear correction is invalid, too tight and you lose the performance benefit.

## 1.7 Residuals

$$\mathbf{r}_{\Delta\mathbf{R}_{ij}} = \mathrm{Log}\!\left(\left[\Delta\tilde{\mathbf{R}}_{ij}(\bar{\mathbf{b}}^g)\,\mathrm{Exp}\!\left(\tfrac{\partial\Delta\bar{\mathbf{R}}_{ij}}{\partial\mathbf{b}^g}\delta\mathbf{b}^g\right)\right]^\top \mathbf{R}_i^\top\mathbf{R}_j\right)$$

$$\mathbf{r}_{\Delta\mathbf{v}_{ij}} = \mathbf{R}_i^\top(\mathbf{v}_j - \mathbf{v}_i - \mathbf{g}\Delta t_{ij}) - \Delta\tilde{\mathbf{v}}_{ij}(\mathbf{b})$$

$$\mathbf{r}_{\Delta\mathbf{p}_{ij}} = \mathbf{R}_i^\top\left(\mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i\Delta t_{ij} - \tfrac{1}{2}\mathbf{g}\Delta t_{ij}^2\right) - \Delta\tilde{\mathbf{p}}_{ij}(\mathbf{b})$$

Plus a bias random-walk factor between consecutive bias nodes:

$$\mathbf{r}_b = \mathbf{b}_j - \mathbf{b}_i, \qquad \boldsymbol{\Sigma}_b = \Delta t_{ij}\,\mathrm{diag}(\sigma_{bg}^2\mathbf{I}, \sigma_{ba}^2\mathbf{I})$$

The IMU factor is therefore a **15-dimensional residual** connecting $\{\mathbf{R}_i,\mathbf{p}_i,\mathbf{v}_i,\mathbf{b}_i\}$ and $\{\mathbf{R}_j,\mathbf{p}_j,\mathbf{v}_j,\mathbf{b}_j\}$ — six variables in GTSAM's `ImuFactor` + `BetweenFactor<Bias>`, or four in `CombinedImuFactor` which folds the bias evolution in.

**The analytic Jacobians in full.** These are taken with respect to the *increments* used to lift each state, under the right perturbation $\mathbf{R} \leftarrow \mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol{\phi})$:

$$\mathbf{R}\leftarrow\mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol{\phi}), \qquad \mathbf{p}\leftarrow\mathbf{p}+\mathbf{R}\,\delta\mathbf{p}, \qquad \mathbf{v}\leftarrow\mathbf{v}+\delta\mathbf{v}, \qquad \mathbf{b}\leftarrow\mathbf{b}+\delta\mathbf{b}$$

| | $\delta\boldsymbol{\phi}_i$ | $\delta\mathbf{p}_i$ | $\delta\mathbf{v}_i$ | $\delta\boldsymbol{\phi}_j$ | $\delta\mathbf{p}_j$ | $\delta\mathbf{v}_j$ | $\delta\mathbf{b}^g_i$ | $\delta\mathbf{b}^a_i$ |
|---|---|---|---|---|---|---|---|---|
| $\mathbf{r}_{\Delta\mathbf{R}}$ | $-\mathbf{J}_r^{-1}(\mathbf{r}_{\Delta\mathbf{R}})\mathbf{R}_j^\top\mathbf{R}_i$ | $\mathbf{0}$ | $\mathbf{0}$ | $\mathbf{J}_r^{-1}(\mathbf{r}_{\Delta\mathbf{R}})$ | $\mathbf{0}$ | $\mathbf{0}$ | see below | $\mathbf{0}$ |
| $\mathbf{r}_{\Delta\mathbf{v}}$ | $\lfloor\mathbf{R}_i^\top(\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\Delta t_{ij})\rfloor_\times$ | $\mathbf{0}$ | $-\mathbf{R}_i^\top$ | $\mathbf{0}$ | $\mathbf{0}$ | $\mathbf{R}_i^\top$ | $-\frac{\partial\Delta\bar{\mathbf{v}}_{ij}}{\partial\mathbf{b}^g}$ | $-\frac{\partial\Delta\bar{\mathbf{v}}_{ij}}{\partial\mathbf{b}^a}$ |
| $\mathbf{r}_{\Delta\mathbf{p}}$ | $\lfloor\mathbf{R}_i^\top(\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\Delta t_{ij}-\tfrac{1}{2}\mathbf{g}\Delta t_{ij}^2)\rfloor_\times$ | $-\mathbf{I}$ | $-\mathbf{R}_i^\top\Delta t_{ij}$ | $\mathbf{0}$ | $\mathbf{R}_i^\top\mathbf{R}_j$ | $\mathbf{0}$ | $-\frac{\partial\Delta\bar{\mathbf{p}}_{ij}}{\partial\mathbf{b}^g}$ | $-\frac{\partial\Delta\bar{\mathbf{p}}_{ij}}{\partial\mathbf{b}^a}$ |

The gyro-bias column of $\mathbf{r}_{\Delta\mathbf{R}}$ is the only genuinely awkward one, because the bias enters *inside* a $\mathrm{Log}$ through another $\mathrm{Exp}$:

$$\frac{\partial\mathbf{r}_{\Delta\mathbf{R}}}{\partial\delta\mathbf{b}^g_i} = -\mathbf{J}_r^{-1}(\mathbf{r}_{\Delta\mathbf{R}})\,\mathrm{Exp}(-\mathbf{r}_{\Delta\mathbf{R}})\,\mathbf{J}_r\!\left(\frac{\partial\Delta\bar{\mathbf{R}}_{ij}}{\partial\mathbf{b}^g}\delta\mathbf{b}^g_i\right)\frac{\partial\Delta\bar{\mathbf{R}}_{ij}}{\partial\mathbf{b}^g}$$

**Why $\mathbf{p}$ is lifted as $\mathbf{p}+\mathbf{R}\,\delta\mathbf{p}$, and why it matters.** That convention is not arbitrary — it is what you get by right-multiplying the pose matrix $\mathbf{T}_i = \begin{bmatrix}\mathbf{R}_i & \mathbf{p}_i\\ \mathbf{0} & 1\end{bmatrix}$ by a perturbation $\delta\mathbf{T}_i$, which gives $\mathbf{R}_i\delta\mathbf{R}_i$ and $\mathbf{p}_i + \mathbf{R}_i\delta\mathbf{p}_i$ together. It keeps the increment body-resolved and consistent with the right perturbation already used for rotation, so $\delta\mathbf{p}$ means "displacement expressed in the body frame."

The payoff is the clean $-\mathbf{I}$ and $\mathbf{R}_i^\top\mathbf{R}_j$ entries above. **If your code instead lifts position additively in the world frame** ($\mathbf{p}\leftarrow\mathbf{p}+\delta\mathbf{p}$, which is what a naive `Vector3` state does), those two entries become $-\mathbf{R}_i^\top$ and $+\mathbf{R}_i^\top$. Both conventions are correct; mixing the analytic Jacobian of one with the retraction of the other is a silent, direction-dependent convergence bug — the optimizer still descends, just along the wrong metric, so it converges slowly rather than failing outright.

## 1.8 Pseudocode

```
struct Preintegrated:
    dR = I3;  dV = 0;  dP = 0;          # mean
    Sigma = zeros(9,9)                   # covariance (or 15x15 combined)
    J_dR_bg = 0; J_dV_bg = 0; J_dV_ba = 0
    J_dP_bg = 0; J_dP_ba = 0             # bias Jacobians
    dt_total = 0
    b_bar = (bg_bar, ba_bar)             # linearization bias

integrate_measurement(P, w_tilde, a_tilde, dt):
    w = w_tilde - P.b_bar.bg
    a = a_tilde - P.b_bar.ba
    dR_k  = Exp(w * dt)
    Jr_k  = right_jacobian(w * dt)

    # --- mean (order matters: use OLD dR for v/p, then update dR) ---
    P.dP += P.dV*dt + 0.5*P.dR*a*dt*dt
    P.dV += P.dR*a*dt
    P.dR  = normalize(P.dR * dR_k)

    # --- covariance ---
    A = build_A(P.dR_old, a, dR_k, dt)   # 9x9, see §1.5
    B = build_B(P.dR_old, Jr_k, dt)      # 9x6
    P.Sigma = A @ P.Sigma @ A.T + B @ Sigma_eta @ B.T

    # --- bias Jacobians (recursions) ---
    P.J_dP_ba += P.J_dV_ba*dt - 0.5*P.dR_old*dt*dt
    P.J_dP_bg += P.J_dV_bg*dt - 0.5*P.dR_old*skew(a)*P.J_dR_bg*dt*dt
    P.J_dV_ba += -P.dR_old*dt
    P.J_dV_bg += -P.dR_old*skew(a)*P.J_dR_bg*dt
    P.J_dR_bg  = dR_k.T @ P.J_dR_bg - Jr_k*dt

    P.dt_total += dt

corrected(P, b_new):
    d_bg = b_new.bg - P.b_bar.bg
    d_ba = b_new.ba - P.b_bar.ba
    if norm(d_bg) > TH_G or norm(d_ba) > TH_A:
        return repropagate(P, b_new)     # full re-integration
    dR = P.dR * Exp(P.J_dR_bg @ d_bg)
    dV = P.dV + P.J_dV_bg@d_bg + P.J_dV_ba@d_ba
    dP = P.dP + P.J_dP_bg@d_bg + P.J_dP_ba@d_ba
    return (dR, dV, dP)
```

Two implementation notes that cost people days:
- **Use midpoint (or RK4) integration**, not Euler, for $\tilde{\mathbf{a}}$ and $\tilde{\boldsymbol{\omega}}$ between samples. VINS-Mono uses midpoint. The accuracy gain is free. Note that Forster's derivation as published *is* Euler — plain zero-order hold, not the higher-order coning/sculling schemes of classical strapdown INS — so this is an implementation upgrade over the paper, not a restatement of it. Swapping in midpoint changes only which $\Delta\bar{\mathbf{R}}_{ik}$ enters each sum; the $\mathbf{A}$/$\mathbf{B}$ structure is untouched.
- **Re-orthonormalize `dR` periodically.** Repeated matrix products drift off $SO(3)$ in float. Quaternion normalization is the usual fix.

## 1.9 Architecture

```
   IMU 200-1000 Hz
        │
        ▼
 ┌──────────────┐   raw (ω̃, ã, t)
 │ IMU ring buf │───────────────────────────────┐
 └──────┬───────┘                               │
        │                                       │  (for deskewing,
        ▼                                       │   image-time interp,
 ┌────────────────────────────────────┐         │   high-rate output)
 │  Preintegrator                     │         │
 │  ┌──────────────────────────────┐  │         │
 │  │ mean:  ΔR, Δv, Δp            │  │         │
 │  │ cov:   Σ (9x9 / 15x15)       │  │         │
 │  │ jac:   ∂Δ/∂b_g, ∂Δ/∂b_a      │  │         │
 │  └──────────────────────────────┘  │         │
 └────────────────┬───────────────────┘         │
                  │ on keyframe trigger         │
                  ▼                             │
        ┌───────────────────┐                   │
        │  ImuFactor(i,j)   │                   │
        └─────────┬─────────┘                   │
                  ▼                             │
   ╔══════════════════════════════════╗         │
   ║  Factor graph  /  sliding window ║         │
   ║  x_i ──[IMU]── x_j ──[IMU]── x_k ║         │
   ║   │            │            │    ║         │
   ║ [vision/lidar factors]           ║         │
   ╚══════════════┬═══════════════════╝         │
                  │ optimized state @ keyframe  │
                  ▼                             ▼
            ┌───────────────────────────────────────┐
            │  IMU forward-propagation to now       │──► high-rate
            │  (re-integrate from last KF state)    │    odometry out
            └───────────────────────────────────────┘
```

The right-hand path is essential and often forgotten: the optimizer produces a state at the *last keyframe*, which is 50–100 ms stale. The controller needs a state *now*. You forward-propagate the raw IMU from the optimized keyframe state. This is exactly the same architectural idea as PX4's output predictor in Chapter 2.
