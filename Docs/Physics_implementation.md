# Physics Implementation Theory — Gravitron Orbit–Dive Robot

This document explains the **physics theory** behind the impact-and-recovery behavior we implement in the simulator. It’s written so a reader can understand (and re-implement) the same mechanics in any codebase.

---

## 1) State & Notation

For each dynamic entity (robot / opponent / projectile), we maintain:

- Mass $m$, radius $r$ (disk model for collisions)  
- Position $mathbf{p} = (x, y)$, velocity $mathbf{v} = (v_x, v_y)$  
- (Optional) Orientation $theta$, angular velocity $omega$, moment of inertia $I$ — only needed if we want **spin**

**Target-relative vectors** (useful for orbit control):

- $mathbf{r} = \mathbf{p} - \mathbf{p}_t$, distance $r = \|\mathbf{r}\|$  
- Unit radial $ \mathbf{e}_r = \mathbf{r} / r $, unit tangent $ \mathbf{e}_t = (-e_{r,y}, e_{r,x}) $ (CCW)

---

## 2) Time Integration

We use **semi-implicit (symplectic) Euler**, stable and common in games:

\[
\mathbf{v} \leftarrow \mathbf{v} + \frac{\mathbf{F}_\text{total}}{m}\,\Delta t,\qquad
\mathbf{p} \leftarrow \mathbf{p} + \mathbf{v}\,\Delta t
\]

with optional **linear drag** for stability:
\[
\mathbf{F}_\text{drag} = -c_d\,\mathbf{v}.
\]

No gravity (top-down). Choose consistent units (e.g., pixels as length units).

---

## 3) Controller Forces (Orbit & Dive emerge from physics)

We don’t teleport to the orbit; we **apply forces** that *produce* orbiting and diving:

- Radial velocity: \( v_r = \mathbf{v}\cdot \mathbf{e}_r \)  
- Tangential velocity: \( v_t = \mathbf{v}\cdot \mathbf{e}_t \)

**Radial PD (keep distance \(R\))**
\[
\mathbf{F}_r = m\left[-k_{pr}(r - R) - k_{dr}\,v_r\right]\mathbf{e}_r
\]

**Tangential P (maintain tangential speed \(v_{t,\mathrm{des}}\))**
\[
\mathbf{F}_t = m\left[-k_{pt}(v_t - v_{t,\mathrm{des}})\right]\mathbf{e}_t
\]

**Centripetal feedforward (optional)**
\[
\mathbf{F}_{\mathrm{ff}} = m\,\frac{v_{t,\mathrm{des}}^2}{R}\,(-\mathbf{e}_r).
\]

Total control force:
\[
\mathbf{F}_\text{ctrl} = \mathbf{F}_r + \mathbf{F}_t + \mathbf{F}_\mathrm{ff},\qquad \|\mathbf{F}_\text{ctrl}\|\le F_\mathrm{max}.
\]

- **Dive:** temporarily set \(R_\mathrm{des}=0\) and \(v_{t,\mathrm{des}}=0\), then restore.  
- **Flip direction:** set the sign of \(v_{t,\mathrm{des}}\) (CW/CCW).

---

## 4) Collisions — Linear Impulses (No Spin)

We treat each body as a **disk / point-mass** for impact. That is sufficient to get “thrown apart” behavior.

Let A be the robot, B be another robot (or the wall). Denote the **contact normal** \(\mathbf{n}\) as the unit vector from B to A at the contact.

- Relative velocity: \(\mathbf{v}_{rel} = \mathbf{v}_A - \mathbf{v}_B\)  
- Normal component: \( v_n = \mathbf{v}_{rel}\cdot\mathbf{n} \)

If \(v_n > 0\) (separating), **no impulse**.

**Normal impulse** with restitution \(e\in[0,1)\):
\[
j = -\frac{(1+e)\,v_n}{\frac{1}{m_A} + \frac{1}{m_B}}.
\]

Apply to the velocities:
\[
\mathbf{v}_A' = \mathbf{v}_A + \frac{j}{m_A}\mathbf{n},\qquad
\mathbf{v}_B' = \mathbf{v}_B - \frac{j}{m_B}\mathbf{n}.
\]

**Friction (tangent) with Coulomb clamp:** Let \(\mathbf{t}=(-n_y, n_x)\). Tangential relative speed \( v_t = \mathbf{v}_{rel}\cdot\mathbf{t} \).

\[
j_t = \mathrm{clamp}\!\left( -\frac{v_t}{\frac{1}{m_A} + \frac{1}{m_B}},\; -\mu|j|,\; \mu|j| \right),
\]
\[
\mathbf{v}_A' \mathrel{+}= \frac{j_t}{m_A}\mathbf{t},\qquad
\mathbf{v}_B' \mathrel{-}= \frac{j_t}{m_B}\mathbf{t}.
\]

**Positional correction** (to remove overlap / prevent sticking). If penetration depth is
\[
\text{pen} = (r_A + r_B) - \|\mathbf{p}_A - \mathbf{p}_B\| > 0,
\]
then push bodies apart along \(\mathbf{n}\):
\[
\Delta \mathbf{p} = \beta \,\frac{\max(\text{pen} - \text{slop}, 0)}{\frac{1}{m_A} + \frac{1}{m_B}}\,\mathbf{n},
\]
\[
\mathbf{p}_A \mathrel{+}= \frac{1}{m_A}\Delta \mathbf{p},\qquad
\mathbf{p}_B \mathrel{-}= \frac{1}{m_B}\Delta \mathbf{p},
\]
with typical values \(\beta\approx 0.2\), \(\text{slop}\approx 0.01\).

> **Walls** are handled as B with **infinite mass** \((1/m_B=0)\); B’s velocity is zero at impact. Use the same formulas; apply correction along the wall normal and clamp \(x\) or \(y\) to the boundary.

---

## 5) Angular Momentum About the Target (Why linear impulse is enough)

Although the orbiting robot’s **angle** changes constantly, we **do not** need an explicit “moment of impulse” to compute collision response. A **linear impulse** \(\mathbf{J}\) at contact automatically changes the robot’s **angular momentum about the target**:

\[
L_\text{about target} = m\,(\mathbf{r}\times \mathbf{v}),\qquad
\Delta L = \mathbf{r}\times \mathbf{J},
\]

where \(\mathbf{r}=\mathbf{p}-\mathbf{p}_t\). Thus, updating linear velocity via the impulse already updates \(L\) correctly.

Only if we want the robot to **spin around its own center** (orientation dynamics) do we add **angular impulses** (see §6).

---

## 6) (Optional) Rotational Impulse (Spin)

If modeling **body rotation**:  
Let \(\mathbf{r}_c\) be the lever arm from center to contact on body A. With moment of inertia \(I_A\) (disk: \(I = \tfrac12 m r^2\)), the **angular velocity** update is:

\[
\Delta \omega_A = \frac{ (\mathbf{r}_c \times \mathbf{J}) }{ I_A }.
\]

The full rigid-body impulse also modifies the denominator to include rotational terms. We avoid this complexity unless spin is required visually/physically.

---

## 7) “Stun and Recover” After Impact (Game Feel)

To show loss of balance after an impact and a **smooth return** to orbit:

1. When an impact occurs, start a **stun timer** \(T_s\) (e.g., \(0.2\)–\(0.5\) s).  
2. While stunned, **scale down** controller force: \(\mathbf{F}_\text{ctrl} \leftarrow \alpha\,\mathbf{F}_\text{ctrl}\), \(0<\alpha<1\).  
3. Drag dissipates excess speed; when stun ends, the controller naturally **re-acquires** the orbit:  
   - Recompute \(\mathbf{e}_r\) and \(\mathbf{e}_t\) from current \(\mathbf{p}\) and \(\mathbf{p}_t\).  
   - Set orbit direction from the sign of \( \mathbf{v}\cdot\mathbf{e}_t \).  
   - Use the forces in §3 to pull \(r\to R\) and \(v_t\to v_{t,\mathrm{des}}\).

---

## 8) Walls — Impulse With Friction (Glide is Emergent)

For an axis-aligned box with left/right/top/bottom boundaries:

- Detect contact when \(x-r<\text{left}\), \(x+r>\text{right}\) (and similarly for \(y\)).  
- Apply **normal impulse** along the wall normal, **friction** along tangent, and **positional correction** to keep the robot inside the arena.  
- With appropriate friction and drag, **gliding** along the wall emerges naturally (no teleport).

---

## 9) Putting It Together (Per-frame Order of Operations)

1. **Controller force** (orbit/dive) → compute \(\mathbf{F}_\text{ctrl}\) from §3.  
2. Add **drag** (and optional soft wall force).  
3. **Integrate** velocities/positions with symplectic Euler (Δt from the clock).  
4. **Collisions:**  
   - Robot ↔ walls (impulses + correction)  
   - Robot ↔ opponent (impulses + correction)  
5. If any collision occurred → **stun timer**.  
6. **Re-acquire orbit direction** from tangential velocity sign; continue.

---

## 10) Recommended Parameters (Starting Point)

- Mass \(m = 1.0\), radius \(r = 6\) px  
- Drag \(c_d = 3\)–\(6\)  
- Restitution \(e = 0.2\)–\(0.3\), friction \(\mu = 0.5\)–\(0.7\)  
- Positional correction: \(\beta=0.2\), \(\text{slop}=0.01\)  
- Controller gains: \(k_{pr}=10\), \(k_{dr}=6\), \(k_{pt}=6\); force cap \(F_\mathrm{max}=1200\)  
- Stun time \(T_s = 0.3\) s, scale \(\alpha=0.5\)

Tune in this order: **drag → restitution/friction → controller gains → force cap**.

---

## 11) Why This Works for Orbiting

Even though the robot’s **angle** (orbital phase) changes every frame, treating impacts with **linear impulses** is correct and efficient. The change in **linear momentum** instantly updates the **angular momentum about the target**, giving you natural “throw-and-recover” behavior. The controller then re-establishes the desired circular motion without teleportation.

---

## 12) Extensions

- Add rotational dynamics for spin & torque-limited thrust.  
- Switch to a physics engine (e.g., `pymunk`) for richer contact models.  
- Add noise/latency to match real sensors/actuators.  
- Multi-robot impacts and team strategies (phase separation).

---

**References (conceptual):**
- Game Physics (symplectic Euler, impulses): common practice in 2D engines (Box2D/Chipmunk)  
- Classical Mechanics: linear impulse–momentum, angular momentum about a point
