# Gravitron: Orbit–Dive Fighting Robot Project

A cross-disciplinary project to build a **fighting robot** that normally **orbits** an opponent at a fixed radius/speed, can **reverse orbit direction** on command, and can **dive/attack** when triggered.  
**Today:** robust 2D simulation of motion and maneuvers.  
**Next:** a simple **game** with scoring and a starter bot.  
**Then:** on-robot control powered by our **Vortex AI**.

---

## Table of Contents
- [Project synopsis](#project-synopsis)
- [Team & ownership](#team--ownership)
- [Current status](#current-status)
- [Robot movement model](#robot-movement-model)
  - [Finite-state machine](#finite-state-machine)
  - [Orbit control](#orbit-control)
  - [Dive & return](#dive--return)
  - [Wall safety: bounce & corner-glide](#wall-safety-bounce--corner-glide)
- [Controls (simulation)](#controls-simulation)
- [Key parameters](#key-parameters)
- [Recent simulation work](#recent-simulation-work)
- [Game plan (next step)](#game-plan-next-step)
- [Vortex AI plan](#vortex-ai-plan)
  - [Observations](#observations)
  - [Actions](#actions)
  - [Rewards](#rewards)
  - [Training phases](#training-phases)
- [Testing scenarios](#testing-scenarios)
- [Engineering notes & bug log](#engineering-notes--bug-log)
- [Roadmap](#roadmap)
- [Contributing](#contributing)

---

## Project synopsis

- **Goal:** Compete with a robot that **maintains a fixed-radius orbit** around a target, can **change orbit direction**, and can **dive to attack** on command—while **never getting trapped by walls/corners**.  
- **AI:** **Vortex AI** will keep the orbit stable and make decisions (direction flips, when to dive).  
- **Scope:** Start in sim → make a minimal game for fast iteration → port control to the real robot.

---

## Team & ownership

- **Computer Engineering:** simulation, game layer, **Vortex AI** model(s), data pipelines.  
- **Controls Engineer:** control laws, hardware actuation mapping, safety interlocks.  
- **Mechanical Engineers (2):** chassis, drivetrain, sensors/placements, integration & testing.

---

## Current status

- ✅ **Simulation** of orbit/dive with robust wall handling:
  - Deterministic **single-wall bounce** (mirror the predicted next position).
  - Corner survival via **WALL_GLIDE** (slide along wall, then rejoin the circle).
  - Fixed radius & tangential speed; dive in/out with radial speed.
  - Time-step stable kinematics; small hysteresis to avoid state flapping.
- ✅ Project log (Obsidian-friendly) describing bugs → fixes → decisions.
- 🔜 **Game v0**: scoring, lives, sprites/SFX, and a “starter bot.”
- 🔜 **Vortex AI**: behavior-cloned baseline → reinforcement learning policy.

---

## Robot movement model

### Finite-state machine

```
ORBIT --(command: dive)--> INWARD --(r ≤ ε)--> OUTWARD --(r ≈ R)--> ORBIT
                           \--(single wall)--> Bounce (same frame, no teleport)
               --(corner)-------> WALL_GLIDE --(safe)--> ORBIT
```

- **ORBIT:** hold radius `R`, rotate with angular speed `ω = v_t / R`.
- **INWARD:** move radially toward target at speed `v_r`.
- **OUTWARD:** move radially back to `R` at speed `v_r`.
- **WALL_GLIDE (corner only):** slide along the wall at tangential speed until a safe step onto the circle is possible again.

### Orbit control

Let the unit vector **u** from target→robot define the phase on the circle:

- Advance phase by `Δθ = ω · dir · dt` (`dir ∈ {+1, −1}`).
- Normalize **u** each step (prevents drift); position = `target + R · u`.

### Dive & return

- **INWARD:** `r ← max(0, r − v_r·dt)`; switch to OUTWARD when near center.
- **OUTWARD:** `r ← min(R, r + v_r·dt)`; snap to ORBIT when close to `R`.

### Wall safety: bounce & corner-glide

- **Bounce (single wall):** predict next position `p_nom`. If it violates exactly one boundary, **reflect `p_nom` across that wall**, recompute **u** from `target→p_ref`, and **reverse the sweep direction**. This yields a clean semicircle bounce **without teleporting**.
- **Corner-glide (both axes violated):** pin to the contacting wall and **slide along it** in the direction of the current tangent (deterministic choice). Rejoin the circle as soon as a fixed-radius step is safely inside the box.

---

## Getting started

```bash
# Clone
git clone https://github.com/<org-or-user>/<repo>.git
cd <repo>

# (Optional) Virtual env
python -m venv .venv
# Windows: .venv\Scriptsctivate
source .venv/bin/activate

# Install
pip install -r requirements.txt

# Run the sim
python Simulation_bounce_with_corner_glide_nocomments.py
```

---

## Controls (simulation)

| Key     | Action                                          |
|---------|-------------------------------------------------|
| Arrows  | Move target (red) for testing                   |
| SPACE   | Dive (INWARD → OUTWARD → ORBIT)                 |
| C       | Flip orbit direction (CW/CCW)                   |
| D       | Toggle debug HUD                                |
| ESC     | Quit                                            |

---

## Key parameters

| Name               | Meaning                            | Default |
|--------------------|------------------------------------|---------|
| `ORBIT_RADIUS`     | Desired standoff distance          | 120 px  |
| `TANGENTIAL_SPEED` | Linear speed along the circle      | 140 px/s|
| `ANGULAR_SPEED`    | Derived: `v_t / R`                 | —       |
| `DIVE_SPEED`       | Radial in/out speed                | 320 px/s|
| `SAFETY_MARGIN`    | Clearance from walls               | 12 px   |
| `EPS`              | Hysteresis near thresholds         | 1.0     |
| `FPS`              | Render cap / dt cadence            | 60      |

---

## Recent simulation work

**Stability & clarity**
- Converted to **dt-stable** speeds (no frame-rate coupling).
- Added FSM **hysteresis** (`EPS`) to prevent flapping at thresholds.
- Consistent unit-vector **normalization** to avoid angle drift/jitter.

**Bounce without teleport**
- Reflect **predicted position** (not direction), recompute phase, **no double rotation** in the same frame; reverse sweep for a natural semicircle.

**Corner survival**
- **WALL_GLIDE**: slide along wall (no teleport) until a fixed-radius step is feasible; deterministic wall/axis choice from orbit tangent.

**Usability**
- Debug HUD (state, radius, angle, orbit dir, counters).
- Clean “no comments” baseline for integration into game layer.

For a detailed changelog & rationale, see `orbit_dive_project_log.md`.

---

## Game plan (next step)

We’ll evolve the sim into **Game v0**:

- **States:** MENU → PLAYING → PAUSED → GAME_OVER  
- **Scoring:**  
  - +1/sec inside orbit band (`|r − R| ≤ band`)  
  - +N on successful dive “hit” (within `hit_radius`)  
  - −N when forced into WALL_GLIDE or if a dive misses  
- **Lives/Timer:** lose a life if cornered too long or repeated misses  
- **Juice:** sprites (robot/target), spark on hit, simple SFX, trails  
- **Env shim:** the game loop calls the same step function used for AI

---

## Vortex AI plan

**Purpose:** keep stable orbit, choose when to flip orbit direction, and decide when to dive.

### Observations
- `(cosθ, sinθ)` of orbit phase  
- `r` and `r_err = r − R`  
- Distances to four walls (normalized)  
- Current FSM state (one-hot)  
- Optional: target velocity (if available on robot)

### Actions
Start discrete (simpler, robust):
- `KEEP` (no change), `FLIP_DIR`, `DIVE`, `FASTER`, `SLOWER`  
Later, move to continuous controls if needed.

### Rewards
- `+0.01` per step while within orbit band  
- `+1.0` on valid hit  
- `−0.02` while in boundary contact; `−0.2` for corner-glide  
- Small action penalty to reduce twitchiness  
- Episode penalties on “destroyed” states (e.g., stuck too long)

### Training phases
1. **Behavior cloning** (supervised):  
   Log (observation, action) from a strong scripted policy / human play → train a baseline bot (scikit-learn or Keras MLP).
2. **Reinforcement learning** (PPO/SAC):  
   Use a Gym-style wrapper around the exact same physics. Start with fixed target motion; then add bounces/corners; then full dynamics.

---

## Testing scenarios

- **Wall bounce:** hold target near each wall; verify clean semicircle bounces with no teleport.  
- **Corner survival:** push target into each corner at varying speeds; ensure glide enters and exits deterministically.  
- **Dive under stress:** command dives near walls/corners; confirm INWARD→OUTWARD completes without clipping.  
- **Direction flips:** flip mid-bounce and mid-glide; no instability.  
- **Parameter sweeps:** vary `R`, `v_t`, `v_r`, `SAFETY_MARGIN`; confirm invariants.

---

## Engineering notes & bug log

See `orbit_dive_project_log.md` for the running log:
- Frame-rate dependence → **dt-scaled speeds**  
- Orbit jitter near walls → **normalize unit vector every frame**  
- Wall “teleport” → **reflect predicted position; no double rotation**  
- Corner trap → **WALL_GLIDE with deterministic tangent-based axis**  
- `global DEBUG` syntax error → removed global at module scope

---

## Roadmap

- **Game v0** (score/lives/HUD, sprites/SFX)  
- **Gym wrapper** + behavior-cloned baseline (**Vortex AI v0**)  
- **RL training** (PPO) with curriculum & evaluation harness  
- **Multi-agent** scenarios (deconfliction, phase offsets)  
- **Hardware bridge**: map sim actions to robot actuators, add sensor noise models  
- **Analytics**: CSV/Parquet logs, episode replays, charts

---

## Contributing

1. Fork → branch → PR.  
2. Keep physics changes isolated (pure functions/classes).  
3. Update the project log (`orbit_dive_project_log.md`) with **bug, cause, fix**.  
4. Add/extend test scenarios where relevant.

---
