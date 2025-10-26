
## **First phase of simulation**

### 🌀 Simulation Summary (Progress so far)

### **1. Entities Defined**

We introduced **three independent elements** in the simulation:
1. **Red Dot** – represents the _target_ (enemy robot).
    - Moves freely under external control (keyboard input in our sim, but sensors in the real robot).
    - Defines the _center of engagement_.

2. **Circle** – represents the _engagement zone / attack radius_.
    - Radius = constant value R.
    - Always centered on the red dot.
    - Acts as a **constraint boundary**: our robot (green dot) must orbit at this distance.

3. **Green Dot** – represents _our robot_.
    - Moves along the circumference of the circle.
    - Can switch between orbiting and performing a **radial dive attack** toward the red dot.

---

### **2. Motion: Orbiting**

We modeled orbiting as **circular motion** around the target.  
The position of the green dot is determined by an angle θ, which increases over time.
Formulas:

	$x_g=x_r+R⋅cos⁡(θ)$
	$y_g=y_r+R⋅sin⁡(θ)$

- $(xr,yr)(x_r, y_r)$ = position of the red dot (center).
- R = orbit radius.
- $\theta$ = orbit angle, updated each frame:
    $θ←θ+ω$
    where ω is the angular speed.

This keeps the green dot moving around the circle **relative to the red dot’s current position**.

---

### **3. Motion: Radial Dive Attack**

We then added the ability for the green dot to **dive inward** toward the red dot and then return back outward, along its radial line of motion.

To achieve this, we introduced a variable `radial_offset` that modifies the effective radius of the orbit:
$x_g=x_r+(R+Δr)⋅cos⁡(θ)$
y$_g = y_r + (R + \Delta r) \cdot \sin(\theta)$

- Δr = radial offset.
    - Normally Δr=0 $\Delta r = 0$ → standard orbit.
    - During dive: Δr decreases toward −R, pulling the robot inward.
    - At the center: Δr=−R.
    - On return: Δr increases back to 0.

This creates a **state machine**:

- **Orbit** → **Inward Dive** → **Outward Return** → **Orbit**.
    

---

### **4. Key Simulation Insights**

- The circle and dot are kept **independent**, not hard-coded together.
    - This separation is crucial for future robotics, where the target (red dot) moves independently, and our robot (green dot) must respond dynamically.
- Orbiting logic uses **polar coordinates** (radius + angle).
- The dive attack is just a controlled **manipulation of radius**, while angle continues to evolve.
- The system is already generalizable to real robotics:
    - Replace keyboard input for the red dot with **sensor data** (enemy tracking).
    - Replace drawing the green dot with **motor commands** that maintain orbit + attack maneuvers.

---

### **5. Current State of Simulation**

- ✅ Red dot can move freely.
- ✅ Circle always follows the red dot at fixed radius.
- ✅ Green dot orbits smoothly around the circle.
- ✅ On command (space press), green dot dives inward to the red dot, then returns outward.
- ✅ Formulas are stable and scalable (more orbiting dots or different radii can be added easily).

---

### **6. Conceptual Mapping to Project Gravitron: Vortex AI**

- **Red dot (target)** → enemy robot.
- **Circle** → effective combat zone around the enemy.
- **Green dot (our robot)** → autonomous attacker, executing orbit + timed dive strikes.
- **Formulas** → already capture the strategy of _circling the opponent_ and _lunging in for attacks_.
