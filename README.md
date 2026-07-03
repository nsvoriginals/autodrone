# auto-drone

A real-time **3D pursuit–evasion simulation** rendered live in your terminal. An
autonomous interceptor drone must track and capture an evasive, jinking target
while both craft dodge obstacles and stay inside a bounded arena. Everything —
physics, state estimation, guidance, control, and the ASCII/Unicode renderer —
is written from scratch in Rust.

The point of the project is to be a compact, readable playground for the classic
math of guided interception: Kalman filtering, proportional navigation, PID
control, and potential-field obstacle avoidance, all wired together in one loop
you can watch run.

```
┌─ TOP VIEW ───────────────────┐  ┌─ TELEMETRY ──────────┐
│        ·        ✳ target      │  │ phase     PURSUE     │
│    ◯obstacle                  │  │ distance  altitude   │
│              ✈ drone ···trail │  │ closing   TTI  LOS   │
└──────────────────────────────┘  └──────────────────────┘
```

---

## Quick start

```bash
# build & run
cargo run --release

# run the tests
cargo test
```

Requires a recent Rust toolchain (the crate uses `edition = "2024"`).

### Controls

| Key                 | Action                                        |
| ------------------- | --------------------------------------------- |
| `q` / `Esc`         | Quit                                          |
| `Space`             | Pause / resume                                |
| `r`                 | Restart with a fresh random scenario          |
| `f`                 | Toggle camera **follow** / **free** mode      |
| `↑ ↓ ← →`           | Pan the camera                                |
| `+` / `-`           | Zoom in / out                                 |
| `<` / `>`           | Rotate the camera                             |

When the drone captures the target the engagement freezes briefly, then a new
random chase starts automatically.

---

## How it works

Each simulation tick (fixed at **Δt = 1/60 s**) runs this pipeline:

```
target moves ──► noisy sensor ──► Kalman tracker ──► guidance ──► speed + PID + avoidance ──► drone moves
   (evader)      (measurement)     (estimate)        (aim/PN)         (steering command)       (integrate)
```

1. **Target** picks a patrol waypoint, and if it senses the drone nearby it
   panics and evades (flee + brake + random "jink").
2. The drone only sees a **noisy position measurement** of the target — never
   its true velocity or acceleration.
3. A **Kalman filter** fuses those measurements into a smooth estimate of the
   target's position, velocity, and acceleration.
4. **Guidance** computes an intercept aim-point and a proportional-navigation
   acceleration from that estimate.
5. A **speed controller**, **adaptive PID**, and **potential-field avoidance**
   turn the guidance into a single steering acceleration.
6. The drone **integrates** its motion, and the loop repeats until the true
   distance drops inside the capture radius.

---

## The math

### Motion & integration

Both craft are point-mass bodies advanced with **semi-implicit (symplectic)
Euler** integration, which is stable for the drag/spring-like forces used here:

```
v(t+Δt) = v(t) + a·Δt
p(t+Δt) = p(t) + v(t+Δt)·Δt        (velocity updated first, then position)
```

Forces acting on a body each tick:

- **Steering** — the control command, clamped to `max_accel`.
- **Linear drag** — `F_drag = -k · v`, opposing motion.
- **Boundary repulsion** — a quadratic ramp `((margin − d)/margin)²` that pushes
  a craft away from the arena walls, floor, and ceiling as it nears them.

Velocity is clamped to `max_speed` after each step. Orientation (used only for
drawing) is derived from the velocity heading, flight-path angle, and a small
bank angle proportional to lateral acceleration.

### Target estimation — Kalman filter

A 9-state **constant-acceleration Kalman filter** (`src/estimation/kalman.rs`)
tracks the target. The state vector is:

```
x = [ px py pz | vx vy vz | ax ay az ]ᵀ
```

**Predict** — propagate the state with the constant-acceleration transition
matrix `F` and grow the covariance with the process noise `Q`:

```
x⁻ = F · x
P⁻ = F · P · Fᵀ + Q
```

where each axis of `F` encodes `p += v·Δt + ½a·Δt²` and `v += a·Δt`, and `Q` is
the standard continuous white-noise-acceleration model (terms in
`Δt⁵/20 … Δt`).

**Update** — correct with the incoming measurement `z` (position only, via the
observation matrix `H`):

```
y = z − H·x⁻                    (innovation)
S = H·P⁻·Hᵀ + R                 (innovation covariance)
K = P⁻·Hᵀ·S⁻¹                   (Kalman gain)
x = x⁻ + K·y
P = (I − K·H)·P⁻
```

`R` comes from the sensor noise standard deviation (`MEASUREMENT_NOISE_STD²`).
The tracker also estimates **evasiveness** from the target's *lateral*
acceleration (the component of `a` rejected from `v`), passed through a sigmoid
and smoothed — this drives how aggressively the drone reacts.

### Guidance — proportional navigation + lead pursuit

Guidance (`src/control/guidance.rs`) blends two ideas:

**Lead pursuit / predictive aim.** Solve for the time-to-intercept `t` assuming
the drone flies at `intercept_speed` toward where the target *will* be. This is
the quadratic from `‖ r + v_tgt·t ‖ = intercept_speed · t`:

```
a·t² + b·t + c = 0
  a = v_tgt·v_tgt − s²
  b = 2 · r·v_tgt
  c = r·r
```

(with `r` = relative position, `s` = intercept speed). The smallest positive
root gives `t`, and the aim-point is the target's predicted position:

```
aim = p_tgt + v_tgt·t + ½·a_tgt·t²
```

**Proportional navigation (PN).** The commanded acceleration is proportional to
the line-of-sight (LOS) rotation rate and the closing speed — the guidance law
used by real interceptors:

```
ω        = (r × v_rel) / ‖r‖²           (LOS rotation rate)
V_c      = −v_rel · r̂                    (closing speed)
a_pn     = N · V_c · (ω × r̂)            (N = PN_GAIN = 4)
```

The two are **range-blended**: far out, the drone mostly flies the direct
lead-pursuit heading; close in, PN takes a larger share for a crisp terminal
intercept.

### Speed control

The speed controller (`src/control/speed.rs`) sets a desired speed from the
target's estimated speed times a margin, then multiplies it by three adaptive
factors:

- **Proximity boost** — speed up as distance closes inside the intercept radius.
- **Evasive boost** — speed up when the target is maneuvering hard.
- **Obstacle factor** — slow down (down to 35 %) when hugging an obstacle.

The result is clamped between the drone's min and max speed.

### Low-level control — adaptive PID

An **adaptive, gain-scheduled PID** (`src/control/pid.rs`) drives the drone's
velocity error `e = v_desired − v` to zero:

```
u = Kp·e + Ki·∫e dt + Kd·(de/dt)
```

The integral term is magnitude-clamped to prevent wind-up. Gains are *scheduled*
each tick from flight conditions:

- `Kp` rises with speed ratio, and by ×1.5 more when the target is very evasive,
- `Kd` eases off at high speed,
- `Ki` doubles inside the intercept radius to kill steady-state error at the
  endgame.

### Obstacle avoidance — potential fields

Obstacles are spheres. Avoidance (`src/planning/potential.rs`) uses the gradient
of a **repulsive potential field** that only acts within an influence radius:

```
a_avoid = strength · (1/d − 1/d₀) · (1/d²) · n̂
```

where `d` is distance to the obstacle surface, `d₀` the influence radius, and
`n̂` points away from the obstacle. This is the classic Khatib repulsive-field
form — force blows up near contact and smoothly vanishes at the influence edge.

### The target's evasion

The evader (`src/target/target.rs`) blends a **patrol** behavior (seek random
waypoints) with **evasion** when the drone enters its detection radius. Evasion
is a weighted sum of:

- a **flee** component directly away from the drone,
- a **brake / perpendicular** component to cut across the drone's approach,
- a random **jink** that re-rolls every 0.6–1.8 s for unpredictability,

all scaled by a `panic_level` that ramps up with proximity and decays when safe.

---

## Project layout

```
src/
├── main.rs            Terminal loop: input, fixed-step scheduler, rendering
├── lib.rs             Module tree
│
├── core/              Math primitives: Vector3, Matrix, Quaternion, math helpers
├── physics/           Rigid bodies, semi-implicit Euler integrator, drag & walls
├── sensors/           Noisy measurement models (gps, imu, camera, noise)
├── estimation/        Kalman filter, target tracker (+ ekf / imm scaffolding)
├── control/           Guidance (PN + lead pursuit), PID, speed, attitude
├── planning/          Potential-field avoidance (+ rrt / obstacle scaffolding)
├── vehicle/           Drone body, tunable parameters, flight-phase state machine
├── target/            The evasive target and its behavior model
├── environment/       Arena bounds and scattered spherical obstacles
├── engine/            Simulation orchestrator, scenarios, metrics, telemetry
├── display/           ratatui renderer, camera, viewport, HUD panels
├── scenarios/         Scenario presets (basic, evasive, interception, obstacles)
└── utils/             Config constants, logging, config/error helpers
```

**Where the interesting bits live:**

| Concept                     | File                              |
| --------------------------- | --------------------------------- |
| Simulation tick / pipeline  | `src/engine/orchestrator.rs`      |
| Kalman filter               | `src/estimation/kalman.rs`        |
| Guidance (PN + lead pursuit)| `src/control/guidance.rs`         |
| Adaptive PID                | `src/control/pid.rs`              |
| Speed control               | `src/control/speed.rs`            |
| Potential-field avoidance   | `src/planning/potential.rs`       |
| Target evasion              | `src/target/target.rs`            |
| Tunable constants           | `src/utils/config.rs`             |

---

## Tuning

Almost every knob lives in **`src/utils/config.rs`** — speeds, accelerations,
drag, PN gain, PID gains, sensor/process noise, capture and intercept radii,
obstacle field strength, and arena size. A few starting points:

| Constant                 | Meaning                                           |
| ------------------------ | ------------------------------------------------- |
| `PN_GAIN`                | Proportional-navigation gain (aggressiveness)     |
| `PID_KP/KI/KD`           | Base PID gains before scheduling                  |
| `MEASUREMENT_NOISE_STD`  | Sensor noise — how blind the drone is             |
| `PROCESS_NOISE`          | Kalman filter's trust in its motion model         |
| `CAPTURE_RADIUS`         | How close counts as a capture                     |
| `DRONE_MAX_SPEED` / `TARGET_MAX_SPEED` | The speed advantage of pursuer over evader |

Turn up `MEASUREMENT_NOISE_STD` to see the Kalman filter work harder; drop
`DRONE_MAX_SPEED` toward `TARGET_MAX_SPEED` to make captures genuinely hard.

---

## Notes

Some modules (`estimation/ekf.rs`, `estimation/imm.rs`, `planning/rrt.rs`,
`sensors/*`, `scenarios/*`) are scaffolding for extensions — an Extended Kalman
Filter, an Interacting Multiple Model estimator, RRT path planning, richer
sensor models, and preset scenarios. The active flight loop uses the linear
Kalman filter, PN/lead guidance, adaptive PID, and potential-field avoidance
described above.
