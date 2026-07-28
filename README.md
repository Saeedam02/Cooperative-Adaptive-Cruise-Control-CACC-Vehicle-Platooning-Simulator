# 🚚 Cooperative Adaptive Cruise Control (CACC) — Vehicle Platooning Simulator

A simulation that answers a real, practical question in autonomous vehicle
control: **if the lead truck in a platoon taps the brakes, does that
disturbance shrink or grow by the time it reaches the 8th vehicle behind
it?**

This is the core engineering problem behind truck platooning and highway
CACC systems — and the reason CACC (which shares acceleration data between
vehicles over V2V) is being deployed instead of plain ACC (which only
senses the vehicle directly ahead). This project builds both, runs the
same disturbance through both, and measures the difference numerically —
not just claims it.


## The result (this isn't a made-up number — it's this repo's own output)

Running an 8-vehicle platoon with a 0.4-second time headway through a
2-second acceleration pulse from the leader:

| | ACC (radar/camera only) | CACC (with V2V acceleration feedforward) |
|---|---|---|
| Peak spacing error, closest follower | 0.97 m | 0.50 m |
| Peak spacing error, last vehicle (8th) | 1.85 m | 0.83 m |
| Amplification (last vehicle ÷ first) | **1.91×** | **1.67×** |

Being fully honest about what this shows: at this headway, neither
controller is *perfectly* string stable by the strict textbook definition
(error never growing hop-to-hop) — but CACC's V2V feedforward keeps peak
spacing errors roughly **half** the size of ACC's, all the way down the
platoon. That's the real, measurable benefit V2V communication buys you,
and it's exactly why CACC is the direction the industry is actually
moving, not a hypothetical improvement.

**Velocity and spacing-error time histories, ACC vs CACC:**

![Platoon comparison — velocity and spacing error](/images/platoon_comparison.png)

**String stability comparison — peak spacing error by platoon position:**

![String stability comparison — peak spacing error per vehicle](/images/string_stability_comparison.png)

Run `python simulate.py --headway 0.3` yourself to see the gap widen
further at a more aggressive (shorter) headway — a regime where ACC starts
approaching genuinely unsafe spacing errors while CACC stays comparatively
controlled.

## Why this matters (the actual engineering concept)

- **ACC** only knows what its own radar/camera can see: the position and
  velocity of the vehicle directly ahead. It can't know that vehicle is
  accelerating until the effect shows up in its velocity a moment later —
  there's an unavoidable sensing delay built into the physics.
- **CACC** adds one thing: the preceding vehicle broadcasts its current
  acceleration directly over V2V. That single piece of information removes
  a full step of reaction lag from the control loop — which is what lets
  real CACC systems safely run shorter, more fuel-efficient headways than
  ACC without the platoon's spacing errors snowballing.
- This is the same class of problem tackled in platooning research more
  broadly (including cybersecurity-hardened variants, where the V2V link
  itself becomes an attack surface worth defending) — this project models
  the clean control-theory baseline that work builds on top of.

## Formulas

Everything the code actually computes, laid out explicitly.

### 1. Vehicle dynamics (`vehicle.py`)

Longitudinal kinematics:

$$\dot{p}_i = v_i, \qquad \dot{v}_i = a_i$$

Actuator lag — commanded acceleration doesn't apply instantly, it's
filtered through a first-order lag with time constant $\tau$:

$$\dot{a}_i = \frac{u_i - a_i}{\tau}$$

where $p_i$, $v_i$, $a_i$ are vehicle $i$'s position, velocity, and actual
acceleration, and $u_i$ is the controller's commanded acceleration.

Integrated numerically each timestep (forward Euler):

$$a_i \leftarrow a_i + \Delta t \cdot \frac{u_i - a_i}{\tau}, \qquad
v_i \leftarrow v_i + \Delta t \cdot a_i, \qquad
p_i \leftarrow p_i + \Delta t \cdot v_i$$

### 2. Spacing policy (`controllers.py`)

Constant Time Headway (CTH) — desired gap grows with your own speed:

$$d_i^{\text{desired}} = d_0 + h \cdot v_i$$

- $d_0$ = standstill distance
- $h$ = time headway (seconds)

Spacing error — actual gap minus desired gap:

$$e_i = \big(p_{i-1} - p_i - L_{i-1}\big) - \big(d_0 + h \cdot v_i\big)$$

where $L_{i-1}$ is the preceding vehicle's length.

### 3. Control laws

**ACC** (radar/camera only — feedback on spacing error and relative velocity):

$$u_i = k_p e_i + k_d (v_{i-1} - v_i)$$

**CACC** (same feedback, plus V2V feedforward of the predecessor's actual
acceleration):

$$u_i = k_p e_i + k_d (v_{i-1} - v_i) + k_{ff}\, a_{i-1}$$

The only structural difference between the two is the $k_{ff}\, a_{i-1}$
term — that's the entire mathematical expression of what V2V communication
buys you.

### 4. String stability metrics (`string_stability.py`)

Peak spacing error for vehicle $i$ over the whole simulation:

$$E_i = \max_t \, |e_i(t)|$$

Amplification ratio (last vehicle vs. first follower):

$$\text{Amplification} = \frac{E_N}{E_1}$$

String stability condition — the platoon is string stable if error never
grows from one vehicle to the next:

$$E_{i+1} \le E_i \quad \text{for all } i$$

Worst-case single-hop growth:

$$\max_i \frac{E_{i+1}}{E_i}$$

**Scope note, stated honestly:** the CACC law here is a feedback +
feedforward design, not a rigorously pole-placed one. A textbook-rigorous
treatment would derive the closed-loop error transfer function
$\Gamma_i(s) = E_i(s)/E_{i-1}(s)$ and require $\|\Gamma_i(j\omega)\|_\infty \le 1$
for all frequencies — the true frequency-domain string stability
condition. What's implemented here is validated empirically instead, by
measuring $E_i$ directly from simulation, which is honest but weaker than
a frequency-domain guarantee. See "Ideas for extending it" below.

## Quickstart

```bash
git clone https://github.com/Saeedam02/Python-Projects.git
cd Python-Projects/platoon-cacc
pip install -r requirements.txt
python simulate.py
```

This prints the string-stability numbers for both controllers to the
console and saves two plots:
- `platoon_comparison.png` — velocity and spacing-error time histories for every vehicle, ACC vs CACC, side by side
- `string_stability_comparison.png` — a direct bar-chart comparison of peak spacing error by platoon position

### Options

```bash
python simulate.py --headway 0.3        # shorter, more aggressive time headway
python simulate.py --vehicles 12        # longer platoon
python simulate.py --duration 60        # longer simulation window
```

## How it's built

- **`vehicle.py`** — longitudinal vehicle model: position, velocity, and
  *actual* acceleration, which lags the commanded acceleration through a
  first-order actuator model (`tau * da/dt = u - a`). This lag is what
  makes string stability a real problem instead of a trivial one.
- **`controllers.py`** — the ACC and CACC control laws, both built on a
  Constant Time Headway (CTH) spacing policy, the same policy real-world
  ACC systems use.
- **`platoon.py`** — assembles a leader + N followers, initializes the
  platoon already at its steady-state desired spacing (so any error that
  appears is caused by the leader's maneuver, not a mismatched starting
  condition), and runs the simulation loop.
- **`string_stability.py`** — measures whether spacing error grows or
  shrinks from one vehicle to the next down the platoon, and reports an
  amplification ratio.
- **`simulate.py`** — runs both controllers through the same maneuver and
  generates the comparison plots.

## Ideas for extending it

- **Communication imperfections**: add latency, packet loss, or a full V2V
  dropout partway through the platoon to see CACC degrade toward ACC-like
  behavior — a genuinely interesting and realistic failure mode
  - Anyone building on this in a security-conscious direction could go
    further and simulate a spoofed/malicious acceleration broadcast — the
    exact class of attack that motivates authenticated V2V protocols
- **Frequency-domain string stability**: derive the closed-loop transfer
  function analytically and check the H∞ norm condition directly, instead
  of only measuring it empirically from one maneuver
- **Nonlinear vehicle dynamics**: replace the simple actuator-lag model
  with a more realistic engine/brake torque model
- **Lane-change/cut-in scenarios**: model a vehicle merging into the
  platoon mid-simulation


## Communication & Interaction

Questions, feedback, bug reports, or ideas for extending this (especially
around the communication-imperfection or security-focused extensions
mentioned above) are welcome.

- **Open an issue** on this repo for bugs, questions, or feature requests — it keeps the discussion visible and searchable for anyone else running into the same thing
- **Pull requests** are welcome if you want to implement one of the extension ideas yourself
- **Email**: saeedaghamohammadi99@gmail.com — for anything you'd rather discuss directly (collaboration, research-related questions, etc.)

If this project was useful or interesting, a star on the repo is always
appreciated — it's the easiest way to signal that and helps others find it.


## License

MIT
