# Signed-axis proximity approaches — 2026-09-21

The existing hybrid proximity sequence now selects `+R`, `-R`, `+V` or `-V`.
The default remains `-R`. Closing, acquisition, controlled approach, holds and
terminal standoff use the selected side. The actual Phase1 chaser and target
states are passed into the same existing handoff impulse and dwell; no state
is reconstructed, reflected or rotated to a fabricated acquisition point.
The legacy cycloid/hop comparison remains unchanged and only accepts its
existing `-R` configuration.

## Conventions and configuration

LVLH follows `orbit_core.relative_state`: R is target-centred radial outward,
H is the target angular-momentum direction, and V = H × R is forward transverse
along-track. V is not defined by an arbitrary rotation of the approach axis.
Velocity includes the existing h/r² rotating-frame correction. Position/length
units are metres, velocities m/s, forces N, times seconds and configured angles
degrees. The mode names specify the **side occupied by the chaser**, not the
direction of its closing velocity.

| Mode | Unit vector u in [R,V,H] | Acquisition / hold / terminal position | Closing direction |
|---|---|---|---|
|+R |[1,0,0] |u ×500 /250 /30 m |Toward −R |
|-R |[−1,0,0] |u ×500 /250 /30 m |Toward +R |
|+V |[0,1,0] |u ×500 /250 /30 m |Toward −V |
|-V |[0,−1,0] |u ×500 /250 /30 m |Toward +V |

```matlab
settings.phase2.autonomous.approach_mode = '+V';
settings.phase2.autonomous.insertion_range_m = 500;
settings.phase2.autonomous.hold_range_m = 250;
settings.phase2.terminal_standoff_m = 30;
result = Run_Nominal_Orbit(settings, struct('verbose',false));
```

`terminal_standoff_m` is the mode-independent terminal-range override. Empty
retains the existing `phase2.S4_R_abs_m` compatibility field; a nonempty value
takes precedence. Internally `p2.S4_R_abs` retains its historical name but is
the positive distance along the selected side, not necessarily a radial
coordinate. The upstream `phase2.S2_m` and its handoff gate are independent of
approach mode and remain[0,−5000,0] m in the comparison.

Existing parameters continue to set acquisition/hold distances, approach
durations, intermediate and final dwell, force norm, control period, gains,
speed limits, gate position/speed tolerances, corridor opening/floor, keep-out
radius and minimum mass. Default order is500→250 m in3200 s,60 s hold,
250→30 m in2800 s and60 s terminal hold. Reference position uses a quintic
smoothstep with zero endpoint velocity and acceleration, now along u. Actual
endpoint position must satisfy the final0.25 m standoff check and0.01 m/s
speed check; intermediate controlled gates use1 m /0.01 m/s. Neither gate
sets the propagated state to its target.

The approach corridor uses axial distance `d = dot(u,r)` and lateral distance
`norm(r-u*d)`. It requires d>0, lateral≤floor +d tan(half-angle), spherical
range≥keep-out, bounded relative speed and remaining mass. Defaults are10°,
1 m floor,20 m keep-out,0.17 m/s controlled-approach speed and1000 kg minimum
mass. These are research assumptions rather than certified docking rules.

## Direction-dependent dynamics and control

The acquisition transfer is recomputed from the actual post-dwell state to
u × acquisition range. The existing CW seed, nonlinear differential correction
and finite time-grid screening are reused; no previous R-bar burn is rotated.
Closing is checked for minimum range and maximum speed before the arrival
impulse and subsequent corridor-controlled segment.

During controlled approach, the reference position, velocity and acceleration
follow the selected axis, but the natural dynamics remain in physical R/V/H:

```text
a_R = 3 n² r_R + 2 n v_V
a_V = -2 n v_R
a_H = -n² r_H
F_requested = mass × (a_reference - a_natural
                     + Kp (r_reference-r) + Kd (v_reference-v))
```

The instantaneous target n, actual r/v, reference derivatives and mass are
evaluated every control tick. R-side holding has a radial gravity-gradient
term; V-side holding does not have that same CW term. V closing produces a
radial Coriolis term, whereas R closing produces an along-track Coriolis term.
Thus reference geometry is parameterized while dynamics/control are recomputed,
not treated as rotationally equivalent. Propagation remains nonlinear J2 in
ECI; the existing CW feedforward and h/r² frame-rate approximation are retained.

Actuation remains ideal direction/throttle with a300 N force-norm limit and
sample-and-hold ECI force over each1 s control interval. There is no attitude
tracking, thruster allocation, plume model or docking-contact dynamics. Closing
and handoff burns remain ideal impulses; their finite peak forces are undefined.
Reported force and saturation metrics therefore cover the controlled final
approach only. Delta-V and duration cover the entire Phase2 sequence.

## Matched comparison and baseline preservation

All four modes start from the **same saved actual nominal Phase1 handoff**,
including position, velocity, target state and remaining mass (about4528.79 kg).
This is `nominal_stage_2026-09-21/nominal_api.mat`, generated by the validated
300→500 km ARD-stack mission. Its SHA-256 is recorded in the stage manifest.
The upstream state is retained in every result for verification.

Requirements are matched: same upstream state,60 s handoff dwell,500/250/30 m
ranges, controller, force/speed/corridor/resource limits, gate tolerances and
closing candidate times[1800,2700,3600,4500,5400] s. The selected closing time
may differ; this is reported rather than hidden as equal total duration. All
controlled approach/hold durations are identical. No new time optimizer was
introduced and the finite grid has no global-optimality claim.

| Mode | Phase2 delta-V (m/s) | Duration (s) | Selected closing (s) | Peak controlled force (N) | Saturation (s) | Max tracking error (m) |
|---|---:|---:|---:|---:|---:|---:|
|-R |7.477427|10680|4500|8.793171|0|0.008587|
|+R |8.417238|10680|4500|8.787768|0|0.008715|
|+V |1.948533|11580|5400|1.476654|0|0.006957|
|-V |1.828599|11580|5400|1.476792|0|0.006658|

All four satisfy terminal standoff with no recorded controlled-approach
constraint violations. Final position errors are approximately0.0003–0.0006 m
and speeds below0.000004 m/s. These numerical residuals are not independent
physical-accuracy estimates. Different costs reflect both dynamics and the
different closing geometry from the common −V handoff; they do not prove one
mode universally superior. Selected/rejected closing candidates and margins
remain in each result.

The `-R` rerun matches the pre-stage saved state history to1e−8 in its SI state
components and total delta-V to1e−9 m/s. Generalization preserves its numerical
behavior; phase labels were made range-independent. A separate public-runner
check enables the prior orbit-correction stage and a+V approach, verifying
that its actual disturbed-and-corrected handoff is consumed unchanged.

## Diagnostics, verification and next dependency

Control outputs now retain requested and delivered force, peak force,
saturation time, reference/actual position and velocity histories, tracking
error, corridor/keep-out margins, termination and individual side/corridor/
keep-out/speed/mass/gate violation flags. If a controlled approach violates a
constraint or gate, its partial actual trajectory is returned with
`completed=false`; mission execution subsequently inhibits deorbit when
`reached_standoff=false`. No failed trajectory is repaired to its reference.
Earlier handoff/closing failures retain their existing error behavior and
closing candidate diagnostics; force metrics are unavailable when the
controlled segment is never reached.

Tests rerun all four modes, verify frozen−R equivalence, target/reference
coordinates, resource accounting, corridor and terminal conditions. A0.01 N
force-limited counterexample records saturation and failure without exceeding
the commanded force norm. A wrong-side initial state records a side violation
and remains unchanged. Aggregate MATLAB regression and public-API checks are
recorded in `approach_stage_2026-09-21`.

Reproduce from repository root:

```matlab
addpath('docs/approach_stage_2026-09-21'); benchmark();
addpath('validation'); Run_All_Validations();
```

Next dependency: broaden matched upstream-state/geometry and parameter cases
before claiming a usable operational envelope. Acceptance requires all-axis
state continuity, retained failure diagnostics, convergence checks and matched
resource/timing requirements. Navigation, frame-rate refinements and actuator
realism remain separate choices; existing ideal-actuator assumptions and the
no-contact standoff endpoint are deliberately retained in this stage.
