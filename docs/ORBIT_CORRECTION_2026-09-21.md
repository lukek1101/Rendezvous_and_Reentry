# Bounded orbit correction — 2026-09-21

This stage adds disturbed execution and bounded correction to the validated
nominal phasing/homing schedule. The nominal plan remains the reference;
correction is opt-in. The same existing J2 propagation, LVLH conventions,
impulsive mass accounting and Phase2 are used. Vehicle models, Python
optimizers, orbital architectures and entry algorithms are unchanged.

## Run and assumptions

```matlab
settings.phase1.correction.enabled = true; % execute a disturbed nominal plan
settings.phase1.correction.corrections_enabled = true;
settings.phase1.correction.seed = 42;
result = Run_Nominal_Orbit(settings, struct('verbose',false));
```

`enabled=false` retains the previously validated undisturbed nominal path.
`enabled=true, corrections_enabled=false` executes the same disturbed initial
state and nominal departure/arrival schedule without retargeting or added
impulses. The mission runner inhibits Phase2 unless the disturbed result meets
the nominal0.5 m /0.001 m/s handoff criteria. For failed cases and paired
comparisons, use `mission.execute_corrected_nominal` directly: it returns
actual states, limits and failed-opportunity diagnostics instead of discarding
them in a mission-level exception. See the benchmark driver for complete inputs.

**State knowledge is PERFECT_INSTANTANEOUS.** There is no sensor model,
navigation estimator, communication delay or knowledge covariance. Current
position, velocity and mass are known exactly, including after each burn;
future random execution errors are not used by the predictor/controller.
The terminal velocity cleanup is an explicitly idealized zero-latency impulse
after the nominal arrival burn. It has its own execution error and is limited
to the configured correction count/capability/budget. These assumptions must
be replaced before interpreting this as operational flight guidance.

Default exploratory uncertainties are independent Gaussian components:
20 m initial chaser position sigma per ECI axis,0.02 m/s velocity sigma,
0.002 fractional burn-gain sigma and0.0002 rad pointing-error-vector sigma.
These are project test assumptions, not published vehicle performance.
Pointing uses a first-order small-angle perturbation:
`delivered = (1 + gain) * (command + cross(pointing_error, command))`.
It is appropriate to the small test angles, not a general attitude model.
The target initial state is unperturbed. Three-axis initial/execution errors
can leave the nominal plane; the correction Jacobian is three-dimensional.
The underlying nominal orbit and target remain the supported polar case.

A private MT19937 stream with explicit seed generates the initial error and
all burn-slot draws before execution. Departure, each configured midcourse
opportunity, arrival and cleanup have fixed slots. Disabling correction does
not shift departure/arrival draws, and the caller RNG is unchanged. The
controller does not invert or compensate the sampled future actuator error.
Gaussian tails are unbounded; four seeds/cases are not a reliability estimate.

## Sequence and bounds

1. Plan from the nominal initial state. Inject the sampled initial error once
   into the actual initial state and propagate it through the nominal wait.
2. Execute the frozen nominal departure command with its sampled gain/pointing
   error. No initial-state or departure-state repair is performed.
3. At35% and75% of the original transfer time, predict the actual handoff under
   the remaining policy. If position misses tolerance, solve for a small ECI
   velocity correction using bounded finite-difference differential correction
   and shared nonlinear propagation. Prediction assumes ideal future commands;
   it does not know future random execution errors. Already-adequate position
   does not trigger an unnecessary impulse; velocity matching is deferred to
   arrival. Failed numerical correction attempts and capability rejection are
   retained separately in the opportunity records.
4. At the original arrival epoch, retarget the **existing** arrival burn from
   the actual state to the requested rotating-LVLH velocity. This changes an
   existing command and its cost, not the count of added impulses. Execute it
   with the arrival-slot error. If needed and allowed, apply one additional
   velocity-cleanup impulse immediately afterward; its execution error remains
   in the final state. There is no iterative terminal reset or hidden extra trim.

| Limit | Default |
|---|---:|
| Added corrections, including terminal cleanup |3 maximum |
| Delivered/commanded magnitude per correction |2 m/s |
| Total added-correction delta-V |3 m/s |
| Each nominal departure/arrival command |150 m/s, also subject to the existing system burn limit |
| Total Phase1 propellant expenditure |400 kg |
| Minimum remaining mass |1000 kg |
| Maximum physical elapsed time, including initial wait |40000 s |
| Minimum time remaining for a midcourse position correction |120 s |
| Midcourse solver |6 iterations /30 trial evaluations per opportunity |
| Execution plus prediction wall-time guard |60 s; separate from nominal planner's60 s guard |

Bounds are configurable mission assumptions;400 kg is the tested4800 kg ARD
stack budget, not a universal HORUS propellant allocation. Integration and
targeting tolerances remain those in `phase1.nominal`; correction seeks
one-quarter of its position tolerance internally. Baseline/post-burn prediction
calls are separate from the30 solver-trial limit and bounded by the finite
opportunity list and wall-time guard. Every future coast ends at the original
handoff epoch: no extra coast/orbit is silently inserted.

Command capability, remaining correction count, cumulative correction delta-V
and predicted propellant are checked before firing. Every **delivered** burn
is then charged using its actual magnitude and the rocket equation. If the
sampled actuator error takes an applied burn over a resource/capability bound,
the delivered state and spent fuel remain recorded with a delivered-limit
violation. The simulator does not undo, clip or reset that burn to claim success.
Checks are not an exhaustive collision/path-constraint certification.

`execution_status`, per-opportunity solver status and `handoff_status` are
separate. An exhausted wall-time/evaluation budget is numerical noncompletion;
it is not proof of infeasibility. A completed disturbed trajectory outside a
handoff tolerance is a demonstrated candidate violation. A denied command is
recorded as a count, capability, delta-V, propellant or time-limit decision.

## Paired results and additional-leg assessment

The same300→500 km,4800 kg ARD stack,2000 kg target, J2-on/drag-off and
Isp200 s scenario from the nominal stage is used. Cases vary initial target
phase90°/60° and seed42/7. The frozen nominal plan, disturbance draws and
original handoff epoch are identical within each disabled/enabled pair.

| Phase / seed | Disabled position miss (m) | Enabled position miss (m) | Disabled velocity error (m/s) | Enabled velocity error (m/s) | Added correction delta-V (m/s) |
|---|---:|---:|---:|---:|---:|
|90° /42 |3373.85|0.002856|1.08650|0.000103805|1.67143|
|90° /7 |1856.63|0.002695|0.034225|0.000041562|1.06598|
|60° /42 |2686.98|0.002271|1.08227|0.000104634|1.28109|
|60° /7 |1253.99|0.001979|0.032698|0.000041722|0.72278|

All four enabled cases meet0.5 m /0.001 m/s using three added impulses.
Enabled Phase1 propellant is271.46–275.59 kg, within400 kg. Execution plus
prediction took about0.55–0.82 s in this run, excluding nominal planning.
The resource comparison also includes the changed arrival-burn cost; added
correction delta-V alone is not the total cost difference. Full paired totals,
fuel, runtime and elapsed time are in `correction_stage_2026-09-21/benchmark.json`.
Physical elapsed times remain32758.90 s (90°) and22305.76 s (60°).

**An additional phasing-orbit leg is not necessary for these intended test
cases:** all enabled cases satisfy handoff and resources at the original epoch.
No additional-leg strategy or general sequence optimizer was implemented.
Count/capability/propellant counterexamples deliberately fail; those failures
do not establish that another orbit is necessary or sufficient. Larger errors,
different spacecraft budgets or delayed navigation require a new bounded
case assessment rather than an automatic extra orbit.

## Accounting, verification and next dependency

`history.execution.burns` records every command and delivered vector, role,
time, full pre/post state and fuel expenditure. Histories contain both sides
of each instantaneous burn; position and target state do not jump. Phase1
budget totals include all delivered impulses, including corrections. The
initial interface records distinguish nominal design state from actual
disturbed initial state. Compatibility contracts include correction settings.

Validation checks paired random draws and caller-RNG isolation; delta-V/fuel
ledger closure; position/target continuity across burns; count, capability,
propellant, delta-V, elapsed-time and numerical limits; integration refinement;
and zero-error execution without added corrections. The full MATLAB aggregate
suite passed, followed by extra focused limit checks. A corrected public-runner
case reaches the existing Phase2 endpoint and preserves total accounting.
Tenfold tighter integration tolerances shift terminal inertial position by
0.01713 m: millimetre shooting residuals do not establish millimetre absolute
propagation accuracy. No new optimizer or expensive orbit search was run.

Reproduce from repository root:

```matlab
addpath('docs/correction_stage_2026-09-21'); benchmark();
addpath('validation'); Run_All_Validations();
```

The stage folder contains paired JSON and per-case MAT histories, verification
logs/results and one integrated API result. Earlier logs include a failed
checkpoint write and the initial fixed-arrival experiment; final JSON/per-case
files record the completed comparison. Earlier migration artifacts and unrelated
user changes were preserved.

Next dependency: define credible navigation error, observation/burn latency,
actuator limits and vehicle-specific propellant allocations before widening
the disturbance envelope. Acceptance: paired fixed-realization tests with
bounded resources and explicit failure reasons, independent propagation checks,
and no state resets. If those intended cases demonstrably require more time,
evaluate one explicitly bounded phasing-leg strategy with defined entry/exit
conditions. Existing deorbit/entry vehicle-domain dependencies remain separate.
