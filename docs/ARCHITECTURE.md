# Architecture

## Public execution API

`Main_Mission_Simulator.m` is an interactive compatibility wrapper. It calls
`Run_Mission`, plots results, and exposes the established budget/history/state
names in the caller workspace. It does not clear variables or close figures.

`Run_Mission(overrides, options)` is the reusable entry point. It returns a
single result structure and creates no figures by default. `overrides` is a
partial `Mission_Run_Config` structure. `options` supports `plot`, `verbose`,
`seed`, and partial physical/scenario overrides under `system`.

```matlab
settings.runtime.allow_environment_overrides = false;
settings.reentry.vehicle_mode = "CAPSULE";
result = Run_Mission(settings, struct('plot', false, 'verbose', false, 'seed', 42));
mission.plot_results(result); % no propagation repeated
```

A supplied seed restores the caller RNG on both success and failure. Without
a seed the runner uses the caller RNG normally. Results record the initial RNG
state, elapsed wall time, MATLAB version, and resolved configuration. Quiet mode
captures console output in `result.log`; verbose mode prints it. Neither mode
changes the numerical model.

## Responsibility boundaries

| Module | Responsibility |
|---|---|
| `Mission_Config` | Physical constants and default orbital scenario |
| `Mission_Run_Config` | Default run settings; optionally accepts physical defaults |
| `mission.configure` | Resolve config precedence and validate loop settings |
| `mission.load_optimizer_config` | Select and read the Python design archive |
| `mission.run` | Initialize states, sequence phases, accumulate budget |
| `Phasing_Propagator` | Custom impulse, Hohmann and preliminary multi-leg transfers |
| `mission.proximity` | Existing cycloid / CW waypoint-impulse standoff approach |
| `mission.deorbit` | Direct entry injection and interface propagation |
| `mission.entry_interface` | Entry state/epoch handoff from deorbit history |
| `mission.entry` | Capsule separation policy, entry propagation, mass accounting |
| `Reentry_Propagator` | Vehicle resolution, atmospheric-entry events/history/diagnostics |
| `mission.report`, `mission.plot_results` | Console summary and figures |
| `orbit_core` | Shared SI J2 acceleration and target-frame relative state |
| `reentry_core` | Reusable entry dynamics, aero, atmosphere-relative geometry/events |
| `paperstudies` | Published-condition equation audits and labeled surrogate adapters |
| `mission_io.py` | Shared Python serialization, archive hashing, atomic JSON/index writes |

Private configuration readers under `+mission/private` retain the existing
JSON/run-setting conversion semantics. Phase-specific numerical helpers remain
local to their owning modules. A package directory is part of MATLAB's package
namespace; add the project root to the path, not every subdirectory with
`genpath`. In particular, keep `legacy` snapshots off the active MATLAB path.

## Configuration and provenance

`Mission_Config` remains the default authority for orbital/physical parameters.
The functional API may overlay `options.system` for an individual experiment
without editing that file. JSON/run control do not overwrite orbital scenario
altitudes or initial geometry. `Mission_Run_Config` defaults are derived from
the selected system before partial run overrides are applied.

Resolution preserves the established policy: run settings inform archive
selection, supported JSON fields are applied, run settings are reapplied, then
permitted environment overrides and vehicle-derived entry settings are resolved.
Consequently the CAPSULE paper-condition option still overrides interface/FPA
with its configured paper reference. `result.config` records both resolved
physical systems (pre/post proximity drag scope), run settings, optimizer JSON,
and phase parameters. Relative JSON paths resolve from the project root.

Use `runtime.allow_environment_overrides=false` and `python_config.mode="FILE"`
with a pinned archive for reproducible campaigns. AUTO retains its existing
scored scenario matching; it is not proof of an exact scenario match.
Unknown API override fields are rejected. Phase timestep/duration/count checks
run before propagation; this is not a full mission-feasibility validator.

## Result interface and phase handoff

- `result.phasing`: history, final chaser/target, delta-V, fuel, position error.
- `result.proximity`: history, relative trajectory, waypoint data, final states,
  resolved waypoint geometry, duration, final position/velocity, standoff flag.
- `result.deorbit`: history, final states, burn info, pre-separation interface
  state, interface epoch/altitude/FPA/speed.
- `result.entry`: history, final vehicle, post-separation initial state, summary,
  and absolute mission entry-start time.
- `result.budget`: per-phase delta-V, fuel, active mass, total-accounted mass.
- `result.config`, `result.metadata`, `result.log`: reproducibility context.

States use metres, seconds, m/s, kg and ECI coordinates; LVLH is [radial outward,
along-track, orbit normal]. Histories use phase-relative seconds, and entry also
records its mission epoch. Capsule residual carrier mass remains accounted for,
but its post-separation trajectory is not propagated. The orbiting relay is the
target satellite. Plotting uses result histories and does not modify states.

## Model boundary and extensions

The current active mission is primarily translational and impulsive. `Env_EOM`
has rotational states, but the full mission is not a closed-loop attitude/GNC
simulation. Phase 2 ends at a standoff; physical docking is not modeled.
Finite-thrust control, signed V-bar corridors and uncertainty distributions
remain design decisions described in `PROXIMITY_CONTROL_PLAN_KR.md`.
`CODE_REVIEW_KR.md` records the handoff-speed, environment consistency and entry
FPA decisions that require a physical-model change rather than a refactor.

Python uses km/km/s internally and exports MATLAB-facing SI fields. MATLAB and
Python constants/low-altitude atmospheres are not yet a single shared model.
Paper packages retain provenance and explicit incomplete/surrogate status.

## Validation

```matlab
addpath validation
Run_All_Validations
```

This checks all active MATLAB files, J2 potential-gradient/LVLH invariants,
existing entry/paper regressions, and a complete pinned nominal mission from
an external working directory. It checks headless execution, mass accounting,
RNG restoration, invalid input paths, and frozen orbital handoff values.
Focused drag-aware deorbit checks exercise both impulsive and finite burns.
`Run_All_Reentry_Validations` remains available for the narrower entry suite.

```shell
python -m unittest discover -s validation -p 'test_*.py'
```

Python checks preserve archived hashes, export round trips, index upserts and
failure behavior. JSON writes are atomic per file; multi-file transactions and
concurrent writers to the same index are not provided.
