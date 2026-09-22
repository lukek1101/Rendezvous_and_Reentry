# Bounded nominal orbital planning — 2026-09-21

This stage provides a fast nominal path for the existing ascending, coplanar
X–Z polar mission with deterministic impulsive burns and orbital drag off.
It uses the existing two-impulse Hohmann architecture and existing Phase 2.
No additional manoeuvre, orbital plane constructor, guidance law or entry
model is introduced. Legacy/custom/Python paths remain explicit comparisons.

## Run and configuration

```matlab
result = Run_Nominal_Orbit(); % ARD stack, Phase 1 + existing Phase 2, stops before deorbit
% Example tolerance overrides:
settings.phase1.nominal.position_tolerance_m = 0.5;
settings.phase1.nominal.velocity_tolerance_m_s = 0.001;
result = Run_Nominal_Orbit(settings, struct('verbose',false));
```

The equivalent ordinary runner configuration is `python_config.mode="NONE"`,
`phase1.mode="HOHMANN"`, `phase1.hohmann_method="NOMINAL_TARGET"`, with
`options.stop_after_proximity=true`. The normal `Run_Mission` default remains
the explicit custom/archive workflow; this stage does not make unsupported
full reference descents succeed. `phase1.hohmann_method="GRID_SEARCH"` selects
the previous Hohmann search. `CUSTOM_IMPULSE` and the existing Python
`optimize_j2_hohmann_rendezvous` / `minimize_delta_v_on_zero_distance_manifold`
remain available. Their legacy export compatibility limitations still apply.

`mission.plan_nominal(sys,Xc,Xt,goal,max_wait,settings,max_burn)` is the
nonthrowing diagnostic API for bounded numerical nonconvergence and evaluated
constraint failures. Invalid inputs/unsupported physics throw. Mission execution
requires `plan.success`; otherwise it aborts before handing off an unaccepted
candidate. Solver status, checked constraints, requested velocity, actual
LVLH position/velocity, work counts, elapsed time and burn vectors are retained
in `result.phasing.history.planning`.

## Algorithm and interfaces

1. Compute a Keplerian Hohmann half-period and departure phase from the initial
   radii and requested target-relative position. Convert the current phase to
   the first nonnegative wait using the difference of circular mean motions.
   Reject a seed beyond the configured wait horizon; do not add another orbit
   or search another architecture automatically.
2. Co-propagate chaser and target through that wait with the existing shared
   J2 dynamics (`mission.proximity_dynamics`, `orbit_core.gravity_j2`). Seed
   the departure burn with the analytical transfer velocity at the propagated
   departure location.
3. Hold wait and transfer duration fixed. Correct the two in-plane components
   of the departure impulse using a finite-difference terminal-position
   Jacobian and bounded Newton updates, with at most three damping trials.
   This is a root solve, not a delta-V minimization.
4. At the propagated arrival position, use the existing arrival impulse to
   match the requested rotating-LVLH velocity (default zero). The target frame
   and its h/r² rotation convention come from `orbit_core.relative_state`.
   Neither position nor epoch is reset. Both impulses change mass through the
   same rocket equation; their costs are charged to Phase 1.
5. Execute the existing Phase 2 unchanged. Its handoff impulse remains present;
   for a zero-velocity nominal arrival its magnitude is numerical roundoff.

The old custom solution places its first impulse in Phase 1 and velocity
matching in Phase 2. Comparing Phase 1 costs alone therefore misrepresents
the tradeoff: report Phase 1 + handoff and the total through Phase 2.
Nominal transfer time is fixed and wait is analytical; no global or local
delta-V optimality is claimed. Reusing a historical parameter triple at a
different initial phase is a comparison experiment, not a compatible design.

## Tolerances and bounded failure semantics

| Category | Default / meaning |
|---|---|
| Numerical integration | ODE45 relative tolerance1e−11, position absolute tolerance1e−4 m, velocity absolute tolerance1e−7 m/s, mass absolute tolerance1e−9 kg, max step60 s |
| Terminal targeting | Euclidean LVLH position error≤0.5 m and velocity error≤0.001 m/s; velocity command defaults to[0,0,0] m/s |
| Correction work | At most8 iterations and40 transfer evaluations; 0.01 m/s finite-difference step, update norm≤25 m/s, damping factors1/.5/.25 |
| Wall-time guard |60 s, checked at dynamics calls and before each transfer evaluation; not a real-time scheduling guarantee |
| Optional optimizer stopping | Existing Python SLSQP `ftol` and iteration budget are separate from terminal acceptance; this benchmark uses ftol1e−9, three feasibility iterations and two SLSQP iterations, plus an external90 s process limit |
| Checked physical/configuration bounds | Wait horizon, configured per-burn delta-V limit and sampled positive altitude. These checks are not exhaustive collision/path-constraint certification |

`WORK_LIMIT`, `ITERATION_LIMIT`, `SINGULAR_JACOBIAN` or `NO_DESCENT` mean the
numerical procedure did not establish an accepted solution. They do not prove
the mission infeasible. `WAIT_BOUND_VIOLATED` identifies this seed's explicit
horizon violation. `DEMONSTRATED_CONSTRAINT_VIOLATION` identifies an evaluated
burn/altitude bound violation even when terminal correction converged.
`target_tolerances_met` is separate from `constraint_status`; inspect both.

## Benchmark and reproducibility

The comparison uses the current ARD mission stack4800 kg (carrier2000 +
capsule2800), target2000 kg,300→500 km, J2 on, drag off, Isp200 s, chaser
argument0°, and initial phases90° and60°. The handoff is[0,−5000,0] m and
zero rotating-LVLH velocity. All successful handoffs run the unchanged hybrid
Phase 2 with its normal five closing-time candidates. A failed50 m position
gate inhibits Phase 2; no new correction is added to rescue it.

`nominal_stage_2026-09-21/benchmark.json` is the authoritative final numerical
comparison, including full achieved LVLH vectors, remaining mass, runtimes,
handoff cost and downstream totals. The grid comparison uses its normal60 s
coarse spacing and±180 s /2 s refinement, but bounds its horizon to1.1 times
the analytical nominal wait. It is a bounded comparison, not an exhaustive
search over the old full synodic horizon. The old search also estimates arrival
at a fixed transfer time but executes to a radius event; this discrepancy
remains in that preserved comparison path.

| Initial phase / method | Planning + Phase1 execution (s) | Position miss (m) | Phase1 delta-V (m/s) | Phase2 handoff correction (m/s) | All Phase2 (m/s) | Total through Phase2 (m/s) |
|---|---:|---:|---:|---:|---:|---:|
|90° nominal |1.457|0.000181|114.072760|<1e−12|7.477427|121.550187|
|90° bounded grid |6.289|17555.4|123.592857|Not executed|Not executed|Not available|
|90° historical replay |1.351|0.134334|57.992997|57.419455|64.891782|122.884779|
|60° nominal |1.125|0.001299|113.256818|<1e−12|7.473014|120.729832|
|60° bounded grid |6.294|15745.3|120.071615|Not executed|Not executed|Not available|
|60° historical replay |2.025|3136.42|57.992997|Not executed|Not executed|Not available|

Both nominal handoff velocity errors are below1e−12 m/s in the selected
rotating-frame convention. The demonstrated grid/replay position misses are
candidate violations, not proof that a different solution cannot meet the
constraints. The nominal cost difference is an observed case result, not
an optimality claim. Phase2 runtimes are recorded separately in the JSON.

Both fresh Python attempts exceeded their external90 s limits without
returning a candidate. Thus fresh optimized cost/state are **unavailable**,
and no equal-quality speedup over a completed optimizer is claimed. The
additional historical comparison reads the SHA-identified audited impulsive
parameter triple and freshly propagates it in MATLAB. Its runtime excludes
the unknown historical optimization time. The original optimizer reported
iteration-limit nonconvergence. Its original2000 kg mass is not adopted:
drag-free impulsive translation is mass-independent, and current4800 kg fuel
and downstream dynamics are recomputed. The60° replay deliberately tests
transferability and must not be silently reused as a new design.

Reproduce from the repository root:

```powershell
python docs/nominal_stage_2026-09-21/benchmark_optimizer.py
matlab -batch "addpath('docs/nominal_stage_2026-09-21'); benchmark();"
matlab -batch "addpath('validation'); Run_All_Validations();"
```

No Python mission archive/index/latest alias is written. The benchmark logs
retain earlier failed/coarse attempts; `benchmark.json` contains the final
normal-resolution comparison and `coarse_grid_attempt.json` preserves the
earlier bounded coarse-search evidence. Timing includes function startup/JIT
effects and is machine-dependent, not a statistical performance claim.

## Verification, limitations and next dependency

Validation covers terminal position/velocity, exactly two burns, fuel accounting,
unchanged position across the arrival impulse, integration refinement, work/time
limits and converged-but-constraint-violating cases. Existing orbit, deorbit,
entry and mission regressions all passed (147-file code analysis included).
Tenfold tighter integration tolerances changed terminal inertial position by
0.01713 m. Consequently the submillimetre solver residual is not a claim of
submillimetre absolute propagation accuracy. Numeric results/logs are stored
in `nominal_stage_2026-09-21`.

Finite burns, thrust uncertainty, orbital drag, nonpolar/noncoplanar transfers
and descending transfers are explicitly outside this nominal path. A missed
analytical wait seed is not repaired by an unbounded search. Optional fuel
optimization is not run during nominal targeting. Earlier vehicle AoA and
legacy changes remain intact.

Next dependency: if cost refinement is desired, first qualify MATLAB–Python
propagation and archive/export contracts against this explicit terminal state
and mass/epoch definition. Acceptance: bounded runtime, independently checked
position/velocity tolerances, full Phase1+handoff+Phase2 cost accounting, and
separate numerical/constraint status. Full deorbit/entry remains dependent on
the previously recorded vehicle-domain and mission-definition work.
