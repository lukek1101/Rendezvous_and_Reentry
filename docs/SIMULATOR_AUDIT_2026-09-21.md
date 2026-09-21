# Simulator audit and migration baseline — 2026-09-21

This audits the **current working tree**, based on commit `e4b6db03935efa5afc7b33843c1cc21a2a08a46c`, including pre-existing uncommitted window-study changes. No simulator implementation or existing study output was changed. Evidence, replay driver, input SHA-256 manifest, original Git status/diff, and newly measured results are in [audit_baseline_2026-09-21](audit_baseline_2026-09-21/). Local source is the authority here; cited paper claims were not independently re-audited.

The active simulator is a connected **3-DOF translational research model**, with impulsive handoff/closing and finite-force final proximity control. It is not a closed-loop 6-DOF mission, docking simulation, or reproduced paper optimum. Two important migration boundaries are the position-only Phase 1 capture (about 57.42 m/s relative speed remains) and the approximate deorbit FPA target (default target −1.16°, measured hybrid interface −1.08812°).

## 1. Active execution and dependency map

| Entry/path | Implemented execution | Scope and dependencies |
|---|---|---|
| `Main_Mission_Simulator.m` | Calls `Run_Mission(...,plot=true)` and publishes compatibility variables | Interactive wrapper; no separate physics path |
| `Run_Mission.m` | Merge physical/options defaults; seed/restore RNG; `mission.configure` → `mission.run`; metadata/log; optional plots | Root plus MATLAB packages must be on path; keep `legacy/` off path. Default is headless. `stop_after_proximity=true` returns before deorbit and rejects full-mission plotting |
| Phase 1 | `Phasing_Propagator`: phase-angle wait → custom burn → capture/closest approach; target co-propagates | Default `CUSTOM_IMPULSE` with impulsive execution. Finite burn is an explicit alternative. `HOHMANN` has numerical wait selection, transfer and circularization. `MULTI_HOHMANN` is executable but preliminary, not the nominal validated path |
| Phase 2 | `mission.proximity` → `autonomous_proximity` → `plan_closing` → `track_rbar` | Default hybrid: charge velocity-null impulse; 60 s free dwell; screen five transfer times; two-impulse closing to −500 m R-bar; ideal-force feedback to −250/−30 m and holds. CW seeds plus up to seven nonlinear corrections per candidate; `ode45` screening, RK propagation/control execution |
| Phase 2 alternative | `LEGACY_IMPULSIVE`: optional S2 cleanup, velocity trim, cycloid to first V-bar crossing, CW hops/refinement, terminal stop | Implemented baseline, not current default. All translational corrections are charged impulses, not teleportation |
| Phase 3 | `mission.deorbit` | Precedence: enabled apogee schedule → drag-enabled design/manual burn → conic FPA-targeted departure impulse. Public phase mode accepts only `HOHMANN`; label alone does not identify the actual burn algorithm |
| Phase 3 optional schedule | `apogee_deorbit` | Finite burns with apogee events, cooldown, duration/mass/time limits; independent of Phase 1 burn model and Python design. Default disabled; 150 m/s split 20/20/60%, 300 N, 200 s Isp, 900 s maximum burn, 600 s cooldown |
| Phase 4 | `entry_interface` → `mission.entry` → `Reentry_Propagator` → `reentry_core` | Entry vehicle propagation, atmosphere-relative aero, heating/LOS diagnostics, terminal events. RK4 plus refined altitude/speed crossings. Dynamic relay is the mission **target**, not the separated carrier |
| Shared orbital physics | `orbit_core.gravity_j2`, `relative_state`, `Env_EOM`, `Atmospheric_Drag_Acceleration` | SI/ECI translation; LVLH `[R,V,H]`; frame angular-rate approximation `h/r²`. `Env_EOM` has a 6-DOF branch, but the integrated nominal mission does not use an attitude-control loop |
| `Run_Footprint_Study` / `entry_design` | Separate bank-profile propagation, sampled footprints, bounded endpoint correction, fixed-policy dispersion | `ode45`; shared entry physics but separate entry controller/termination policy. Standalone example prescribes 120 km, 7500 m/s air-relative, −3° FPA, 90° heading at lat/lon `[0,0]`; not mission-generated entry |
| `Run_Deorbit_Window_Study` | Uses actual successful Phase 2 terminal state, free wait, deorbit, entry target search | Current uncommitted implementation is active. Default 0:600:86400 s grid, zero-bank screen, `fminbnd` time refinement and bounded bank correction. Endpoint is 20 km, target `[36.5,130.5]`; sampled witnesses, not continuous windows or touchdown. Waiting is free coast, not docked stationkeeping |
| `Run_Paper_Reproduction_Suite` | Zhang/Saito source-condition, equation, coordinate/grid audits; optional shared-core forward adapters | Default does not optimize or propagate a full trajectory. Forward models explicitly surrogate. `legacy/` snapshots and historical GNC controllers are excluded from nominal execution |

## 2. Configuration precedence (actual code)

1. `Run_Mission` recursively merges `options` into its defaults, including `options.system` into `Mission_Config()`. Unknown override fields error. Physical/scenario defaults can therefore be overridden programmatically despite “sole authority” wording in documentation.
2. `mission.configure` merges user run overrides into **all** `Mission_Run_Config(sys)` defaults. It applies this resolved run configuration once before Python archive selection.
3. Selected JSON overlays supported system/phase fields; then resolved run settings are reapplied. Consequently **nonempty run defaults**, not only explicitly supplied user overrides, win over JSON. Empty Phase 1 angle/delta-V/gamma fields deliberately retain JSON values. Empty optional numerical fields generally retain the lower-layer default; inspect getters rather than treating `[]` as deletion.
4. Permitted environment overrides apply last to their respective destinations; derived Hohmann phase is refreshed. `runtime.allow_environment_overrides=false` suppresses these overrides. Phase 1/2 use a separate `proximity_system` copy with orbital drag disabled when `apply_from_phase="PHASE3"`; resolved `system` is used thereafter. Entry aero remains enabled independently of orbital drag.
5. Capsule `use_paper_entry_conditions=true` forces **120 km and 1.16° FPA magnitude** during run-to-system application. This supersedes run Phase 3 altitude/FPA inputs. It does not prescribe the actual achieved FPA or overwrite speed with 7970 m/s.

**Exceptions to the documented authority boundary:** JSON `maneuver.initial_mass_kg` replaces `sys.Chaser_Mass_Init`; JSON can also replace target mass, Cd/area and rotation rate when those are not reapplied by run settings. Thus `options.system` is not an unconditional final physical override. JSON scenario altitude/initial angles are not applied to the mission. JSON hash/schema/optimizer-success metadata is not a runtime acceptance gate.

**Archive selection:** environment file (`RENDEZVOUS_CONFIG_JSON`, then legacy `RENDEZVOUS_CONFIG`) precedes even `NONE`/`FILE`; otherwise `NONE/OFF/DISABLED`, `FILE`, or index selection. Index supports case/hash/environment selectors. AUTO hard-filters burn model, scores altitude pair and drag, and chooses the last equal-scoring index entry. It does **not** strictly validate geometry, initial angles, mass, or optimizer success; ordering, rather than an independent timestamp sort, breaks ties. Missing index falls back to the latest alias even before exact case/hash handling. Exact `CASE_ID` matches bypass the burn filter; HASH selection still filters burn model. Pin an archive file and record its content hash.

Other environment controls are `RENDEZVOUS_BURN_MODEL`, `RENDEZVOUS_ATMOSPHERIC_DRAG`, `RENDEZVOUS_REENTRY_SHAPE`, and `RENDEZVOUS_PHASE3_MODE`. Index case/hash variables apply within index selection. The default allows environment overrides; the baseline below disables them.

With no selected JSON, hard-coded Phase 1 fallback values are 3.5663°, 64.2342 m/s and −1.416°. They resemble the finite-burn archive while the default execution is impulsive; `NONE` is therefore not an equivalent replay of the nominal pinned case.

**Deorbit design selection differs:** AUTO does not search `drag_deorbit_solution_index.json`; it loads the configured file, as FILE does. With orbital drag on, nonempty manual delta-V takes priority. JSON interface altitude and FPA are checked when present, but actual ignition state/start geometry and mass/environment compatibility are not comprehensively enforced. Default saved drag design targets 4°, so simply enabling drag with default paper-condition capsule settings causes an FPA mismatch rather than a valid 1.16° design.

## 3. MATLAB–Python coupling and optimizer boundaries

- Coupling is offline JSON, not `py.*`, subprocess optimization, or a live bridge. MATLAB runs without invoking Python. Python uses km/km/s internally and exports named SI maneuver fields; MATLAB uses metres/m/s, with explicit degree/radian parsing.
- `J2PolarHohmann.py`: polar J2 propagation, `solve_ivp`, scalar event/timing searches. `J2PolarHohmannShooting.py`: finite-difference shooting, then SciPy SLSQP; variables are phase angle, delta-V and gamma. The selected objective is first burn plus terminal relative speed; it does not optimize all mission phases. The actual SLSQP constraint list uses equality residuals; the nearby 0.1 m inequality function is defined but unused.
- `DragDeorbitDesigner.py`: repeated single-burn/coast propagation and scalar delta-V root search, with bounded scalar minimization fallback; optional finite retrograde firing is not a full low-thrust optimal-control solution. Designing from its prescribed circular polar start does not establish equivalence with the actual Phase 2 terminal state.
- `mission_io.py` shares serialization, short hashes, aliases, archives and index upserts. Writes are atomic **per file**, not a multi-file transaction or concurrent-writer protocol. `requirements.txt` has unpinned NumPy/SciPy/Matplotlib. MATLAB nominal paths use base numerical functions; `atmosisa` is an optional toolbox-dependent branch with fallback.
- Physics is not identical across languages: MATLAB J2 `1.08263e-3`, Python `1.08262668e-3`; MATLAB lower atmosphere uses layers/optional `atmosisa`, Python uses a log-density table. Shared gravity/entry packages unify MATLAB callers, not the Python models.
- Both stored Phase 1 archives have `optimizer.success=false` (iteration limit). The impulsive archive has Python miss `2.161372574e-7 km`, terminal speed `57.419473595 m/s`, two-impulse objective `115.412470337 m/s`; the finite archive has miss `2.033328511e-6 km`, duration `421.291658 s`, objective `146.192489401 m/s`. These are recorded candidate outputs, not proven optima. Fresh MATLAB impulsive miss is `0.134334108 m`.

## 4. Geometry, vehicle assumptions and phase handoffs

| Boundary/quantity | Actual behavior and migration implication |
|---|---|
| Initial orbit | `mission.run/circular_polar_state` forces `r=r[cos(u),0,sin(u)]`, tangential X–Z velocity with J2-adjusted speed. `sys.inc=pi/2` is declarative and is **not read** by this initializer. No arbitrary inclination/RAAN/epoch initial-state API is used. Defaults: 300/500 km insertion/target, u=0°, target +90° |
| Earth/vehicle | `mu=3.986004418e14`, Re=6378137 m, g0=9.80665, target/base chaser=2000 kg each, Isp=200 s, reference thrust=300 N. Default capsule adds 60 kg to the stack (2060 kg). Inertia `[800,800,600]`, sensor-noise fields and legacy attitude gains do not create active navigation/attitude dynamics |
| Phase 1 → 2 | Actual propagated position/velocity/mass and same-epoch target are passed. Capture primarily concerns position; no automatic velocity-match solution. Hybrid requires S2 position within 50 m, then charges approximately 57.419455 m/s to null relative velocity. No uncharged velocity reset. Phasing reconstructs the 14-state with identity quaternion and zero rates |
| Proximity geometry | S2 `[0,-5000,0]` m; autonomous targets negative R-bar 500/250/30 m. Research limits: 300 N ideal vector force, 0.17 m/s final approach, 0.01 m/s capture speed, 0.25 m position capture, 20 m keep-out, 10° cone. These are defaults/assumptions, not flight-qualified vehicle specifications. Passing ends at standoff, not docking/berthing contact |
| Phase 2 → 3 | Successful terminal state continues directly into deorbit; no relocation to a synthetic circular orbit or docking-state reset. Failure inhibits downstream phases. Drag may switch on here by policy. Conic departure is a charged velocity impulse; an FPA target does not imply terminal-angle closure under J2 |
| Phase 3 → 4 | `entry_interface` linearly interpolates bracketing position/velocity/mass, or chooses the nearest recorded altitude if no crossing exists; empty history uses fallback state/time zero. Reconstructs identity quaternion/zero rates. This helper alone does not certify a true interface crossing; nominal deorbit has its own event handling |
| Separation | Default zero-impulse capsule separation preserves translation, changes active mass to 60 kg and retains residual carrier mass in the ledger. Carrier atmospheric trajectory is **not propagated**. ATTACHED keeps stack mass with capsule aero; it is a sensitivity model, not a full stack aerodynamic design |
| Relay/time | Target state at start of Phase 3 is independently advanced to entry for relay use. Paper TDRS option prescribes 42164 km geocentric radius, 77° longitude, equator. Integrated mission has no UTC epoch; Greenwich angle defaults to 0°. Window study accepts an optional mission-t=0 UTC/approximate GMST convention |
| Entry vehicle | Capsule area 0.554 m², Cd=1.3, L/D=0.25, trim AoA=0°, surrogate nose radius 0.420 m. Four spaceplane shape tables prescribe dimensions/areas/nose radii; nominal COMPROMISE uses area `pi*1.944*1.296/4`, nose 0.054 m. Polynomial spaceplane aero and speed-scheduled AoA supersede legacy constant-Cd/L/D tables on that path |
| Entry controls/events | Integrated bank is constant (default 0°); spaceplane AoA scheduled, capsule trim constant. Central spherical entry gravity is default versus orbital J2. Capsule speed event 240 m/s, ground safety floor 0 m, maximum 2500 s; spaceplane default altitude event 20 km. “Guidance active” capsule diagnostic is not a connected bank controller. No parachute descent or touchdown propagation |
| Independent study entries | Footprint initial lat/lon, speed, FPA/heading and paper-study entry constructors are externally prescribed. They must not be mistaken for continuous Phase 3 handoffs. `entry_design` adds a rate-limited bank state and speed-fraction command profiles, unlike constant-bank integrated Phase 4 |

Implemented but limited: apogee scheduling, finite-force R-bar tracking, target/window witness search, fixed-policy perturbation studies, public Saito guidance algebra and Zhang augmented bank equations. Not integrated/established: arbitrary orbital-plane scenario construction, navigation filter, realistic attitude/RCS allocation, finite-force whole-rendezvous execution, docking/contact, executable safe abort, full closed-loop capsule paper guidance, global reachability, landing, RF/plasma link model, or paper optimal trajectories. Thermal output is a Sutton–Graves surrogate, not validated paper heat compliance. Entry-design default thermal/load limits are infinite.

Documentation drift: README's later Phase 2 narrative still describes the selectable cycloid/hop baseline; `CODE_REVIEW_KR.md` says the finite-force controller is not implemented and discusses old failure policy; those statements predate the active hybrid path. Broad “no footprint/dispersion” limitation wording is also stale for the separate study tools. Conversely, 6-DOF equations and paper guidance function names are not evidence of integrated closed-loop execution.

## 5. Reproducible baseline and existing results

Replay from repository root:

```powershell
python -m unittest discover -s validation -p 'test_*.py' -v
matlab -batch "run('docs/audit_baseline_2026-09-21/run_baseline.m')"
matlab -batch "addpath('validation'); Check_Project_Code()"
```

The audit driver is a harness only. It runs small core/deorbit/paper checks, the existing pinned legacy mission regression, then **one** pinned hybrid mission. No Python optimization, footprint regeneration, window scan, or Monte Carlo was run. The hybrid runner itself performs its existing bounded five-candidate proximity screen. Avoid `addpath(genpath(root))`, which introduces legacy name collisions.

Environment: Windows PCWIN64, MATLAB R2025a Update 1 (`25.1.0.2973910`), Python 3.13.14, NumPy 2.4.6, SciPy 1.17.1, Matplotlib 3.10.9. Baseline pins `configs/python_runs/impulsive_drag-off_h300.0-500.0_phase90.0_31c1e45c_410e0069_20260630_134559Z.json`; settings hash `31c1e45c970413bc`, result hash `410e00697144186c`. Seed=123, environment overrides=false, thrust noise=false, orbital drag=false, impulsive Phase 1, default paper-condition capsule. Full resolved configuration, histories and initial RNG are saved in `baseline.mat`; manifest hashes are stronger content identifiers than Git HEAD alone.

Aerospace Toolbox/Blockset and Simulink are installed; `atmosisa` resolves to the Aerospace Toolbox on this host. The focused deorbit fixture explicitly uses the built-in atmosphere. To replay the frozen hybrid resolved configuration without re-reading changed defaults/aliases, load `baseline.mat`, restore `result.metadata.initial_rng`, and call `mission.run(result.config)`; use a caller RNG cleanup if embedding that call in another experiment.

| Fresh measured result | Legacy regression | Current hybrid |
|---|---:|---:|
| Phase 1 elapsed / terminal miss | 32996.731705844 s / same Phase 1 | 32996.731705844 s / 0.134334108 m |
| Phase 2 duration | 16778 s | 10680 s |
| Phase 2 delta-V | Existing fixture: 61.128982557 m/s | 64.891782444 m/s |
| Phase 2 terminal position error | Existing fixture: 0.015596320 m | 0.000710994 m |
| Phase 2 terminal speed | Existing fixture: ~1.03e-13 m/s | 3.831642435e-6 m/s |
| Phase 3 time to interface | 2173.238658595 s | 2159.841734018 s |
| Hybrid interface | — | 120000 m, 7924.647559121 m/s, −1.088120078° |
| Mass after phases 1/2/3 | 1999.981205554 / 1938.608942624 / 1821.636469201 kg | 1999.981205554 / 1934.893298310 / 1817.993689105 kg |
| Entry outcome | PARACHUTE_SPEED, 60 kg | PARACHUTE_SPEED, 60 kg; total accounted mass 1817.993689105 kg |

Legacy values marked “existing fixture” are from `output/proximity/study_results.json`, not newly extracted by the architecture test. The fresh test checks frozen orbital state to 1e-4, masses to 1e-7 kg, time tolerances, headless/external-directory execution, RNG restoration and invalid inputs. Hybrid run elapsed ~19.94 s on this host; this is not a benchmark claim.

Existing artifacts, **inspected without regenerating**:

- `output/proximity/study_results.json`: chosen closing 4500 s; hybrid max final force 3.775128 N; half-step terminal difference 0.000204278 m and delta-V difference 4.02235e-6 m/s; 10/10 initial-delivery perturbation trials (seed 1809). Trials exercise final R-bar control, not full mission uncertainty or a navigation filter.
- `configs/drag_deorbit_runs/`: two July 24 archives for 500 km / 120 km / 4°; saved delta-V 274.955631 m/s, predicted coast 1200.292874590 s, FPA −4.000003977°. Neither is a fresh optimization or a matched default capsule mission design.
- `output/footprint/footprint_results.json`: separate prescribed-entry two-vehicle example, 20 km endpoint, unbounded load/heat envelope. `validation_results.json`: 150 m/s three-burn test, interface FPA −1.932745084°, duration 13219.405777715 s, endpoint convergence 0.0473051 m, deterministic two-trial dispersion. `target_refinement.json`: propagated witness miss 24.7331437 m within 100 m after 20 correction evaluations.
- `output/deorbit_window/verification.json`: 159 samples, **zero verified candidates**, best miss 248197.203188 m at delay 9215.096700 s, finer-propagation endpoint change 0.176275 m. CSV/MAT/plot accompany it. This is an unresolved search result, not proof the target is unreachable.

## 6. Validation coverage and gaps

Fresh bounded checks passed: six Python serialization/archive tests; orbital gravity-gradient/LVLH invariants; shared-entry extraction equivalence (frozen-state max difference 2.79397e-9, event-time and RAAP error zero); impulsive and finite deorbit execution; selected paper audit; pinned legacy mission; one pinned hybrid mission. The finite-deorbit test uses **20000 N** to keep the fixture fast; it does not qualify the nominal 300 N case.

Static `Check_Project_Code` also passed for 132 active MATLAB files. Final SHA-256 comparison found zero changes among the pre-existing source/configuration/document/JSON inputs in `input_manifest.json`. Python test result and runtime versions are recorded in `python_checks.txt`; MATLAB checks are in the two log files.

Existing suites additionally cover entry events, LOS/RAAP/blackout/relay modes, capsule accounting, paper adapters/coordinate round trips and uncertainty grids. `Run_All_Validations` invokes analyzer, orbit, deorbit, entry-core, entry, paper and **legacy** mission regression. It does **not** invoke `Study_Proximity_Options`, `Validate_Entry_Design`, `Validate_Footprint_Target` or `Validate_Deorbit_Window_Study`; these are separate checks with output-writing behavior. The full entry/paper-forward suites and those separate studies were not rerun in this audit.

Coverage is predominantly regression/internal consistency. Missing acceptance evidence includes strict archive compatibility and precedence matrices; a shared MATLAB/Python physical golden model; arbitrary inclination/RAAN scenarios; broad default-hybrid failure/dispersion coverage in the aggregate runner; continuity and terminal FPA acceptance across all branches; independent aero/heating validation; realistic finite-force handoff; and touchdown or full paper reproduction. Preserve these distinctions when reporting “all validations pass.”

## 7. Migration checklist (no implementation authorized here)

| Order | Dependency / action | Acceptance criterion |
|---|---|---|
| 1 | Freeze this working tree and data before migrating | Preserve HEAD **and** dirty/untracked inputs; input hashes match; replay logs, resolved configs, versions, RNG and result metadata retained |
| 2 | Define configuration/schema contract; depends on 1 | Tests enumerate default → JSON → run → environment, mass exceptions, capsule FPA override and drag scope. Exact selection either verifies all required scenario/model fields or fails explicitly; nonconverged optimizer candidates remain clearly labeled |
| 3 | Define geometry/frame/epoch and common physics contracts; depends on 2 | Explicit SI/angle conventions, initial state or orbital elements, target epoch, atmosphere branch and constants; rotated/nonpolar golden cases and MATLAB–Python parity tolerances agreed before behavior changes |
| 4 | Specify phase interface records; depends on 2–3 | Each handoff includes actual state, mass ledger, epoch, event/success reason and tolerances. No uncharged translational reset; attitude reconstruction/separation declared. Missing entry crossing rejected or explicitly classified |
| 5 | Decide rendezvous endpoint and actuator fidelity; depends on 4 | Position **and speed** limits specified; ~57.42 m/s handoff impulse separately budgeted or replaced only under a new validated model. Standoff/abort/docking claims distinguished; current hybrid and legacy replay tolerances preserved until intentionally revised |
| 6 | Align deorbit targeting and entry model; depends on 3–5 | Design replay starts from actual Phase 2 state and correct mass/environment; FPA tolerance checked on propagation. Apogee/impulsive/finite branches share declared success and mass criteria; 60 kg separation preserves total mass |
| 7 | Migrate studies/guidance only after model choices; depends on 3–6 | Prescribed vs mission-derived entry explicit; paper-surrogate blockers retained; finite thermal/load limits chosen for mission claims; every reachable claim has a propagated witness and unresolved scans remain unresolved |
| 8 | Establish bounded regression gate; depends on 1–7 | Fast core/IO checks plus pinned legacy and hybrid mission tests; selected failure/epoch/mass tests included explicitly. Optimization requires a named objective, parameter bounds, evaluation/time cap, matching model and saved convergence status; no unbounded reoptimization in routine tests |

Migration is ready for planning from this baseline, not for claims of higher physical fidelity. No new feature or model correction was implemented by this audit.
