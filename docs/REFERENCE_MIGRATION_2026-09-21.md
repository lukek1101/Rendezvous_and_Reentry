# Reference configuration and phase-interface migration

**Evaluation stage, 2026-09-22:** [technical report](INTEGRATED_EVALUATION_2026-09-22.md)
adds reproducible bounded design and fixed-plan/component robustness studies,
saved configurations/histories, resource and sensitivity plots, and explicit
failure classifications. Production models/policies were unchanged. Eight
designs reach the entry interface but all exceed aerodynamic domains. Nine
of sixteen selected disturbed Phase 1 cases pass with correction (none without);
six final-approach component cases and eleven standalone entry cases complete.
Counts are not reliability estimates. Paired draws, frozen plan, resource/state
accounting and fresh orbit/profile/Apollo checks passed. Entry step refinement
changes endpoint by 0.01739 m, not evidence of physical accuracy. Perfect
navigation, absent entry guidance and unspecified path/vehicle fuel limits
restrict interpretation. Next qualify high-altitude atmosphere/Mach conventions
alongside entry aero coverage: the helper fixes temperature above 84.852 km.
Only then extend actual-state entry and define full fixed-design robustness
interfaces and operational uncertainty models.

**Implemented capsule replacement:** [Apollo 7 migration](APOLLO7_MIGRATION_2026-09-21.md)
now supersedes the selection-only status below. Apollo 7 preflight trim is the
default capsule, ARD remains selectable, and HORUS-2B is unchanged. Source
area, mass and curvature are qualified; Mach 0.40–27.72 trim data are active
with strict domain checks. Standalone open-loop propagation reaches a declared
7.62 km pre-parachute endpoint. All MATLAB validations passed. The actual orbital
handoff is Mach 28.7946, outside the 27.72 table ceiling; integrated entry is
therefore not yet supported. Next qualify high-Mach coverage using the already
obtained operational data book, then implement bounded guidance with explicit
recovery and control/path contracts. Touchdown is not implemented.

**Capsule reassessment:** [quick comparison](CAPSULE_REASSESSMENT_2026-09-21.md)
recommends Apollo 7/CSM-101 preflight trim aerodynamics as the first capsule
guidance implementation target, retaining ARD for comparison. A primary NASA
report supplies trim CL/CD/AoA across Mach0.40–27.72 plus separate postflight
data. No vehicle was changed. Next dependency is source-qualified reference
area/heating geometry, table transcription and recovery/bank/path contracts
before implementing the bounded controller. This supersedes ARD-first
implementation priority, not the preserved historical ARD results.

**Entry-guidance review:** [method comparison and acceptance checklist](ENTRY_GUIDANCE_REVIEW_2026-09-21.md)
recommends a bounded adaptation of Lu's numerical predictor-corrector,
following review of the supplied Saito and Lu papers, retaining open-loop propagation. No controller has yet
been implemented. The user requires complete entry toward recovery, rather
than only an intermediate atmospheric handoff; the current reference
aerodynamic domains do not cover the paper's low-speed endpoint. Existing reference/profile
checks passed again; no optimization was run. Source provenance, numerical
results, remaining data gaps and implementation acceptance criteria are in
the review. Lu's full-text access dependency is resolved; its signed residual,
energy endpoint, lateral logic and predictive constraint treatment are mapped
to the current core. A bounded positive-right bank convention check passed.
Next dependency is bounded aerodynamic coverage for the complete descent and
explicit recovery-interface/path requirements before controller implementation.

**Signed-axis proximity stage:** [approach-mode record](PROXIMITY_APPROACH_MODES_2026-09-21.md)
adds selectable+R/−R/+V/−V geometry, a mode-independent terminal-range override,
axis-relative corridors and references, and recomputes closing/control in the
physical R/V/H frame. Actual upstream state is retained. The saved−R history
and cost are preserved within regression tolerance. Four matched cases reach
standoff without saturation/approach violations; peak controlled force is
about8.79 N for R and1.48 N for V cases. Partial failed controlled trajectories
retain violation flags. Ideal actuators remain; no contact model was added.
Next dependency is broader matched-case/convergence coverage before extending
the operational envelope or changing navigation/actuator assumptions.

**Orbit-correction follow-up:** [bounded correction record](ORBIT_CORRECTION_2026-09-21.md)
adds opt-in reproducible initial and burn errors, perfect instantaneous state
knowledge, two mid-transfer opportunities, retargeting of the existing arrival
burn and one bounded terminal velocity cleanup. All delivered impulses and
fuel change the propagated state and budget; no state reset repairs errors.
Four identical disturbed off/on pairs passed with correction enabled, using
three added impulses and0.72–1.67 m/s correction delta-V; disabled misses were
1.25–3.37 km. No extra phasing leg was needed for these cases. MATLAB aggregate
and bounded-resource tests passed. Next dependency is credible navigation,
latency, actuator and propellant assumptions before expanding the envelope.

**Nominal orbital planning stage:** [bounded planning record](NOMINAL_ORBIT_PLANNING_2026-09-21.md)
adds `Run_Nominal_Orbit`, an analytical Hohmann seed with bounded numerical
position correction and the existing arrival impulse for velocity matching.
It stops after existing Phase2; no new manoeuvres or entry changes. Integration,
targeting and optimizer limits are separate. Existing grid/custom/Python paths
remain comparison options. Both nominal300→500 km cases passed0.5 m /0.001 m/s
handoff tolerances in1.1–1.5 s; the MATLAB aggregate suite passed. Both fresh
Python comparisons hit90 s budgets without candidates; no infeasibility or
optimality is inferred. Artifacts are in `nominal_stage_2026-09-21`.
Next dependency is independently qualified MATLAB–Python state/physics/export
parity before optional cost refinement; full entry still needs the documented
vehicle-envelope work.

**Latest HORUS choice:** the user-selected Shuttle-inspired speed schedule
now replaces the published time schedule as HORUS's fallback:15° through
2000 m/s, linear to40° at5000 m/s, then40°, using atmosphere-relative speed.
It is explicitly a user assumption. The published schedule remains selectable
as `HORUS_2B_PUBLISHED_TIME`; ARD is unchanged. The
[profile record](REFERENCE_PROFILES_2026-09-21.md) records source provenance,
bounded propagation checks and unchanged aerodynamic limitations.

**AoA follow-up:** the [reference profile stage](REFERENCE_PROFILES_2026-09-21.md)
supersedes the fixed-AoA descriptions below. ARD now uses the digitized mean
AoA/altitude history and HORUS the book's AoA/entry-time history when no user
command is provided. ARD's alpha15–25° constant-body-coefficient rotation is
explicitly an exploratory extension; Mach10–26 is unchanged. HORUS remains
clean/untrimmed with its original missing cells and Mach20 ceiling. Earlier
fixed-command baseline results below remain reproducible with explicit20°/40°
overrides and have not been replaced by claims of flight validation.

The follow-up verified digitization, precedence, profile bounds, compatibility
changes and a20 s time-varying cross-propagator case, then passed the full
MATLAB regression suite. Results are in `profile_stage_2026-09-21`.
No optimization or guidance was added. Next dependency remains a bounded,
source-labeled force-model extension before full-entry verification.

This stage implements the user's selection: HORUS-2B for spaceplane mode and ARD for capsule mode. It follows the [audit checklist](SIMULATOR_AUDIT_2026-09-21.md) and [source review](REFERENCE_VEHICLE_RESEARCH_2026-09-21.md). No guidance algorithm, optimizer run, flight-data fitting or unrelated refactoring was added.

## Configuration and reference selection

`Mission_Config("ARD")` (also the default) builds the capsule reference. `Mission_Config("HORUS_2B")` builds the spaceplane reference. `Run_Mission` selects these from an explicit `overrides.reentry.vehicle_mode` of `CAPSULE` or `SPACEPLANE`; `options.preset` can explicitly select a reference or legacy preset. A contradictory reference/mode pair is rejected. `entry_design.vehicle` can select either reference vehicle independently for standalone studies.

Order is reference preset → `options.system` physical overrides → run overrides → explicitly enabled environment overrides. Environment overrides are now disabled by default. Python JSON supplies Phase 1 manoeuvre parameters only; it no longer changes system masses, atmosphere, entry properties, Phase 2 or Phase 3 configuration. Nonempty resolved run manoeuvre settings still override corresponding JSON parameters; empty angle/delta-V fields permit the verified artifact to supply them. `result.metadata.provenance` retains the supplied overrides and precedence; `cfg.provenance` and `cfg.contract` retain the selected reference and resolved compatibility contract. Reference values remain separate from the original user-override structures.

The four old active definitions (`COMPROMISE`, `HEATLOAD_MIN`, `PAYLOAD_MAX`, `TPS_MIN`) were deleted from `Mission_Config`; no separate mesh/model files with those names existed. Historical baseline artifacts were preserved. `LEGACY_CAPSULE_60KG` retains the former capsule. `LEGACY_PAPER_RLV` retrieves only the former default spaceplane from the frozen audit artifact under an explicit legacy name, for the existing Zhang surrogate. The other three are not selectable. Historical regression fixtures read the archived system; they are not reference-model validation. Saito and Zhang standalone adapters explicitly select their legacy presets instead of accidentally inheriting ARD/HORUS data.

| Quantity | Explicit contract / interpretation |
|---|---|
| Translation | Earth-centred inertial position m and velocity m/s; seven-state core `[r;v;m]`, compatibility mission state `[r;v;q;rates;m]` with mass at14 |
| Relative frame | `[R,V,H]`: radial outward, transverse forward, orbital normal; existing validated transformations reused |
| Angles | Internal radians unless field ends `_deg`; FPA positive outward/climbing; standalone heading clockwise from local north; latitude geocentric on the spherical entry Earth |
| Orbit | Existing fixed X–Z polar initializer; arbitrary `inc` overrides now error instead of being ignored. No new orbit-plane constructor |
| Epoch | Seconds from mission epoch; UTC explicitly `UNSPECIFIED` by default. Earth angle at epoch is a separate explicit setting, not an inferred real flight date or J2000 claim |
| Propulsion | 300 N and200 s remain project assumptions, not HORUS/ARD propulsion specifications. Impulse Isp, finite-burn Isp/thrust, target/base/stack mass, environment and model-source hashes are in the compatibility record |
| ARD mass | Published2800 kg entry vehicle; the existing integrated carrier/add-on policy makes the default initial stack4800 kg. This carrier arrangement is a project mission assumption, not ARD's actual launch architecture |
| HORUS mass | Published26029 kg used in standalone entry. Integrated initialization currently uses the same number as an explicitly labeled wet-mass assumption; burns reduce it and entry never restores the reference mass. A flight-faithful propellant budget remains unresolved |

## Implemented reference physics and validity

**HORUS:** `reference_vehicle.horus_data` transcribes M-692 Tables5.3/5.6 (printed pp.12–13): clean, untrimmed CD/CL, alpha0–45°, Mach1.2–20. Bilinear interpolation uses only nonzero weights. Four missing CD and seven missing CL cells remain NaN; queries needing them error. Exact valid grid points adjacent to a missing cell still work. No extrapolation, subsonic completion, trim solver, control-surface increments or high-Mach extension was invented. Area110 m² and reference mass26029 kg are published; nose radius.8 m comes separately from the 2016 reference case. Constant40° AoA is an explicit open-loop command, not the published reference guidance or a claim of trimmed flight.

**ARD:** published mass2800 kg, diameter2.8 m, areaπD²/4 and derived nose radius3.36 m are implemented. The allowed simplified option is labeled `ARD_FIXED_HYPERSONIC`: approximate graphical AEDB20 levels CA=1.36 and CN=−.07 from the supplied RTO chapter's Figs.29–30. These are not exact tabulated flight measurements. The implementation defines its positive20° AoA magnitude convention with CD=CA cos20°−CN sin20° and CL=CA sin20°+CN cos20°. This gives a constant effective CD/L/D and fixes alpha20°. Its deliberately narrow evidence domain is Mach10–26; all other alpha/Mach queries error. It is not a general Mach/AoA database or ARD's actual closed-loop guidance. No transonic/subsonic/parachute model is supplied.

Both use the existing atmosphere, gravity and Sutton–Graves core. Heat output remains a surrogate, not validated vehicle TPS heating. Published standalone EI conditions are stored under `sys.reference.entry`; they are not copied into an integrated state. The HORUS clean-table domain and ARD hypersonic domain do **not** cover an entire reference descent. A full default reference mission/study may fail at the compatibility or aerodynamic-domain gate. This is intentional; a successful short propagation is not a verified replacement for a complete mission.

## Offline Python results

Every selected archive must pass a resolved compatibility check after local settings and permitted environment overrides. An artifact with a `compatibility` object must match the contract and report `optimizer.success=true`. The contract includes scenario geometry, initial stack mass, target mass, propulsion, atmosphere/drag scope, reference identity/settings, epoch/Earth angle, desired LVLH endpoint and physical-source SHA-256 identifiers. Selection scoring cannot override rejection. Loaded file path, SHA-256 and selection reason are recorded.

Existing version1 Python exports lack this contract and are rejected. The existing Python writers are **not yet parity-qualified exporters of the new contract**; regenerating with those unchanged writers does not remove this limitation. Do not stamp an old archive with the current contract to make it pass. Model alignment and trustworthy exporter construction are a next dependency, not something this stage claims to have validated.

One exception is deliberately explicit: `python_config.mode="FILE"`, `allow_legacy_replay=true`, preset `LEGACY_CAPSULE_60KG`, the original baseline conditions and the exact SHA-256-pinned audited impulsive archive. It emits a warning that the result is nonconverged and lacks a full contract. Changes to geometry, mass, relevant physical constants, atmosphere, noise or LVLH endpoint reject this replay. The original2000 kg Python versus2060 kg integrated stack discrepancy remains identified as legacy evidence, not silently qualified compatibility.

With Python disabled, `CUSTOM_IMPULSE` now requires explicit phase angle, delta-V and gamma. It cannot silently use the historical fallback numbers. `HOHMANN` remains an explicit existing alternative. Offline drag-deorbit designs additionally require the exact propagated ignition state including mass and a matching contract; old circular-start designs are rejected. Explicit manual delta-V and existing conic/apogee propagation remain available.

## Phase and entry interfaces

`mission.state_record` records units/frame/epoch, position, inertial velocity, air-relative velocity, both FPA conventions and mass. Integrated results expose initial, Phase1→2, Phase2→3 and Phase3→4 records. Existing charged manoeuvres remain charged. Capsule separation is recorded separately, with unchanged translation and the existing carrier mass ledger.

`mission.entry_interface` now accepts the actual deorbit terminal event without reconstructing its quaternion, interpolating a chord, selecting a nearest altitude or using an empty-history fallback. It requires a descending terminal state within1 mm of the interface and agreement with terminal history. Missing/off-interface/mismatched events error. Requested altitude and **inertial deorbit target FPA** are recorded separately from the achieved state. Reference air-relative FPA is not automatically interpreted as an identical inertial target; preset deorbit angles are initial targeting assumptions, not closure guarantees.

The integrated `use_paper_entry_conditions` switch now errors when enabled; standalone constructors still accept prescribed air-relative speed/FPA. Their optional second output labels the conditions `STANDALONE_PRESCRIBED`. Constructor and study atmosphere rotation must agree; the study no longer silently forces a different atmosphere co-rotation policy. No translational state is reset to force FPA agreement.

## Verification and reproducibility

Run from the repository root:

```matlab
run('docs/migration_stage_2026-09-21/run_verification.m')
```

```powershell
python -m unittest discover -s validation -p 'test_*.py' -v
```

The driver runs the MATLAB aggregate suite, including the new migration checks, plus one pinned legacy-vehicle hybrid mission. It does not optimize or regenerate footprint/window studies. Results/configuration inputs are saved in `migration_stage_2026-09-21/verification.mat`, `summary.json` and logs. Old baseline artifacts remain unchanged.

Evidence: data knots, missing cells, interpolation and domain rejection; ARD force conversion; configuration FPA precedence; compatible/incompatible/missing-contract cases; air-relative coordinate round trip;20 s prescribed-entry propagations at65 km/4500 m/s, alpha20° ARD and40° HORUS, with.5/.25 s steps. Position refinement differences were4.42e−7 m and2.14e−6 m respectively. These establish numerical consistency only.

The aggregate static/orbit/deorbit/entry/paper/legacy-mission checks passed; six Python IO/archive tests passed. The hybrid replay preserved Phase3's terminal state exactly. Requested FPA was−1.16° inertial; achieved FPA was−1.088120078° inertial and−1.087663215° air-relative. Entry altitude120000 m, inertial speed7924.647559 m/s, stack mass1817.993689 kg; separation preserved total accounted mass. Keeping this mismatch visible is the intended behavior.

## Next dependencies and acceptance criteria

1. **Reference operating envelope:** decide/obtain HORUS trim and high-Mach coverage, or a separately labeled bounded extension; obtain additional ARD coefficient/control information or define another justified surrogate. Acceptance: explicit domains, source/assumption labels, uncertainty and independent comparison data; no silent extrapolation.
2. **Mission mass/epoch/geometry definition:** supply the carrier/propellant budget and any required real flight epoch/plane. Acceptance: published entry mass distinguished from integrated remaining mass; all frame/rotation assumptions explicit; no artificial state restoration.
3. **MATLAB–Python contract producer:** align the actual exporter physics and units before emitting compatibility records. Acceptance: independent matched golden propagation, rejection tests for mass/geometry/environment changes, successful bounded design or separately approved candidate policy, content-traceable archives.
4. **Full replacement verification:** only after1–3, run a bounded actual deorbit-to-entry reference case through its declared endpoint. Acceptance: phase continuity, requested/achieved conditions and uncertainties, numerical convergence and independent physical comparison. Preserve labeled legacy presets until then. No new guidance is required by the current stage.
