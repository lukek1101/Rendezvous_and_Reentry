# Apollo 7 capsule preset migration

Follow-up: the [2026-09-22 evaluation](INTEGRATED_EVALUATION_2026-09-22.md)
confirms the integrated-domain boundary and adds conditional resource/error
maps. The high-altitude Mach calculation also needs qualification: the current
atmosphere fixes temperature at 186.946 K above 84.852 km. The next dependency
is therefore atmosphere/Mach convention **and** compatible aerodynamic coverage,
not an automatic extension or clipping of the table. No physics changed in that stage.

The default capsule is now `APOLLO7_PREFLIGHT_TRIM`. HORUS-2B remains the
spaceplane reference. `ARD`, `LEGACY_CAPSULE_60KG` and `LEGACY_PAPER_RLV`
remain explicit comparison/replay presets. This stage changes the reference
vehicle and verifies open-loop propagation; it does not implement entry guidance.

**Integrated-mission limitation:** actual nominal deorbit handoff is Mach
28.7946, above the table's 27.72 ceiling. The default capsule is implemented,
but the orbital mission cannot yet propagate through entry with this strict
preset. Its state is retained and the domain error is reported. The already
obtained operational data book is the next source to qualify for high-Mach
coverage; no inaccessible-paper request is pending for that step.

## Source and model contract

| Input | Implemented value / interpretation | Primary source |
|---|---|---|
| Aerodynamics | 12 preflight trim knots, Mach 0.40–27.72; linearly interpolate CL, CD and trim angle in Mach | [MSC 69-FM-89](https://apollojournals.org/afj/ap07fj/pdf/a07-entry-postflight-analysis-19740072689.pdf), Table IIb, printed p21 / PDF p28 |
| Angle convention | Simulator angle = 180° minus published Apollo body angle; trim range 12.83°–27.03°. One-dimensional trim curve, **not** a rectangular Mach/AoA database | Same table; positive published CL gives positive lift in the existing bank convention |
| Reference entry mass | 12,364.1 lb × 0.45359237 = 5,608.261421917 kg | Same report, printed p8 / PDF p15, post-separation weight |
| Aerodynamic reference area | 129.4 ft² × 0.3048² = 12.021653376 m² | [Operational Data Book Rev. 2](https://www.ibiblio.org/apollo/Documents/HSI-208962.pdf), Table 6-1 footnote, printed p6-5 / PDF p650 |
| Reference diameter | 154.0 in = 3.9116 m | Same footnote; use published area normalization, not a newly calculated circle area |
| Heating radius | 4.694 m spherical aft heat-shield curvature | [NASA 20070025192](https://ntrs.nasa.gov/api/citations/20070025192/downloads/20070025192.pdf?attachment=true), p3, Block II geometry |
| Entry metadata | BET geodetic latitude 29.926°, longitude −92.444°, altitude 397,802.1 ft, inertial speed 25,848.512 ft/s, inertial FPA −2.055°, inertial azimuth 87.552°, GET 935,608 s | MSC 69-FM-89, Table Ia, printed p19 / PDF p26; metadata only |

Source hashes and inspected pages are in [sources.json](apollo7_stage_2026-09-21/sources.json).
The preflight coefficients are deliberately paired with the same flight's
reported separation mass: a documented reference construction, not an exact
preflight replay. The later operational-data-book coefficients are not mixed
into Table IIb. Block II curvature supplies only the existing Sutton–Graves
surrogate; TPS response, recession and heating compliance are not validated.

Table IIa postflight values remain comparison material in the source PDF,
not the active model. Pages 7–8 explain that postflight L/D was inferred from
PIPA data, CD and trim came from six-degree-of-freedom simulation, and CL was
calculated. These are not three independent flight measurements. The report
used a reconstructed atmosphere; ISA76 here is a separate project approximation.

## Configuration, state and validity

`Mission_Config()` and mission entry points select Apollo by default.
`Mission_Config('ARD')` and `options.preset='ARD'` retain the previous reference.
Selecting spaceplane mode continues to select HORUS-2B. No propulsion values,
orbital architecture or HORUS aerodynamic cells were changed.

Preset data precede system and run overrides. The existing profile resolver
records user profile, explicit angle or reference fallback provenance. Apollo
accepts only its interpolated trim angle: an incompatible user AoA is rejected
instead of assigning trim coefficients to an unsupported attitude. Mach outside
[0.40,27.72] also raises an error; no extrapolation or endpoint holding occurs.
A custom off-trim vehicle needs its own aerodynamic model. Density/coefficient
scales remain explicit user assumptions. Inherited Saito mass, target, inertia
and uncertainty claims are cleared for Apollo.

Translation remains ECI in metres and seconds. Drag uses air-relative velocity
in the co-rotating atmosphere. Epoch is relative time plus an explicit Earth
angle; no historical UTC reconstruction is implied. Table Ia's Cartesian
entries are platform coordinates and are not imported as ECI. Its inertial
speed/FPA are not passed to the standalone air-relative state constructor.

The integrated interface is spherical altitude 121,920 m (400,000 ft), with
requested inertial FPA −2.055°. This project targeting contract is distinct
from the BET altitude. Actual propagated position, velocity, mass and epoch
are accepted unchanged; requested and achieved FPA remain separate. The
existing 2,000 kg carrier plus reference capsule is a project architecture,
not Apollo's service module. At configured capsule separation the carrier is
removed from the stack budget. Already separated states use the existing
`separation_mode='ATTACHED'` option to retain supplied mass (no new separation).

Default mission endpoint: 7,620 m (25,000 ft), a **project pre-parachute study
boundary**, not parachute deployment or landing. Standalone settings still
control their own endpoint (existing default 20 km). Path limits are unbounded
until requirements are supplied; numerical completion does not establish safety.

The compatibility contract contains preset, revision, full entry configuration,
vehicle properties, mass/propulsion and the new table's SHA-256. An ARD archive
cannot be silently reused as Apollo. Explicit legacy replay remains available.

## Verification and reproduction

`validation/Validate_Apollo7.m` checks all source knots, rounded published L/D,
an independent transonic anchor, interpolation, angle conversion, domain and
off-trim rejection, default/mode selection and stale-archive rejection. It checks
a nonreference mass and identical initial ECI state through both propagators,
separation accounting, and preservation of a deorbit interface state whose
achieved FPA differs from the request.

The bounded open-loop case uses ISA76, latitude/longitude 25°/40°, 121,920 m,
7,400 m/s air-relative, FPA −2°, heading 70°, constant 30° bank, reference mass,
and endpoint 7,620 m. Maximum duration 1,800 s; ode45 relative tolerance 1e-9,
maximum step 2 s. These are project conditions, not an Apollo flight replay.
The 20 s cross-integrator case uses 65 km, 4,500 m/s, −3° FPA, zero bank,
0.25 s steps and ode45 relative tolerance 1e-10.

From the repository root:

```matlab
addpath('validation');
checks = Run_All_Validations();
nominal = Run_Nominal_Orbit(struct(),struct('verbose',false));
```

Results and logs are in `apollo7_stage_2026-09-21/`. Preserved regression cases
remain pinned to their original presets. No expensive optimization was started.

Final verification: all MATLAB validations passed, including Code Analyzer
(154 files), orbit/correction/proximity, reference profiles, deorbit, entry,
paper adapters and frozen mission regression. The suite took 79.93 s; the
default Apollo orbital sequence reached standoff in 6.18 s wall time.
Its initial stack mass is 7,608.2614 kg; Phase 1 and Phase 2 costs are
114.0728 and 7.4774 m/s. A bounded continuation of that saved state to the
entry interface cost 152.2766 m/s and retained the achieved state unchanged.
Requested inertial FPA is −2.055°; achieved is −2.048858°.

Standalone entry reached 7,620 m in 729.1147 s, traversing Mach
27.1252–0.4577, with peak dynamic pressure 9.7164 kPa and peak load 2.7491 g.
Peak surrogate heat flux was 0.42289 MW/m²; no finite path limits or thermal
compliance claim were applied. Short RK4/ode45 position disagreement was
2.63e-8 m. These are numerical baseline results, not flight validation.
See [summary.json](apollo7_stage_2026-09-21/summary.json) for exact values,
requested/achieved conditions and the high-Mach domain failure.

To regenerate saved artifacts from the root:

```matlab
t=tic; results=Run_All_Validations(); results.elapsed_s=toc(t);
save('docs/apollo7_stage_2026-09-21/validation.mat','results');
t=tic; nominal=Run_Nominal_Orbit(struct(),struct('verbose',false));
nominal_runtime_s=toc(t);
save('docs/apollo7_stage_2026-09-21/nominal.mat','nominal','nominal_runtime_s');
addpath('docs/apollo7_stage_2026-09-21');
report=Summarize_Apollo7_Baseline();
```

The summary helper repeats bounded deorbit from the saved proximity state,
records the actual event, and tests the aerodynamic domain without propagating
an unsupported trajectory. `interface.time_s` is phase-local; its achieved
state record includes the accumulated mission epoch, matching the mission runner.

## Next dependencies and acceptance criteria

- First qualify compatible high-Mach Apollo data for the observed Mach 28.7946
  integrated handoff. Acceptance: source/CG/convention traceability, checked
  interpolation at the join, and representative propagation from the saved
  actual state without clipping Mach, resetting FPA or substituting velocity.

- Add the recommended bounded Lu-style predictor-corrector to this open-loop
  core, with explicit recovery-interface target, path limits, bank magnitude,
  rate and response assumptions. Compare terminal error, constraints, control
  activity, predictor convergence and initial-state sensitivity with identical
  open-loop cases. Keep bank response separate from full attitude dynamics.
- Use actual integrated entry states; never adjust FPA to force a target.
- Extend below the aerodynamic boundary only with compatible data and a declared
  recovery/parachute model. The operational data book offers a broader database,
  but coefficient revision, CG and angular conventions need separate qualification.
- Qualify flight validation separately: consistent atmosphere, coordinates,
  epoch and bank history are needed for an Apollo 7 trajectory comparison.
  Heating and off-trim validation remain unresolved.
