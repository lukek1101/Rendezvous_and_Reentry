# Conditional evaluation of the integrated simulator

Evaluation date: 2026-09-22. Repository base: `08e0e07cd0b6df4a4a1254350c4bd2f37ba973e9`
plus the existing uncommitted migration work. Production physics, planners and
controllers were not modified. Source hashes, scripts, configurations, histories,
CSV/JSON results and exportable PNG/PDF figures are in
[evaluation_stage_2026-09-22](evaluation_stage_2026-09-22/).

## Findings and scope

Eight redesigned nominal missions reached proximity standoff and a propagated
deorbit entry interface. **None establishes end-to-end mission success:** all
eight handoffs lie outside the active aerodynamic domain. This is an
implementation/model-coverage limit, not demonstrated physical infeasibility.

For a frozen Apollo nominal Phase 1 design, online correction accepted nine of
sixteen selected disturbed realizations; disabled correction accepted none.
Both zero-error controls passed. Seven corrected cases were denied midcourse
commands above the 2 m/s capability and retained a terminal position miss.
These are selected-case counts, not reliability estimates or confidence bounds.

Six separate fixed-reference final-approach trials completed with no sampled
constraint violation or saturation. Eleven standalone fixed-bank entry cases
reached 7.62 km, with substantial endpoint sensitivity. Their states are not
substituted for the unsupported integrated handoffs.

## Experimental separation and assumptions

| Experiment | What is fixed | What changes / scope |
|---|---|---|
| Nominal design sweep, 8 cases | Existing 300→500 km polar architecture, J2, drag off, Isp 200 s; existing planners and ideal actuators | Apollo initial phase 60/90/120° at −R; Apollo 90° at +R/+V/−V; ARD and HORUS 90° at −R. Nominal transfer and proximity closing are redesigned per case. Propagate actual upstream state into deorbit. |
| Orbital robustness, 36 runs | **One** saved Apollo 90° nominal plan: wait, transfer time and nominal commands. Same correction opportunities, budgets and tolerances in enabled/disabled pairs | 3×3 initial/execution error levels × seeds 7/42 × correction off/on. Only the existing midcourse corrections, arrival retargeting and terminal cleanup may adapt. Stop at the Phase 1 handoff; no downstream closing redesign is called. |
| Allocation controls, 2 runs | Same frozen design, zero error | Compare inherited 400 kg Phase 1 propellant allowance with 634.0218 kg, obtained by preserving the prior 400/4800 stack-mass fraction. An exploratory allocation, not Apollo tank capacity. |
| Final-approach robustness, 6 runs | Saved nominal −R acquisition state, reference, timing and feedback controller | Add +H displacement 0/1/5 m and +H velocity 0/0.02 m/s to truth. Perfect state knowledge. No new closing plan. These are component cases, not continuations of the disturbed Phase 1 runs. |
| Standalone entry sensitivity, 11 runs | Apollo trim model, mass, 121.92 km altitude, 7400 m/s air-relative speed, latitude/longitude 25°/40°, heading 70°, initial/commanded bank 30°, endpoint 7.62 km | FPA −2.5/−2/−1.5° × density scale 0.9/1/1.1; two additional L/D scales 0.9/1.1 at −2° and unit density. No redesign, target search or online entry guidance. |

Navigation error is **zero only**: perfect instantaneous state knowledge is
the implemented capability. Nonzero measurement errors, estimator covariance,
observation latency and sensor outages are unimplemented, so no navigation
robustness region is asserted. Perturbing the true initial state is not a
navigation-error experiment. Orbital drag, winds and atmospheric-model mismatch
in an entry predictor are also outside this evaluation: the nominal planner
rejects drag and there is no closed-loop entry predictor.

The orbital base errors reuse the previously verified exploratory levels:
independent zero-mean Gaussian ECI components with σr=20 m, σv=0.02 m/s,
fractional burn gain σ=0.002 and small-angle pointing-vector σ=0.0002 rad.
Initial and execution scales independently take 0/1/3. Threefold levels test
the existing correction limit rather than assert spacecraft performance.
Private MT19937 streams use fixed slots for scheduled burns; enabled/disabled
pairs have identical initial errors and actuator draws. Future actuator errors
are unavailable to the controller. The two deliberately selected seeds do not
support confidence intervals; Gaussian tails do not define bounded envelopes.

The entry ±0.5° FPA bracket and ±10% density/L/D factors are local sensitivity
choices around the checked baseline, not measured Apollo dispersions. L/D
scaling uses the existing uncertainty hook and is explicitly a perturbation of
the reference model, not another published coefficient table. The proximity
offsets probe small fractions of the 500 m acquisition geometry and 0.17 m/s
speed ceiling. Results apply only to these sampled directions and levels.

## Resource requirements and conditional success regions

### Redesigned integrated cases

| Preset / initial phase / approach | Phase 1 ΔV (m/s) | Phase 2 ΔV (m/s) | Deorbit ΔV (m/s) | Propellant to entry (kg) | Peak controlled force (N) | Entry Mach |
|---|---:|---:|---:|---:|---:|---:|
| Apollo / 60° / −R | 113.257 | 7.473 | 152.906 | 990.751 | 13.984 | 28.793 |
| Apollo / 90° / −R | 114.073 | 7.477 | 152.277 | 991.395 | 13.938 | 28.795 |
| Apollo / 120° / −R | 115.806 | 7.485 | 152.675 | 998.609 | 13.891 | 28.828 |
| Apollo / 90° / +R | 114.073 | 8.417 | 152.386 | 994.933 | 13.929 | 28.795 |
| Apollo / 90° / +V | 114.073 | 1.949 | 151.959 | 971.640 | 2.341 | 28.838 |
| Apollo / 90° / −V | 114.073 | 1.829 | 151.959 | 971.234 | 2.341 | 28.838 |
| ARD / 90° / −R | 114.073 | 7.477 | 178.925 | 681.800 | 8.793 | 28.724 |
| HORUS / 90° / −R | 114.073 | 7.477 | 129.365 | 3125.721 | 47.683 | 28.869 |

The four Apollo 90° approaches use matched acquisition/hold/standoff ranges,
corridors, control limits and the same closing candidate durations; the planner
may select different durations. ±V uses about 909 s more total time to entry
than −R and less delta-V in these cases. This is not an optimality claim.
Other vehicle rows have different mass, entry altitude and requested FPA;
they are resource scenarios, not an equal-mission vehicle ranking.

Controlled force is for finite final approach only; instantaneous impulses
have no modeled peak force. All eight nominal final approaches have zero
saturation; sampled maximum tracking errors are 0.00666–0.01148 m. Apollo
time to entry spans 34,745–55,662 s across selected phases. The 120° design
is not certified under the correction executor's separate 40,000 s Phase 1
time bound. Vehicle dry mass/tank capacity is not established; positive
remaining mass is not proof that these propellant requirements are available.

![Design resources](evaluation_stage_2026-09-22/design_resources.png)

### Frozen-design correction

Accepted enabled cases out of the two selected seeds at each cell:

| Initial-error scale \ execution-error scale | 0 | 1 | 3 |
|---|---:|---:|---:|
| 0 | 2/2 controls | 2/2 | 2/2 |
| 1 | 2/2 | 2/2 | 1/2 |
| 3 | 0/2 | 0/2 | 0/2 |

Disabled correction passes only the zero-error cell. Acceptance means position
error ≤0.5 m and rotating-frame velocity error ≤0.001 m/s at the original
handoff epoch, with executor limits satisfied. No continuous region between
sampled cells is inferred. The seven rejected enabled cases retain predicted
capability-limit reasons and observed terminal misses of 5.52–10.12 km.
They do not prove that another correction schedule or extra orbit is necessary.

The inherited 400 kg allocation rejects even the zero-error arrival burn:
nominal complete Phase 1 requires 429.881 kg. It stops with 222.293 kg already
spent and does not reset velocity to complete the maneuver. Under the declared
634.022 kg allocation, enabled cases expend 418.881–436.826 kg, including
failed cases. Lower expenditure in a failed case is not a performance gain.
All delivered corrections and changes to the existing arrival command are
included in total maneuver cost and the mass ledger.

![Fixed-design orbital trials](evaluation_stage_2026-09-22/orbital_robustness.png)

### Fixed-reference final approach and standalone entry

All six acquisition-error cases complete the same 6120 s controlled sequence.
Final approach costs 6.13945–6.16215 m/s and 22.4197–22.5024 kg. Peak force
rises from 13.94 to 23.98 N; saturation remains zero, sampled corridor/side/
keep-out/speed/mass/gate flags remain false. Maximum tracking error includes
the initial perturbation and reaches 5.077 m. This does not test navigation
uncertainty, actuator dynamics, docking or disturbed upstream closing.

All eleven entry cases reach the altitude endpoint in 660.1–825.0 s. Across
them, peaks span 8.936–10.361 kPa dynamic pressure, 2.559–2.899 g aerodynamic
load and 0.3600–0.4813 MW/m² surrogate heat flux. There are **no finite path
limits or accepted landing target** in these configurations. Completion is
an endpoint event, not successful recovery or thermal compliance.

Relative to the −2° baseline, FPA −2.5°/−1.5° changes the endpoint by
462.8/669.4 km at unit density. Density ×0.9/1.1 changes it by 23.57/21.25 km;
L/D ×0.9/1.1 changes it by 157.54/166.91 km. These are ECEF chord distances
between endpoints at their respective event times, not flight-target errors
or downrange-only distances. Bank is constant from initialization, so command
activity and response transients are zero. The retained 60° magnitude, 10°/s
rate and 2 s response settings are unexercised limits, not attitude validation.

![Standalone sensitivity](evaluation_stage_2026-09-22/entry_sensitivity.png)

## Public benchmarks and validity

| Reference | Supported comparison | What this evaluation does not reproduce |
|---|---|---|
| [NASA Apollo 7 postflight analysis, MSC 69-FM-89](https://www.nasa.gov/wp-content/uploads/static/history/afj/ap07fj/pdf/a07-entry-postflight-analysis-19740072689.pdf), Table IIb | Source-knot, interpolation, trim-angle convention and mass/geometry checks; fresh Apollo checks passed | Historical atmosphere, bank history, onboard navigation, trajectory or splashdown. Table IIa postflight coefficients are not substituted into the preflight model. |
| [JAXA HTV2 press kit](https://iss.jaxa.jp/en/htv/mission/htv-2/library/presskit/htv2_presskit_en.pdf), §2.5.2, pp2-21–22 | −R geometry matches 500/250/30 m stages; simulated controlled peak speed 8.840 m/min is within the documented 1–10 m/min approach range. Stops/ramp speeds are not claimed to obey its lower bound | The subsequent 10 m capture, yaw maneuver, RGPS/RVS, ISS orbital plane and flight telemetry. Other signed axes are hypothetical alternatives, not HTV flight modes. |
| [Mooij, HORUS-2B M-692](https://repository.tudelft.nl/record/uuid:514cefb5-8768-40ba-aabb-a18a1f10f339) and prior migration source ledger | Preserved clean coefficient table and explicit missing-cell/domain rejection; fresh reference-profile check passed | Trimmed/full-flight trajectory, published guidance or an atmospheric descent from this integrated handoff. The user-supplied speed/AoA policy remains an assumption. |
| ARD supplied EN-AVT-130 chapter and [ESA Bulletin 109](https://www.esa.int/esapub/bulletin/bullet109/chapter6_bul109.pdf) | Preserve the previously documented hypersonic surrogate; identify incompatibility at the actual handoff | Flight aerodynamics across the full descent, reconstructed thermal histories or recovery accuracy. |
| [NASA CR-185676](https://ntrs.nasa.gov/citations/19930020463), relative-motion equations | Existing frame/dynamics conventions provide a reference; fresh potential-gradient and rotated-frame checks passed | An independent public flight-trajectory benchmark or a new quantitative reproduction of the report. |

The JAXA online PDF fetch timed out this stage; the previously obtained full
86-page local copy was available and pp71–72 were reread. No missing article
was silently replaced by a summary. Primary-source locations/hashes from the
research and Apollo stages remain in their source ledgers.

Apollo's active table ends at Mach 27.72, ARD at 26, HORUS at 20. The current
atmosphere helper uses a density table but constant temperature **186.946 K
above 84.852 km** and a calorically perfect speed-of-sound calculation. Thus
the high-altitude Mach/domain mismatch requires atmosphere and Mach-convention
qualification as well as aerodynamic coverage; it is not necessarily evidence
that the physical vehicle lacks high-speed capability. No temperature, Mach,
AoA, velocity or FPA was clipped or reset to bypass the validity check.

## Failure semantics, precision and verification

`MODEL_DOMAIN_UNSUPPORTED` means required physics is outside the declared
model. `RESOURCE_OR_POLICY_LIMIT` includes a denied propellant command.
`CORRECTION_CAPABILITY_REJECTED_WITH_OBSERVED_MISS` retains both predicted
capability rejection and actual propagated terminal error. A plain
`HANDOFF_TOLERANCE_VIOLATION` is a completed candidate miss.
`NUMERICAL_NONCOMPLETION` and `NUMERICAL_NONCONVERGENCE_WITH_OBSERVED_MISS`
are reserved for bounded work/solver failures; none occurred in the completed
matrix. A rejected search is never labeled physical infeasibility. Unexpected
programming exceptions are rethrown, not counted as scientific failures.

Orbital integration uses RelTol 1e-11, absolute position/velocity tolerances
1e-4 m / 1e-7 m/s and maximum step 60 s. Targeting thresholds are separately
0.5 m / 0.001 m/s. Nominal correction limits are 8 iterations / 40 evaluations;
online correction limits are 6 / 30 per opportunity, with separate 60 s runtime
guards. These are root-solving/work limits, not optimization convergence or
optimality criteria. Entry uses ode45 RelTol 1e-9, AbsTol 1e-7 and max step 2 s.

Entry refinement to RelTol 1e-10 and max step 1 s changes the nominal ECEF
endpoint by 0.01739 m and event time by 6.35e-7 s. This is numerical consistency
under identical physics; it does not validate atmosphere, heating or flight
accuracy. Earlier orbital refinement evidence remains separately documented.

The evaluation took 137.9 s on this host, including selected fresh validations;
runtimes include startup/JIT effects and are not statistically characterized.
Verification covers frozen-plan identity, paired initial/burn draws, caller RNG
isolation, burn-position continuity, rocket-equation and aggregate fuel/delta-V
closure, unchanged deorbit handoff and saved-result consistency. Existing orbit,
reference-profile and Apollo checks passed. No expensive optimization was run.

## Reproduction and next dependency

From the repository root in MATLAB:

```matlab
addpath('docs/evaluation_stage_2026-09-22');
results=evaluate_simulator();
results=finalize_evaluation(); % saved-data checks and readable summary
```

For figures, use Python 3.12 with
`pip install -r docs/evaluation_stage_2026-09-22/plot_requirements.txt`, then
`python docs/evaluation_stage_2026-09-22/plot_results.py`. This run installed
plot dependencies only under `tmp/evaluation_plot_deps`; no system Python or
simulator environment was changed. The source manifest records the dirty
working-tree source identity, since the base Git commit alone is insufficient.
MAT files retain actual configurations, plan, realizations and trajectories;
JSON nulls indicate unavailable quantities, never zero cost or successful runs.

Next, qualify high-altitude atmosphere/Mach conventions and compatible entry
aerodynamics, then repeat actual-state entry propagation. Acceptance requires
source-qualified domains and a continuous, unmodified upstream handoff.
After that, implement the separately recommended entry guidance with declared
recovery target and path/control limits. Before reliability claims, define
vehicle-specific resource allocations and a navigation/actuator/environment
error model with justified distributions and an adequate sampling plan. A full
fixed-design integrated robustness evaluator also needs explicit frozen closing
and deorbit design interfaces; this stage does not silently replan those phases.
