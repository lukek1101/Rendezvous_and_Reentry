# Entry guidance method review

Update: [Apollo 7 migration](APOLLO7_MIGRATION_2026-09-21.md) implements and
checks the replacement capsule trim model. ARD remains available. The active
Apollo table covers Mach 0.40–27.72; a bounded open-loop descent reaches 7.62 km.
This resolves the initial capsule-model dependency for a pre-parachute guidance
study. The actual integrated handoff is Mach 28.7946, so compatible high-Mach
coverage is still required first; the operational data book is already obtained.
Closed-loop guidance and parachute/touchdown dynamics remain unimplemented.

Status: recommendation and dependency review; no controller implemented. The
user requested a method comparison before implementation if the separate
research had not selected one. The relevant research discussion and supplied
strategy review do not establish a selected closed-loop algorithm.

## Sources and interpretation

Saito et al., *Guidance strategies for controlled Earth reentry of small
spacecraft in low Earth orbit*, Acta Astronautica 229 (2025), 684–697,
[DOI](https://doi.org/10.1016/j.actaastro.2024.12.054), supplied full text.
Sections 3.2–3.3 and 4.1, PDF pages 8–10 / printed pages 691–693, were read
and visually checked, including equations 13–26, Table 5 and Figures 10–11.
This is a small HSRC-like capsule study, not ARD or HORUS flight validation.

The supplied *LEO Reentry Trajectory Strategies — Direct Deorbit vs.
Intermediate Orbits* was inspected as a background synthesis, including its
nine tables and source qualifications. Its cited flight figures were not
independently revalidated in this stage and were not imported into the models.
Its statements about orbital positioning, entry range and skip trajectories
do not authorize new mission architectures or select a guidance algorithm.
Source paths and SHA-256 identifiers are in
`entry_guidance_stage_2026-09-21/sources.json`.

Lu, *Entry Guidance: A Unified Method* (2014), DOI 10.2514/1.62605,
Saito reference 34, was subsequently supplied by the user. The relevant
formulation and constraints were reviewed as recorded in the follow-up below;
the former access gap is resolved. NASA
records 20210025125 and 19850008593 also returned HTTP 403 during this review;
their full text was not used for the comparison below.

## Comparison and recommendation

| Published approach | Required information / burden here | Assessment |
|---|---|---|
| Reference-trajectory feedback, Saito §3.2, equations 13–22 | Vehicle-specific reference range, drag, altitude rate and gains versus speed; Table 5 is for the paper's capsule. Separate regeneration and tuning are required for ARD and HORUS. | Low online cost, but copying gains would mix incompatible vehicles. Not the first choice for a configurable exploratory simulator. |
| Real-time numerical predictor-corrector, Saito §3.3, equation 23 / Figure 10 | Predict from the current state at two bank commands; use predicted range sensitivity to update command. Requires numerical safeguards, a defined endpoint, lateral policy and bounded prediction cost. | Recommended starting family: reuses the current force model and propagator, without an imported reference-gain database. |
| Predictor-corrector with force estimation and terminal bank guidance, equations 24–25 / Figure 11 | Adds measured force scale estimates, filter tuning, scheduled crossrange reversals and a terminal switch. Paper does not supply every tuning value needed for exact replication. | Defer the estimator and extra switching modes. Their reported performance cannot be attributed to the simpler first mode. |

Recommendation, refined after reading Lu: one explicitly labeled, bounded
adaptation of Lu's baseline numerical predictor-corrector with perfect state
knowledge and the existing open-loop baseline. This remains a recommendation,
not an implemented or verified FNPEG controller. A future
implementation must define a signed downrange residual separately from
crossrange: an unsigned ground miss distance is not automatically a
well-conditioned scalar root for a secant correction. Bank magnitude alone
does not independently control both coordinates; a stated lateral/sign policy
and explicit residual reporting are necessary.

## Material endpoint dependency

The user clarified that a partial atmospheric handoff is insufficient: the
intended scope is complete entry toward the recovery region. The proposed
intermediate-endpoint-only implementation is therefore not the accepted
deliverable. Resolve the force-model coverage for that descent first; do not
silently substitute a high-altitude endpoint for the recovery requirement.
The user has supplied Lu's paper for the guidance assessment. Complete entry
guidance still does not imply parachute, runway flare or touchdown/contact
dynamics, which the current simulator does not implement.

ARD currently supports Mach 10–26 and alpha 15–25 degrees using the labeled
fixed-body-coefficient surrogate. HORUS supports clean, untrimmed Mach 1.2–20
and alpha 0–45 degrees, with missing interpolation cells still rejected.
AoA profile endpoint holding does not extend either force model's validity.
The paper's Mach-3 guidance cutoff and 240 m/s propagation endpoint are
outside ARD's supported domain. Its HSRC coefficient treatment does not
supply missing ARD or HORUS data. HORUS's orbital-speed entry interface can
also lie above its Mach ceiling; an integrated run must retain and check that
actual state, not jump to a lower starting altitude.

## Proposed implementation and acceptance checklist

1. Resolve endpoint semantics and source/model envelope first. Store endpoint
   altitude or speed, Earth-fixed target location, epoch/rotation convention,
   terminal position tolerance, and any terminal speed/FPA requirements
   separately. Choose an interior validity margin for numerical stages.
2. Reuse `reentry_core` and the established phase interface. Preserve actual
   integrated position, velocity, epoch and remaining mass. Standalone
   constructors may prescribe air-relative FPA; sweeps change only the named
   initial-condition variables. No mass restoration or trajectory state reset.
3. Keep `OPEN_LOOP` selectable. Define a sampled guidance update with explicit
   prediction count/iteration/time bounds, sensitivity safeguards and failure
   behavior. Proposed initial update period is 1 s, motivated by Saito's RPC
   study, not a verified onboard timing requirement. Do not claim global
   optimality or feasibility from a failed numerical search.
4. Keep commanded bank separate from actual bank. Reuse the existing
   first-order, rate-limited response concept in both prediction and execution;
   preserve the historical baseline configuration for comparison. Current
   standalone defaults are 60 deg magnitude, 10 deg/s rate and 2 s response.
   These are project assumptions, not certified ARD/HORUS actuator limits.
   No moments, RCS torque allocation or attitude-control dynamics are implied.
5. Predict only within supported aerodynamics. Distinguish domain exit,
   exhausted search, failed integration, unreachable tested target, timeout
   and measured path-constraint violations. Record attempted commands,
   accepted command, residuals and rejected candidate reasons. An exhausted
   search does not prove physical unreachability.
6. Verify bank sign/lift direction, frames and epoch, event refinement,
   predictor/executor agreement and exact phase-state continuity. Verify
   magnitude/rate/response limits, small-sensitivity handling and finite
   candidate budgets. Tighten integration tolerances separately from guidance
   targeting tolerance and correction stopping criteria.
7. Compare identical nominal and initial-condition-perturbed cases with
   guidance off/on. Freeze vehicle, AoA fallback and target across each set.
   Include FPA, speed and lateral-state changes plus a combined case. Report
   downrange/crossrange/ground miss, terminal altitude/speed/FPA, peak dynamic
   pressure/load/surrogate heating, accumulated heat, bank variation,
   reversals, saturation duration, rate and response error, runtime and
   prediction count. State finite path limits explicitly; current infinite
   defaults cannot establish vehicle safety. Heating remains a surrogate.

## Verification performed

Re-ran `Validate_Reference_Profiles` and `Validate_Reference_Migration` only.
Both passed. They cover existing data/precedence/domain/frame/compatibility
checks and bounded reference propagation, not new guidance performance.
Profile RK4/ODE45 position disagreement was 5.01e-8 m; the HORUS speed-profile
case was 1.12e-7 m. Saved result:
`entry_guidance_stage_2026-09-21/reference_checks.mat`.

Reproduce from the repository root:

```matlab
addpath('validation');
results.profiles=Validate_Reference_Profiles();
results.migration=Validate_Reference_Migration();
```

No optimization, guidance performance sweep or full reference descent was run.
Only this review, source manifest, verification result and migration summary
were added in this stage. Existing implementation and unrelated changes were
preserved. Next dependency: establish bounded,
source-labeled aerodynamic extensions for complete entry before implementing
and verifying the single guidance mode. Needed force information is ARD below
Mach 10 and HORUS above Mach 20 plus any missing cells encountered on descent:
CD/CL or CA/CN with their angle/sign/reference-area definitions, or a justified
approximation with explicit limits and uncertainty. Published AoA histories
alone do not close this dependency. The final guidance endpoint and downstream
recovery interface must be explicitly defined; no touchdown capability is
claimed by the existing altitude-endpoint propagator.

## Lu full-text follow-up

**Coverage clarification:** Mach 10 is the active ARD surrogate's lower
implementation limit, not the lower limit of all supplied evidence. The
earlier source review records RTO chapter Figures 29–30 extending approximately
to Mach 4. These should be assessed/digitized before requesting duplicate
Mach 4–10 data. The main remaining low-speed evidence gap is below that
coverage, particularly transonic/subsonic flight toward recovery. Likewise,
the supplied 2024 HORUS book already fills the older M-692 missing cells;
the recorded later-source completion has not been activated. Those cells
are an integration/provenance task, not an outstanding request for new data.
See `REFERENCE_PROFILES_2026-09-21.md` and its table-difference record.

The supplied 16-page paper is now accessible. Its formulation in Sections
II–IV and the predictive constraint treatment in VI.B–C were inspected;
PDF pages 3–6 and 11–12 (printed 715–718 and 723–724) were rendered and
visually checked. This is a targeted implementation review, not replication
of every numerical example. The source hash is in the source manifest.

Lu is a stronger common starting point than transferring the Saito capsule
controller: its examples cover CEV, X-33 and CAV-H, spanning low to high lift.
Those examples support the method choice, not ARD/HORUS model validation.

| Item | Published formulation | Consequence for this simulator |
|---|---|---|
| Corrector | Linear bank-magnitude profile versus energy (19), signed range residual (20), squared-residual objective (21), finite-difference Newton step with halving line search (22–25). | Bound iterations, predictions, line-search steps and bank magnitude. A stationary residual is not necessarily a target hit; separately test physical terminal tolerances. No global-optimality claim. |
| Range sign | Propagated range state (17–18) permits negative remaining range; unsigned great-circle distance cannot distinguish overshoot. The range-rate approximation neglects heading offset. | Reuse actual 3-D translation, explicitly document any signed-range adaptation, and check actual Earth-fixed terminal miss independently, especially for lateral dispersions. |
| Terminal event | Final energy combines requested radius and speed (10–13); the paper allows final altitude error. | Energy convergence alone cannot pass separately specified altitude/speed/FPA requirements. Record all achieved quantities and residuals. Do not reset them. |
| Lateral guidance | Bank sign changes when heading error crosses a velocity-dependent deadband (26). Deadband depends on vehicle lift capability and terminal speed. | One common algorithm may use separate documented ARD/HORUS deadbands. Exact deadband curves are not supplied in this paper; tuning must be labeled and frozen before disturbance comparisons. |
| Activation | 1 Hz guidance when total aerodynamic acceleration is at least 1.52 m/s²; otherwise hold current bank (p.717). | This is different from Saito's 0.20-g **drag** threshold and the existing diagnostic. Do not mix them silently. Threshold and update period remain explicit settings. |
| Bank response | Predictions in baseline Section III assume instantaneous reversals; executed 3DOF examples limit rate and acceleration. Capsule: 20 deg/s, 10 deg/s²; X-33/CAV-H: 10 deg/s, 5 deg/s² (p.717). | Reuse our finite-response model in both prediction and execution, documenting that adaptation. These example limits are not ARD/HORUS hardware values. Our 60-deg default magnitude cannot inherit performance from plots reaching much larger magnitudes. |
| Path limits | Baseline Section III does not enforce them. VI.B–C adds predictive altitude-rate feedback (35–45), evaluated throughout every candidate integration, then at the current state. | Postprocessing limit flags or clipping the final bank alone are not Lu's constraint treatment. Finite-response feasibility and actual violations still need evaluation. |
| Constraint approximations | Locally constant CL/CD and approximately dV/dt = −D are used in (36); finite feedback tracking may violate limits (p.724). | Respect coefficient changes and actuator limits. Report measured violations even when the corrector converges. Do not describe this as guaranteed safety. |
| Heating | Paper uses a V^3.15 heating correlation for a 0.3048-m curvature radius (14). | Current core uses Sutton–Graves V³ and each vehicle's nose radius. Re-derive any heating-rate feedback for the actual core; do not copy the 3.15 coefficient in (43) or change vehicle radii. |

Table 1 illustrates the endpoint distinction: CEV requests 7.6 km /150 m/s
at zero remaining range; X-33 requests 30.4 km /908 m/s at 30 nautical miles
remaining. Neither is a touchdown simulation. These are literature examples,
not new defaults for ARD or HORUS. The user's complete-entry objective is
retained; no intermediate-only substitute has been accepted.

Coordinate verification: `reentry_core.lift_direction` was exercised for
equatorial northward flight. Positive 90-degree bank produces east/right
lift, negative bank west/left, matching Lu's positive-right convention (p.715).
Result saved in `entry_guidance_stage_2026-09-21/lu_bank_convention.mat`.
The paper uses Earth-relative velocity and clockwise-from-north heading,
consistent with the standalone constructor; its nondimensional equations
must not be copied into the SI core without conversion.

This follow-up adds source review and the bounded bank-sign check only.
No implementation or aerodynamic domain was changed, and no optimization or
new propagation campaign was run. The prior reference checks remain the
baseline evidence. The guidance-source dependency is resolved; the next
dependency is complete-descent aerodynamic coverage and explicit recovery
interface/finite path limits. Simplified extensions are permitted by the
user, but still require a stated physical basis, bounded domain and uncertainty;
the CEV/X-33 data in this paper do not supply ARD/HORUS coefficients.
