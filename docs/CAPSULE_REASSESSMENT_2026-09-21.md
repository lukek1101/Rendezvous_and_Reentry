# Capsule reference reassessment

Subsequent implementation: [Apollo 7 migration](APOLLO7_MIGRATION_2026-09-21.md)
records the now-active default capsule, source transcription, passing checks
and the remaining high-Mach integrated-entry boundary. The review below is
the preserved selection-stage record.

Recommendation: replace ARD as the **first implementation target** with an
Apollo 7 / CSM-101 trim-entry reference. Keep ARD selectable for later
comparison. This is a source review and recommendation only; no preset,
aerodynamic implementation, guidance controller or mission was changed.

The earlier ARD priority did not justify treating it as automatically the
best-supported capsule. The newly inspected Apollo 7 primary report is a
material improvement over the earlier AS-202-only comparison.

## Comparison

P = published; D = derivable; M = not resolved in this quick review.

| Candidate | Available force/attitude evidence | Mission and validation evidence | Remaining burden / decision |
|---|---|---|---|
| **Apollo 7 / CSM-101 trim reference** | **P:** preflight CL, CD and trim AoA versus Mach 0.40–27.72; separate postflight table 0.688–27.720. **D:** L/D cross-check, SI conversion. | **P:** entry-state alternatives, post-separation weight/CG/inertia, atmosphere choice and reconstructed guidance/trajectory plots in the same report. | **Preferred.** Restricted trim model suits bank-only guidance. Resolve coefficient reference area, heating geometry, frames, physical limits and recovery endpoint before activation. No arbitrary-AoA capability or touchdown model follows from these tables. |
| **Apollo generic database in supplied Mooij book; AS-202 as separate comparison** | **P:** book Tables B.1–B.3 give total drag/lift/moment on Mach 0.9–10 and total AoA 0–40 deg. AS-202 report provides reconstructed flight aerodynamics. | Supplied book has body/force conventions; NASA AS-202 report has flight comparisons and uncertainty discussion. | Useful for a later off-trim model, but book alone leaves high/low-Mach gaps. Generic book geometry/CG and AS-202 must not be spliced into Apollo 7 without reconciliation. |
| **ARD** | **P/G:** reference mass/geometry, AoA history and CA/CN figures approximately Mach 4–26. Active model's Mach-10 floor is a software restriction, not the end of source evidence. | Supplied RTO chapter and other papers provide flight/CFD comparisons. | Good retained reference; more digitization and lower-speed completion than Apollo 7. Complete 2-D aero grid not established. |
| **Orion 2011 development configuration** | **P:** public database-development papers cover hypersonic and supersonic/subsonic regimes, coefficient/uncertainty formulation and plots. Full PDFs obtained. | Strong model-development evidence; Lu's earlier CEV example is a different configuration. | Promising second choice, but a coherent executable coefficient export and matching mission/mass/CG set were not established in this quick review. More assembly and variant reconciliation than Apollo 7; do not identify 2011 data with flown Artemis I automatically. |
| **Saito HSRC-like research capsule** | Supplied paper describes guidance but does not provide its complete underlying coefficient database. | Simulation cases, not ARD flight data. | Does not remove the present data dependency. |

The supplied Huygens material is not a direct substitute for Earth-entry
guidance; using it would require a different planetary/environment reference
and expand the requested mission scope.

## Strongest new primary source

Frank G. Skerbetz, *Apollo 7 Entry Postflight Analysis*, MSC Internal Note
69-FM-89, 26 May 1969. [Full NASA report, archival mirror](https://apollojournals.org/afj/ap07fj/pdf/a07-entry-postflight-analysis-19740072689.pdf).
Read Sections 4.1–4.4 and inspected Table II visually (printed p.21, PDF28).
Preflight Table IIb is the recommended predictive baseline. Postflight IIa
combines inferred L/D with simulation-derived CD and trim angle; it is not
an independent measured force database. Its reconstruction atmosphere was
selected for agreement, so comparisons must disclose shared assumptions.
The report includes state alternatives, mass properties and histories for
load, roll command, errors and altitude. A modern Lu controller would be a
new study on this vehicle, not replication of flown Apollo guidance.

## Other reviewed sources and access

- Supplied Mooij *Re-entry Systems*, Appendix B.1, printed pp.1483–1489:
  revisited definitions and visually checked B.1/B.2/B.3. Numerical tables
  are more useful than a constant L/D assumption but do not alone cover a
  full Earth entry. They are a published secondary compilation of older
  wind-tunnel data, not an Apollo 7 mission-specific prescription.
- Hillje, NASA TN D-4185, *Entry Flight Aerodynamics from Apollo Mission
  AS-202*: retained earlier source assessment; [full primary report](https://ntrs.nasa.gov/api/citations/19670027745/downloads/19670027745.pdf).
- Bibb et al., Orion static database [Part I](https://ntrs.nasa.gov/api/citations/20110013644/downloads/20110013644.pdf?attachment=true)
  and [Part II](https://ntrs.nasa.gov/api/citations/20110013645/downloads/20110013645.pdf?attachment=true):
  full PDFs downloaded after initial access failures. Scoped review of
  coverage, configuration and database construction; no complete table
  transcription or flight-case qualification performed.
- [Apollo 7 postflight trajectory](https://apollojournals.org/afj/ap07fj/pdf/as205-postflight-traj-rep-19920075302.pdf):
  accessible companion identified; detailed reconstruction not reviewed here.
- Apollo 10 operational planning and AS-202 MIT GSOP documents were also
  located. They concern different cases and were not used to fill Apollo 7
  inputs. No inaccessible indispensable article remains from this review.

## Next implementation dependencies and acceptance

1. Freeze an explicitly named `APOLLO7_PREFLIGHT_TRIM` dataset from IIb;
   retain IIa separately. Verify every row and CL/CD ratio against the scan.
   Translate the 150–170-degree Apollo AoA convention explicitly; never
   feed it directly into the existing positive-small-angle ARD convention.
2. Confirm CSM-101 coefficient area and appropriate heating geometry from
   the matching data book; reconcile post-separation mass with propagated
   remaining mass. Record source revision and units. No silent borrowing of
   AS-202/Orion mass, area, CG or nose radius.
3. Define pre-parachute terminal conditions, bank response and finite path
   limits. Check the actual propagated EI state and every predictor stage
   against Mach coverage, including the upper-atmosphere model assumptions.
   A range-spanning table is not proof that every candidate trajectory is
   covered. Recovery below the table or under parachutes needs a separate
   explicit model; do not clamp coefficients to claim splashdown capability.
4. Implement the previously recommended bounded Lu adaptation with the shared
   dynamics and actual upstream state, keeping open loop. Compare identical
   initial-condition sweeps with frozen vehicle data. Assess physics first
   using prescribed bank history, then assess guidance separately; differences
   from flown Apollo control must not be labeled aerodynamic-model errors.

Verification in this review: source text/scan consistency, table domains,
preflight/postflight separation and existing-file review. No simulation,
optimization or runtime benchmark was necessary or performed. The repository
implementation and unrelated changes were preserved. Recommendation is to
switch the implementation priority, not to claim a verified replacement yet.
