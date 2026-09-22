# Consolidated plotting entry points (2026-09-22)

- Consolidated eight plotting/analysis scripts into four entry points:
  `plot_trajectory.py`, `plot_flight_constraints.py`,
  `analyze_radar_interaction.py`, and `analyze_weight_sensitivity.py`.
  Removed the seven replaced source files, unused imports, and an unused
  spline-resampling helper. The root now contains 22 Python files instead of 26.
- Trajectory views share one 3-D hemisphere/cylinder renderer. Radar clearance
  and sampled penetration remain separate functions with shared input/environment.
  Speed and turn-radius figures share validated final samples; raw curves remain
  diagnostic proxies. The altitude profile now uses 3-D cumulative distance.
- Entry points have main guards and do not read results or draw on import.
  Trajectory/radar entries default to all views and support `--view` selection.
  Main's eight plotting calls use the new entries and explicit views, preserving
  its coarse/fine figure handling. All historical PNG names are retained; the
  two heatmap names now use the same figure and one implementation.
- Updated README, quick-start commands and the paper plan's script inventory.
  The core planners, data readers, preflight checks, tests and experiment data
  remain independent. Earlier sections below are historical revision records.
- Validation: 42 regression tests and all 11 preflight checks passed. All 22
  Python files parse; active source/usage instructions have no deleted-script
  references. The real saved balanced sweep case produced nine valid PNGs via
  the three merged trajectory/radar CLIs. Synthetic 72-row fixture rendering
  produced three weight figures (including the heatmap filename alias); those
  are test data, not new sweep results. Logs and reports are retained under
  `output_5zones/validation_merged_plots/`, `merge_regressions.log`,
  `merge_preflight.log`, and `merge_weight_fixture.log` in `output_5zones/`.
- All eight sweep-hashed source files are unchanged; the saved balanced case
  still passes signature/trajectory validation. This refactor does not require
  repeating the sweep. Main's source signature changes with its plotting calls,
  so prior main caches require regeneration. No full 72-case search or main
  fine-search run was performed for this consolidation.

# Unified analysis and trajectory plotting (2026-09-21)

- The shared workbook reader now requires 72 successful current-version cases,
  one consistent source signature and a match to the local source/baseline.
  A missing new workbook no longer silently selects an old workbook. All four
  sweep analysis scripts and the representative selector use the same checks.
- Core tradeoff plots use the shared representative selector instead of fixed
  weights. Display-only batch min-max scaling is explicitly labelled and handles
  constant columns. Exposure C is reported in seconds, including heatmaps.
- Fixed radar v2's wrong `C_risk` key (which displayed zero); it now reads
  `Crisk`. Sample-segment counts are no longer called penetration events, and
  the probability average is identified as a sampled fused max-Pd diagnostic.
- All six trajectory/radar plotting scripts use `trajectory_data_loader.py`.
  They accept an explicit `TRAJECTORY_FILE` sweep case, validate its source,
  saved metrics, spline samples and speed profile, and isolate its figures by
  case name. Default inputs remain current main.py caches. Final speed and
  curvature plots reuse saved samples, rather than resampling/replanning.
- Main caches gain a signature covering shared physics, main and fine search.
  Main invokes plotting with its own Python executable and isolates its
  coarse/fine inputs from standalone plotting environment overrides.
- The eight files hashed by the sweep are unchanged. The existing validated
  balanced cache still matches signature
  `9c24c99ea0b720f403f59d473140d4251d3fbd6e9901e80810fd3967b2d96001`.
  No sweep rerun is needed due to these postprocessing edits. Old unsigned
  main.py caches need regeneration before being used as current final results.
- Validation: 36 automated tests passed, including executing all four sweep
  analysis scripts on a synthetic complete table. All six trajectory plotting
  scripts executed successfully against the real saved balanced case and
  generated nine PNGs under `output_5zones/validation_postprocessing/`.
  Their individual logs and `smoke_report.json` are retained there. Full A*
  searches and a new main.py fine-search experiment were not repeated.

# Terminal-heading follow-up (2026-09-21)

- Smoother version: `V5.3.2-terminal-heading-repair`. The V5.3.1 evaluation
  schema and output filenames are retained; source hashes distinguish runs.
- If all original smoothing candidates fail and any violates terminal heading,
  rotate the final control-polygon edge in XY and rerun the candidate search,
  including altitude/climb repairs and every original hard constraint. Existing
  successful candidates take the original path. Limits remain 12 degrees for
  heading and 15 degrees for climb. No search physics or anchor values changed.
- Reject singular spline tangents; include vertical tangents in the climb-angle
  test instead of ignoring them. Preserve smoothing version and repair diagnostics
  through Excel export/import. Verify cached weights and final-row provenance
  before recovering a complete case.
- The previously failing pure-time anchor now passes the complete coarse
  postprocessing pipeline: heading error 0.074350 degrees, maximum climb
  14.651145 degrees, minimum radius 1084.554692 m, speeds 33-55 m/s.
  Final T=2506.515208 s, C=155.832585, F=7.563341 kg. This is a coarse-path
  verification, not a replacement for the paper's planned fine-grid comparison.
- Added regressions cover heading repair, endpoint/altitude preservation,
  singular/vertical splines, unchanged successful candidates, rejection of
  other constraint failures, cache identity, and the complete 72-case orchestration
  through Excel export/import, representative selection and resume. The 72-case
  orchestration test uses simulated search results, not 72 actual searches.
- All 28 regression tests and every original preflight check passed; all 24
  Python files passed syntax parsing. Regression log:
  `output_5zones/validation_v5_3_2/regressions.log`.
- An additional 16,385-point check of the repaired pure-time spline passed
  altitude/no-fly tests, with minimum radius 1051.438098 m and maximum climb
  14.656091 degrees. These denser extrema supplement the 4096-point routine
  validation above; the checks remain numerical sampling, not an analytic
  proof of constraints at every point on the continuous curve.
- Real-map postprocessing passed for all four saved raw paths: pure-time,
  pure-risk, pure-fuel and balanced (0.3,1,0.2). All passed the strict speed
  profile/evaluator and additional 16,385-point altitude/no-fly checks.
  Risk, fuel and balanced did not invoke the new heading repair. The balanced
  final path and T/C/F agree with the previous validated result; its complete
  case cache has been regenerated with the current source signature and can
  be recovered by the default sweep without searching or smoothing again.

| Real case | Final T (s) | Final C | Final F (kg) |
| --- | ---: | ---: | ---: |
| Time (1,0,0) | 2506.515208 | 155.832585 | 7.563341 |
| Risk (0,1,0) | 2856.137976 | 0 | 14.336817 |
| Fuel (0,0,1) | 2482.749544 | 184.564177 | 7.636832 |
| Balanced (0.3,1,0.2) | 2578.928872 | 18.922463 | 7.737316 |

These are validation cases, not the representative weights selected from a
completed new sweep. The full 72 actual searches have not been executed in
this follow-up. The three stored raw anchors still reproduce their original
L/T/C/F. Detailed local records (including source signature and environment)
are in `output_5zones/validation_v5_3_2/verification_report.json`; generated
records remain outside Git. Start the formal sweep with `python -u sweep_2d.py`.

## Previous V5.3.1 evaluation consistency and repeatable sweep

- Final speed segments now use constant tangential acceleration: squared
  speed is linear in distance. Time, exposure and fuel use the same subsegment
  time increments; constant Pd=1 gives C=T.
- The mission begins at an airborne boundary with v(0)=v_stall=33 m/s.
  No ground-roll or landing-roll model is claimed. Endpoint coordinates and
  altitude envelope are unchanged. All samples obey the stall-speed floor.
- Strict evaluation also validates externally supplied speed profiles and
  computes acceleration from the actual speeds, rather than trusting supplied
  acceleration diagnostics. NaN inputs and infeasible profiles are rejected.
- Representative cases follow paper Sec. 16: minimum C, minimum T, minimum
  Euclidean ideal-point distance after sweep min-max normalization. Fuel is
  a reference point. Exact ties use other metrics then ascending weights;
  constant metric columns contribute zero. Incomplete, failed, duplicate,
  nonfinite or old-schema formal datasets are rejected before selection.
- Sweep outputs use a new v5_3_1 suffix; old V5.3 outputs are preserved.
  Every checkpoint row is checked against schema, source/configuration hash
  and weights. Failed cases are retried and replaced instead of skipped.
- Raw A* paths are atomically saved before postprocessing. Complete case files
  additionally retain spline, validation, final path, speeds and accelerations.
  Compatible retries reuse raw paths; completed case files can recover a lost
  aggregate checkpoint update. Source/configuration changes invalidate reuse.
- Pickles and Excel use temporary writes followed by replacement. If Excel is
  locked, close it and run the sweep again: successful cases are skipped and
  the workbook is exported from saved records.
- Search-stage segment costs are unchanged, so a matching existing three-anchor
  baseline can be reused. Prefer `python -u sweep_2d.py`; `--fresh` deliberately
  recomputes anchors and all cases and is unnecessary for this version upgrade.
- Run `python -m unittest -v test_regressions` and `python preflight_check.py`
  before the formal sweep. Historical SHA256SUMS.txt describes the original
  release only; Git commits and the experiment signature identify this revision.

## Historical limitation (resolved by the terminal-heading follow-up above)

The cached pure-time anchor (1,0,0) is a valid raw search path and its raw
metrics remain unchanged. Passing it directly through the coarse smoothing
pipeline does not produce a feasible spline: the best diagnostic candidate
has about 33.32 degrees of terminal-heading error (limit: 12 degrees).
The smoothing implementation is unchanged in this revision. This case is not
one of the 72 w_risk=1 screening cases. Do not present the raw anchor as a
validated final trajectory in the Traditional comparison; that comparison
still needs its own trajectory generation/validation. No heading limit was
relaxed to make this diagnostic pass.

## Validation completed on 2026-09-21

- 21 automated regression tests passed, including the selection CLI, failed-case
  retries, recovery of a complete case, atomic checkpoint failure and Excel lock.
- All original preflight checks passed. All 24 Python files passed syntax parsing.
- Re-evaluated the three stored raw anchors: L/T/C/F agree with their stored
  metrics within numerical tolerance; the original baseline remains compatible.
- Real-map risk case (0,1,0), using its stored anchor path: final T=2856.137976 s,
  C=0, F=14.336817 kg; speed 33-55 m/s, analytical Rmin=1384.871952 m,
  analytical maximum climb=14.253859 degrees; altitude and spline checks passed.
- Real-map balanced case (0.3,1,0.2), including a new A* search: 57,385 iterations
  and 19,355 expanded nodes, matching the original run. Final T=2578.928872 s,
  C=18.922463, F=7.737316 kg; speed 33-55 m/s, analytical Rmin=1307.108896 m,
  analytical maximum climb=14.805177 degrees; altitude and spline checks passed.
  Its full case file is saved for recovery by the next default sweep run.
- Validation environment: Python 3.13.5, NumPy 2.1.3, SciPy 1.15.3,
  pandas 2.2.3, matplotlib 3.10.0, openpyxl 3.1.5.
- The full revised 72-case sweep has not been run. Formal paper metrics must
  come from the completed V5.3.1 run, not this two-case validation.

---

# V5.3 final sweep-stability revision

This revision targets the 11 failed cases observed in the previous 72-case
coarse-grid sweep. The previous workbook showed:
- 6 failures caused only by the 2000 m smoothing-locality limit;
- 4 failures caused only by cubic B-spline climb-angle overshoot above 15 deg;
- 1 failure containing both issues;
- no failures caused by the mission altitude envelope or hard no-fly collision.

## 1. Grid-scaled smoothing trust region

The coarse sweep uses a 2000 m horizontal / 500 m vertical grid. The smoothing
locality metric is a 3-D Euclidean distance, so V5.3 defines the trust radius
from one maximum 26-neighbour coarse-grid step:

    d_grid = sqrt(2*2000^2 + 500^2) = 2872.28 m
    d_trust = 1.05*d_grid = 3015.90 m

This replaces the old fixed 2000 m locality limit. It remains a trust region,
not a new planner: hard no-fly collision, altitude limits, turn radius and
climb angle are all rechecked on the final spline, and final T/C/F are
reintegrated on the final trajectory.

## 2. Adaptive climb-angle B-spline repair

The raw coarse-grid climbing edge 500/2000 corresponds to about 14.04 deg and
is feasible, but cubic interpolation can overshoot above the 15 deg aircraft
limit. V5.3 does NOT relax 15 deg. Instead, when needed it:
- keeps the candidate smooth x/y route;
- projects only z onto |dz/ds_h| <= fraction*tan(15 deg);
- preserves exact mission endpoints;
- simultaneously respects the altitude floor;
- refits the corrected dense curve with cubic splprep(s=0);
- revalidates every original hard constraint.

The slope fractions are attempted conservatively from 0.92 down to 0.76, and
the first fully feasible repair is retained.

## 3. Experiment audit / resume safety

- sweep schema version is now `v5.3-grid-scaled-trust-climb-repair`, so old
  V5.1/V5.2 interim rows cannot be silently resumed.
- Excel output records trust radius, 3-D grid diagonal, altitude-repair flag,
  climb-repair flag, climb-repair fraction and maximum final smoothing
  deviation.
- Failure rows retain best-candidate diagnostics for debugging.
- `python sweep_2d.py --fresh` is a shorthand for
  `--no-resume --force-baseline`.

## 4. Validation completed before packaging

The packaged preflight performs 11 checks, including:
- objective normalization/additive A* equivalence;
- global s-candidate evaluation;
- endpoint and terminal-heading constraints;
- altitude envelope and adaptive altitude repair;
- joint altitude + climb-angle repair;
- functional old-2000/new-3015.9-m locality test;
- exactly 72 unique weight combinations and V5.3 schema;
- full sweep post-processing smoke test:
  coarse path -> B-spline -> curvature -> speed -> unified evaluator;
- fuel-unit sanity.

---

# V5.2 adaptive altitude-envelope repair

This revision fixes the runtime case where A* is feasible but every globally
smoothed B-spline candidate is rejected only by the mission altitude envelope.

Root cause:
- the hard altitude envelope rises approximately linearly in the departure and
  terminal transition zones;
- V5.1 used a sin^2 parameter taper for the B-spline altitude-repair buffer;
- sin^2 has zero endpoint slope, so a cubic refit could still undershoot a
  linearly rising altitude floor near the endpoints or transition boundaries.

V5.2 changes:
1. The altitude-repair buffer now scales with the actual local altitude floor,
   giving a linear-compatible transition instead of a zero-slope sin^2 taper.
2. Repair projection density is increased.
3. Repair buffer is adaptive. It starts small and increases from the measured
   post-refit altitude deficit, up to the smoothing trust-region limit.
4. Every repaired candidate is still fully revalidated for:
   turn radius, climb angle, fixed endpoints, terminal heading, fine-path
   locality, altitude envelope, and hard no-fly collision.
5. The hard altitude constraint is NOT relaxed.

The final trajectory remains a cubic C2 B-spline generated through splprep().

---

# V5.1 Altitude-envelope revision

## Mission endpoints are unchanged

The paper experiment still uses:

- start = `(2000, 2000, 0)` m
- goal = `(98000, 98000, 0)` m

No endpoint coordinate or endpoint altitude was changed.

## New altitude hard constraint

A common mission altitude envelope is now used by coarse A*, fine A*, B-spline
validation, final trajectory evaluation, baseline anchors and sweep experiments:

- departure transition: endpoint altitude -> 1000 m over 5 km
- en-route segment: 1000 m <= h <= 10000 m
- terminal transition: 1000 m -> endpoint altitude over the final 5 km

The 5 km transition is longer than the approximately 3.732 km minimum required
to climb 1000 m at the configured 15 degree maximum climb angle.

`altitude_constraints.py` is the single shared implementation.

## B-spline handling

The altitude envelope remains a hard constraint.  If a globally smoothed
`splprep()` candidate sags below the lower envelope, the candidate is not
accepted as-is.  Its vertical component is projected upward to the legal
envelope and refit with a cubic `splprep()` spline, then every hard constraint
(turn radius, climb angle, endpoints, terminal heading, locality, altitude and
no-fly collision) is checked again.  The selected trajectory remains a C2 cubic
B-spline.

## Baseline / sweep cache

The baseline signature/version now includes the altitude-envelope parameters.
Old `baseline.pkl` will therefore not match.  For a clean paper rerun, delete:

- `output_5zones/baseline.pkl`
- `output_5zones/sweep2d_interim.pkl`
- old `sweep_2d_time_fuel.xlsx`

Run `python preflight_check.py`, then `python main.py`.  Only after main is
visually/numerically confirmed should the 72-case sweep be rerun.

---

# V5 revision notes

## What changed

1. **One multi-objective definition everywhere**
   - Reported objective: `J = w_T*T_n + w_R*C_n + w_F*F_n`.
   - `T_n`, `C_n`, `F_n` use the shared three-anchor payoff-table ranges.
   - A* edge increments use the same denominators but do **not** subtract the minima on every edge; those offsets are path-independent constants. This preserves A* additivity while keeping exactly the same complete-path ordering as the paper formula.
   - Raw A* statistics and final trajectory statistics both use `AStarRiskPlanner.evaluate_path()` / the same segment integrator.

2. **Shared `baseline.pkl`**
   - Added `baseline_manager.py`.
   - Anchors: time `(1,0,0)`, risk `(0,1,0)`, fuel `(0,0,1)`.
   - Ideal values are the corresponding single-objective minima.
   - The maxima are the largest cross-objective values in the three-anchor payoff table (reproducible nadir approximation).
   - Cache carries a configuration signature and is automatically regenerated if the environment/grid/dynamics/start-goal configuration changes.

3. **Fuel and risk units**
   - Risk is consistently `integral(Pd dt)` in exposure-seconds.
   - Fuel is consistently kg using power / `fuel_heat_value_j_per_kg` (43 MJ/kg default).
   - Search edges always compute fuel, even when `w_fuel=0`, so anchor cross-metrics are valid.

4. **B-spline V5**
   - `splprep()` retained.
   - All candidate `s` values are evaluated; no first-feasible `break`.
   - Feasible candidates are ranked by
     `J_s = 0.4*J_smooth + 0.3*J_curvature + 0.2*J_deviation + 0.1*J_length`.
   - `J_curvature` is based on the requested curvature-continuity integral
     `J_kappa_prime = integral((dkappa/ds)^2 ds)`.
   - Added strong endpoint retention, terminal-heading constraint, analytic curvature, collision/locality/turn-radius/climb checks.

5. **Final evaluation order**
   - `A* -> B-spline -> analytic curvature -> adaptive speed -> time/risk/fuel/objective`.
   - `main.py` no longer contains its own duplicate fuel/risk/time formulas.

6. **Sweep output expanded**
   - Adds min turn radius, max curvature, max climb angle, smoothing score, curvature variation integral, terminal heading error, max `|dv/dt|`, normalized fuel, and final `J`.
   - Keeps raw A* metrics as audit columns.
   - Excel now includes a normalization-baseline sheet and extra pivot sheets.

7. **Other fixes found during audit**
   - A* map/altitude bounds now come from the environment; climb angle comes from the dynamics model instead of duplicated hard-coded values.
   - Risk-only anchors use a lexicographic geometric tie-break to avoid arbitrary long zero-risk paths without changing the primary objective.
   - `analyze_radar_interaction_v2.py` now uses 3-D hemisphere membership and `calculate_risk()` instead of the nonexistent `risk` key at `z=0`.
   - Analysis scripts no longer hard-code a Windows `D:` path or assume exactly 18 Excel columns; `analysis_data_loader.py` supports the new schema and old 18-column workbooks.
   - Removed the old incorrect interpretation that `(w_time,w_fuel)=(0,0)` is a “zero-weight tie”; `w_risk=1` is still active.
   - Radar-radius documentation aligned with the actual 15 km configuration.

## Recommended run order

```bash
python preflight_check.py
python main.py
python sweep_2d.py --no-resume
```

After the sweep completes, run the analysis scripts as needed. They default to `output_5zones/sweep_2d_time_fuel.xlsx`; `SWEEP_FILE` and `ANALYSIS_OUTDIR` can override those locations.

## Important experiment note

`sweep_2d.py` intentionally keeps the original coarse 2000 m / 500 m A* grid so the 72-case scan remains computationally practical. It now performs B-spline, curvature, speed and final physical evaluation for every successful coarse-grid solution, but it does not run the expensive fine-corridor A* stage 72 times.
