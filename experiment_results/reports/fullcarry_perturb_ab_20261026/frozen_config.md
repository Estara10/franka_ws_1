# Full-carry disturbance A/B batch: frozen configuration

## Scope

- Batch: `fullcarry_perturb_ab_20261026`
- Code base: Git `HEAD 81b7204` plus the current local working-tree revisions present at batch freeze. The task package and `franka_hardware` were built from that working tree; no commit was made.
- A: internal ABCD group A, JTC position control, basic shared object path, rod load feedforward disabled.
- B: internal ABCD group C, independent Cartesian impedance, same basic shared object path, rod load feedforward enabled.
- Trajectory optimization is disabled. No `CONTACT_TRANSPORT_PAIRING` setting is used; common transport uses the direct fixed-time object path.

## Fixed scene and control configuration

- Start object pose: `(0.500, 0.000, +10.0 deg)`; target: `(0.400, 0.000, +20.0 deg)`.
- Lift height: `0.400 m`; contact hold: `2.0 s`; grasp force request: `10 N per finger`.
- Rod model, mass, friction, grip settings, collision checks, task criteria, controller transitions and placement/release/retreat logic are held at the currently successful full-carry configuration.
- The same common nominal object path and quintic smootherstep progress are used for both groups. The main transport is `3.040 s`; any measured-start connection is separately logged and excluded from the main-segment disturbance timing.
- No 8 s suspended wait is used. No control, stiffness, grip, friction, protection or acceptance parameter is changed for this batch.
- B rod load feedforward: `3.1392 N` per arm along world `+Z`, ramped over `0.5 s` at MMC activation after its first accepted reference. A rod-load feedforward is zero. Robot-body compensation is unchanged.
- Disturbance in both groups: one-shot `+2 N` world `X` force on `obj_rod_01` at its MuJoCo body origin/COM, armed at the logged transport-main start, applied after `1.000 s` simulation time, then cleared after `0.500 s`. The hardware physics-thread interface records `armed`, `pulse_start`, and `pulse_end` in `rod_force_events.csv`.
- All runs are headless and sequential, one MuJoCo instance at a time.

## Run order

Pretrials: A, then B; each is separate from formal statistics.

Formal schedule is frozen in `schedule.csv`. It contains ten A and ten B attempts, with the balanced order repeated as `A1 → B1 → B2 → A2`, then `A3 → B3 → B4 → A4`, continuing through `A9 → B9 → B10 → A10`. Each A/B index shares the same nominal scene input; all runs start from independent simulation initialization. No failed formal attempt is replaced.

## Commands

```bash
python3 experiment_results/reports/fullcarry_perturb_ab_20261026/run_batch.py pretrial
# Review both pretrial task logs, reference CSVs and force-event CSVs; record the decision in pretrial_review.md.
python3 experiment_results/reports/fullcarry_perturb_ab_20261026/run_batch.py formal
```

The runner stores each run in a unique `experiment_results/a1_runs/` directory, writes progress to `attempts.csv`, refuses to reuse an existing run directory, and does not repeat any action that has already started. A pre-action launch exception is separately recorded and may be retried up to three times; an action-started failure remains that attempt.
