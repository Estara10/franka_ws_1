# Pretrial review

## Decision

The formal A/B comparison may proceed with the frozen configuration. Both conditions have a completed full-carry pretrial with the requested force pulse, main-segment event, reference samples and usable placement result. Pretrials are excluded from the formal sample count.

## Records

- Initial A attempt: `fullcarry_perturb_ab_20261026_pretrial_A_i1`. It reached and completed the common transport main segment, and recorded the requested pulse, but the lower plan was rejected by `E_CONTACT_ROD_SUPPORT_PENETRATION`. The cause was a condition-specific planning mismatch: the computed lowering distance targeted the flat support extent, while JTC preserved the slightly tilted transport attitude. This record is retained as a pre-fix software failure and is not a formal sample.
- A recheck after the minimal correction: `fullcarry_perturb_ab_20261026_pretrial_recheck_A_i1`. The JTC lower plan now uses the same levelled release attitude as the impedance condition. The task completed release and retreat; force and reference records are present.
- B pretrial: `fullcarry_perturb_ab_20261026_pretrial_B_i1`. The task completed release and retreat; force and reference records are present. B already used the levelled release attitude, so the A-only correction did not change B's motion.

## Transport and disturbance comparison

Both completed pretrials used the fixed `3.040 s` common main path. The actual start connection was `0.500 s` in both, recorded separately from the main segment. The nominal main-path start differed by 0.0098 mm in Z, 0.0052 mm in Y and 0.0086 deg in yaw between the two pretrials; the X start and commanded X displacement differed by 0.0024 mm. These differences are negligible for the fixed scene comparison. Measured object-to-common-reference offsets at connection start were 8.62 mm for A and 4.48 mm for B and remain a distinct transition metric.

- A: main event at `32.258 s`; pulse `33.258–33.758 s`, i.e. 1.000 s after arm and 0.500 s duration at the 2 ms physics step.
- B: main event at `108.642 s`; pulse `109.644–110.146 s`, i.e. 1.002 s after arm and 0.502 s duration after physics-step quantization.
- Both event files report a single +2 N world-X pulse on `obj_rod_01`; both contain `armed`, `pulse_start` and `pulse_end` records. The arm event is published once at transport-main start; the physical thread records the actual application times.
- B records payload feedforward enabled at `3.1392 N` per arm. A records it disabled/zero. The fixed impedance parameters and robot-body compensation are unchanged.

## Version freeze note

After the successful A/B checks, the final targeted build added only JTC reference samples for the lift and lower phases and corrected the pre-dispatch metadata field so it no longer reports an unset actual connection duration. These changes add or clarify output records; they do not change control, trajectory generation, transition, placement, protection or evaluation behavior. The earlier pretrial reference files remain valid for the main transport comparison; their actual connection is taken from the dispatch/main-event logs and the time-stamped reference CSV rows.

Final targeted build: `colcon build --packages-select dual_arm_carry_task franka_hardware` followed by `colcon build --packages-select dual_arm_carry_task`. The frozen source is local working-tree state based on Git `81b7204`; no commit was made.
