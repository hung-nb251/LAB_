# Robot force calibration review — 2026-09-07

## Scope and decision

Offline inspection of the six real-run CSVs in `cocarry_logs/` dated
20260907, from 103328 through 111336. Original CSVs and runtime calibration
parameters were not changed. No robot commands were sent.

All 2,136 rows contain paired finite human/robot XYZ force values. Every robot
sample is `UNCALIBRATED:torque_nm`, frame `base_link`, with calibration false.
This confirms acquisition, not calibrated Newtons or isolated contact force.

## Evidence

Pearson correlation below compares contemporaneous logged samples, without
delay compensation. It is descriptive, not evidence of a physical force sign.

| Trial suffix | Rows | Duration s | corr X | corr Y | corr Z |
| --- | ---: | ---: | ---: | ---: | ---: |
| 103328 | 237 | 15.73 | .648 | .778 | .581 |
| 105302 | 545 | 36.27 | .495 | .639 | .546 |
| 105600 | 628 | 41.80 | .501 | .536 | .429 |
| 110033 | 146 | 9.67 | .413 | .622 | .478 |
| 110303 | 308 | 20.47 | .685 | .717 | .433 |
| 111336 | 272 | 18.07 | .626 | .577 | .241 |

Last trial robot XYZ medians: [-0.162, 0.132, 0.697] in provisional N;
human medians: [-2.260, 2.560, 1.923] N. These are moving-run medians,
NOT zero-load offsets. Actual EE spans 0.383/0.403/0.275 m in that trial.
Robot sample age p95 across trials is 84–89 ms. Human sample age p95 reaches
207 ms in 103328 and 97 ms in 111336. Filter latency is additional and is
not measured by those age columns.

Diagnostic affine fitting: ordinary least squares H = [R, 1] B, trained on
five complete trials and evaluated on the excluded sixth (no random-row
split). This is only a test of a tempting static mapping, not a calibration
model. No fitted coefficients were deployed.

| Held-out trial | RMSE X N | RMSE Y N | RMSE Z N |
| --- | ---: | ---: | ---: |
| 103328 | 5.27 | 2.58 | 2.35 |
| 105302 | 4.81 | 4.56 | 2.61 |
| 105600 | 6.45 | 5.92 | 4.66 |
| 110033 | 7.91 | 7.96 | 2.97 |
| 110303 | 5.17 | 5.98 | 4.56 |
| 111336 | 4.39 | 4.84 | 4.11 |

Held-out Z R-squared for 111336 is -0.06. Thus this simple mapping is not
supported as a reliable cross-run calibration. No claim is made that all
possible estimators would fail.

## Code interpretation

- `sensorless_force_node.py` uses the shared base_link/tool0 geometric
  Jacobian, converts efforts with scale 1 in torque_nm mode, subtracts the
  configured constant six-joint bias (currently zero), then solves
  J.T W ~= tau with damped least squares. The linear part is logged.
- This path has no pose-dependent gravity, friction or dynamic compensation.
  Zero constant bias is not proof those contributions are absent.
- Human force is separately filtered and compensated in `axia_sensor_ui.py`.
  Its operational deadband means a logged zero is not a reliable no-contact
  label. The CSV does not provide a verified six-direction calibration
  protocol or explicit no-load interval labels.
- Expressing both vectors in base_link does not require them to point the
  same way. External force on the robot and force exerted by the robot have
  different sign conventions; actuator-equivalent force need not equal either
  without a dynamics model. Do not fit against velocity direction.

## Required next acquisition, pending operator confirmation

1. Confirm unchanged tool/payload/mounting and validated Axia axis directions;
   determine whether desired output is external-on-robot or robot-on-human.
2. Verify installed effort units/scales independently; do not infer them from
   URDF torque limits or force amplitude alone.
3. With an operator-managed safe static setup, record explicitly labelled
   no-contact baselines and small known forces in +/-X, +/-Y, +/-Z. Repeat
   at multiple poses and force levels; log contact position/moment effects.
   Do not infer no-contact from deadbanded Axia zero. Do not tare under load.
4. Align source timestamps and account for filtering; estimate offsets/scale
   only on training trials and validate on separate repeated poses/trials.
5. Keep any candidate calibration offline and calibration_confirmed=false
   until magnitude, cross-axis error, sign, zero return and repeatability are
   validated. Do not change admittance or role selection during acquisition.

Current blocker: existing runs are moving co-carry trials without confirmed
reference-load/zero-load labels. Ask the operator whether such intervals exist
and confirm tool configuration before choosing or applying calibration.
