# P0 — Kết quả calibration F_robot hợp nhất ngày 19/09/2026

P0 đã hoàn thành ở mức kiểm kê, hợp nhất, fit và kiểm chứng offline. **Chưa
hoàn tất calibration đủ tin cậy để dùng cho điều khiển.** Candidate mới không
vượt model đang chạy trên cả T2-r4 và lượt runtime dài. Giữ model 18/09 ở
shadow; không thay launch/config, không bật `calibration_confirmed` hoặc
`role_valid`, không gửi lệnh robot.

## Kết quả chính

| Đối chứng cùng mẫu/reference | Model runtime 18/09 | P0 frozen, refit train+r3 |
|---|---:|---:|
| T2-r4: RMSE vector lực | 2,306 N | 3,134 N |
| Runtime 17:22: RMSE vector lực | 7,144 N | 8,771 N |

Reference ở 17:22 được suy ngược radial deadband 4 N từ CSV, chỉ dùng các
mẫu mà hai đầu nội suy đều khác 0; đây không phải raw Axia được ghi trực tiếp.
17:42 còn thiếu raw M310/baseline/q0 nên không thể replay candidate mới;
model đang chạy vẫn có RMSE khoảng 6,817 N trên reference có điều kiện này.

Con số T2-r4 2,306 N khác 2,164 N của audit cũ vì P0 ghép theo timestamp
trung bình của sáu M310–M315, thay vì timestamp M321, và dùng quality mask
thống nhất. Hai model trong bảng được so trên cùng 21 mẫu hợp lệ. Không dùng
sự khác nhau giữa hai protocol để kết luận model cũ thay đổi.

Candidate P0 được chọn là torque+pose, gain đầy đủ, ridge 0,1. Model trước
refit đạt RMSE trung bình theo trial 2,474 N trên hai lượt r3. Sau lựa chọn,
refit train+r3 và giữ r4 ngoài fit. Đây vẫn là kiểm chứng lịch sử: r3/r4 và
các runtime log đã được xem trong những audit trước, không phải test mới.

## Dữ liệu đã hợp nhất

- Kiểm kê và hash **77 `events.jsonl`**: 61 log trong kho calibration và 16
  log lịch sử ngoài kho này. Có cả metadata, register counts, marker,
  fault, thời gian scan, nhãn mode/tuyến, lý do dùng hoặc loại.
- Giữ **12 pose tĩnh không tiếp xúc** riêng biệt. Các lần ghi lặp cùng pose
  vẫn có trong manifest; baseline chính lấy lần hợp lệ mới nhất.
- Tập hiện tại có **54 đoạn baseline**, **25 trial lực**, **3.993 mẫu lực**
  sau ghép và kiểm tra chất lượng, từ ngày 15–18/09.
- Các log 12/09 được kiểm kê nhưng không trộn vào fit hiện tại vì cấu hình
  cũ chưa xác nhận tương thích. Các log thiếu raw M310 không được suy ngược
  từ force đã tính để tạo torque giả.
- Các lượt GRU/GRU+MJM ngày 17/09 thiếu M310–M315. Chúng không cung cấp dữ
  liệu torque cho calibration, dù có M320 và Axia.
- Chưa có sidecar `*.calibration.jsonl` trong kho dữ liệu tại thời điểm audit.
- Chưa tìm thấy đoạn baseline **chuyển động không tiếp xúc có marker**.
  Không suy nhãn này từ force sau deadband bằng 0, trạng thái Stop hay tên MJM.

Baseline GT đầu trial có marker `baseline`, khớp đứng yên và Axia healthy
được nhận dù controller đang RUNNING, phù hợp protocol GT cũ. Baseline cuối
phải không RUNNING, đứng yên và residual Axia dưới 2 N. Các bài lực tại pose
cố định có mode STATIC được đánh giá dù không chạy Admittance.

## Observability và độ ổn định

| Tập baseline | Rank số học | Rank với ngưỡng tương đối 1% | Condition đã chuẩn hóa |
|---|---:|---:|---:|
| 12 pose tĩnh | 6 | 5 | 101,12 |
| 54 baseline, trừ trung bình theo session | 6 | 5 | 196,80 |

Kết luận rank=2 trong audit 18/09 chỉ thuộc hai baseline cuối được chọn lúc
đó. Bộ hợp nhất đủ rank số học, nhưng một hướng vẫn bị kích thích yếu.
Ngưỡng 1% là thước đo chẩn đoán được công khai, không phải tiêu chuẩn chứng
nhận calibration. Condition phụ thuộc chuẩn hóa và không so trực tiếp với
condition của Jacobian robot.

Hướng yếu của ma trận `dq/scale` có vector khoảng
`[0.053, -0.519, 0.787, -0.091, 0.309, 0.069]`, nổi bật ở tổ hợp J2/J3/J5.
Thay đổi joint/XYZ lớn trong các tuyến tương quan với nhau chưa chắc tăng
được thông tin theo hướng này. Orientation tool giữa 12 pose có độ lệch
trung vị 4,32°, lớn nhất 16,36° theo FK.

Có năm baseline có độ lệch chuẩn ít nhất một kênh torque trên 0,5 Nm, kể cả
một số pose tĩnh. Đặc biệt baseline T1 lúc 12:47 ngày 17/09 lệch torque tới
khoảng 5,43 Nm so với baseline gần cùng joint pose trong session. Đây là
bằng chứng bất ổn cần kiểm tra, chưa xác định nguyên nhân là nhiễu, drift,
preload hay trạng thái estimator phía controller.

Các cặp gần cùng pose, sai khác mỗi joint dưới 0,01 rad, có chênh lệch
vector baseline torque, đo bằng **norm chênh lệch giữa hai median**:

- Cùng session: median 0,450 Nm, P95 1,020 Nm.
- Khác session: median 0,660 Nm, P95 1,223 Nm.

Các cặp này có tương quan; không coi số cặp là số thí nghiệm độc lập. Mô
hình baseline tuyến tính theo pose cũng chưa thắng baseline hằng trên
leave-one-pose-out: RMSE vector torque lần lượt 4,298 và 4,116 Nm.

Kiểm tra thăm dò khi bỏ các baseline có scatter >0,5 Nm không đảo ngược
kết luận: baseline-pose + gain đầy đủ đạt 3,853 N ở r4, vẫn chưa vượt model
đang chạy. Ngưỡng này chỉ dùng cho sensitivity, không âm thầm loại mẫu
khỏi phân tích chính hoặc dùng để chọn candidate.

## Thiết kế kiểm chứng

Đã so năm họ model, mỗi họ ở ridge 0,01 / 0,1 / 1:

1. Baseline hằng + gain đường chéo.
2. Baseline hằng + gain đầy đủ 6×6.
3. Baseline theo pose fit riêng + gain đường chéo.
4. Baseline theo pose fit riêng + gain đầy đủ 6×6.
5. Fit chung torque+pose để hiệu chỉnh torque, rồi khôi phục wrench qua Jacobian.

Baseline được trừ trung bình theo session khi fit shape để tránh ép offset
khác session thành phụ thuộc pose. Mỗi trial có trọng số tổng bằng nhau khi
fit gain; báo cả RMSE gộp và trung bình RMSE theo trial.

Ngoài split thời gian train/r3/r4, có kiểm chứng loại toàn session, toàn ngày
và toàn tuyến. Baseline thuộc tập bị giữ lại không vào fit; chỉ baseline đầu
trial test được phép làm tare đầu vào. Có assertion kiểm tra giao tập rỗng.
Các fold ridge 0,01 là cấu hình định trước; fold dùng ridge được chọn từ r3
là thăm dò, không gọi là nested CV.

Khả năng chuyển giao vẫn yếu: với torque+pose ridge cố định 0,01, giữ ngoài
toàn session GRU 18/09 cho RMSE trung bình trial khoảng 6,418 N; giữ ngoài
toàn tuyến T2 cho khoảng 8,032 N. Không thể dùng kết quả tốt ở trial ngắn
để khẳng định calibration áp dụng tốt cho session hoặc tuyến khác.

Reference tái dựng dùng Axia raw, baseline đầu trial, payload phía Axia
1,126 kg, TF được ghi và góc gá -90° như pipeline hiện tại; moment arm
`[0, 0.120, 0.0354] m` theo audit trước. Không thay frame/gá/payload.
Moment gravity/CoG, lực quán tính và UI settings thực tế chưa được ghi đầy
đủ vẫn là giới hạn. Reference chính qua trung bình đối xứng 0,10 s để so
sánh offline; có metric đối chứng reference chưa làm trơn. Không tối ưu lag.

## Phần dữ liệu cần bổ sung có mục tiêu

Trong bộ M310 hiện có, đếm mẫu Axia ≥4 N và nằm trong cone 30° quanh trục:

| Tuyến/pose khai báo | X+ | X− | Y+ | Y− | Z+ | Z− |
|---|---:|---:|---:|---:|---:|---:|
| Home | 17 | 31 | 36 | 42 | 50 | 47 |
| T1 | 58 | 73 | 93 | 80 | 78 | 58 |
| T2 | 0 | 0 | 19 | 0 | 5 | 21 |
| New pose | 31 | 52 | 68 | 68 | 49 | 67 |

Đây là độ phủ trên cả tuyến, không phải tất cả lực đã được đo ở đúng pose
đích. Phần thiếu rõ nhất là **X± và Y− trong vùng T2** có M310 đồng thời;
các bài lực T2 dùng M320 cũ không bù được phần này. Chỉ thu hướng đó ở pose
đã được phép vận hành, với raw M310/Axia/q và sidecar logger.

Nếu cần baseline động, bổ sung đoạn không tiếp xúc có marker trên tuyến
đã có quy trình cho phép. Nếu cần nhận dạng baseline sáu chiều ngoài vùng
hiện tại, chọn thêm pose kích thích hướng yếu J2/J3/J5 và kiểm tra singular
values sau từng pose; không lặp lại 12 pose cũ hoặc đặt số pose cố định.
Tọa độ mới phải được kiểm tra reachability/clearance trước khi người vận
hành thực hiện. P0 này không phát sinh lệnh chuyển động.

Sau khi chọn và đóng băng model mới, cần một test session mới có raw đầy
đủ; không refit/chọn ngưỡng bằng test đó. Hiện chưa có cơ sở để thay model
runtime hoặc dùng `F_robot` chọn vai trò.

## Artifacts và tái lập

- Script: [calibrate_hc_p0_unified.py](../scripts/calibrate_hc_p0_unified.py).
- Input chính: `cocarry_logs/hc_force_calibration/20260919_p0_unified_inputs_v2/`.
  Có manifest JSON, baseline medians, metadata và các mẫu ghép trong NPZ.
- Kết quả chốt: [20260919_p0_unified_final_v1](../cocarry_logs/hc_force_calibration/20260919_p0_unified_final_v1/).
  Có manifest CSV, toàn bộ metric, model ở từng giai đoạn, quyết định,
  kiểm tra runtime, source snapshot, đồ thị lực và độ phủ pose.
- Các thư mục `inputs_v1`, `evaluation_v1/v2` là kết quả trung gian được
  giữ nguyên, không phải bộ kết quả chốt.

```bash
cd /home/hungnb/cocarry_ws
OPENBLAS_NUM_THREADS=1 python3 scripts/calibrate_hc_p0_unified.py \
  --prepare-only --output <thu_muc_input_moi>
OPENBLAS_NUM_THREADS=1 python3 scripts/calibrate_hc_p0_unified.py \
  --prepared <thu_muc_input_moi> --output <thu_muc_ket_qua_moi>
```

Script từ chối thư mục output đã tồn tại và kiểm tra hash của cả 77 log
cùng metadata trước khi đánh giá cache. `candidate_frozen.json` là model
train+r3; `candidate_selection_stage.json` là model dùng để chọn trên r3;
`candidate_refit_all_exploratory.json` có cả r4 nên không nhận metric test
của model frozen.

Kiểm chứng phần mềm: `py_compile` và 15 numerical/data-isolation tests đạt.
Chạy pytest với `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` vì plugin ROS
`launch_testing` cài sẵn không tương thích pytest 9; không đổi môi trường.
SciPy hệ thống cảnh báo NumPy 1,26,4 nằm ngoài dải hỗ trợ khai báo; các
phép kiểm tra số đã đạt, cảnh báo được giữ nguyên. Không sửa package ROS
nên không phát sinh build/deploy hay thử robot trong P0.
