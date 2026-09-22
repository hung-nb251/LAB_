# Calibration T1 — kết quả lượt 18:00 và lượt Z 18:31 ngày 19/09/2026

> **Cập nhật 18:55.** Mục 3b bổ sung kết quả sau khi có lượt Z± 18:31. Kết
> luận ở mục 3 (chỉ dữ liệu 18:00) đã bị thay bởi mục 3b. Phần phân tích từ
> 18:53 do Claude thực hiện theo yêu cầu người dùng, tiếp quản nhánh Codex
> đang dừng dở; vì vậy **không còn lớp review độc lập** giữa hai nhánh.

## Kết luận và việc thu tiếp

**Candidate hợp nhất tốt hơn model runtime trên mọi phép đối chứng hiện có,
gồm cả kiểm tra chuyển giao sang 10 trial T1 lịch sử mà lần trước bị kém đi.
Vẫn chưa được phép thay runtime, vì không còn tập dữ liệu nào chưa bị xem.**

Lượt Z± 18:31 đã lấp đúng phần thiếu mà bản 18:24 chỉ ra: từ 13 mẫu Z+ và
0 mẫu Z− lên **54 mẫu Z+ và 37 mẫu Z−**. Sau khi đưa lượt này vào, candidate
hợp nhất hạ RMSE vector trên cả hai holdout nội bộ và hạ macro RMSE lịch sử
từ 2,792 xuống 2,015 N.

Điều kiện còn thiếu duy nhất là **một session mới với model đã đóng băng**.
Mọi split lịch sử và cả hai khối nội bộ trong ngày đều đã được xem trong quá
trình phát triển, nên không con số nào dưới đây là test độc lập.

F_robot tiếp tục shadow: `calibration_confirmed=false`, `role_valid=false`,
không thay launch/config/model đang chạy. Không tiếp tục điều tra nguyên nhân
mất ACK theo yêu cầu người dùng; mốc dừng chỉ dùng để cắt dữ liệu không hợp lệ.

## 1. Dữ liệu và chất lượng

Nguồn: `cocarry_logs/hc_force_calibration/20260919_t1_local_dev_v1/`.

- Lượt chính: `cocarry_admittance_3d_20260919_180049.csv` và sidecar.
- Baseline cuối: `cocarry_admittance_3d_20260919_180506.csv.calibration.jsonl`.
- CSV chính có 1.878 dòng. Sidecar có 1.147 scan M310, trong đó 615 scan có
  lực runtime. Dòng CSV lặp timestamp lực không được tính thành mẫu độc lập.
- RUNNING khoảng 125,20 s. Giữ 612 scan sau khi ghép reference/joint và loại
  ba scan không nằm hoàn toàn trong cửa sổ đã cắt: sau Start và trước fault
  tối thiểu 1 s. Loại phần quanh/sau fault khỏi fit và đánh giá calibration.
- M310 khoảng 5,10 vector/s trên toàn log; trong RUNNING khoảng 4,9 Hz.
  Median span sáu thanh ghi khoảng 145 ms toàn log, 150 ms khi RUNNING.
  Đây là đọc tuần tự, chưa phải vector đo đồng thời.
- Axia ghi được raw 6D và cấu hình thực: filter `median3_adaptive`, payload
  1,126 kg, radial deadband 4 N, bias không đổi; các bản tin processed đều
  có `calibrated=true`, `tf_valid=true`.
- Không dùng phép suy ngược deadband. Reference chính tái dựng từ raw force,
  bias, mass và effective rotation được ghi, không thêm yaw lần thứ hai.
  Raw được ghép nội suy với giới hạn gap 50 ms; có metric đối chứng reference
  processed trước deadband. Không tối ưu lag hoặc làm trơn đối xứng để fit.
- Replay force runtime bằng q được ghi và model snapshot: sai khác lớn nhất
  khoảng `2,84e-14 N`. Replay lực processed Axia: `1,07e-14 N`. Điều này kiểm
  chứng công thức tái lập, không phải chứng nhận độ chính xác vật lý của lực.

Baseline đầu có marker rõ ràng, khoảng 29,08 s sau bỏ biên, 157 scan M310;
joint đứng yên. Axia raw đã bù có norm median 0,035 N, P95 0,057 N. Baseline
M310 chốt tại Start khác median đoạn marker 0,1 Nm ở J1/J4, các kênh còn lại
không khác. Độ lệch chuẩn từng kênh trong đoạn marker <=0,165 Nm.

Baseline cuối có khoảng 15,87 s được đánh dấu sau bỏ biên, joint đứng yên,
Axia residual median 0,407 N, P95 0,430 N. **Không có bất kỳ mẫu M310 nào trong
file baseline cuối**, nên không đánh giá được torque drift đầu–cuối. Baseline
cuối không dùng để tare/refit các mẫu trước đó.

## 2. Độ phủ và phạm vi áp dụng

Đếm mẫu có Axia norm >=4 N và hướng trong cone 30° quanh trục:

| Tập | X+ | X− | Y+ | Y− | Z+ | Z− |
|---|---:|---:|---:|---:|---:|---:|
| Toàn lượt hợp lệ | 44 | 59 | 19 | 83 | 13 | 0 |
| Train | 21 | 28 | 10 | 42 | 13 | 0 |
| Validation | 11 | 17 | 0 | 25 | 0 | 0 |
| Đoạn cuối giữ ngoài fit | 12 | 14 | 4 | 13 | 0 | 0 |

Đây là mẫu liên tiếp, không phải số thí nghiệm độc lập. Không có mẫu đủ tiêu
chí Z− không có nghĩa bạn chưa hề tác động lực xuống; nghĩa là dữ liệu chưa
có lực xuống đủ rõ theo điều kiện phân tích đã công bố. Z+ chỉ xuất hiện rõ
trong phần train nên chưa kiểm chứng được khả năng chuyển giao hướng này.

EE được quan sát trong hộp bao X=[−0,228; 0,407], Y=[0,707; 1,110],
Z=[0,432; 0,717] m. Hộp bao chỉ mô tả dữ liệu, không phải workspace được phép
vận hành hay bảo đảm mọi điểm trong hộp đều đã được kiểm chứng.

Joint pose có rank số học 6, rank ở ngưỡng tương đối 1% là 3, condition chuẩn
hóa khoảng 2.321. Đây là **độ phủ pose động của riêng lượt T1**, không phải
rank của toàn bộ baseline P0. Không dùng một đoạn baseline đầu để fit mới
hàm baseline sáu chiều theo pose. Condition Jacobian median khoảng 9,03,
max 17,59; không đồng nhất với condition của ma trận dữ liệu.

## 3. Model đã thử và kết quả

Giữ phép tính runtime làm prior: baseline/q0 từ đầu lượt, ma trận torque+pose
đang chạy và nghịch đảo regularized Jacobian. Fit correction gain trên torque
đã hiệu chỉnh, dạng đường chéo 6 hệ số hoặc đầy đủ 6×6, ridge
0,01 / 0,1 / 1 / 10. Loss chỉ dùng force XYZ; không tuyên bố đã calibrate
moment hoặc lực chủ động của robot.

Lần chọn đầu dùng 60% thời gian đầu để train, 20% tiếp validation, 20% cuối
giữ ngoài fit; có guard 1 s mỗi phía ranh giới. Tương ứng 362/112/118 mẫu,
20 mẫu guard chỉ báo mô tả. Chọn bằng validation rồi refit train+validation,
không đưa đoạn cuối vào refit. Đây là kiểm chứng nội bộ cùng session, không
phải test độc lập. Các thử nghiệm tiếp theo đã xem kết quả này nên được ghi
rõ là thăm dò hồi cứu.

| Đối chứng | Runtime 18/09 | Candidate đầy đủ được chọn |
|---|---:|---:|
| Validation, model chỉ fit train | 2,532 N | 1,987 N |
| Đoạn cuối 118 mẫu, sau refit train+validation | 2,922 N | 2,041 N |
| Góc median trên 46 mẫu chung của đoạn cuối | 13,28° | 9,66° |
| Góc P95 trên 46 mẫu chung | 35,04° | 22,68° |
| RMSE trung bình theo trial của 10 trial T1 lịch sử | 2,792 N | 3,278 N |

Metric góc chỉ dùng mẫu norm cả reference và hai estimate >=4 N; báo cùng
tập 46 mẫu để so sánh trực tiếp. Trên toàn lượt, runtime RMSE 2,673 N;
candidate 1,893 N là con số có chứa dữ liệu fit, không gọi là test accuracy.

Ở 13 mẫu Z+ của phần phát triển, RMSE vector tăng từ **4,881 lên 5,455 N**,
dù X/Y cải thiện. Candidate full có singular value gain lớn nhất khoảng
22,04 do các hướng torque yếu được chuẩn hóa riêng; hệ số đó không có đủ
bằng chứng vật lý để diễn giải thành calibration torque hoàn chỉnh.

Đã thử lại prior đồng đều theo hệ số để không nới penalty ở hướng yếu, và
đối chiếu cả gain đường chéo. Kết quả:

| Candidate | RMSE đoạn cuối cùng lượt | Macro RMSE 10 trial T1 cũ |
|---|---:|---:|
| Runtime | 2,922 N | 2,792 N |
| Full, prior chuẩn hóa riêng | 2,041 N | 3,278 N |
| Diagonal, prior chuẩn hóa riêng | 2,519 N | 3,209 N |
| Full, prior đồng đều | 2,065 N | 3,386 N |
| Diagonal, prior đồng đều | 2,566 N | 3,057 N |

Đối chứng lịch sử dùng cùng mask/reference P0 cho từng cặp model; kiểm tra
hash raw và metadata của cả 10 trial đã dùng. P0 reference có smoothing
offline 0,10 s; có metric raw đối chứng. Không so RMSE tuyệt đối giữa protocol
cũ/mới để kết luận cải thiện. Runtime prior đã học một phần dữ liệu lịch sử;
các trial đó là phép kiểm tra suy giảm, không phải holdout độc lập cho runtime.

Đã thử fit chung dữ liệu T1 ngày 17/09 và phần train lượt mới, validation trên
T1 ngày 18/09 cùng phần validation lượt mới, mỗi trial có trọng số bằng nhau.
Dùng raw reference để giảm khác biệt preprocessing giữa nguồn. Full ridge
0,01 cải thiện nhóm cũ nhưng làm validation mới tăng từ 2,532 lên 2,675 N.
Diagonal ridge 10 chỉ giảm score trung bình hai nhóm từ 3,114 xuống 3,107 N
(khoảng 0,2%); chưa cho thấy lợi ích đủ rõ để thay runtime. Không chọn/refit
thêm bằng đoạn cuối rồi gọi đó là test mới.

**Quyết định:** giữ runtime shadow. File `candidate_frozen.json` trong analysis
v1 chỉ lưu nghiệm fit để review/tái lập, chưa được chấp thuận deploy. Kết luận
chốt ở `20260919_t1_local_review_v2/decision.json`.

## 3b. Sau khi có lượt Z± 18:31 — kết quả hợp nhất

Lượt `t1_dev_z_02` gồm file chính `183108` (169 s, 853 scan M310) và file
baseline cuối `183502`. Ghép và lọc chất lượng cho **430 scan dùng được**.

Độ phủ hướng lực của riêng lượt này, cone 30° và norm >=4 N:

| X+ | X− | Y+ | Y− | Z+ | Z− |
|---:|---:|---:|---:|---:|---:|
| 19 | 30 | 0 | 0 | **54** | **37** |

Y± bằng 0 trong lượt này; thông tin Y vẫn chỉ đến từ lượt 18:00 và dữ liệu
lịch sử. Không được coi vùng Y đã được lượt mới củng cố.

Candidate được chọn là `full`, ridge 0,01, giữ nguyên prior là phép tính
runtime (baseline/q0 đầu lượt, ma trận torque+pose đang chạy, nghịch đảo
regularized Jacobian). Loss chỉ dùng force XYZ.

| Đối chứng | Runtime 18/09 | Candidate hợp nhất |
|---|---:|---:|
| Holdout nội bộ lượt 18:00, n=118 | 2,922 N | **2,536 N** |
| Holdout nội bộ lượt Z 18:31, n=80 | 2,661 N | **2,245 N** |
| Macro RMSE 10 trial T1 lịch sử | 2,792 N | **2,015 N** |
| Góc P95, holdout 18:00 | 35,0° | 27,6° |
| Góc P95, holdout Z 18:31 | 41,4° | 25,3° |

Đây là thay đổi so với bản 18:24, khi candidate chỉ fit từ dữ liệu XY làm
macro RMSE lịch sử **xấu đi** 2,792 → 3,278 N. Candidate XY-only đó cũng
không đạt khi kiểm tra tiến cử trên lượt Z mới. Việc bổ sung Z± đã đảo
ngược kết luận.

RMSE từng trial lịch sử của candidate hợp nhất, 10 trial:

| Trial | Mode | RMSE |
|---|---|---:|
| `20260917_101035_..._target1_x` | GROUND_TRUTH | 1,435 N |
| `20260917_102540_..._target1_y` | GROUND_TRUTH | 1,869 N |
| `20260917_103040_..._target1_z` | GROUND_TRUTH | 2,319 N |
| `20260917_104200_..._target1_z` | GROUND_TRUTH | 0,829 N |
| `20260917_125927_..._target1_static_force` | STATIC | 1,930 N |
| `20260918_155647_..._gru_home_to_target1` | GRU | 2,612 N |
| `20260918_155939_..._gru_home_to_target1` | GRU | 3,491 N |
| `20260918_161818_..._gru_home_to_target1` | GRU | 1,768 N |
| `20260918_162314_..._gru_home_to_target1` | GRU | 2,178 N |
| `20260918_163243_..._gru_home_to_target1` | GRU | 1,723 N |

Singular values của correction gain: `[1,445; 1,192; 1,015; 0,998; 0,970;
0,468]`. Bốn hướng gần như giữ nguyên, một hướng bị co còn 0,468. Đây là
hiệu chỉnh vừa phải quanh model hiện tại, khác hẳn gain lớn 22,04 của thử
nghiệm prior riêng lẻ trước đó. Vẫn chưa đủ cơ sở diễn giải nó thành
calibration torque vật lý hoàn chỉnh.

Macro validation dùng để chọn model gần như hòa: 3,378 N của candidate so
với 3,450 N của runtime. Phần cải thiện thật nằm ở các holdout và ở dữ liệu
lịch sử, không ở bước chọn.

### Vì sao vẫn chưa được deploy

`decision.json` giữ nguyên `KEEP_CURRENT_RUNTIME_SHADOW;
SAVE_COMBINED_CANDIDATE_OFFLINE; REQUIRE_NEW_SESSION_TEST`. Lý do:

- Mọi split lịch sử và cả hai khối nội bộ trong ngày **đều đã được xem**
  trong quá trình phát triển. Không còn tập nào đóng vai test độc lập.
- Lượt Z mới ban đầu là phép kiểm tra tiến cử cho candidate XY; sau khi dùng
  nó để fit, nó trở thành dữ liệu phát triển và mất vai trò test.
- Y± không có trong lượt mới.
- Phạm vi kết luận giới hạn ở vùng T1 đã đo, đúng tool/tải/gá của session này.

### Giới hạn của chính dữ liệu này

- File `183502` được ghi liên tục hơn 15 phút thay vì 15 giây và **chỉ có một
  marker** `post_baseline_for:t1_dev_z_02`. Thiếu `no_contact_begin/end`, nên
  không có nhãn không tiếp xúc theo protocol. Logger được đóng lúc 18:52 theo
  yêu cầu người dùng. Đoạn baseline cuối vì vậy chỉ dùng chẩn đoán, đúng như
  `protocol.json` quy định, không làm tare hay đầu vào fit.
- File `183108` thiếu `no_contact_end:...:start`, `interaction_pending` và
  không có `run_status=False` trước `logger_stop`.
- Bản phân tích `t1_z_analysis_v4` của Codex đã đọc `183502` khi file còn
  đang được ghi, nên sha256 trong manifest của nó không còn khớp. Bản dùng
  cho kết quả trên là `t1_z_analysis_v5`, chạy sau khi file đã đóng.
- Sáu kênh M310–M315 lệch nhau trung vị 144,8 ms; khi robot chuyển động điều
  này gây sai lệch vector torque trung vị 3,04 Nm, P95 8,16 Nm. Xem
  `m310_reader_rate_audit_20260919_vi.md`. Ảnh hưởng của nó lên các con số
  trên chưa được tách ra.

## 4. Lượt bổ sung tối giản cho một người

Mục tiêu là bổ sung thông tin, không lặp bài X/Y đã dùng được:

1. Giữ vùng bắt đầu gần T1, Ground Truth và cấu hình đã chốt. Reader phải
   có mẫu M310 mới trước bài; không đổi backend giữa một lượt.
2. Ghi 15 s không tiếp xúc trước Start theo runbook.
3. Thực hiện Z+ rồi giảm lực/nhả có kiểm soát; Z− rồi giảm lực/nhả. Mỗi lần
   tác động khoảng 3–5 s, mỗi lần giảm lực khoảng 5 s; lặp 2–3 vòng, tổng
   khoảng 45–60 s. Đi trong vùng đã vận hành, không cần đi xa hoặc tăng lực
   để đạt số mẫu. Ground Truth có lực kéo về pose capture khi nhả.
4. Stop/Disable, ghi tiếp 15 s không tiếp xúc ở file baseline cuối và xác
   nhận file đó thực sự có M310. Không dùng mẫu cũ để thay baseline bị thiếu.

Mã lượt tại Terminal 4: `export HC_T1_TRIAL=t1_dev_z_02`; mọi lệnh logger và
marker dùng như [runbook](hc_force_t1_work_assignment_runbook_20260919_vi.md).
Nếu tool/tare/reader thay đổi, tạo session mới và ghi rõ cấu hình. Không cần
gõ marker từng hướng lúc đang vận hành; Codex phân đoạn bằng raw force.

Đây vẫn là **development**, chưa phải test cuối. Thu Z không tự bảo đảm hết
sai số hoặc nhận dạng được baseline toàn workspace. Codex sẽ đánh giá phần
mới kết hợp dữ liệu cũ; chỉ khi có candidate đạt tiêu chí mới đóng băng và thu
một lượt test độc lập. Y+ hiện ít mẫu hơn X/Y− nhưng chưa yêu cầu thêm một bài
riêng, để giảm công việc tại lab.

Claude tiếp tục phụ trách reader; thông tin cần bàn giao là rate khoảng 5 Hz,
span khoảng 150 ms và baseline cuối không có M310. Cần logger ghi timestamp
từng register và chất lượng scan để kiểm chứng calibration sau khi nâng rate.
Tài liệu này không giao thêm nhiệm vụ điều tra ACK.

## 5. Artifact và tái lập

- [Script audit sidecar](../scripts/audit_hc_t1_sidecar.py).
- [Script review và fit thăm dò](../scripts/review_hc_t1_candidates.py).
- [Tests](../tests/test_hc_t1_sidecar.py): 9 numerical/data-isolation tests đạt;
  `py_compile` đạt. Không sửa package runtime nên không build/deploy ROS.
- [Audit, model fit và mẫu đã ghép](../cocarry_logs/hc_force_calibration/20260919_t1_local_analysis_v1/).
- [Kết quả review chốt](../cocarry_logs/hc_force_calibration/20260919_t1_local_review_v2/).
- [Đồ thị lực](../cocarry_logs/hc_force_calibration/20260919_t1_local_analysis_v1/force_comparison.png).

```bash
cd /home/hungnb/cocarry_ws
OPENBLAS_NUM_THREADS=1 python3 scripts/audit_hc_t1_sidecar.py \
  --main cocarry_logs/hc_force_calibration/20260919_t1_local_dev_v1/cocarry_admittance_3d_20260919_180049.csv.calibration.jsonl \
  --post cocarry_logs/hc_force_calibration/20260919_t1_local_dev_v1/cocarry_admittance_3d_20260919_180506.csv.calibration.jsonl \
  --csv cocarry_logs/hc_force_calibration/20260919_t1_local_dev_v1/cocarry_admittance_3d_20260919_180049.csv \
  --output <thu_muc_audit_moi>
OPENBLAS_NUM_THREADS=1 python3 scripts/review_hc_t1_candidates.py \
  --analysis <thu_muc_audit_moi> \
  --history cocarry_logs/hc_force_calibration/20260919_p0_unified_inputs_v2 \
  --output <thu_muc_review_moi>
```

Output phải mới; log gốc và model runtime được giữ nguyên. Review v1 là kết
quả trung gian, v2 bổ sung metric theo hướng và diễn đạt đầy đủ kết quả fit
hợp nhất; không thay số liệu/model của analysis v1.
