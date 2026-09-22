# P2 — Audit băng thông và độ trễ pipeline co-carrying, 19/09/2026

Đã hoàn tất audit P2 offline và sau đó thực hiện các A/B robot do người vận hành
chạy, được ghi tại mục 5c–5e. Agent không Enable/Start/Servo hay gửi lệnh chuyển
động. Không sửa force/workspace/joint limit, Tool Data, tare, frame, payload hay
model calibration. Trạng thái runtime mới nhất được ghi tại mục 5e.

> **Đính chính trong chính phiên này.** Bản nháp đầu tiên của tài liệu kết luận
> "trần vận tốc Cartesian 0,15 m/s là nút thắt". **Kết luận đó sai.** Nó dùng
> hằng số mặc định trong source streamer, trong khi launch thật truyền
> `--max-vel 0.25 --max-accel 1.00`. Toàn bộ số liệu dưới đây đã chạy lại với
> hằng số đúng. Trần vận tốc thật **chưa bao giờ bão hòa trên cả 21 trial**.

## Kết luận ngắn

Robot chậm **không phải** vì bộ lọc reference `tau=0,4 s`, **không phải** vì
jerk, **không phải** vì IK, và **không phải** vì trần vận tốc Cartesian.

Nguyên nhân là một tương tác ít ai để ý giữa hai khâu:

```text
command-lead clamp cho reference toi da 40 mm truoc EE do duoc
  - queue execution lag: streamer khoi dong lai moi tick tu pose queue DA ACCEPT,
    pose nay von da chay truoc EE do duoc khoang 29 mm
  = chi con ~10 mm cho profile giam toc cua streamer
  -> nhanh giam toc sqrt(2*a*d) chi cho phep ~0,10-0,15 m/s
  -> robot chay o khoang 40% toc do da duoc phep (0,25 m/s)
```

Ngân sách command-lead 40 mm bị queue execution lag ăn mất gần hết. Vì vậy
tăng jerk, tăng trần vận tốc hay hạ `tau` đều không giải quyết được — đúng như
hiện tượng đã ghi ngày 18/09: "tăng jerk lại mất mượt mà không nhanh hơn".

## 1. Dữ liệu và cách đo

- **21 trial robot thật**, 8.627 mẫu, 573,7 s, ngày 18–19/09, toàn bộ mode GRU,
  `role=OFF`. Bỏ 1 trial dưới 100 hàng.
- Chất lượng mẫu sạch: **0 timestamp không đơn điệu, 0 khoảng trống >200 ms**.
  Queue chạy đều **14,94 Hz**, `point_dt` 66,7 ms — không có hiện tượng đói queue.
- Audit đọc module runtime từ **working tree đang chạy**, không phải HEAD. HEAD
  đã commit cũ hơn đáng kể: `prediction_reference.py` trong HEAD *chưa có*
  velocity-lead; `cartesian_streamer_hc10dtp.py` trong HEAD *chưa có*
  `_publish_motion_diagnostics`. Audit theo HEAD sẽ mô hình hóa code chưa từng chạy.

### Hằng số thật sự đã chạy

Đây là điểm đã làm sai ở bản nháp đầu. **Launch ghi đè cả YAML lẫn hằng số
module**, nên không được đọc YAML hay source constant rồi cho là giá trị chạy:

| Tham số | YAML / source | **Launch thật truyền** |
|---|---:|---:|
| Cartesian max velocity | 0,15 m/s | **0,25 m/s** |
| Cartesian max acceleration | 0,50 m/s² | **1,00 m/s²** |
| Cartesian max jerk | 10,0 m/s³ | 10,0 m/s³ |
| `max_command_lead_m` | 0,03 m | **0,04 m** |
| `max_virtual_velocity_mps` | 0,15 m/s | **0,25 m/s** |
| Joint vel [S,L,U,R,B,T] | [0,20…] | **[0,50; 0,50; 0,50; 0,08; 0,50; 0,40]** |

Log xác nhận độc lập: command lead bão hòa đúng `0,0400 m`, khớp giá trị launch.

### Kiểm chứng mô hình offline khớp runtime

Replay `PredictionReference(tau=0,4; lead=0,15; cap 0,02 m)` cộng clamp
`prediction_max_nominal_lead_m=0,05` trên raw GRU đã log, so với `nominal_xd`:

| Chỉ số | Trung vị 21 trial | P95 | Lớn nhất |
|---|---:|---:|---:|
| RMSE vector | 0,503 mm | 0,654 mm | 4,64 mm |
| RMSE sau khi trừ offset `capture_ee` | 0,489 mm | 0,640 mm | 4,58 mm |

Hai lượt dài nhất đạt **0,20 mm và 0,25 mm**. Đủ để dùng mô hình offline, nhưng
**không phải khớp bit-exact như P0 đạt với F_robot (2,3e-7 N)**: `capture_ee`
không được log, `dt` lấy theo ROS clock thay vì `time.monotonic()` của
controller, và CSV ghép mẫu latest-sample-and-hold. Phần dư 0,2–0,5 mm chưa
được quy về một nguyên nhân cụ thể.

Kiểm tra đồng nhất `reference = nominal + error` cho phần dư RMS **2,7 mm**.
Đây *không* phải lỗi công thức mà là độ lệch ghép mẫu của logger giữa ba
callback, và là lý do không đo được trễ liên tầng dưới một tick từ CSV.

## 2. Trễ từng tầng

Trễ hiệu dụng = `khoảng cách / tốc độ lệnh`; chỉ dùng mẫu có tốc độ ≥0,02 m/s.

| Tầng | Trung vị | P95 | P99 |
|---|---:|---:|---:|
| Axia → controller (`force_age_ms`) | 5,2 ms | 8,8 ms | — |
| UDP gap Axia | 10,0 ms | — | — |
| EE pose → controller (`pose_age_ms`) | 9,1 ms | 23,9 ms | — |
| GRU prediction (`prediction_age_ms`) | 30,2 ms | 31,0 ms | 31,3 ms |
| raw GRU → nominal (tau 0,4 + lead 0,15) | **191 ms** | 467 ms | ~680 ms |
| **reference → actual EE (streamer+IK+queue)** | **363 ms** | 565 ms | 761 ms |
| Queue ACK round-trip | 14,6 ms | 29,2 ms | 33,4 ms |

Tổng chuỗi khoảng **585 ms trung vị**. Trễ streamer nằm trong dải hẹp
**335–392 ms** ở cả 21 trial.

**Tầng gây trễ chính: `Cartesian streamer + IK + queue` (363 ms)**, gần gấp đôi
tầng lọc reference (191 ms) và lớn hơn một bậc so với GRU (30 ms) hay Axia (5 ms).

Lưu ý số học: 363 ms ≈ 40 mm ÷ 0,11 m/s. Trễ của tầng này **chính là** ngân sách
command-lead chia cho tốc độ — robot luôn bị bỏ lại đúng một lần ngân sách lead.

Cross-correlation cho trung vị 0 ms, P95 33 ms. Con số này không mâu thuẫn mà
chỉ cho thấy độ phân giải của nó bằng một chu kỳ 66,7 ms, quá thô; không dùng
nó làm kết quả trễ.

## 3. Tầng nào đang giới hạn băng thông

| Limiter | Mức chạm | Ngưỡng |
|---|---:|---|
| **Command lead** | **ghim đúng 40,0 mm ở 20/21 trial**, 68,4% số mẫu | 0,04 m |
| **Queue execution lag** | **27,5–31,4 mm** (chính là tracking error) | stop ở 50 mm |
| Tracking error | trung vị 30,5 mm, P95 35,1 mm | stop ở 50 mm |
| Trần vận tốc Cartesian | **0/21 trial bão hòa**; EE thật 0,093–0,109 m/s | 0,25 m/s |
| `joint_scale` (clamp khớp đồng bộ) | **1,000 ở 100% mẫu** hai lượt dài | — |
| Joint velocity | max 0,45 rad/s | limit 0,50 rad/s |
| IK fail (chuỗi liên tiếp) | **2 sự kiện** trên 573 s | 3 lần thì stop |
| Queue rate | 14,94 Hz đều | 15 Hz |
| Queue BUSY/retry | **không đo được** | — |

### Cơ chế thắt cổ chai, đã kiểm chứng số

Streamer clamp reference ≤40 mm trước EE **đo được**, nhưng bộ smoother lại
khởi động lại mỗi tick từ **pose queue đã accept** (`_on_queue_result` gán
`_current_ee_pose = queued_pose`). Pose đó đã chạy trước EE đo được một khoảng
bằng queue execution lag. Khoảng cách mà profile thực sự nhìn thấy là:

```text
d_eff = command_lead - queue_lag = 40 mm - 29 mm ~ 10 mm
v = min( v_max, sqrt(2*a*d_eff), d_eff/dt )
```

Đối chiếu công thức này với số đo trên cả 21 trial:

| Trial | lead | queue lag | d_eff | v dự đoán | v đo thật |
|---|---:|---:|---:|---:|---:|
| 20260919_103009 | 40,0 mm | 29,5 mm | 6,6 mm | 0,099 m/s | **0,095 m/s** |
| 20260918_174205 | 40,0 mm | 31,4 mm | 6,1 mm | 0,091 m/s | **0,100 m/s** |
| 20260918_155858 | 37,9 mm | 27,5 mm | 6,4 mm | 0,096 m/s | **0,093 m/s** |
| 20260918_172207 | 40,0 mm | 28,2 mm | 9,9 mm | 0,141 m/s | **0,097 m/s** |

Tỷ lệ dự đoán/đo trung vị trên 21 trial là **1,14**: công thức bậc một hơi
dự đoán cao vì bỏ qua pha tăng tốc và bộ giới hạn jerk, nhưng đúng bậc độ lớn
và đúng chiều. Nếu smoother có đủ 40 mm, nó sẽ chạm **0,25 m/s** — tức robot
hiện chạy khoảng **40% tốc độ đã được phép**.

Một phần của queue lag đến từ `QUEUE_PREBUFFER_POINTS = 3`: ba điểm × 66,7 ms
≈ 200 ms lịch trình mà robot đang chạy phía sau. Đây là giả thuyết có cơ sở
từ đọc code, **chưa được kiểm chứng bằng thực nghiệm**.

## 4. So sánh candidate offline

Bước tổng hợp 0,10 m, chạy qua cùng chuỗi hạ nguồn với trần thật (v=0,25;
a=1,00; jerk=10), trễ phản hồi 81,3 ms lấy từ ACK queue đo được:

| Candidate | t90 | Peak speed | Jerk P95 | Tích phân jerk | Overshoot | Chạm lead | t90 chỉ governor |
|---|---:|---:|---:|---:|---:|---:|---:|
| Baseline tau 0,4 + lead 0,15 | 600 ms | 0,250 | 6,12 | 4,27 | 0 | 5 | 600 ms |
| tau 0,15 + rate/accel bound | 533 ms | 0,224 | 10,84 | 8,18 | 20,5 mm | 1 | 467 ms |
| Bậc hai critically damped 0,45 s | 600 ms | 0,213 | **5,72** | **3,69** | **0** | **0** | 600 ms |
| Bậc hai + jerk-limit + feed-forward | 533 ms | 0,231 | 12,73 | 13,22 | 6,1 mm | 4 | 400 ms |

Sàn động học của bài test này là 400 ms. Mọi candidate nằm trong 533–600 ms.
Candidate nhanh nhất chỉ hơn baseline **67 ms (11%)**, và phải trả giá bằng
jerk P95 gấp đôi.

**Quan trọng:** mô phỏng bước này **không** mô hình hóa cơ chế queue-pose ở
mục 3, nên nó cho peak 0,21–0,25 m/s trong khi robot thật chỉ đạt 0,10–0,17 m/s.
Vì vậy **không được đọc bảng này như dự báo tốc độ trên phần cứng**; nó chỉ so
sánh tương đối các governor khi ngân sách lead không bị ăn mất.

Replay open-loop trên ba trial dài cho **mọi candidate chênh nhau dưới 1,5%**.
Giá trị tuyệt đối của replay đó là artefact phân kỳ open-loop, không phải số đo trễ.

### Độ nhạy theo trần hạ nguồn

| v_max | a_max | t90 bước 0,10 m | Jerk P95 |
|---:|---:|---:|---:|
| **0,25 / 1,00 (hiện tại)** | | **600 ms** | 6,12 |
| 0,25 | 1,50 | 533 ms | 10,0 (chạm trần jerk) |
| 0,35 | 1,00 | 600 ms | 6,42 |
| 0,35 | 1,50 | 600 ms | 10,0 |
| 0,45 | 1,50 | 600 ms | 10,0 |

**Nâng trần vận tốc lên 0,35 hay 0,45 m/s không cải thiện gì** (vẫn 600 ms), vì
vận tốc vốn đã không phải ràng buộc. Đây là bằng chứng định lượng cho việc
không nên đụng vào trần vận tốc.

### Sweep ngân sách command-lead — đòn bẩy thật sự

Áp công thức đã kiểm chứng ở mục 3, với queue lag hiện tại 29 mm:

| command_lead | a_max | d_eff | v dự đoán | so với hiện tại | chạm trần vận tốc |
|---:|---:|---:|---:|---:|---|
| **0,04 (hiện tại)** | 1,00 | 11 mm | 0,148 m/s | 1,0× | không |
| 0,05 | 1,00 | 21 mm | 0,205 m/s | **1,4×** | không |
| 0,06 | 1,00 | 31 mm | 0,249 m/s | **1,7×** | gần chạm |
| 0,08 | 1,00 | 51 mm | 0,250 m/s | 1,7× | có |

Nếu thay vào đó giảm queue lag xuống 20 mm (prebuffer nông hơn) mà **giữ
nguyên** command lead 0,04 m: d_eff 20 mm → 0,200 m/s, tức **1,35×** mà không
nới bất kỳ giới hạn an toàn nào.

## 5. Dữ liệu còn thiếu

> **Cập nhật:** các mục 1, 3 và 5 đã được bổ sung đo lường trong chính đợt này
> (xem mục 5b). Chúng vẫn thiếu **trong các log 18–19/09 đã phân tích ở trên**,
> nhưng sẽ có từ lần chạy robot kế tiếp.

1. **Đếm queue BUSY/retry/reject không tồn tại ở bất kỳ topic hay cột CSV nào.**
   Streamer chỉ giữ chúng trong biến đếm cửa sổ 5 s in ra console. Đây là khoảng
   trống đo lường lớn nhất ở đúng tầng gây trễ nhiều nhất. **Đã bổ sung.**
2. **Không có lượt chạy với bước dịch chuyển có kiểm soát.** Trial 126 s cho
   đúng **một** đoạn chuyển động liên tục, nên `t90` từ log tự nhiên (~8 s) phản
   ánh nhịp tay người chứ không phải băng thông pipeline. Mọi số t90 ở mục 4 đến
   từ mô phỏng, chưa có đối chứng phần cứng.
3. **Chưa đo trực tiếp thành phần của queue execution lag.** Chưa tách được phần
   do `QUEUE_PREBUFFER_POINTS=3`, phần do buffer/nội suy của YRC1000, và phần do
   servo lag. Nếu phần lớn là do controller thì giảm prebuffer sẽ không giúp nhiều.
4. **CSV ghép latest-sample-and-hold ở tick 15 Hz**; không phân giải được trễ
   liên tầng dưới ~66 ms.
5. **Trạng thái limiter nội bộ streamer không được log**; số jerk là proxy đạo
   hàm số từ EE thật. **Đã bổ sung.**
6. **`_capture_ee` không được log**, phải xấp xỉ bằng mẫu EE đầu tiên.
   **Đã bổ sung.**
7. Chưa biết **speed override trên pendant** lúc chạy các trial này. Người dùng
   xác nhận không nhớ mức đã đặt. Nếu nó không ở 100% thì một phần tốc độ thiếu
   là do nó chứ không do phần mềm. **Đây là biến chưa xác định**, phải kiểm tra
   và ghi lại thủ công vào ghi chú trial trước lần chạy tới; phần mềm không đọc
   được giá trị này.

## 5b. Đo lường đã bổ sung trong đợt này

Theo lựa chọn của người dùng: **chỉ bổ sung đo lường, không đổi tham số điều
khiển nào.** Các thay đổi dưới đây thuần chẩn đoán, không đụng vào bất kỳ nhánh
quyết định điều khiển nào, không đổi giới hạn an toàn.

`src/hc10dtp_bringup/scripts/cartesian_streamer_hc10dtp.py`
— `motion_diagnostics_json` nâng lên **schema 2**, thêm:

- `queue_busy_total`, `queue_retry_total`, `queue_reject_total`,
  `accepted_points_total`, `auto_recovery_count` — bộ đếm **tích lũy**, không bị
  xóa mỗi 5 s như bộ đếm cửa sổ cũ.
- `smoother_velocity`, `smoother_acceleration` — trạng thái nội bộ của profile
  limiter, để jerk/gia tốc không còn phải suy ra bằng đạo hàm số từ EE đo được.
- `limits` — **giới hạn thực sự đang có hiệu lực sau khi launch ghi đè**:
  `max_cartesian_velocity/acceleration/jerk`, `max_joint_velocities`,
  `prebuffer_points`, `stream_hz`, `queue_dt_sec`, `max_tracking_error_m`,
  `joint_coordination`, `fail_closed`.

`src/cocarry_admittance_control/.../admittance_controller.py`
— `hybrid_status_json` thêm:

- `capture_ee` — gốc tương đối của lượt chạy, để replay offline không phải đoán
  bằng mẫu EE đầu tiên.
- `limits` — `max_command_lead_m`, `prediction_max_nominal_lead_m`,
  `prediction_reference_tau_sec/lead_sec/max_lead_m`, `max_virtual_velocity`,
  `max_virtual_acceleration`, `control_rate_hz`, các timeout lực.

Trường `limits` ở cả hai nơi chính là biện pháp chống lặp lại lỗi hằng số đã
mắc trong đợt này: từ nay log tự mô tả giới hạn của nó, và
`audit_hc_p2_pipeline_bandwidth.py` **ưu tiên đọc giới hạn từ log**, chỉ rơi về
hằng số khi log cũ hơn schema 2, đồng thời báo `source` và `mismatches`.

Không cần `colcon build`: `install/` hiện dùng `--symlink-install`, file
streamer là symlink trỏ thẳng vào source và `cocarry_admittance_control` là
egg-link, nên sửa source có hiệu lực ngay ở lần launch kế tiếp.

Chi phí: `limits` được lặp lại mỗi tick nên CSV dài thêm khoảng vài MB mỗi
trial. Đây là đánh đổi có chủ ý, đổi dung lượng lấy khả năng tự mô tả.

## 5c. Lượt mốc schema 2 — hai nguyên nhân gốc đã xác định

Người dùng chạy hai lượt mới ngay sau khi bổ sung đo lường:
`20260919_120505` (530 mẫu, 35,3 s) và `20260919_120622` (888 mẫu, 59,1 s),
đều GRU/FOLLOWER, cấu hình **không đổi**. Kết quả tại
`cocarry_logs/20260919_p2_schema2_baseline_v1/`.

Đo lường mới hoạt động: `effective_limits.source = "log"`, `mismatches: none`
— runtime tự xác nhận trần thật là `0,25 m/s` và `1,00 m/s²`, đúng như bản
đính chính, và bác bỏ dứt điểm hằng số `0,15/0,50` của bản nháp đầu.

### Các giả thuyết bị loại bỏ dứt điểm

| Nghi phạm | Đo được | Kết luận |
|---|---:|---|
| Queue BUSY / retry / reject | **0 / 0 / 0** | Không đóng góp gì |
| Auto-recovery queue | **0** | Queue chưa từng bị drop |
| IK fail | **0** | Không phải nguyên nhân |
| Joint velocity clamp | **0 / 887 tick** | Không hề chạm |
| Trần vận tốc 0,25 m/s | lệnh tối đa 0,18–0,19 m/s | **Chưa từng chạm** |

Điểm mù lớn nhất của bản audit trước — queue BUSY/retry — nay đã có số liệu và
câu trả lời là **không có lần nào**. Không còn lý do nghi ngờ tầng queue.

### Nguyên nhân gốc: khoảng cách smoother được phép đuổi chỉ ~15–18 mm

Đo trực tiếp khoảng cách `dist` mà `_smooth_pose` nhìn thấy mỗi tick
(`|reference_k − ik_request_{k−1}|`, hợp lệ vì backpressure gán
`_current_ee_pose = queued_pose`):

| | 120505 | 120622 |
|---|---:|---:|
| `dist` trung vị | 15,4 mm | 18,3 mm |
| `dist` P95 | 22,0 mm | 22,8 mm |

Con số nhỏ này rơi đúng vào hai ngưỡng có sẵn trong code, và gây ra **hai**
hậu quả khác nhau:

**(1) Bộ giới hạn jerk bị tắt — giải thích "mất mượt".**
`_smooth_pose` có nhánh `if dist < 0.020: max_da = float('inf')`, cố ý bỏ giới
hạn jerk khi đã gần đích để tránh overshoot. Nhưng trong co-carry, "gần đích"
lại là **chế độ vận hành bình thường**:

- Tỷ lệ tick có `dist < 20 mm`, tức **jerk không bị giới hạn**: **74,8%** và **62,0%**.
- Jerk **từng trục** đo được: trung vị 4,6–5,5; P95 **15,5–16,3**; lớn nhất
  **22,2–26,6 m/s³**, trong khi trần cấu hình là **10,0**.
- **21,3–21,4% số mẫu có jerk từng trục vượt trần 10 m/s³.**

Đây là lý do vì sao **tăng `--max-jerk` không có tác dụng**: tham số đó không
được áp dụng trong phần lớn thời gian chạy.

**(2) Profile luôn ở nhánh phanh — giải thích trần tốc độ.**
Nhánh giảm tốc kích hoạt khi `dist < v²/(2a) = 31,2 mm`:

- Tỷ lệ tick ở nhánh phanh: **99,4%** ở cả hai lượt.
- Tỷ lệ tick được chạy full speed 0,25 m/s: **0,6%**.
- Trần tốc độ mà profile cho phép từ `dist`: **0,176–0,191 m/s** trung vị,
  tức **thấp hơn hẳn trần cấu hình 0,25 m/s**.

Vì vậy nâng `--max-vel` lên 0,35 hay 0,45 m/s là vô nghĩa: hệ chưa bao giờ
tới được 0,25 m/s.

**Cần nói rõ giới hạn của kết luận này.** Tốc độ lệnh thực tế
(0,094–0,099 m/s trung vị) vẫn **thấp hơn** trần 0,176–0,191 mà profile cho
phép. Nghĩa là ở nhịp trung bình, profile **không phải** ràng buộc duy nhất:
reference do người và admittance kéo đi cũng chỉ di chuyển ở tốc độ đó. Không
được kết luận rằng nới `dist` sẽ tự động cho tốc độ cao hơn ở mọi thời điểm;
điều chắc chắn là nó **nâng trần** và **bật lại bộ giới hạn jerk**.

### Cả hai hậu quả có chung một nguyên nhân

`dist ≈ command_lead (40 mm) − queue execution lag (28,5 mm)`. Ngân sách 40 mm
bị queue lag ăn mất gần 3/4, đẩy điểm vận hành xuống dưới **cả hai** ngưỡng
20 mm và 31 mm. Nới `dist` xử lý đồng thời cả tốc độ lẫn độ mượt.

Ước lượng phần đóng góp của prebuffer: `QUEUE_PREBUFFER_POINTS = 3` × 66,7 ms
≈ 200 ms lịch trình; ở 0,1 m/s tương đương ≈ 20 mm trong tổng 28,5 mm đo được.
Giảm còn 2 điểm sẽ bỏ đi ≈ 6,7 mm, đưa `dist` trung vị từ 15–18 mm lên
≈ 22–25 mm, tức **vượt ngưỡng 20 mm** và tiến gần ngưỡng 31 mm. Đây là ước
lượng bậc một từ dữ liệu, **chưa được kiểm chứng trên robot**.

## 5d. Thay đổi đã áp dụng ngày 19/09 theo yêu cầu người dùng

Người dùng yêu cầu thực hiện **đồng thời hai** thay đổi. Cả hai nằm trong
`launch/cocarry_admittance_real_gui.launch.py`, ở **hai commit riêng** để có
thể revert độc lập.

| # | Thay đổi | Có nới giới hạn an toàn? |
|---|---|---|
| 1 | Thêm `--prebuffer 2` cho streamer (trước đây dùng mặc định 3) | **Không** |
| 2 | `command_lead_m` default `0.04` → `0.05` | **Có** |

Thay đổi 1 dùng CLI arg `--prebuffer` **đã có sẵn**, không sửa hằng số
`QUEUE_PREBUFFER_POINTS` của module, nên mặc định của các launch khác
(simulation, co-drawing) không đổi.

**Cảnh báo về quy kết.** Đổi hai thứ cùng lúc đi ngược nguyên tắc "chỉ đổi một
tầng mỗi lần" trong task spec và trong `CODEX.md`. Hai thay đổi này lại còn
**tương tác** với nhau: giảm prebuffer làm *giảm* queue lag nên *giảm* tracking
error, trong khi tăng command lead cho phép chạy nhanh hơn nên *tăng* tracking
error. Nếu tracking error thay đổi, không suy ra được khâu nào gây ra. Nếu kết
quả xấu đi, revert **từng commit một** để phân biệt.

**Rủi ro riêng của prebuffer 2.** Prebuffer quyết định độ sâu hàng đợi ở trạng
thái ổn định, vì streamer gửi một điểm mỗi tick và robot tiêu thụ một điểm mỗi
tick. Còn 2 điểm thay vì 3 nghĩa là ít đệm hơn khi một tick bị trễ: nguy cơ
queue underrun, chuyển động giật, hoặc queue bị drop. Lượt mốc có
`queue_busy_total = 0` nên còn dư địa, nhưng **đây là chỉ số phải xem đầu tiên**.

**Rủi ro riêng của command lead 0,05.** Cho reference ngồi xa EE đo được hơn.
Tracking error ở lượt mốc đã là 33,1–33,5 mm P95 so với ngưỡng dừng 50 mm.

**Bắt buộc rebuild.** Khác với hai file runtime ở mục 5b, launch file được cài
dưới dạng **bản copy** trong `install/.../share/`, không phải symlink. Đã chạy
`colcon build --symlink-install --packages-select cocarry_admittance_control`
và xác minh bản trong `install/` có đúng cả hai thay đổi. Nếu sau này sửa lại
launch file, **phải build lại**, nếu không sẽ chạy cấu hình cũ mà tưởng là mới.

### Dự đoán và cách kiểm chứng

`dist` trung vị dự kiến tăng từ 15–18 mm lên khoảng 32–35 mm (≈ +6,7 mm từ
prebuffer, +10 mm từ command lead), tức vượt **cả hai** ngưỡng 20 mm và 31,2 mm.
Nếu đúng, lượt sau phải thấy:

- Tỷ lệ tick `dist < 20 mm` giảm mạnh từ 62–75% xuống gần 0 → **jerk từng trục
  P95 phải tụt xuống dưới 10 m/s³**. Đây là chỉ số quyết định vế "mất mượt".
- Tỷ lệ tick ở nhánh phanh giảm từ 99,4%, trần tốc độ profile tăng từ
  0,176–0,191 m/s lên tới 0,25 m/s.

Đây là **dự đoán chưa kiểm chứng**. Quan hệ giữa prebuffer và queue lag là ước
lượng bậc một, chưa đo trên robot.

Chạy lại audit trên log mới bằng đúng lệnh ở mục 8 rồi so với bảng mốc ở mục 6.

## 5e. Kết quả A/B `command_lead=0,04`, `prebuffer=2` và trạng thái mới nhất

Người dùng chạy lượt `20260919_125048` sau khi hoàn nguyên riêng command lead
về 0,04, giữ prebuffer 2. Log tự xác nhận đúng cấu hình. Audit nằm tại
`cocarry_logs/20260919_p2_lead004_prebuffer2_v1/`.

| Chỉ số | Baseline 0,04/3 (`120505`, `120622`) | 0,05/2 (`122735`) | **0,04/2 (`125048`)** |
|---|---:|---:|---:|
| Tốc độ lệnh trung vị | 0,094 / 0,099 m/s | 0,113 m/s | **0,0946 m/s** |
| Tốc độ lệnh P95 | 0,129 / 0,133 | 0,153 | **0,133** |
| Peak EE thật | 0,150 / 0,172 | 0,174 | **0,157** |
| Queue lag không gian, median | 28,6 / 28,5 mm | 32,7 mm | **27,6 mm** |
| Tracking error P95 | 33,5 / 33,1 mm | 39,0 mm | **32,2 mm** |
| Trễ reference→EE, median | 393 / 385 ms | 402 ms | **398 ms** |
| Jerk lệnh P95 | 18,1 / 17,5 m/s³ | 17,8 | **18,3** |
| BUSY/retry/reject | 0/0/0 | 0/0/0 | **0/0/0** |
| IK fail | 0 | 0 | **0** |

Kết luận A/B: giảm prebuffer 3→2 chỉ giảm tracking/queue lag không gian khoảng
1 mm, không giảm lag theo thời gian và không tăng tốc có ý nghĩa. Command lead
0,05 tăng nhẹ tốc độ nhưng đẩy tracking P95 lên 39 mm và max 44,6 mm, quá gần
ngưỡng dừng 50 mm. Vì vậy launch đã trở về baseline `command_lead_m=0.04`,
`prebuffer=3`; không tiếp tục dùng hai tham số này làm đòn bẩy tốc độ.

Đọc sâu source cũng thu hẹp vế mất mượt. Trong co-carry continuous, `dist<20
mm` là trạng thái phổ biến nhưng code cũ tắt jerk limiter tại đó. Đồng thời ACK
ở chế độ synchronized ghi thẳng vận tốc FK và sai phân bậc hai của nó vào trạng
thái smoother. Candidate mới:

- luôn giữ jerk bound trong continuous mode, trừ khi đã thực sự tới đích
  (`dist<1e-5`);
- reconcile feedback vận tốc ACK qua giới hạn velocity/acceleration/jerk thay
  vì gán tức thời;
- giữ hành vi lịch sử cho camera/demo non-continuous;
- log riêng raw accepted velocity/acceleration và cờ reconciliation.

Candidate này không nới giới hạn và đã qua test offline, nhưng chưa chạy robot
thật. Nó nhằm khôi phục độ mượt/hiệu lực jerk limiter, **không phải** lời hứa
tăng tốc; queue execution lag khoảng 0,29–0,30 s vẫn là giới hạn nền. Lượt robot
kế tiếp phải giữ baseline 0,04/3 và chỉ đánh giá jerk, overshoot, tracking cùng
queue health. Nếu có dao động hoặc tracking P95 >40 mm thì dừng và hoàn nguyên
candidate.

## 6. Metric và điều kiện dừng đề xuất cho P5

Chỉ thực hiện khi người vận hành đồng ý, chạy robot thật, speed override thấp,
sẵn sàng E-stop. **Mỗi lần chỉ đổi một tầng.**

Bước đo lường đã xong (mục 5b) và lượt mốc schema 2 đã có (mục 5c). Việc còn
lại chưa làm: **kiểm tra và ghi lại speed override trên pendant** — phần mềm
không đọc được giá trị này.

Sau kết quả mục 5c, thứ tự ưu tiên đã thay đổi so với bản trước, vì nay biết
mục tiêu là **nới `dist` lên trên 20 mm** chứ không phải nới trần tốc độ:

1. **Giảm `QUEUE_PREBUFFER_POINTS` 3 → 2** (ưu tiên cao nhất). Không nới bất kỳ
   giới hạn an toàn nào. Dự kiến `dist` tăng ≈6,7 mm, đưa trung vị từ 15–18 mm
   lên ≈22–25 mm, tức **vượt ngưỡng 20 mm nên bật lại bộ giới hạn jerk** và
   đồng thời nâng trần tốc độ của profile. Xử lý cả hai nguyên nhân gốc bằng
   một thay đổi duy nhất, không đụng giới hạn an toàn nào.
   Rủi ro: buffer mỏng hơn → dễ queue underrun/BUSY. Lượt mốc cho BUSY = 0 nên
   còn dư địa, nhưng đây chính là chỉ số phải theo dõi sát nhất.
2. **Sửa ngưỡng bỏ giới hạn jerk** `dist < 0.020` trong `_smooth_pose`. Đây là
   cách xử lý tận gốc vế "mất mượt": dựa vào việc `dist` tình cờ nằm trên 20 mm
   là một sự phụ thuộc mong manh. Nhưng nhánh này tồn tại để chống overshoot
   khi tới gần đích, nên hạ ngưỡng có thể làm robot dao động quanh điểm dừng.
   **Chỉ nên làm sau khi bước 1 đã đo xong**, để biết phần nào của độ mượt do
   `dist` và phần nào do chính ngưỡng.
3. **Tăng `command_lead_m` 0,04 → 0,05.** Đây **là** nới một giới hạn an toàn:
   cho reference chạy xa EE thật hơn. Rủi ro trực tiếp là tracking error tăng,
   mà nó hiện đã ở **57–67% ngưỡng dừng 50 mm** (28,5–33,5 mm) — có thể gây
   safety stop giả. Chỉ dùng nếu bước 1 chưa đủ; không tăng quá 0,05 lần đầu.
4. Chỉ sau các bước trên mới cân nhắc governor bậc hai.

**Không** đụng trần vận tốc/gia tốc: mục 4 và 5c đều cho thấy hệ chưa bao giờ
chạm 0,25 m/s, nên nâng trần lên 0,35–0,45 m/s không có tác dụng.

Metric ghi lại mỗi lượt, so với mốc trong tài liệu này:

Mốc lấy từ hai lượt schema 2 ngày 19/09 (mục 5c), không phải từ log 18/09:

| Metric | Mốc schema 2 | Ý nghĩa |
|---|---:|---|
| **`dist` smoother nhìn thấy** | **15,4 / 18,3 mm** trung vị | **nguyên nhân gốc** |
| **Tỷ lệ tick `dist` < 20 mm (tắt jerk limit)** | **74,8% / 62,0%** | **vế mất mượt** |
| **Jerk từng trục P95 / max** | **15,5–16,3 / 22,2–26,6 m/s³** | trần cấu hình 10,0 |
| Tỷ lệ mẫu jerk vượt trần 10 | 21,3–21,4% | mục tiêu giảm |
| Tỷ lệ tick ở nhánh phanh | 99,4% | vế chậm |
| Trần tốc độ profile cho phép | 0,176–0,191 m/s | so trần cấu hình 0,25 |
| Tốc độ lệnh thực tế | 0,094–0,099 m/s | nhịp người + admittance |
| Queue execution lag | 28,5–28,6 mm | thành phần ăn ngân sách |
| Tracking error | 28,5–28,6 / 33,1–33,5 mm P95 | biên an toàn, stop 50 mm |
| Trễ hiệu dụng reference → actual | 385–393 ms / 544–550 ms P95 | tầng chính |
| Tỷ lệ chạm command-lead | 62,1% / 67,9% | mức bão hòa |
| **Queue BUSY / retry / reject** | **0 / 0 / 0** | phải giữ bằng 0 |
| IK fail / joint clamp | 0 / 0 | phải giữ bằng 0 |

Điều kiện dừng ngay, quay lại cấu hình mốc:

1. Tracking error P95 vượt **40 mm** (80% ngưỡng dừng 50 mm), hoặc có bất kỳ
   safety stop do tracking. Mốc hiện tại 33,1–33,5 mm nên biên còn lại rất mỏng.
2. Có **bất kỳ** IK fail nào (mốc hiện tại là 0), hoặc joint clamp bắt đầu chạm.
3. **`queue_busy_total` hoặc `queue_retry_total` khác 0** — mốc hiện tại bằng 0
   nên bất kỳ giá trị dương nào cũng là thay đổi thật, không phải nhiễu. Đây là
   chỉ số cảnh báo sớm quan trọng nhất khi giảm prebuffer.
4. Jerk P95 proxy tăng quá 50% so với mốc, hoặc người vận hành báo mất mượt —
   **ưu tiên đánh giá của người vận hành hơn số đo**.
5. Xuất hiện overshoot nhìn thấy được tại điểm dừng, hoặc dao động quanh đích.
6. Bất kỳ fault lực/pose/readiness nào không tái lập được nguyên nhân.

## 7. Những gì chưa được phép kết luận từ log hiện tại

- **Không kết luận candidate nào nhanh hơn trên robot thật.** Toàn bộ so sánh là
  mô phỏng offline không mô hình hóa IK, soft joint limit, queue admission, và
  cũng không mô hình hóa chính cơ chế queue-pose ở mục 3.
- **Không coi `t90` ở mục 4 là số đo phần cứng.** Chưa có lượt bước có kiểm soát.
- **Không coi sweep ở mục 4 là dự báo tốc độ sẽ đạt được.** Nó là công thức bậc
  một khớp số đo ở tỷ lệ 1,14, không phải mô phỏng đầy đủ chuỗi tick.
- **Không kết luận giảm prebuffer sẽ giảm queue lag tương ứng.** Chưa tách được
  thành phần của queue lag; phần thuộc về YRC1000 nằm ngoài tầm điều chỉnh từ ROS.
- **Không kết luận mô hình offline khớp runtime ở mức bit-exact.** Phần dư
  0,2–0,5 mm chưa được quy về nguyên nhân cụ thể.
- ~~Không kết luận queue/BUSY không phải vấn đề chỉ vì không đo được.~~
  **Đã giải quyết:** lượt mốc schema 2 đo được BUSY/retry/reject = 0/0/0 trên
  cả hai lượt, nên queue thực sự không đóng góp. Kết luận này chỉ áp dụng cho
  cấu hình hiện tại; nếu giảm prebuffer thì phải đo lại.
- **Không kết luận nới `dist` sẽ làm robot nhanh hơn ở mọi thời điểm.** Tốc độ
  lệnh thực tế (0,094–0,099 m/s) còn thấp hơn trần profile (0,176–0,191 m/s),
  nên ở nhịp trung bình profile không phải ràng buộc duy nhất. Điều chắc chắn
  là nới `dist` nâng trần và bật lại bộ giới hạn jerk.
- **Không kết luận giảm prebuffer sẽ đưa `dist` lên đúng 22–25 mm.** Đó là ước
  lượng bậc một từ `3 × 66,7 ms × 0,1 m/s`; quan hệ giữa prebuffer và queue lag
  chưa được kiểm chứng bằng thực nghiệm.
- **Không coi 2 sự kiện IK fail là bảo đảm IK an toàn** cho cấu hình nhanh hơn;
  con số đó thuộc về cấu hình hiện tại với reference bị throttle.
- **Không dùng audit P2 làm bằng chứng để đưa `F_robot` vào vòng điều khiển.**
  P2 không đánh giá calibration lực; kết luận P0 giữ nguyên, `F_robot` vẫn shadow.
- **Không suy rộng ra mode LEADER/MJM.** Toàn bộ 21 trial có `role=OFF`; pha
  LEADER thuộc phạm vi P3.

## 8. Artifacts và tái lập

- Script audit: [audit_hc_p2_pipeline_bandwidth.py](../scripts/audit_hc_p2_pipeline_bandwidth.py)
- Script mô phỏng: [simulate_hc_p2_reference_candidates.py](../scripts/simulate_hc_p2_reference_candidates.py)
- Test: `tests/test_hc_p2_pipeline_audit.py` — 18 test đạt.
- **Lượt mốc schema 2 (kết quả mới nhất, mục 5c):**
  `cocarry_logs/20260919_p2_schema2_baseline_v1/`.
- **Kết quả trên log 18–19/09:** `cocarry_logs/20260919_p2_pipeline_audit_v4/` và
  `cocarry_logs/20260919_p2_candidates_v3/`.
  `v4` chạy bằng script đã biết đọc giới hạn từ log; với log 18–19/09 nó báo
  `effective_limits.source = "assumed_constants"` vì các log đó có trước
  schema 2. Số liệu của `v4` trùng `v3`; khác biệt là `v4` ghi rõ nguồn giới hạn.
- `*_v2` là kết quả trung gian dùng hằng số vận tốc/gia tốc **sai** (0,15/0,50);
  giữ lại để đối chiếu lịch sử nhưng **không dùng cho kết luận**. `*_v1` đã bị
  xóa trong phiên này vì dùng cả command-lead sai 0,03 m; không phải dữ liệu
  thực nghiệm mà chỉ là kết quả phân tích sinh lại được.

```bash
cd /home/hungnb/cocarry_ws
OPENBLAS_NUM_THREADS=1 python3 scripts/audit_hc_p2_pipeline_bandwidth.py \
  --output cocarry_logs/<thu_muc_moi>
OPENBLAS_NUM_THREADS=1 python3 scripts/simulate_hc_p2_reference_candidates.py \
  --output cocarry_logs/<thu_muc_moi_2>
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_hc_p2_pipeline_audit.py -q
```

Cả hai script từ chối thư mục output đã tồn tại và không ghi đè log hiện có.
Pytest dùng `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` vì plugin ROS `launch_testing`
cài sẵn không tương thích pytest hiện tại; không đổi môi trường hay package ROS.
SciPy hệ thống vẫn cảnh báo NumPy 1.26.4 ngoài dải hỗ trợ khai báo — không đổi.

## 9. Bài học quy trình

Lỗi hằng số ở bản nháp đầu đáng ghi lại: **đọc YAML hoặc hằng số module rồi coi
đó là giá trị đang chạy là sai**, vì launch file ghi đè cả hai. Trước mọi phân
tích định lượng sau này, phải truy giá trị thật theo thứ tự
`launch arguments → parameter override → YAML → hằng số module`, và nếu có thể
thì kiểm chứng chéo bằng chính log (ở đây log bão hòa đúng 0,0400 m đã chỉ ra
ngay giá trị command lead thật).
