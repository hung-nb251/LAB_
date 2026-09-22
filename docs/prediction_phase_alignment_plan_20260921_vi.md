# Kế hoạch: khóa pha nhịp điều khiển với luồng prediction — 21/09

## 1. Vấn đề

Chuỗi nominal đi qua ba lưới 15 Hz độc lập, không có gì đồng bộ chúng:

| Node | Tham số | Giá trị |
|------|---------|---------|
| `ee_position_tracker` | `publish_rate_hz` | 15.0 |
| `trajectory_predictor` | `inference_rate_hz` | 15.0 |
| `cocarry_admittance_controller` | `control_rate_hz` | 15.0 |

Ba timer ROS tự do chạy. Pha tương đối được chốt lúc các node khởi động và giữ
nguyên suốt phiên, nên một mẫu prediction nằm chờ trong controller một khoảng
**hằng số phụ thuộc lần launch, trong [0, 66,7] ms**. `admittance_controller.py`
đo đúng khoảng này: `received_at = time.monotonic()` tại lúc bản tin tới
(dòng 536), rồi `prediction_age = now(tick) − received_at`.

### Bằng chứng

Các giá trị `prediction_age_ms` median quan sát được ngày 21/09 — 5,4 / 18,2 /
21,4 / 33,2 / 53,0 / 63,7 / 64,5 / 66,7 ms — trải đúng một chu kỳ điều khiển,
và trong mỗi lượt cực kỳ ổn định (p95 lệch dưới 3 ms so với median).

Cặp đối chứng do người vận hành tạo ra, cùng tác vụ, cùng người, cùng deadband
2,5 N, cùng `logging_profile:=diagnostic`, chỉ khác lần khởi động launch:

| Lượt | `prediction_age` | `rev/s` | Tốc độ EE | Ripple |
|------|------------------|---------|-----------|--------|
| `134314` | 33,2 ms | 0,000 | 0,1007 m/s | không |
| `140717` | 64,5 ms | 0,073 | 0,0855 m/s | có |

Gom theo nhóm trên toàn bộ lượt GRU trong ngày: tuổi ≤ 33 ms cho `rev/s` median
khoảng 0,01; tuổi ≥ 53 ms cho median khoảng 0,085.

Ngoài ra pha ≈ 5 ms có một chế độ hỏng riêng: prediction tới sát ngay trước
tick nên jitter làm nó thỉnh thoảng trượt sang tick sau, gây **16% tick dùng
lại giá trị cũ** và nhịp nominal thực chỉ còn 12,60 Hz (lượt `120649`).

### Vì sao phương án A không đủ

Phương án A (bù lead theo tuổi mẫu đo được, đã triển khai trong
`prediction_reference.py`) làm giảm **sai số bám vị trí** của nominal. Lượt
`140717` chạy với nó và ripple vẫn xuất hiện. Trễ một chu kỳ vẫn nằm nguyên
trong vòng kín `robot EE → tracker → predictor → nominal → admittance →
reference → robot`; số hạng lead lại bị kẹp ở 20 mm và vận tốc đã qua lọc
`velocity_tau = 0,15 s`, nên ở tần số dao động của vòng (~0,3 Hz) nó gần như
không bù được pha. Phương án A chữa sai số bám, không chữa biên pha.

## 2. Mục tiêu

Đưa `prediction_age` về một giá trị **tất định, không phụ thuộc lần launch**,
nằm trong vùng đã cho kết quả tốt. Mục tiêu là **nửa chu kỳ, 33,3 ms**: nó có
biên jitter về cả hai phía, tránh được vùng ≥53 ms gây ripple và vùng ≈5 ms gây
rơi mẫu, và là đúng pha của hai lượt duy nhất trong ngày đạt `rev/s = 0,000`.

Không nhằm giảm trễ xuống 0. Nhằm làm cho nó **giống nhau ở mọi lần chạy**, để
kết quả thí nghiệm có thể so sánh được và ripple không còn là chuyện may rủi.

## 3. Thiết kế

### 3.1 Bỏ lưới của predictor — HOÃN, làm riêng sau

`InferenceSchedule.ready()` có sẵn nhánh:

```python
if self.period == 0.0:
    return True   # Historical camera profile: every input sample.
```

Đặt `trajectory_predictor.inference_rate_hz: 0.0` sẽ khiến predictor chạy theo
mọi mẫu của tracker, xóa một lưới và xóa luôn chế độ rơi mẫu do dung sai 5 ms.

**Nhưng không làm trong đợt này.** Hai lý do:

1. Khóa pha ở mục 3.2 bám theo `self._prediction_time`, tức thời điểm prediction
   **thực sự tới controller**. Nó tự xử lý bất kỳ pha nào ở thượng nguồn, nên
   3.1 không cần thiết để đạt mục tiêu tất định.
2. Gộp hai thay đổi vào một đợt sẽ làm nhiễu biến, đúng cái bẫy đã khiến so
   sánh deadband hôm nay trở nên vô giá trị.

Thêm một rủi ro cần cân nhắc trước khi làm 3.1: rate gate cũng đang là lưới
chặn quá tải. Nếu ai đó đặt `ee_position_tracker.publish_rate_hz: 0` (phát theo
mọi `/joint_states`, 50–100 Hz) thì inference sẽ chạy ở 100 Hz. Khi làm 3.1
phải chặn trường hợp đó.

3.1 chỉ nên làm sau khi 3.2 đã được nghiệm thu, và đo riêng bằng tỷ lệ tick
trùng lặp prediction.

### 3.2 Khóa pha timer điều khiển — nội dung của đợt này

Căn pha giữa chuỗi tracker→predictor và timer của controller, một lần, tại
`PREPARING`.

Chọn `PREPARING` chứ không phải `RUNNING` vì ở trạng thái đó **robot chưa
chuyển động**, nên việc tạo lại timer không rơi vào giữa một quỹ đạo đang chạy.

Cơ chế:

1. Thêm hai tham số: `prediction_phase_align_sec` (mặc định `0.0333`) và
   `prediction_phase_align_enabled` (mặc định `true`, đặt `false` để đối chứng).
2. Trong `PREPARING`, khi đã nhận được ít nhất một prediction và chưa căn pha
   cho lượt này, tính thời điểm tick mong muốn:
   `t_target = t_prediction_gần_nhất + offset + k/rate`, với `k` nhỏ nhất sao
   cho `t_target` nằm trong tương lai.
3. Tạo một timer một lần với chu kỳ bằng `t_target − now`. Trong callback của
   nó: hủy chính nó, tạo lại timer điều khiển chu kỳ `1/rate`, rồi gọi
   `_control_tick()` ngay.
4. Ghi log pha đạt được. Không cần thêm công cụ đo: `prediction_age_ms` đã có
   sẵn trong CSV, nên kiểm chứng là tự động.

### 3.3 Ràng buộc an toàn

- Chỉ căn pha **một lần cho mỗi lượt**, trong `PREPARING`, khi robot đứng yên.
- Nếu chưa có prediction nào, hoặc `_mode != 'prediction'` (Ground Truth, MJM),
  **không căn pha** — giữ nguyên timer hiện tại.
- Khoảng gián đoạn tối đa của `_control_tick` là **dưới một chu kỳ (66,7 ms)**.
  Watchdog lực dùng ngưỡng 0,20 s giữ và 0,50 s fault, watchdog pose tương tự,
  nên bỏ lỡ nhiều nhất một tick nằm trong dung sai. Dù vậy, vì `_control_tick`
  là nơi chạy toàn bộ watchdog, việc này phải được nêu rõ và không được mở rộng
  sang trạng thái `RUNNING`.
- Không bao giờ để node ở trạng thái không có timer điều khiển: timer mới phải
  được tạo trong cùng callback đã hủy timer cũ, và bọc bằng `try/finally`.
- Không đổi `control_rate_hz`, giới hạn vận tốc/gia tốc, workspace, joint
  limits, force limits hay tracking threshold.

## 4. Kiểm thử

**Unit** — trong `test_prediction_reference.py` hoặc file mới:

- Hàm tính `t_target` trả về đúng pha mong muốn với nhiều `t_prediction` và
  `now` khác nhau, luôn cho kết quả trong tương lai và trong một chu kỳ.
- Không căn pha khi chưa có prediction, khi ở Ground Truth/MJM, hoặc khi đã
  căn rồi trong cùng lượt.
- Trạng thái sau khi căn pha luôn có đúng một timer điều khiển.

**Mô phỏng** — domain 42, fake hardware. Chạy ba lượt, mỗi lượt khởi động lại
launch hoàn toàn, và xác nhận `prediction_age_ms` median hội tụ về 33 ± 5 ms ở
cả ba, thay vì rải trên [0, 67] ms.

**Robot thật** — người vận hành chạy. Tiêu chí nghiệm thu:

1. `prediction_age_ms` median nằm trong 33 ± 5 ms ở **mọi** lượt, bất kể số lần
   khởi động lại launch. Đây là tiêu chí chính và đo được trực tiếp.
2. `rev/s` không còn tương quan với lần launch.
3. Tỷ lệ tick trùng lặp prediction dưới 1%.

Chạy `prediction_phase_align_enabled:=false` để có nhóm đối chứng.

## 5. Trạng thái triển khai — 21/09

Mục 3.2 đã triển khai xong và build thành công; **chưa chạy robot thật**.

| File | Thay đổi |
|------|----------|
| `admittance.py` | Thêm hàm thuần `phase_aligned_delay(sample_time, now, period, offset)`, trả về khoảng chờ trong `(0, period]` |
| `admittance_controller.py` | Timer điều khiển thành thuộc tính thay được; thêm `_should_align_phase`, `_align_control_phase`, `_finish_phase_alignment`, `_swap_control_timer`; gọi trong nhánh `PREPARING` của `_control_tick`; reset cờ tại Start Run |
| `cocarry_admittance_params.yaml` | `prediction_phase_align_enabled: true`, `prediction_phase_align_sec: 0.0333` |
| `test_admittance.py` | 5 test cho `phase_aligned_delay`, gồm bảy pha thực đã quan sát và các đầu vào không hợp lệ |
| `test_phase_alignment.py` (mới) | 12 test cho phần nối dây: điều kiện căn pha, bất biến "luôn đúng một timer sống", thứ tự tạo trước hủy sau, thoát an toàn khi offset sai, và một test ghép toàn chuỗi với đồng hồ giả |

Kiểm chứng đã chạy: **228/228 test đạt**,
`colcon build --symlink-install --packages-select cocarry_admittance_control`
thành công.

Hai bất biến được bảo vệ bằng test vì chúng là chỗ dễ hỏng nhất:

- Timer mới luôn được tạo **trước** khi hủy timer cũ, nên không có khoảnh khắc
  nào node không có tick điều khiển — nếu có, mọi watchdog sẽ đứng.
- Timer vừa nghỉ chỉ bị hủy ở lần swap **kế tiếp**, không bao giờ hủy ngay
  trong callback của chính nó.

## 6. Việc còn để ngỏ

- Kế hoạch này làm trễ **tất định**, không làm nó **nhỏ**. Vẫn còn khoảng
  33 ms trễ trong vòng kín. Nếu sau khi khóa pha mà ripple vẫn còn ở mức khó
  chấp nhận thì bước tiếp theo là giảm độ lợi vòng hoặc chuyển sang tick theo
  sự kiện, chứ không phải chỉnh thêm pha.
- Số phận của phương án A chưa quyết. Nó không sửa được ripple; mặt khác lượt
  `140717` có hiệu suất quỹ đạo median 0,75 và cửa sổ tệ nhất 0,33 — tốt nhất
  trong ngày ở chỉ số thứ hai — nhưng đó là n=1 và chưa đủ kết luận. Cần đánh
  giá riêng sau khi pha đã được khóa, vì khi đó mới so sánh được.
- Chưa xử lý trường hợp `ee_position_tracker` hoặc predictor bị khởi động lại
  giữa phiên mà controller thì không; khi đó pha sẽ lệch lại cho tới lượt
  `Start Run` kế tiếp.
