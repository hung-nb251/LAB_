# Đo tốc độ và chất lượng đọc M310–M315 — 19/09/2026

Nhánh Claude theo phân công tại
[hc_force_t1_work_assignment_runbook_20260919_vi.md](hc_force_t1_work_assignment_runbook_20260919_vi.md).
Nguồn số liệu là lượt development T1 `20260919_t1_local_dev_v1`. Toàn bộ phép
đo là offline từ sidecar; **không mở client đọc register thứ hai**, không
Enable/Start, không đổi cấu hình điều khiển, không deploy controller.

## 1. Kết luận

1. Reader đạt **5,10 Hz** vector hoàn chỉnh. Không có timeout, không lỗi
   service, không khoảng trống >1 s trong suốt 224,8 s của lượt thu.
2. **Không thể đạt 15 Hz bằng reader scalar hiện tại.** Trần lý thuyết khi bỏ
   hết scan gap và bỏ lãng phí timer là khoảng 7,7 Hz. Đạt 15 Hz cần
   ≤11,11 ms/register, trong khi giá trị nhanh nhất quan sát được là
   15,81 ms/register. Batch phía controller là điều kiện bắt buộc.
3. **Sáu kênh lệch nhau trung vị 144,8 ms.** Khi robot chuyển động, độ lệch
   này làm vector torque sai trung vị **3,04 Nm**, P95 **8,16 Nm** theo norm.
   Để so sánh, P0 ngày 19/09 đo chênh lệch baseline giữa các pose gần nhau chỉ
   0,450–0,660 Nm trung vị. Sai số do đọc không đồng thời lớn hơn khoảng năm
   lần nhiễu baseline mà P0 đang tìm cách mô hình hóa.
4. **Reader dừng vĩnh viễn sau một response bị mất.** Đây là nguyên nhân file
   baseline cuối `..._180506.csv.calibration.jsonl` không có mẫu M310 nào.
5. Register vẫn đổi giá trị ở 99,8% số scan liên tiếp, nên 5 Hz đang **lấy mẫu
   thiếu** so với tốc độ cập nhật thật của controller, không phải lấy dư.

## 2. Ngân sách thời gian đo được

| Thành phần | Trung vị | P95 | Tỷ lệ chu kỳ |
|---|---:|---:|---:|
| Chu kỳ một vector | 195,40 ms | 225,58 ms | 100% |
| `register_scan_span_ms` (6 lần gọi) | 144,84 ms | 174,82 ms | 74% |
| `scan_gap_sec` | 50,00 ms | — | 26% |
| Một register (round trip) | 24,14 ms | 29,14 ms | — |

Round trip 24,14 ms gồm thời gian service (P05 là 19,16 ms, phù hợp con số
~20 ms đã ghi trong các báo cáo trước) cộng phần chờ timer. Timer chạy 0,005 s;
code cũ xử lý xong một response rồi `return`, nên mỗi register mất thêm tới một
chu kỳ timer, tức khoảng 15 ms lãng phí mỗi scan.

| Cấu hình | Trần tốc độ |
|---|---:|
| Như đang chạy | 5,12 Hz |
| `scan_gap_sec=0` | 6,90 Hz |
| `scan_gap_sec=0` + gửi ngay trong cùng tick | 7,70 Hz |
| **Mục tiêu** | **15,00 Hz** |

Các trần trên suy từ span đã đo, là ước lượng chứ chưa phải benchmark chạy
thật với `scan_gap=0`. Benchmark đó phải làm khi robot Stop/Disable.

## 3. Chi phí của việc đọc không đồng thời

Đoạn RUNNING, 615 mẫu. `|dτ/dt|` đo giữa hai scan liên tiếp, rồi nhân với span
144,8 ms để ra độ lệch tích lũy giữa kênh đọc đầu và kênh đọc cuối:

| Kênh | Trung vị | P95 | Lệch trung vị | Lệch P95 |
|---|---:|---:|---:|---:|
| M310 | 5,01 Nm/s | 27,33 Nm/s | 0,752 Nm | 4,100 Nm |
| M311 | 9,70 Nm/s | 44,88 Nm/s | 1,455 Nm | 6,732 Nm |
| M312 | 5,25 Nm/s | 28,58 Nm/s | 0,788 Nm | 4,287 Nm |
| M313 | 0,93 Nm/s | 9,62 Nm/s | 0,139 Nm | 1,443 Nm |
| M314 | 0,50 Nm/s | 2,93 Nm/s | 0,075 Nm | 0,440 Nm |
| M315 | 0,50 Nm/s | 6,43 Nm/s | 0,075 Nm | 0,964 Nm |
| Norm vector | 20,27 Nm/s | 54,42 Nm/s | **3,041 Nm** | **8,163 Nm** |

Đây là ước lượng bậc nhất từ chính chuỗi bị lấy mẫu thiếu, nên là **cận dưới**:
chuyển động nhanh hơn giữa hai scan không quan sát được. Phép tính này chưa
chuyển sang sai số lực qua Jacobian; việc đó thuộc phần của Codex.

Hệ quả cần Codex cân nhắc: một "vector" M310–M315 khi robot chuyển động không
phải một trạng thái nhất quán. Nó có thể là một phần lý do RMSE ở các lượt
động/runtime là 6–7 N trong khi bài tĩnh chỉ 2–3 N. Đây là giả thuyết được số
liệu gợi ý, **chưa được kiểm chứng**.

Kiểm tra lấy mẫu thiếu: chỉ 1/614 cặp vector liên tiếp trùng nhau hoàn toàn.
Giá trị là bội số của 0,1 Nm và bước nhảy giữa hai scan lên tới 0,7 Nm trên
M310, nên controller cập nhật nhanh hơn nhịp đọc.

## 4. Reader dừng vĩnh viễn — nguyên nhân thiếu baseline cuối

Trong bản cũ, khi một request quá `request_timeout_sec`, `_tick` chỉ ghi log
rồi `return`, giữ nguyên `self._pending`. Không có đường hủy request, không
gửi lại, và **không publish mẫu nào**. Nếu future không bao giờ hoàn tất,
node sống nhưng câm vĩnh viễn.

Bằng chứng tại thời điểm audit:

- `..._180049.csv.calibration.jsonl`: 1.147 mẫu M310, mẫu cuối lúc t+224,80 s.
- `..._180506.csv.calibration.jsonl`: **0 mẫu M310** trên 30,3 s, dù có đủ
  3.029 `axia_raw` và 1.919 `joints`.
- Tiến trình `mregister_force_node.py` (pid 53218) vẫn chạy sau đó 13 phút.
- `/sensorless_force/status` và `/sensorless_force/sample` không phát gì trong
  5 s chờ, trong khi `/read_mregister` vẫn được quảng bá, `/joint_states` vẫn
  chạy và `/axia/connected` vẫn true.

Nên trạng thái này là reader kẹt, không phải mất kết nối controller. Sự cố
dừng robot ở cuối lượt là dịp làm lộ ra lỗi, chưa xác định được cơ chế chính
xác làm mất response.

Hệ quả vận hành: một lần mất response duy nhất làm mất toàn bộ M310 của phần
còn lại trong buổi, **mà không có dấu hiệu nào trong log**.

## 5. Candidate đã sửa

File: `src/hc10dtp_bringup/scripts/mregister_force_node.py`. Chỉ sửa phần
acquisition. Không đụng phép đổi đơn vị `(raw − 10000) × 0,1`, estimator,
baseline, Jacobian, deadband hay interface cũ.

1. **Khôi phục sau timeout.** Quá `request_timeout_sec` thì publish mẫu
   `INVALID:mregister_request_timeout` để trạng thái kẹt hiện trong log. Quá
   thêm `request_recovery_sec` (mặc định 2,0 s) thì gọi
   `Client.remove_pending_request()` để **bỏ hẳn request chưa trả lời** rồi mới
   bắt đầu scan mới từ M310. Không bao giờ có hai request cùng lúc.
2. **Bỏ lãng phí timer.** Sau khi xử lý xong một response, request kế tiếp được
   gửi ngay trong cùng tick. Ước tính tiết kiệm ~15 ms mỗi scan.
3. **Tham số chỉnh được lúc chạy.** Thêm `add_on_set_parameters_callback` cho
   `scan_gap_sec`, `request_timeout_sec`, `request_recovery_sec`. Trước đây các
   giá trị này chỉ nạp lúc init nên `ros2 param set` không có tác dụng thật.
   Các tham số khác bị từ chối. Giá trị không hữu hạn hoặc âm bị từ chối.
4. **Mở rộng chẩn đoán.** Thêm khối `acquisition` vào sample, giữ nguyên toàn
   bộ trường cũ và `schema: 1`:

   ```text
   acquisition.registers[]  address, raw (integer), value_nm,
                            request_ns, response_ns, rtt_ms,
                            request_monotonic, response_monotonic
   acquisition              scan_id, scan_first_ns, scan_end_ns, span_ms,
                            scan_gap_target_ms, request_timeout_ms,
                            restarts_this_scan, errors_cumulative,
                            joint_age_ms, reader_backend, reader_version
   ```

   `errors_cumulative` đếm `service_error`, `not_success`, `request_timeout`,
   `abandoned_request`.

Kiểm chứng: `py_compile` đạt; 40 test của `hc10dtp_bringup` đạt, gồm 9 test
mới tại `src/hc10dtp_bringup/test/test_mregister_acquisition.py`; 88 test của
`cocarry_admittance_control` đạt sau khi source môi trường ROS. Test dùng
service giả theo đúng kiểu AST + mock của các test sẵn có, không tạo ROS graph
nên không chạm domain 10.

**Chưa chạy trên robot thật. Chưa build lại. Chưa đo được mức cải thiện thật.**
Con số 7,7 Hz là ước lượng, không phải kết quả đo.

### Triển khai và hoàn nguyên

`install/` dùng symlink nên file source chính là file được chạy; thay đổi chỉ
có hiệu lực sau khi relaunch Terminal 2. Relaunch T2 cũng khởi động lại Axia
UI, nên **phải calibrate lại bias không tải** theo CODEX.md mục 5.

```bash
# Hoàn nguyên bản gốc (file này chưa được track trong git)
cp cocarry_logs/hc_force_calibration/20260919_m310_reader_audit_v1/\
baseline_source/mregister_force_node.py.orig \
   src/hc10dtp_bringup/scripts/mregister_force_node.py

# Tắt riêng phần khôi phục mà vẫn giữ các sửa khác
ros2 param set /sensorless_force_node request_recovery_sec 86400.0
```

SHA256 bản gốc và của YAML/srv liên quan:
`cocarry_logs/hc_force_calibration/20260919_m310_reader_audit_v1/baseline_source/sha256.txt`.

## 6. Source server `/read_mregister`

`src/motoros2_interfaces` là repo **upstream** `Yaskawa-Global/motoros2_interfaces`
phiên bản `0.2.0-3-gff22d75`, không phải phần mở rộng cục bộ. Header SPDX ghi
Yaskawa America và TU Delft, Apache-2.0. Điều này **đính chính** giả định trong
`cocarry_progress_next_steps_20260918_vi.md` rằng service nhiều khả năng là bản
mở rộng tự viết.

Hệ quả:

- Server là ứng dụng MotoPlus của MotoROS2 chạy trên controller YRC. **Không có
  source trong workspace.** Source ở `Yaskawa-Global/motoros2`, build bằng
  MotoPlus SDK và phải cài lên controller.
- Trong `motoros2_interfaces` 0.2.0 không có service đọc nhiều M register.
  `ReadMRegister` là scalar; `ReadGroupIO`/`ReadSingleIO` là IO, không thay thế
  được.
- `/joint_states` có trường `effort` nhưng toàn số 0, nên không phải đường
  thay thế. CODEX.md cũng cấm thay lực robot bằng joint effort.

Vì vậy mục tiêu 15 Hz **không thực hiện được chỉ ở phía host**. Hai hướng:

1. Thêm service/topic batch sáu register trong MotoROS2 và cài lên controller.
   Cần SDK, license và quy trình cài firmware. Ngoài phạm vi được phép của
   nhánh này; cần người dùng quyết định.
2. Nếu không sửa được controller, chốt lại mục tiêu: giữ scalar, bỏ scan gap và
   lãng phí timer để lên khoảng 7–7,7 Hz, đồng thời **ghi timestamp từng
   register** để Codex bù lệch thời gian trong model thay vì coi sáu kênh là
   đồng thời. Khối `acquisition` ở mục 5 đã cung cấp dữ liệu cho hướng này.

Một response batch, nếu có, vẫn không tự chứng minh sáu kênh được lấy mẫu cùng
lúc trong controller. Phải hỏi rõ quy ước timestamp trước khi tin.

## 7. Việc chưa làm

- Benchmark thật với `scan_gap_sec=0` khi robot Stop/Disable. Cần người vận
  hành cho phép một pha dừng riêng; chưa thực hiện.
- Đo tải hệ thống và jitter của controller ở nhịp cao hơn.
- Đối chiếu với maintainer MotoROS2 về API batch.
- Chưa có số đo nào cho candidate mới trên phần cứng thật.

## 8. Tái lập

```bash
cd /home/hungnb/cocarry_ws
OPENBLAS_NUM_THREADS=1 python3 scripts/benchmark_m310_reader.py \
  --output <thu_muc_moi> <duong_dan>/*.calibration.jsonl
```

Script từ chối thư mục output đã tồn tại. Kết quả của lần chạy này:
`cocarry_logs/hc_force_calibration/20260919_m310_reader_audit_v1/benchmark_v1/results.json`.
