# P1 — Benchmark bộ lọc Axia ngày 19/09/2026

Đã hoàn tất **P1 offline** và tích hợp candidate vào `axia_sensor_ui.py`.
Runtime hiện chạy candidate trước gravity compensation; topic
`/axia/raw_wrench` vẫn là packet raw, còn `/axia/human_force` nhận output đã
lọc như trước. Chưa dùng candidate cho force safety, F_robot hoặc role selection.

Candidate được chọn theo validation 17/09 là `median3_adaptive`: median ba
mẫu rồi low-pass thích nghi cutoff 1,5–15 Hz. Cả ba trục dùng chung alpha.
Lựa chọn này ưu tiên noise thấp và phản ứng nhanh hơn preset EMA mạnh;
không phải cải thiện đồng thời mọi chỉ số so với preset Raw hiện tại.

## So sánh chính

Dữ liệu giữ lại ngày 18/09, không dùng để chọn preset:

| Chỉ số | Raw thật trước lọc | Raw UI: median-5 | Median-5 + EMA 0,1 | Candidate median-3 adaptive |
|---|---:|---:|---:|---:|
| Median RMS vector khi nghỉ | 0,02864 N | 0,01717 N | 0,00943 N | 0,00937 N |
| Median P95 norm khi nghỉ | 0,04754 N | 0,02770 N | 0,01622 N | 0,01589 N |
| Noise/raw, median tỷ lệ theo window | 1,000 | 0,579 | 0,325 | 0,317 |
| t90 trung vị, bước lực tổng hợp | 0 ms | 20 ms | 230 ms | 40 ms |
| Rise 10–90%, bước lực tổng hợp | 0 ms | 0 ms | 200 ms | 30 ms |
| Vượt ngưỡng 4 N, bước 0→8 N | 0 ms | 20 ms | 80 ms | 20 ms |
| Góc sai P95, vector quay tổng hợp 0,5 Hz | 0,27° | 3,75° | 19,47° | 4,28° |

RMS/P95 nghỉ đo trên **raw ghi thật** trong các đoạn không tiếp xúc. Chỉ số
bước lực/hướng quay đo trên **tín hiệu tổng hợp có đáp án**, cộng noise lấy
từ log không tiếp xúc của đúng split. Chúng không phải số đo độ trễ ý định
người hay robot trên phần cứng thật. Độ phân giải thời gian thử là 10 ms.

Candidate giảm RMS nghỉ khoảng **45% so với preset Raw UI**, nhưng thêm
khoảng **20 ms để đạt 90% bước lực**. Với bước 0→8 N, thời điểm vượt 4 N
vẫn khoảng 20 ms. So với EMA 0,1, mức noise tương đương và đáp ứng nhanh hơn
rõ rệt. Mức noise tuyệt đối chỉ vài chục mN trong các log này; không có cơ
sở coi việc giảm nó là lời giải cho sai số F_robot nhiều N ở P0.

## Giới hạn quan trọng: xung kéo dài

P95 tỷ lệ peak đầu ra do xung chèn, so với amplitude xung, đo bằng chênh
lệch giữa hai lần replay cùng tín hiệu nền có/không xung:

| Độ dài xung tại 100 Hz | Raw UI median-5 | EMA 0,1 | Candidate |
|---|---:|---:|---:|
| 1 mẫu / 10 ms | 2,42% | 0,62% | 2,08% |
| 2 mẫu / 20 ms | 2,01% | 0,60% | 73,49% |
| 3 mẫu / 30 ms | 99,91% | 27,09% | 86,36% |

Candidate tốt với xung một mẫu, nhưng **kém hơn median-5 hiện tại khi xung
kéo dài hai mẫu**. Vì vậy chưa đề xuất thay filter runtime. Không dùng bộ
lọc này làm bằng chứng bảo vệ khỏi va chạm/quá lực: xung lực vật lý ngắn có
thể bị loại như nhiễu, và burst nhiễu có thể bị nhận như thay đổi chủ đích.

Kiểm tra hậu kỳ toàn bộ **782.750 mẫu raw của 44 log nguồn** không tìm thấy
xung đơn theo tiêu chí: lệch centered median-9 trên max(0,5 N, 6σ robust),
hai mẫu lân cận quay về trong 0,15 N, không nằm cạnh khoảng thiếu dữ liệu.
Không kết luận cảm biến không có nhiễu: phép kiểm này không bao phủ burst,
xung nhỏ hơn, dữ liệu ngoài 44 log hoặc biến động sinh ra ở bù trọng lực/TF.
Hiệu quả loại xung hiện được kiểm chứng bằng xung chèn có kiểm soát, chưa
được xác nhận trên sự cố nhiễu xung thực tế đã có nhãn.

## Dữ liệu và protocol

- Dùng packet raw trước software filter trong `/axia/raw_wrench`, không
  suy ngược `/axia/human_force` qua deadband. Hardware tare có thể đã áp dụng.
- 44 log nguồn; trích **74 đoạn liên tục, 105.389 mẫu raw** để benchmark:
  49 đoạn nghỉ và 25 đoạn interaction. Bỏ 2 s warmup mỗi đoạn.
- Đoạn nghỉ kế thừa marker không tiếp xúc, health và stationarity từ P0;
  lấy tối đa 12 s cuối cửa sổ. Interaction lấy tối đa 32 s gồm prefix,
  không đại diện mọi phần của mỗi trial dài.
- Development đến hết 16/09: 18 đoạn nghỉ + 4 interaction.
- Validation 17/09: 17 đoạn nghỉ + 10 interaction.
- Historical holdout 18/09: 14 đoạn nghỉ + 11 interaction.
- Không nội suy force qua khoảng raw thiếu >50 ms. Không dùng mẫu đã ghép
  M310 tần số thấp của P0 để giả lập raw Axia 100 Hz.

So tám preset: raw; median5/alpha1; median5/alpha0,1; median3/alpha0,35;
median3 adaptive; Hampel-confirm adaptive; median3 One Euro; median3
low-pass bậc hai critically damped cutoff -3 dB 6 Hz.

Screen được ghi trước khi chạy benchmark: RMS/raw ≤0,50; P95 residual xung
một mẫu ≤10%; t90 lớn nhất ≤80 ms; sai số hướng P95 ≤10° ở vector quay
0,5 Hz; không bỏ lỡ bước. Chọn t90 trung vị thấp nhất rồi noise ratio thấp
nhất. Đây là tiêu chí sàng lọc kỹ thuật, **không phải ngưỡng an toàn robot**.
Median3 adaptive, Hampel-confirm adaptive và One Euro vượt screen validation.
Hai adaptive có t90 40 ms; median3 adaptive được chọn nhờ noise ratio thấp
hơn một chút trên validation. Không khẳng định sự khác biệt nhỏ này có ý
nghĩa thống kê hay candidate là tối ưu duy nhất.

## Thao tác thật và tách các stage

Đối chứng interaction dùng proxy offline centered median-5 rồi mean-5.
Đây là reference chẩn đoán, không phải ground truth. Không dùng metric
proxy để chọn preset vì nó thiên về bộ lọc giống chính nó.

Trên 11 interaction holdout, median RMSE so với proxy là 0,382 N với Raw
UI, 1,018 N với EMA 0,1 và 0,506 N với candidate. Kết quả phù hợp với đánh
đổi độ trễ; không diễn giải thành sai số tuyệt đối của force đo.

Lọc diễn ra trong sensor frame trước khi bù trọng lực/rotation. Các stage
raw → despiked/median → filtered → base trước deadband → sau radial deadband
4 N được lưu riêng cho các đoạn đại diện. Reference base dùng TF/FK,
payload 1,126 kg và baseline có marker từ P0; không khẳng định khôi phục
đúng UI bias/tare/filter lịch sử vì các settings đó chưa được ghi đầy đủ.

Adaptive ước lượng scatter từ MAD sai phân để tránh coi slope đều là noise.
Innovation lớn hơn nền noise làm tăng cutoff, nhằm bám nhanh khi lực đổi.
Hampel-confirm là biến thể riêng, chấp nhận thay đổi bền sau hai mẫu để
tránh giữ mãi một step. One Euro dùng cutoff phụ thuộc tốc độ theo
[mô tả của tác giả](https://gery.casiez.net/1euro/), với mở rộng vector dùng
norm đạo hàm và alpha chung; không phải ba filter scalar độc lập.

## Bước tiếp theo

Giữ filter điều khiển hiện tại. Candidate đã sẵn sàng cho **kênh shadow**
nhận cùng packet Axia, ghi đầy đủ raw/stage/timestamp/state/reset; không
publish đè `/axia/human_force`. Cần quan sát burst thực tế và xác nhận
giới hạn 2–3 mẫu trước khi cân nhắc thay filter vận hành.

Việc tích hợp chỉ thay stage lọc của UI; chưa chạy robot trong lượt này.
Có thể tiếp tục P2 audit băng thông bằng log hiện có. Nếu sau này đổi
reference Axia, đánh giá lại calibration trên cùng reference trước khi
so sánh RMSE F_robot; không gán cải thiện của filter thành cải thiện model.

## Artifacts và kiểm chứng

- [axia_filter_candidates.py](../scripts/axia_filter_candidates.py): module thuần,
  không được UI/ROS import; preset và logic causal.
- [benchmark_axia_p1.py](../scripts/benchmark_axia_p1.py): trích raw, replay, tạo
  tín hiệu thử, chọn preset và xuất báo cáo.
- [audit_axia_p1_raw_impulses.py](../scripts/audit_axia_p1_raw_impulses.py): kiểm tra
  xung đơn trên toàn raw của các file nguồn.
- Input: `cocarry_logs/hc_force_calibration/20260919_p1_axia_inputs_v1/`.
- [Kết quả benchmark](../cocarry_logs/hc_force_calibration/20260919_p1_axia_benchmark_v1/):
  `summary.csv/json`, metric từng đoạn/case, cấu hình candidate, stage NPZ,
  đồ thị, source snapshot và hashes.
- Audit raw: `cocarry_logs/hc_force_calibration/20260919_p1_raw_impulse_audit_v1/`.

Kiểm chứng: 35 tests đạt (27 filter/benchmark và 8 P0), `py_compile` đạt,
hash source filter/benchmark khớp artifacts và hash `axia_sensor_ui.py`
không đổi. Các test kiểm tra không phụ thuộc mẫu tương lai, tương đương
hai preset UI, reset/NaN/gap, step bền và ổn định với dt thay đổi.

Pytest dùng `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` để tránh plugin ROS cũ
không tương thích pytest hiện tại. SciPy hệ thống vẫn cảnh báo phiên bản
NumPy ngoài dải hỗ trợ khai báo; không đổi môi trường hay package ROS.

```bash
cd /home/hungnb/cocarry_ws
OPENBLAS_NUM_THREADS=1 python3 scripts/benchmark_axia_p1.py \
  --prepare-only --output <input_moi>
OPENBLAS_NUM_THREADS=1 python3 scripts/benchmark_axia_p1.py \
  --prepared <input_moi> --output <ket_qua_moi>
OPENBLAS_NUM_THREADS=1 python3 scripts/audit_axia_p1_raw_impulses.py \
  --prepared <input_moi> --output <audit_raw_moi>
```

Mỗi lệnh yêu cầu thư mục output mới. Không ghi đè log, model calibration
hoặc cấu hình điều khiển hiện có.
