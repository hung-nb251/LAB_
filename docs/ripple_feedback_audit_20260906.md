# Ripple GRU simulation — 2026-09-06

Nguồn: `cocarry_logs/simulation/cocarry_admittance_sim_3d_20260906_105435.csv`
(602 mẫu, khoảng 40.07 s) và ảnh
`/home/hungnb/Pictures/Screenshots/Screenshot from 2026-09-06 10-58-00.png`.
Đã đối chiếu log node phiên 10:52:44, không dùng log robot thật hôm trước.

## Kết luận chẩn đoán

- Trial dùng GRU TFLite, raw output, history 20 mẫu/6 feature delta_position,
  stationary HOLD tắt, role OFF (không MJM).
- Inference median/p95/max = 0.470/0.728/1.371 ms; prediction age
  median/p95/max = 24.287/24.961/25.446 ms.
- Không có force-stale HOLD trong trial. Force age tối đa 125.0 ms,
  UDP gap tối đa được CSV lấy mẫu 141.73 ms; log Axia không ghi gap >=150 ms
  trong chính khoảng trial. Không loại trừ mọi jitter, nhưng không có dấu hiệu
  mạng gây chuỗi HOLD/resume trong lần này.
- Stream tick/send/ACK đều khoảng 15 Hz, BUSY/retry/reject/IK-fail bằng 0 trong
  các cửa sổ runtime. Đây là mock, không đánh giá đường LAN robot thật.
- Ripple có trong actual EE, nominal và reference, không chỉ ảnh/UI. Trong
  đoạn 22–31 s, X liên tục đổi chiều trong khi lực X biến thiên chậm hơn nhiều.

Vòng feedback `EE -> GRU dự đoán tương lai -> nominal tức thời -> Admittance +
lead limiting -> streamer -> EE` có thể tự duy trì dao động. Giới hạn nominal
50 mm và reference 30 mm chỉ là giới hạn khoảng cách, không kiểm soát băng
thông/độ biến thiên của nominal. Khi chạm command lead, anti-windup ghi lại
`e=reference-nominal`; do đó nominal thay đổi cũng làm trạng thái `e` thay đổi.
Không thể coi độ mềm K=5 của riêng khối Admittance là bảo đảm ổn định của cả vòng.

Đã tái hiện bằng replay offline với GRU thật, force CSV, Admittance và smoother;
bỏ mạng và bỏ IK vẫn còn ripple. Thay GRU bằng ngoại suy vận tốc đơn giản cũng
tạo dao động trong thử nghiệm đó. Đây là bằng chứng về ghép feedback, không
phải bằng chứng GRU đã train sai/OOD hay conversion TFLite bị lỗi.
Joint clipping/ACK-state mismatch có thể góp phần nhưng không cần thiết để
tạo ripple trong phép tái hiện; chưa thay chúng trong đợt này.

## Thay đổi

- Thêm `PredictionReference`: nominal bậc một với
  `alpha = 1-exp(-dt/tau)`, thời gian monotonic và dt tối đa 0.1 s.
- Chỉ launch simulation override `prediction_reference_tau_sec=0.4`. YAML và
  robot thật mặc định 0.0, tức passthrough như trước.
- Controller điều hòa nominal trước khi cộng Admittance, giữ raw GRU và raw
  CSV nguyên trạng. Không thêm filter vào inference worker/predictor.
- Lead limit vẫn áp sau conditioner; khi clamp, cập nhật cả state conditioner
  về nominal đã được chấp nhận để không tích lũy state ẩn ngoài giới hạn.
- Reset tại Start/realign; đóng băng state trong force HOLD. Ground Truth,
  MJM LEADER và các kiểm tra freshness không đi qua conditioner này.
- Giữ K=5, M/D, giới hạn Cartesian 0.15 m/s, 0.50 m/s², R/B/T=0.08 rad/s,
  workspace, force limits/watchdog và HOLD đứng yên vẫn tắt.

0.4 s là tham số thử nghiệm có đánh đổi độ trễ, không phải chứng minh ổn định
cho mọi quỹ đạo/model. Nó thay đổi nominal điều khiển chứ không cải thiện độ
chính xác thống kê của raw prediction. Không tự áp lên robot thật.

## Kiểm thử và đối chứng

67 pytest đạt; py_compile và colcon build package co-carry thành công.
Test mới gồm passthrough, hội tụ, jitter/repeated tick, stall cap, reset,
nominal clamp/state, bypass Ground Truth/MJM, prediction timeout và force HOLD.

Replay toàn bộ ROS trên localhost/domain 77: fake HC10DTP + mock MotoROS2,
worker GRU thật, chuỗi lực XYZ từ CSV được nội suy phát 100 Hz; không mở Axia
UI, không micro-ROS/robot thật. Khởi tạo cùng home, không hybrid; logger bật
ở lúc bắt đầu phát force replay. Cấu hình trước sửa dùng tau=0; cấu hình sau
dùng tau=0.4. Có sai khác phase DDS/JTC và việc force được giữ cố định thay vì
người điều chỉnh theo robot: không phải tái dựng chính xác từng tick trial gốc.

Đồ thị đối chứng: [Actual EE trước/sau](ripple_feedback_replay_20260906.png).
Quỹ đạo tổng thể khác nhau vì prediction có feedback; không dùng chênh lệch
vị trí giữa hai lượt để kết luận tracking/model accuracy tốt hơn.

Script/CSV/log kiểm tra tạm ở `/tmp/cocarry_ripple_meZlFc/` (có thể mất khi
reboot). Các launch mock đã được Stop/Disable và kết thúc. Shutdown bằng SIGINT
có traceback rclpy/KeyboardInterrupt như lần kiểm tra trước, không phải fault
trong trial; chưa sửa phần shutdown.

Kết quả đoạn 22–31 s (actual được nội suy 15 Hz; đếm đổi dấu vận tốc X với
ngưỡng bỏ jitter 0.01 m/s; gia tốc là sai phân vị trí, không phải cảm biến):

| Nguồn | Đổi chiều X | Gia tốc X RMS (m/s²) |
|---|---:|---:|
| Trial gốc 10:54:35 | 11 | 0.321 |
| ROS replay tau=0, CSV 11:10:44 | 11 | 0.415 |
| ROS replay bản cuối tau=0.4, CSV 11:14:14 | 1 | 0.134 |

Hai lượt đối chứng đều 601 mẫu, khoảng 40 s. Gia tốc X RMS giảm khoảng 68%
so với replay tau=0. Biên độ di chuyển X toàn lượt gần như giữ nguyên
(0.546 m so với 0.546 m); không đạt kết quả bằng việc giữ robot đứng yên.
Không thấy force-stale HOLD hay safety fault trong hai lượt. Đây là đối chứng
cho một chuỗi lực cụ thể, không phải chứng nhận độ mượt/ổn định robot thật.

## Chạy lại

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cocarry_admittance_control cocarry_admittance_sim_gui.launch.py \
  prediction_reference_tau_sec:=0.4
```

Đối chứng: dừng launch rồi thay bằng `prediction_reference_tau_sec:=0.0`.
Không chạy đồng thời hai launch cùng domain. Cần người vận hành xác nhận lại
cảm giác đáp ứng với Axia thật trước khi cân nhắc chuyển tham số sang robot thật.
