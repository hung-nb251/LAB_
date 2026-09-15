# Phối hợp khớp, tracking và bù trễ reference — 15/09/2026

Đã triển khai theo yêu cầu tiếp nối profile tốc độ
`[0.50, 0.50, 0.50, 0.08, 0.50, 0.40] rad/s` trên robot thật.
Trial gần nhất `cocarry_admittance_3d_20260915_100026.csv` và log streamer
`python3_14603_1789441162302.log` xác nhận profile này đã chạy;
các cửa sổ runtime ghi 15 Hz, IK fail=0.

## Thay đổi

- Khi một khớp chạm trần vận tốc, tất cả khớp cùng tiến theo một tỷ lệ
  `s=min(1, min_i(vmax_i*dt/abs(delta_q_i)))`.
  Cách cũ cắt riêng từng vận tốc, làm đổi tỷ lệ chuyển động giữa các khớp.
  Tỷ lệ mới giữ hướng của bước trong joint space; không bảo đảm đường thẳng
  Cartesian tuyệt đối hay tự tăng tốc khi không có khớp nào bão hòa.
- Sau ACK, cập nhật vận tốc/gia tốc Cartesian nội bộ từ FK của chuyển động
  đã được chấp nhận và thời gian giữa các điểm. Bộ smoothing không tiếp tục
  tích phân từ vận tốc yêu cầu lớn hơn vận tốc đã gửi được.
- Ở chế độ synchronized, tạm dừng tiến smoother/IK khi chờ ACK và gửi lại
  nguyên điểm pending trước khi sinh điểm mới. ACK cập nhật hết trạng thái
  rồi mới nhả cờ inflight. Chờ ACK quá 0.5 s thì fail-closed.
- Sửa lỗi BUSY: timestamp điểm retry đã được giữ nguyên thì đồng hồ tích lũy
  cũng phải giữ nguyên. Trước đây callback trừ một chu kỳ khỏi đồng hồ,
  khiến điểm sau retry có thể trùng/lùi timestamp. Đây là lỗi tìm trong code;
  không quy lỗi tracking sáng nay cho BUSY vì cửa sổ log đó không có BUSY.
- Tracking synchronized nội suy tuyến tính XYZ giữa hai FK endpoint queue
  đã nhận, thay vì giữ bậc thang endpoint trước. Không ngoại suy sau điểm cuối,
  không dịch epoch để bám theo feedback, giữ ngưỡng 50 mm/5 mẫu hiện tại.
- Giữ bộ lọc nominal tau=0.4 s và bù tiến theo vận tốc đã lọc với horizon
  0.15 s, tối đa 20 mm. Vận tốc được ước lượng từ bước của vị trí đã lọc,
  qua thêm lọc tau=0.15 s. Bù không đi quá desired trên từng trục; reset khi
  Start/realign/HOLD, không mang vận tốc cũ qua thời gian gián đoạn >0.1 s.
  Raw prediction không thay đổi. Các giới hạn command lead, v/a và workspace
  phía sau vẫn được áp dụng.

Phối hợp/tracking áp dụng ở streamer cho Ground Truth, GRU/SVGP, Hybrid và
GRU+MJM Test. Bù nominal chỉ tác động nhánh prediction/FOLLOWER;
Ground Truth và MJM LEADER vẫn bypass bộ lọc nominal.
Hai launch co-carry bật thuật toán mới; simulation giữ profile tốc độ riêng.
Launch camera không bật synchronized hoặc bù nominal mới.

## Chạy và đối chứng

Sau khi kết thúc và đóng phiên launch cũ, mở terminal tại workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch cocarry_admittance_control cocarry_admittance_real_gui.launch.py
```

Mặc định hiện tại: `joint_coordination:=synchronized`,
`prediction_reference_lead_sec:=0.15`, `prediction_reference_tau_sec:=0.40`.
Launch robot thật dùng Cartesian `vmax=0.25 m/s`, `amax=1.00 m/s²`,
`jerk=10 m/s³`; `reference_tau=0.40 s`, `command_lead=0.04 m`.
Launch không tự Enable/Start; người vận hành dùng UI như trước.

So sánh riêng tác động bù trễ (giữ phối hợp mới, tắt bù trễ):

```bash
ros2 launch cocarry_admittance_control cocarry_admittance_real_gui.launch.py \
  prediction_reference_lead_sec:=0.0
```

Đối chứng cách cắt vận tốc và tracking bậc thang trước đây:

```bash
ros2 launch cocarry_admittance_control cocarry_admittance_real_gui.launch.py \
  joint_coordination:=independent prediction_reference_lead_sec:=0.0
```

Đối chứng này vẫn chứa sửa lỗi timestamp BUSY và telemetry mới;
không hoàn nguyên các sửa lỗi đó. Mỗi lần đổi cấu hình phải kết thúc phiên
launch trước, không chạy hai publisher điều khiển robot cùng lúc.

Kịch bản so sánh: cùng pose xuất phát, kéo X+ rồi X− ở vùng giữa, lên/xuống Z,
giữ yên, sau đó tới gần Target 2 và quay về. Hybrid thử thêm
FOLLOWER→LEADER→FOLLOWER. Đánh giá thời gian di chuyển, cảm giác đổi chiều,
dao động lúc dừng, tỷ lệ bão hòa khớp và tracking error.

## Dữ liệu chẩn đoán mới

Topic `/cartesian_streamer/motion_diagnostics` (String JSON), khoảng 15 Hz,
được logger ghép vào cột cuối `motion_diagnostics_json` của CSV mỗi trial:

- `joint_scale`: 1 là không bị giới hạn khớp, <1 là giới hạn tiến đồng bộ.
- Vận tốc yêu cầu/đã gửi, trần vận tốc, joint yêu cầu/đã gửi/ACK.
- `send_monotonic_ns`, `ack_monotonic_ns`, `due_monotonic_ns`, epoch và thời
  gian điểm để kiểm tra độ trễ queue. Timestamp monotonic chỉ so sánh trên PC đó.
- `actual_xyz`, `expected_xyz`, `tracking_error_m` và timestamp riêng cho
  mẫu tracking. Khi chưa đủ điều kiện watchdog, trường tracking có thể vắng
  hoặc mang timestamp cũ; không diễn giải như một mẫu mới ở mỗi hàng CSV.
- XYZ yêu cầu IK, seed, cận IK, số lỗi IK liên tiếp. Khi fault, terminal ghi
  thêm snapshot diagnostics để không mất nguyên nhân cuối nếu CSV dừng trước.

Đây là snapshot các giai đoạn, không phải tất cả dữ liệu đo đồng thời.
Cần dùng timestamp từng giai đoạn khi đối chiếu.

## Kiểm chứng và giới hạn kết luận

Test offline gồm giới hạn tiến đồng bộ qua đảo chiều ngẫu nhiên, FIFO tracking,
không dùng future trước anchor/không ngoại suy, chuỗi production gửi→BUSY→retry→ACK
với FK thật và RPC giả, reset, bù trễ trên ramp, nhiễu tĩnh, thời gian lặp/gián
đoạn, cùng regression controller/Hybrid/IK: 109 tests đạt. Hai package ROS
`hc10dtp_bringup` và `cocarry_admittance_control` build thành công; kiểm tra
`--show-args` của launch đã cài xác nhận hai mặc định mới được nạp đúng.

Trên ramp tổng hợp 0.1 m/s ở 15 Hz: lag vị trí bộ lọc cũ khoảng 36.8 mm,
có bù khoảng 21.8 mm (giảm 15 mm). Đây là phép thử bộ lọc độc lập,
không phải dự báo tốc độ hoặc độ trễ vòng kín robot-EE→GRU→admittance.

Nội suy tracking là xấp xỉ giữa FK endpoint; chưa biết chính xác thời điểm
controller bắt đầu thực thi và spline nội bộ. Vì vậy không khẳng định đã loại
bỏ mọi nguyên nhân fault 54 mm. Không nới ngưỡng tracking để đạt test.

Giới hạn gia tốc/jerk hiện có vẫn nằm ở bộ smoothing Cartesian. Phép tiến
đồng bộ mới bảo đảm trần vận tốc khớp, chưa phải bộ sinh quỹ đạo có bảo đảm
gia tốc/jerk từng khớp. Mức độ êm và tốc độ thực cần được xác nhận bằng trial mới.

Sau đó người dùng yêu cầu áp dụng trực tiếp cho robot thật: source và symlink
install dùng upper J2=J3=1.50 rad; margin 3° giữ nên cận IK hữu hiệu khoảng
1.4476 rad. Đây là cấu hình thử nghiệm mới, chưa mở lower limits, workspace,
URDF hay Axia. Hãy theo dõi nghiệm IK, khoảng cách tới cận, tracking và
clearance trong lần chạy đầu.
