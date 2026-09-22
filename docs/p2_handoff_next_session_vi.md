# Bàn giao P2 — trạng thái sau A/B ngày 19/09/2026

Đọc theo thứ tự:

1. `CODEX.md`, đặc biệt mục cập nhật P2 ở đầu file.
2. `docs/p2_pipeline_bandwidth_audit_20260919_vi.md`, nhất là mục 5e.
3. File này.

## Kết luận A/B mới nhất

Lượt `cocarry_admittance_3d_20260919_125048.csv` chạy đúng
`command_lead_m=0.04`, `prebuffer=2`. So với hai lượt baseline `0.04/3`, tốc
độ lệnh trung vị vẫn khoảng 0,095 m/s và trễ reference→EE vẫn khoảng 0,40 s.
Queue lag không gian chỉ giảm khoảng 1 mm; BUSY/retry/reject và IK fail đều 0.

Lượt `0.05/2` trước đó chỉ tăng nhẹ tốc độ nhưng đưa tracking error P95 lên
39,0 mm, max 44,6 mm so với ngưỡng dừng 50 mm. Không giữ cấu hình đó.

Quyết định hiện tại:

- `command_lead_m=0.04`;
- `prebuffer=3`;
- không tăng velocity/acceleration/jerk hoặc tracking threshold;
- không dùng prebuffer hay command lead làm đòn bẩy tốc độ nữa.

Artifact lượt A/B cuối:
`cocarry_logs/20260919_p2_lead004_prebuffer2_v1/`.

## Candidate runtime đang chờ kiểm chứng robot

P2 chỉ ra hai nguyên nhân làm jerk limiter mất hiệu lực trong co-carry:

1. continuous mode hoạt động phần lớn dưới `dist<20 mm`, trong khi code cũ tắt
   jerk limiter ở toàn bộ vùng này;
2. ACK synchronized ghi thẳng vận tốc FK và sai phân bậc hai thô vào trạng thái
   velocity/acceleration của smoother.

Source hiện tại đã sửa:

- continuous mode giữ jerk bound cả dưới 20 mm, chỉ reset khi thực sự tới đích;
- feedback ACK được reconcile qua velocity/acceleration/jerk bounds;
- camera/demo non-continuous giữ hành vi lịch sử;
- diagnostics ghi `accepted_ee_velocity`, `accepted_ee_acceleration_raw` và
  `accepted_state_reconciled`.

Các file chính:

- `src/hc10dtp_bringup/scripts/cartesian_streamer_hc10dtp.py`
- `src/hc10dtp_bringup/scripts/motion_conditioning.py`
- `src/cocarry_admittance_control/launch/cocarry_admittance_real_gui.launch.py`

Đã build `hc10dtp_bringup` và `cocarry_admittance_control`. Test offline đạt
50, skip 1 test tích hợp phụ thuộc môi trường. Agent chưa chạy robot thật.

## Lượt robot tiếp theo

Người vận hành phải ghi speed override trên pendant. Chạy ngắn cùng route GRU,
giữ mọi tham số khác. Mục tiêu là kiểm chứng độ mượt/jerk, không tuyên bố tăng
tốc. Dừng nếu có dao động/overshoot, cảm giác mất mượt, tracking error P95 vượt
40 mm, bất kỳ BUSY/retry/reject, IK fail, joint clamp đáng kể hoặc safety stop.

Sau khi có log, chạy audit vào thư mục output mới rồi so với baseline schema 2.
Nếu candidate không giảm jerk hoặc tạo overshoot, hoàn nguyên riêng thay đổi
smoother; không nới giới hạn để bù.

`F_robot` vẫn shadow và không liên quan tới quyết định P2 này.
