# Kế hoạch F_robot phục vụ conflict/confidence — 17/09/2026

## Phạm vi và đính chính

- Người vận hành xác nhận `new_pose` là vị trí gần Target 2. Giữ tên thư mục
  gốc để truy xuất; xem đây là validation vùng Target 2, không phải điểm giữa
  Home–Target 1 và không khẳng định trùng chính xác saved Target 2.
- Mục tiêu là tín hiệu conflict của outer loop, không đưa lực register vào
  admittance. Báo cáo ba pose trước đó chưa chứng minh calibration cho mục tiêu này.
- Chưa cần yêu cầu đo thêm 3–5 pose. Trước hết phải chốt đúng đại lượng và
  tận dụng dữ liệu đã có. Các gain cục bộ vẫn chỉ là kết quả fit offline.

## Đối chiếu kiến trúc và code

`simulation_hri/outer_loop.py:compute_disagreement` dùng
`dot(f_h, f_r)/(norm(f_h)*norm(f_r))`, không dùng đạo hàm lực.
`config.py` đặt PHI_ANGLE=90 độ; arbitration dùng số mẫu conflict liên tiếp,
confidence liên tiếp, hysteresis và cooldown. Không chuyển vai ngay theo một mẫu.
Mô phỏng còn có quy ước riêng khi lực nhỏ; cần thiết kế lại trạng thái không đủ
tín hiệu trước khi chuyển cơ chế này sang robot thật.

`simulation_hri/inner_loop.py` sinh `f_r = K_p @ e_pos + K_d @ e_vel`;
đó là lực điều khiển trong mô phỏng. CODEX.md ghi Hybrid thật hiện điều khiển
vai thủ công; estimator lực chỉ diagnostic, `calibration_confirmed=false`.

Theo tài liệu register đã ghi trong `hc_force_register_plan_20260912_vi.md`,
M310–315 là estimated external joint torque, M320–325 là estimated external
TCP wrench. Đây không mặc nhiên là lực chủ động của bộ điều khiển.
Fit lực external register vào Axia rồi so cosine giữa hai nguồn có thể chỉ đo
sự nhất quán hai phép đo của cùng lực tiếp xúc, không đo xung đột ý định.
Đổi dấu toàn bộ cũng không giải quyết sự khác biệt về định nghĩa.

## Thứ tự công việc

1. Chốt nguồn F_robot độc lập với phép đo F_ext. Nếu tái hiện mô phỏng, cần
   định nghĩa lực điều khiển tương đương trên robot position-controlled; không
   tự coi output PD ảo là lực thật đã đo. Nếu dùng hướng chuyển động danh định
   để đánh giá ý định thì đó là thay đổi metric, cần thống nhất trước triển khai.
2. Đánh giá offline dữ liệu hiện có tại Home, Target 1 và gần Target 2:
   timestamp từng register, scan hoàn chỉnh, baseline/release, dấu/frame,
   drift, độ trễ và chuyển động thực. Loại phần sau timeout của trial đầu
   new_pose khỏi các cặp register/Axia. Không sửa URDF hoặc mounting RPY.
3. Ưu tiên Axia raw với phép bù/frame có truy xuất; không mặc định phép đảo
   deadband 4 N luôn đúng. Đánh giá cả vector ba chiều và coupling, không chỉ
   gain riêng từng trục. Tách lượt fit và lượt kiểm chứng, thử giữ nguyên một
   pose làm holdout; không coi lỗi fit thấp là khả năng tổng quát hóa.
4. Với mục tiêu cosine: báo cáo sai số góc, tỷ lệ sai dấu, độ trễ, tỷ lệ mất
   mẫu; chỉ đánh giá khi cả hai norm đủ lớn. Scale dương chung triệt tiêu trong
   cosine, nhưng offset, sai frame/dấu, gain lệch trục và trễ không triệt tiêu.
   Dữ liệu lực tĩnh hiện có chưa có nhãn đồng thuận/chống lại ý định robot.
5. Sau khi chốt nguồn lực và đạt kiểm chứng offline, thu ngắn các tình huống
   thuận hướng/chống hướng/thả tay trên tuyến Home–T1–T2. Ghi reference và
   vận tốc danh định, pose/vận tốc thật, lực hai nguồn với timestamp, role,
   confidence thực của classifier và nhãn thao tác. Chạy đánh giá quan sát,
   chưa tự chuyển vai. Không dùng confidence cố định thay cho xác suất GMM.
6. Đo rate/latency thực của đường đọc register; không giả định scan 24 register
   tuần tự đủ nhanh cho arbitration. Chọn nhóm register cần thiết, kiểm tra
   tải service khi streaming và quy định INVALID/STALE; không giữ mẫu cũ như
   lực mới. Chỉ tích hợp tự đổi vai sau khi thống nhất ngưỡng và kiểm chứng.

Không cần suy inertia chỉ để tính cosine. Nếu chọn đường ước lượng lực từ
torque truyền động khi robot chuyển động, yêu cầu mô hình động lực học là
bài toán riêng. Khối lượng 2.350 kg và dữ liệu hiện tại không tự xác định được
toàn bộ inertia hay chứng nhận Tool Data.

Tài liệu này là kế hoạch/đối chiếu code; chưa tạo calibration mới, chưa đổi
cấu hình, chưa điều khiển robot hoặc bật tự chuyển vai.
