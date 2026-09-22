# Kết quả so sánh hai nhánh calibration động — 17/09/2026

**M310–M315 + Jacobian là ứng viên tốt hơn trên Ground Truth; chưa xác nhận calibration hoàn chỉnh trên GRU/GRU+MJM.**

Đã phân tích 6 lượt Ground Truth giữ lại trong báo cáo tiến độ và 4 lượt
GRU/GRU+MJM. Các lượt AI chỉ có M320–M325, không có M310–M315, nên chưa thể
kiểm chứng nhánh torque trên những lượt đó. Không dùng joint_states.effort
thay thế và không tái tạo torque từ M320 để làm kiểm chứng vòng tròn.

Hình học người dùng cung cấp trong phiên này: Axia→flange=(0,−120,0) mm;
Axia→tâm thanh=(−160,0,0) mm, bỏ qua tấm đệm. Phân tích quy moment về
flange bằng flange→Axia=(0,+120,0) mm, diễn giải theo trục đo Axia với
correction yaw hiện có. Không sửa URDF/TF. Tâm thanh chưa phải CoG toàn tải.

## Kết quả kiểm chứng độc lập

Mỗi lượt test được giữ hoàn toàn ngoài tập fit; cùng 6 lượt GT và cùng mẫu
cho cả hai nhánh. Ma trận lực 3×3 được so với ma trận torque 6×6 rồi
khôi phục wrench qua Jacobian. Ridge cố định, không chọn tham số theo test.

**Chỉ tính đoạn EE di chuyển >5 mm/s:**

| Lượt test | Mẫu động | RMSE nhánh lực (N) | RMSE nhánh torque + J (N) |
|---|---:|---:|---:|
| Home X | 71 | 8,04 | 3,89 |
| Home Y | 75 | 8,16 | 3,42 |
| Home Z | 53 | 7,22 | 4,79 |
| Target 1 X | 69 | 4,94 | 3,94 |
| Target 1 Y | 55 | 4,93 | 2,50 |
| Target 1 Z | 15 | 2,43 | 2,51 |
| **Gộp** | **338** | **6,76** | **3,72** |

Nhánh torque giảm khoảng 45% RMSE động, nhưng không tốt hơn ở mọi lượt.
Toàn RUNNING gồm cả nhả lực/đứng yên có RMSE 3,56 N và 2,08 N; không dùng
hai số thấp hơn này thay cho kết quả khi chuyển động.

Trong 132 mẫu Axia ≥4 N, nhánh lực có 53 mẫu đủ norm để tính góc, nhánh
torque có 125 mẫu. Góc trung vị/P95 lần lượt 47,70°/97,33° và
24,12°/46,45°. Hai tập góc khác độ phủ, không diễn giải chúng là cùng tập
mẫu hay độ chính xác phân loại conflict. Các mẫu lực ước lượng <4 N vẫn
được giữ trong RMSE và được báo thiếu thông tin hướng.

Giữ toàn vùng Home hoặc Target 1 ngoài tập học vẫn cho nhánh torque tốt
hơn ở 5/6 lượt, nhưng sai số động còn 2,55–4,93 N. Rotation thuần không
giải quyết được sai khác. Gain/coupling và baseline không thể được thay
bằng một ma trận xoay duy nhất trong các phép thử này.

## GRU/GRU+MJM

Nhánh lực fit chỉ GT chuyển sang 4 lượt AI có RMSE động 7,13 / 5,79 /
5,29 / 4,62 N. Khi đưa các lượt AI khác vào training và thêm q−q0, vẫn
giữ nguyên cả lượt test ngoài tập học, kết quả là 4,26 / 3,59 / 2,99 /
2,88 N. Tuy nhiên, độ phủ góc chỉ 10/31, 46/52, 26/26 và 20/34 mẫu Axia
≥4 N. Chưa đủ cơ sở triển khai chung. Trial GRU+MJM được đánh giá tổng
RUNNING, chưa tách role LEADER/FOLLOWER từ CSV controller.

## Giới hạn và phần thiếu cụ thể

- Axia được bù trọng lực 1,126 kg và bias đầu lượt; moment trừ baseline,
  đổi frame, cộng r×F. Chưa bù đầy đủ quán tính động. Orientation ghi
  trong các lượt gần cố định, biến thiên tối đa khoảng 0,05°.
- Dữ liệu được ghép theo trung điểm request/response từng register;
  nội suy và làm trơn đối xứng chỉ dành cho offline, không tăng băng thông
  hoặc chứng minh estimator online cùng chất lượng. Nhóm all khoảng 1,3 Hz.
- Baseline đầu Axia có scatter ngắn hạn khoảng 0,031–0,042 N trong cửa sổ
  đã chọn; đây không phải độ chính xác tuyệt đối. Baseline sau chuyển động
  vẫn lệch, có lượt gần 1,9 N khi joint đã trở về gần vị trí đầu.
- Quét độ trễ bằng validation lồng nhau chưa tìm được hằng số ổn định.
- Các ma trận là hiệu chỉnh thực nghiệm, chưa phải bộ tham số cảm biến
  vật lý đã được nhận dạng duy nhất. Vẫn cần baseline không tiếp xúc đầu lượt.
- Phần thu bổ sung có giá trị nhất: M310–M315 cùng M320–M325 trên các
  tuyến GRU/GRU+MJM, kèm role và timestamp; chưa cần thu lại toàn bộ dữ liệu tĩnh.
- Chưa chọn tiêu chí đạt về lực/góc/độ trễ theo mục đích conflict và chưa
  kiểm chứng session độc lập hoặc toàn workspace.

## Tái lập và artifacts

- [Báo cáo đầy đủ](../cocarry_logs/hc_force_calibration/20260917_dynamic_branches_v2_geometry/report_vi.md)
- [Kết quả JSON](../cocarry_logs/hc_force_calibration/20260917_dynamic_branches_v2_geometry/results.json)
- [CSV so sánh](../cocarry_logs/hc_force_calibration/20260917_dynamic_branches_v2_geometry/metrics.csv)
- [Đồ thị Home Y](../cocarry_logs/hc_force_calibration/20260917_dynamic_branches_v2_geometry/trial_1.png)
- [Ứng viên fit toàn GT, chưa dùng điều khiển](../cocarry_logs/hc_force_calibration/20260917_dynamic_branches_v2_geometry/full_GT_candidates.json)
- [Script phân tích](../scripts/analyze_hc_dynamic_branches.py)
- [Kiểm thử số học](../tests/test_hc_dynamic_branches.py)

`py_compile` đạt, 4 kiểm thử số học đạt. Pytest cần tắt autoload plugin
ROS không tương thích; môi trường có cảnh báo SciPy/NumPy. Không đổi
controller, safety limits hoặc log gốc. Không chạy robot, commit hay push.

Thư mục `v1` là thử nghiệm với offset=0 trước khi người dùng cung cấp
hình học; kết luận hiện tại dùng `v2_geometry`. Log/artifact nằm trong
`cocarry_logs` bị Git ignore, cần giữ/sao lưu cùng báo cáo để tái lập.
