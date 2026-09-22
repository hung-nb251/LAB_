# Kế hoạch ngày 18/09/2026 — kiểm chứng calibration torque khi chuyển động

## Mục tiêu của buổi đo

Kiểm chứng nhánh M310–M315 → hiệu chỉnh torque → Jacobian → force trên
tuyến Home→Target 1 và Home→Target 2 trong GRU/GRU+MJM. Ghi đồng thời
M320–M325 để so sánh hai nhánh trên cùng lượt, không chỉ ghi torque.

Căn cứ: [phân tích ngày 17/09](hc_dynamic_force_branches_20260917_vi.md).
GT cho thấy nhánh torque có triển vọng nhưng các log GRU/MJM cũ không có
M310–M315. Không cần thu lại toàn bộ pose tĩnh hoặc GT X/Y/Z.

Đầu ra mong muốn là kết luận có/không chuyển giao được mô hình trên lượt
độc lập và xác định phần sai số còn lại. Không mặc định sẽ hoàn tất
calibration trong một ngày hoặc tự bật conflict/đổi vai.

## 1. Xác nhận hình học và chuẩn bị CAD — đã hoàn thành phần mô hình

Thông tin đã có, trong hệ trục Axia người dùng mô tả:

- Axia→flange = (0,−120,0) mm.
- Axia→tâm thanh = (−160,0,0) mm.
- Hai khoảng trên bỏ qua độ dày tấm đệm.
- Tổng cụm sau flange khoảng 2,350 kg; tải phía Axia dùng trong source
  1,126 kg. Đây là hai tập bộ phận khác nhau.

Đã kiểm tra assembly STEP ngày 18/09: bốn solid kín, không có giao thoa,
bounding box 401 × 200 × 72,4 mm. CAD đặt trục tâm Axia tại (0,0), tâm
flange theo plate tại (0,−118) mm và tâm thanh tại (−160,0) mm. Khi phân
tích tiếp tục dùng khoảng đo danh nghĩa −120 mm do người vận hành cung cấp;
chênh 2 mm được đưa vào kiểm tra độ nhạy, không âm thầm thay số đo bằng CAD.

Sơ đồ frame và công thức quy wrench nằm tại
[hc_tool_geometry_frames_20260918_vi.md](hc_tool_geometry_frames_20260918_vi.md).
Thông tin này đủ dựng sơ đồ các tâm và thử chuyển wrench sơ bộ. Chưa đủ
xác định rotation thực giữa Axia/tool0, CoG và inertia chính xác.

Việc cần làm trước đo:

1. Đánh dấu chiều +X/+Y/+Z trên cảm biến và xác nhận tọa độ mô tả thuộc
   trục đo Axia hay frame nominal trong URDF. Xác nhận chiều trục tool0
   và flange tương ứng; không tự cộng thêm một yaw correction lần nữa.
2. Xác định “tâm Axia” là gốc đo wrench theo bản vẽ hãng hay tâm hình học.
3. Đo lại vector flange→gốc đo Axia, bao gồm tấm đệm và hướng lắp của
   từng tấm; không cộng máy móc 10+8 mm vào một trục chưa xác định.
4. Ghi điểm cầm/tác động trên thanh, thứ tự lắp, ảnh có trục tọa độ và
   một bản vẽ kích thước đơn giản. Giữ gá/cáp/tải như buổi 17/09 nếu còn nguyên.
5. Ghi Tool number và Tool Data thực sự hiện hành, lịch sử Torque Origin,
   tare và filter. Không tự đổi Tool Data hoặc Torque Origin để làm số đẹp.

### CAD và bằng chứng hình học hiện có

**Không bắt buộc có CAD đầy đủ mới được thu dữ liệu.** Với phép chuyển
wrench về flange, cần transform đúng (vị trí + hướng); bản vẽ đo tay có
kích thước và trục rõ ràng có thể cung cấp thông tin này.

Thêm vào kế hoạch việc tìm/lấy CAD có sẵn của Axia, gá, tấm đệm và thanh.
Ưu tiên STEP hoặc file assembly gốc, bản vẽ xác định gốc/trục đo cảm biến.
Nếu có sẵn thì kiểm tra ngay; nếu chưa có, không dành cả buổi để dựng chi tiết
ren/bu-lông rồi bỏ lỡ đo. Có thể dựng mô hình khối đơn giản từ kích thước đo.

Ảnh đo ngày 18/09 đã xác nhận mục 1–2 của checklist: tâm Axia là tâm vòng
sáu lỗ, tâm flange là tâm vòng bốn lỗ, khoảng tâm là 120 mm và ký hiệu Y+
trên plate là Y+ của Axia. Ảnh assembly rear của Person2 cũng phù hợp với
thứ tự lắp thực tế. Vì vậy không cần dựng lại CAD hoặc đo lại khoảng XY này.

Còn phải ghi riêng thành phần Z từ mặt flange tới gốc đo wrench, cùng
rotation Axia→tool0/base_link trong TF runtime. Không tự coi tâm hình học là
gốc đo lực nếu bản vẽ ATI hoặc TF không xác nhận điều đó.

Để bù động lực học đáng tin cậy, cần mass/CoG/inertia của đúng phần tải
được Axia đỡ, và thông tin toàn cụm khi kiểm tra Tool Data robot. CAD hữu ích
nếu có vật liệu/mật độ/khối lượng đúng; hình học CAD đơn thuần không tự cho
mass properties chính xác. Có thể bổ sung bằng cân/đo/nhận dạng thực nghiệm.
Tâm thanh (−160,0,0) không được mặc định là CoG của cả phần tải.

Deliverable hình học: bảng bộ phận, khối lượng, kích thước, transform và
độ không chắc chắn; đánh dấu riêng thông số chưa xác nhận. Dùng cho offline
trước, chưa sửa URDF điều khiển chỉ để phản ánh bản vẽ mới.

## 2. Kiểm tra logger trước khi chạy tuyến — 20–30 phút

Dữ liệu bắt buộc:

- M310–M315: 6 external joint torques.
- M320–M325: đủ force và moment TCP để đối chiếu.
- Axia raw đủ 6D, human_force, connected/calibrated.
- Joint name/position/velocity, TF; timestamp gửi/nhận từng register.
- RUNNING/Stop/Fault và role FOLLOWER/LEADER, control_phase, target/reference.
- CSV controller và hybrid events đi kèm; ghi tên file đối ứng với trial.

**Trạng thái code hiện tại:** `scripts/hc_force_trial_logger.py` chỉ hỗ trợ
`--group all` (24 register, có torque) và `--group wrench` (6 register,
không có torque). Ngày mai không dùng `--group wrench` cho bài so sánh.

Đã thêm nhóm `torque_wrench`, chỉ đọc 12 register M310–M315 và M320–M325.
Giữ CSV controller để ghép role; status ROS vẫn được logger ghi qua
`/cocarry/status`. Không chạy hai logger cạnh tranh cùng service.

Đo 30–60 s để xác nhận đủ cả hai nhóm, tần số, độ lệch thời gian trong scan
và timeout. Với group all cũ khoảng 1,3 Hz, nội suy không khôi phục được
đỉnh nhanh. Không hứa nhóm 12 sẽ đạt một tần số cụ thể trước khi đo.
Nếu chỉ đạt tốc độ thấp, coi pilot là khảo sát chuyển động chậm; giải quyết
thu dữ liệu trước khi làm cả bộ trial. Không tăng watchdog điều khiển.

## 3. Hai lượt pilot trước khi thu nhiều — 20–30 phút

Thực hiện một lượt GRU Home→T1 và một lượt GRU+MJM Home→T2 trong các
pose/tuyến đã được xác nhận vận hành. Người vận hành thực hiện mọi lệnh
Enable/Start/Stop và trở về Home, theo cấu hình an toàn hiện có.

Quy trình mỗi lượt:

1. Logger chạy trước baseline. Ở Home, không tiếp xúc, ghi 15–20 s.
2. Ghi marker bắt đầu, chạy mode tương ứng. Trong GRU+MJM, ghi rõ lúc bấm
   LEADER và xác nhận role thực từ status/CSV; tên mode chưa chứng minh đã chạy MJM.
3. Tác động lực nhẹ, thay đổi từ từ trong giới hạn hiện có, có đoạn tăng/
   giảm lực và nhả. Không cố tạo xung nhanh hoặc kéo ngược mạnh để lấy số.
4. Khi đến đích, Stop theo quy trình, chờ ổn định và ghi không tiếp xúc
   15–20 s. Không để FOLLOWER tiếp tục kéo rời đích rồi gọi đó là baseline T1/T2.
5. Khi người vận hành đưa robot về Home bằng cách đã được phép, ghi rõ đoạn
   trở về và baseline Home sau vòng di chuyển. Không gộp phần Go Home/đổi
   orientation vào trial cùng orientation nếu thực tế có thay đổi.
6. Giữ tare trong session. Nếu tare/reconnect/đổi gá/Tool Data thì tách
   session và ghi sự kiện, không gộp như cùng điều kiện.

Phân tích pilot ngay: đủ torque, không dùng phần sau lỗi, lực Axia có
đoạn rõ hơn nền, giữ được role và dữ liệu trở về Home. Nếu không đạt,
sửa phần thu thiếu trước khi lặp hàng loạt.

## 4. Bộ kiểm chứng chính nếu pilot đạt — khoảng 60–90 phút

| Mode | Tuyến | Số lượt dự kiến |
|---|---|---:|
| GRU | Home→Target 1 | 3 |
| GRU | Home→Target 2 | 3 |
| GRU+MJM | Home→Target 1 | 3 |
| GRU+MJM | Home→Target 2 | 3 |

Pilot có thể tính là lượt 1 nếu chất lượng đạt và cấu hình không đổi.
Luân phiên tuyến nếu thuận tiện để tránh mọi lượt T1/T2 nằm ở hai giai
đoạn nhiệt/drift khác nhau. Giữ điểm cầm, tải và hướng tool như cũ.

Các lượt phải có lực thay đổi đủ rõ, không chỉ robot chạy trơn rồi đứng
yên. Giữ một kịch bản tăng lực nhẹ/nhả tương tự giữa các lần; không dùng
nhãn “conflict” khi chưa thiết kế và xác nhận bài kiểm chứng conflict.

Nếu vận hành hiện có cho phép MJM đi tuyến mà không cần người chạm,
ghi thêm một lượt không tiếp xúc trên mỗi tuyến để khảo sát baseline động.
Đây là dữ liệu bù, không thay thế lượt có tác động lực. Không tự tạo đường
chạy hoặc bỏ qua điều kiện capture target để thực hiện bài này.

## 5. Chia tập trước khi xem kết quả — 30–60 phút phân tích

1. **Kiểm chứng chuyển giao:** đóng băng ứng viên torque fit GT ngày 17/09,
   thử trên các log mới trước khi refit. Không đưa Axia của đoạn test vào
   estimator; cho phép baseline không tiếp xúc đầu lượt như thiết kế hiện tại.
2. **Hiệu chỉnh bổ sung:** nếu cần, chỉ dùng lượt 1–2 mỗi điều kiện để fit.
   Giữ lượt 3 ngoài training, lựa chọn mô hình, lag và ngưỡng. Không chia
   ngẫu nhiên các mẫu liền kề của cùng lượt.
3. Báo riêng RUNNING, đang di chuyển, lực nhỏ, FOLLOWER và LEADER. So hai
   nhánh trên cùng mẫu; ghi RMSE, góc, độ phủ, drift Home, tốc độ đọc và trễ.
4. Kiểm tra độ nhạy với hình học đã đo, moment và thành phần động; chỉ cần
   hoàn thiện CAD/mass properties ở mức sâu hơn nếu phần sai số còn lại
   thực sự phụ thuộc tải/gia tốc hoặc cần mở rộng orientation.
5. Chốt tiêu chí chấp nhận lực/góc/độ trễ theo mục tiêu nghiên cứu trước khi
   mở tập test cuối. Cải thiện tương đối không tự đồng nghĩa calibration đủ tốt.

## Cập nhật sau 7 lượt GRU và cách thu hai lượt kiểm chứng

Launch thực đã tích hợp bộ ước lượng shadow từ M310–M315. Mỗi lượt cần
đứng yên, không tiếp xúc tại Home ít nhất khoảng 5 s trước khi Start để
node thu đủ tối thiểu 12 scan, lấy median làm `tau0` và đóng băng `q0`.
Sau đó vận hành GRU Home→T1 hoặc Home→T2 và Start/Stop logger như thường lệ.
Không chạy `scripts/hc_force_trial_logger.py` đồng thời vì cả hai tiến trình sẽ
tranh chấp service `/read_mregister`.

Trong CSV controller:

- `f_robot_x/y/z` là lực khôi phục từ M310–M315 bằng nghiệm regularized
  của `J(q)^T W = tau_cal`, không còn lấy từ `joint_effort`;
- `joint_torque_est_nm_j1..j6` là `tau_cal` sau hiệu chỉnh baseline,
  hệ số theo joint và pose;
- các cột `f_robot_mregister_m310_nm..m315_nm`, `..._delta_...`,
  `f_robot_baseline_q0_j1..j6`, `f_robot_mregister_scan_span_sec` và
  `f_robot_calibration_file` giữ dữ liệu nguồn để tái lập phép tính;
- `joint_effort_raw_j1..j6` chỉ còn là tín hiệu chẩn đoán độc lập.

Kết quả này mang trạng thái `SHADOW_VALID:mregister_calibrated_pose` và
`role_valid=false`: nó được ghi để đánh giá, chưa được dùng bởi controller,
MJM hay quyết định LEADER/FOLLOWER. Với hai lượt kiểm chứng hiện tại, chỉ
cần một lượt GRU Home→T1 và một lượt GRU Home→T2, mỗi lượt tạo một CSV
riêng. Axia vẫn phải được ghi trong cùng CSV để đánh giá sai số ngoài mẫu.

## Lệnh logger rời cho thí nghiệm cô lập (chỉ đọc)

Ví dụ này dùng nhóm 12 `torque_wrench`. `--mode` chỉ là nhãn,
không chọn mode UI hay chạy robot. Chỉ ghi tool-number 0 nếu thực tế đúng.

```bash
cd /home/hungnb/cocarry_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10
python3 scripts/hc_force_trial_logger.py \
  --session 20260918_tool0_dynamic_tau_v1 \
  --category dynamic_force \
  --trial gru_home_to_target1_r1 \
  --mode gru \
  --pose home_to_target1 \
  --tool-number 0 \
  --group torque_wrench \
  --timeout 5.0 \
  --scan-gap 0.10 \
  --output /home/hungnb/cocarry_ws/cocarry_logs/hc_force_calibration \
  --notes 'pilot; same_tare_session; geometry_measured_separately; controller_csv_recorded'
```

Đổi mode/pose/trial cho từng lượt; dùng marker CLI hiện có. Marker lạ hiện
có thể bị cảnh báo; role thật lấy từ controller, không chỉ marker người dùng.
Các thời lượng trên là dự kiến, không rút ngắn bước kiểm tra để đủ số lượt.

## Nguồn và phạm vi

- [Quan hệ Jacobian và wrench — Modern Robotics](https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-2-statics-of-open-chains/): cơ sở ánh xạ torque tiếp xúc, không phải tổng torque động cơ.
- [Yaskawa — Alarm 6022](https://knowledge.motoman.com/hc/en-us/articles/4411798364311-Alarm-6022-EXTERNAL-FORCE-ESTIMATION-ERROR-TORQUE-SENSOR-CALIBRATION-REQUIRED): yêu cầu đúng mass/CoG/tool active; hãng cũng nêu khả năng ước lượng tải bằng controller.
- [Kết quả hai nhánh ngày 17/09](hc_dynamic_force_branches_20260917_vi.md).

Bộ ước lượng trên chỉ thay nguồn `F_robot` trong log và không thay Tool Data,
Torque Origin, giới hạn hay luật điều khiển. Nếu khác các giả định lịch sử
trong runbook cũ, dùng kế hoạch cụ thể này cho buổi 18/09; không tự nhập CoG
fit sơ bộ vào Tool Data.
