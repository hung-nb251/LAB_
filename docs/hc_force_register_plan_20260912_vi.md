# Kế hoạch thu và kiểm chứng HC force — 12/09/2026

## Kết luận từ tài liệu

- Discussion #509: Ted Miller xác nhận /joint_states.effort lấy từ
  mpSvsGetVelTrqFb, API đơn vị 10^-6 Nm, MotoROS2 nhân 10^-6; đã tính
  gear ratio, là tổng torque ước lượng từ dòng motor, chưa bù tải/dynamics.
  Torque Spec trên Servo Monitor là % rated specification. Không đồng nhất
  % trên pendant với đơn vị của ROS effort và không nhân thêm gear ratio.
- HW1484764, mục 10, trang 10-1/10-2 do Ted đính kèm PR #188:
  M310–315 = estimated external torque S,L,U,R,B,T;
  M320–325 = estimated TCP external Fx,Fy,Fz,Mx,My,Mz;
  M330–335 = sensor S,L,U,R,B,T CH1;
  M340–345 = sensor S,L,U,R,B,T CH2.
- Với external torque/wrench, giải mã (raw - 10000)*0.1, force N,
  moment Nm. 9500 ở M310 = -50 Nm; 11055 ở M321 = 105.5 N.
  Trang 10-2 có câu scale tổng quát nhưng không giải thích đặc tính CH1/CH2;
  giữ CH1/CH2 nguyên bản, chưa suy ra đó là external torque đã bù hoặc dùng
  cùng scale để tính lực cho tới khi đối chiếu trang Sensor Output/manual đúng revision.
- Tài liệu Yaskawa xác nhận cảm biến torque HC ở đầu ra mỗi hộp giảm tốc.
  Torque Origin ảnh hưởng safety/PFL; không coi là thao tác tare diagnostic.
- PR có người báo ±40 N khi không tiếp xúc trên HC20. Đây là báo cáo cá biệt,
  chưa phải dung sai chính thức; Ted trả lời cần thử nghiệm thêm.

## Chuẩn bị và thứ tự thực hiện

### 1. Xác nhận cấu hình và kết nối (20–30 phút)

Ghi model/controller/software, loại pendant, Tool number, khối lượng tool,
CoG, inertia và transform tool0→Axia→điểm tiếp xúc, mounting, tải thực tế,
trạng thái torque origin hiện tại. Lấy manual đúng HC10DTP/YRC1000micro.
Chụp/ghi Servo Monitor > Torque Spec và Safety Func > Force Monitor cùng pose.
Torque Spec cần bảng rated specification từng trục và phía quy chiếu từ Yaskawa;
không lấy URDF effort limit làm rated torque. Cùng Nm vẫn phải phân biệt tổng
actuator torque với external torque đã bù.

Robot thật dùng ROS_DOMAIN_ID=10 (start_microros.sh); domain 42 là simulation.
Lần dò trước trên 42 không đủ kết luận robot mất kết nối.

`mpReadIO()` là API C của MotoPlus chạy **bên trong controller Yaskawa**. Mã
MotoROS2 triển khai service `ReadMRegister` bằng cách nhận địa chỉ M từ ROS 2,
đổi nó sang địa chỉ I/O nội bộ (ví dụ M310 thành 1000310), rồi gọi `mpReadIO()`.
Vì máy Ubuntu không chạy trực tiếp MotoPlus SDK, cách gọi từ phía chúng ta là:

```text
ros2 service call /read_m_register
  -> MotoROS2 ReadMRegister service
  -> mpReadIO() trên YRC1000micro
  -> giá trị uint16 của M-register trả về ROS 2
```

Chỉ cần tự gọi `mpReadIO()` nếu xây dựng/chỉnh sửa ứng dụng MotoPlus `.out`
chạy trên controller, như cách PR #188 tạo topic liên tục. Với MotoROS2 v0.2.1
hiện tại, dùng service là đường read-only có sẵn và không cần sửa firmware.

```bash
cd /home/hungnb/cocarry_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10
ros2 service list -t
ros2 interface show motoros2_interfaces/srv/ReadMRegister
```

Trên hệ thống HC10DTP hiện tại, service được phát hiện với tên
`/read_mregister` (không có dấu gạch dưới giữa `m` và `register`):

```bash
ros2 service call /read_mregister motoros2_interfaces/srv/ReadMRegister '{address: 310}'
ros2 service call /read_mregister motoros2_interfaces/srv/ReadMRegister '{address: 321}'
```

API nhận 310/321, tự cộng 1000000. Có interface cài trên PC chưa chứng minh
server khả dụng. Đọc đủ bốn nhóm, kiểm tra success/result_code trước giải mã.
Không diễn giải raw=0 hay giá trị đứng yên thành force hợp lệ nếu chưa xác nhận
trạng thái cập nhật. Đối chiếu pendant khi giữ pose, chấp nhận khác thời điểm lấy mẫu.

### 2. Bổ sung logger độc lập trước trial (30–45 phút)

Cập nhật triển khai: đã thêm logger độc lập `hc_force_trial_logger.py` ở gốc
workspace và hướng dẫn `docs/hc_force_trial_runbook_vi.md`. Chạy trực tiếp bằng
Python sau khi source ROS; không thay logger điều khiển hiện có. Đã kiểm tra
syntax và unit test scale/timeout; chưa kiểm chứng rate trên robot thật.
Logger có profile all (24 thanh ghi) hoặc wrench (6 thanh ghi), dừng gửi mới
khi timeout, lưu JSONL từng response/topic và marker. Tool/servo/tare/runtime
UI cần người vận hành ghi notes; không tự đọc Tool Data/servo từ controller.
Yêu cầu thiết kế và phần mở rộng còn cần kiểm chứng:
- Client chỉ đọc 24 địa chỉ, một request đang chờ tại một thời điểm, timeout,
  không chồng chu kỳ. Bắt đầu 1–2 lượt quét/s rồi đo độ trễ để quyết định rate.
- Ghi từng response: địa chỉ, raw, success/code, thời gian gửi/nhận, scan_id;
  ghi scan_start/end. Không gọi 24 giá trị tuần tự là một mẫu đồng thời.
- Lưu M310–315 Nm, M320–325 N/Nm riêng; CH1/CH2 raw riêng.
- Ghi joint position/velocity/effort, Axia raw đủ 6 trục, timestamp nguồn/nhận,
  trạng thái servo, Tool number, pose_id, phase, trial_id và cấu hình tare/filter.
- Logger torque_validation hiện đã nhận /axia/raw_wrench và joint states nhưng
  chưa đọc M-register. Không thay cột f_robot cũ bằng nguồn khác mà không ghi provenance.
- Test timeout, response lỗi, đúng scale/địa chỉ và dữ liệu không bị lặp giả;
  build trước chạy thật. Không đưa lực mới vào chọn vai trò.

### 3. Baseline và tác động tĩnh (45–60 phút)

Người vận hành giữ robot bằng chế độ vận hành phù hợp; không bật Ground Truth
admittance cho bài tĩnh. Không dùng Servo OFF làm bằng chứng zero cảm biến.
- Chọn 3 pose đã được xác nhận khả thi: pose chuẩn, pose khác trong vùng làm việc,
  pose thấp hơn trong vùng đã chạy; ghi q thực tế, không tự chỉ định pose mới.
- Mỗi pose ghi không tiếp xúc 15–20 s, không zero lại HC giữa các pose.
- Pose đầu: tác động X+/X−, 2 mức nhẹ trong giới hạn vận hành, giữ 3–5 s,
  nhả 5–10 s, lặp 3 lần. Mở rộng Y±, Z± nếu dữ liệu ổn.
- Giữ cùng điểm tiếp xúc; lực đi qua Axia, tránh lực tắt qua thanh/gá khác.
- Ghi tare Axia và trọng lực tool; ưu tiên so sánh chênh lệch so baseline cùng
  pose để tránh đồng nhất Axia đã tare với HC có bù tải.

### 4. Ground Truth động (30–45 phút)

Sau khi đo tĩnh đạt tính nhất quán, người vận hành chạy Ground Truth:
X+ nhẹ → nhả → X− nhẹ → nhả, lặp 3 lần; sau đó Y/Z nếu phù hợp.
Ground Truth có K>0 nên nhả tay robot trở về pose capture tại Start Run.
Đây là kiểm chứng chuyển động, không phải thủ tục Torque Origin.
Giữ nguyên cấu hình trial, đo sampling thực tế; nếu quét 24 thanh ghi quá chậm,
ưu tiên nhóm wrench cho bài động và ghi CH1/CH2 thưa hơn có timestamp.

### 5. Phân tích và quyết định (30 phút)

- Baseline: mean/std, drift, trở về zero, khác biệt giữa pose; không fit bù
  pose rồi dùng cùng dữ liệu tuyên bố calibration đúng.
- Đối chiếu wrench: frame, điểm quy chiếu, dấu/action-reaction, độ trễ,
  sai số từng trục, cross-axis và độ lặp lại. Không suy dấu từ vận tốc.
- Transform Axia về cùng điểm với Jacobian: F=R*F_sensor;
  M=R*M_sensor + r×F, r từ điểm quy chiếu tới sensor, trong cùng frame.
- So Δtau_M310 với J(q)^T ΔW_Axia tại pose tĩnh; xác nhận joint order/sign.
  Jacobian dự án base_link/tool0; frame base trong PR chưa được mặc định
  đồng nhất với base_link. Không ép tổng actuator torque bằng external torque.
- CH1/CH2 kiểm tra đáp ứng/lặp lại, không lấy trung bình thành lực khi chưa có
  định nghĩa chuyển đổi. Không suy raw baseline hai kênh phải bằng nhau.
- Chốt dung sai theo mục đích nghiên cứu với mentor; ±40 N trong PR không là
  mức chấp nhận. Dùng lần lặp/pose giữ lại để kiểm chứng độc lập.

## Torque Origin: chỉ thực hiện nếu được người phụ trách xác nhận cần thiết

Yaskawa yêu cầu Safety Security Mode, đúng Tool Data/loading, sau khi đổi Tool
phải di chuyển TCP ít nhất 5 mm trước calibration. Standard Pendant:
SAFETY FUNC → TORQUE SENSOR ORG POS → EDIT → SELECT ALL AXIS → xác nhận.
YBS3.05 cao hơn ngưỡng phần mềm được bài viết nêu cho any-posture; vẫn cần
xác nhận loại pendant và manual áp dụng. Nguồn đã đọc chưa chốt Servo ON/OFF
trong từng bước: không tự suy đoán. Không calibration dưới ngoại lực.
Lưu cấu hình/baseline trước, đo lại cùng pose sau nếu người phụ trách thực hiện;
việc này ảnh hưởng PFL/pull-back, không chỉ thay offset CSV.

## Nguồn

1. https://github.com/Yaskawa-Global/motoros2/discussions/509 — Ted Miller, API torque/gear ratio/Torque Spec.
2. https://github.com/Yaskawa-Global/motoros2/files/13300071/register.list.pdf — HW1484764 §10, 10-1/10-2.
3. https://github.com/Yaskawa-Global/motoros2/pull/188 — code, Force Monitor và báo cáo residual.
4. https://knowledge.motoman.com/hc/en-us/articles/15827842161175-HC-Robot-Series-Torque-Sensor-Origin-Position-Calibration
5. https://knowledge.motoman.com/hc/en-us/articles/18195286709783-HC-Series-Improper-Tool-Data-Effects

Đầu ra tối thiểu ngày mai: log đủ 24 register + Axia/joints, bảng mapping đối
chiếu pendant, baseline nhiều pose, X± tĩnh và Ground Truth, danh sách điểm
chưa xác minh. Chưa tuyên bố calibrated trước khi có kết quả đối chứng.
