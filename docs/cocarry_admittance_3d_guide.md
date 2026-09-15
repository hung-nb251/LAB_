# Co-carrying 3D bằng Admittance Control

Pipeline này độc lập với pipeline co-carrying dùng camera và với package
`codrawing_control`. Nó dùng `/joint_states` để tạo chuỗi `robot_ee`, mô hình dự
đoán tạo `x_d`, và lực Axia `/axia/human_force` tạo sai lệch admittance trên đủ
ba trục XYZ. Không có camera hoặc `coord_transform/transform_node` trong launch.

Luật điều khiển:

`M(xr_ddot-xd_ddot) + D(xr_dot-xd_dot) + K(xr-xd) = Fh`

Với cấu hình hiện tại: `M=[1,1,1]`, `K=[5,5,5]` và D được tính tự động theo
`D_i=2*sqrt(M_i*K_i)=[4.4721,4.4721,4.4721]`. Giới hạn an toàn lực là 20 N
trên mỗi trục và 30 N cho chuẩn lực tổng. Tần số là 15 Hz, giới hạn vận tốc
Cartesian 0.15 m/s và gia tốc 0.50 m/s² theo lịch sử Git của streamer.

Ground Truth và FOLLOWER dùng bộ gain trên. `Ground Truth` giữ `x_d` cố định
tại pose được capture lúc Start Run. Backend được chọn khi launch (`SVGP` hoặc
`GRU`) dùng output dự đoán tương đối từ chuỗi robot EE làm `x_d`. Chế độ
predictor+MJM bắt đầu bằng FOLLOWER, sau trigger tạm
`mjm.t_switch` chuyển sang LEADER và gửi điểm MJM trực tiếp, bỏ qua Admittance.
Chọn mode trước Start Run; không đổi mode giữa một lần chạy.

Z không bị khóa theo pose ban đầu. Controller dùng robot EE và đặt biên dưới
`EE_z=0.2314 m`, tương ứng đầu thanh dài 181.4 mm còn cách sàn ít nhất 0.05 m.
Biên trên EE là 1.50 m. Streamer cũng kiểm tra joint state, IK, joint limits,
workspace và tracking error theo chế độ fail-closed.

Giới hạn mềm trên J3 hiện là `1.25 rad` (~71.6°). Streamer vẫn trừ margin 3°,
nên giới hạn IK hữu hiệu là khoảng `1.198 rad` (~68.6°). Mức này áp dụng cho
cả mô phỏng và robot thật; X/Y workspace và năm joint còn lại không đổi. Không
được bỏ margin hoặc mở tiếp chỉ để che lỗi IK.

## Chạy robot thật

Terminal 1:

```bash
cd ~/cocarry_ws
./start_microros.sh
```

Máy 2 Ubuntu (user `binhdangnguyen`) chạy driver Axia và gửi UDP tới IP Wi-Fi
của máy `hungnb`:

```bash
sudo /home/binhdangnguyen/axia_driver/.venv/bin/python \
  /home/binhdangnguyen/axia_driver/axia_sensor_driver.py \
  enxf8e43b7aeaf2 --ip <IP_WIFI_HUNGNB> --port 50000 --hz 100
```

Terminal 2 trên máy `hungnb`:

```bash
cd ~/cocarry_ws
source install/setup.bash
ros2 launch cocarry_admittance_control cocarry_admittance_real_gui.launch.py
```

Launch mặc định dùng GRU robot-EE và UI hiển thị `GRU`/`GRU+MJM`. Để dùng
SVGP M100 với NumPy `.npz`, thêm `prediction_model:=svgp`:

```bash
ros2 launch cocarry_admittance_control \
  cocarry_admittance_real_gui.launch.py prediction_model:=svgp
```

Profile GRU dùng raw model output, không áp dụng proximity clamp, rate limiter
hoặc EMA của predictor. Các giới hạn safety ở controller/streamer vẫn hoạt
động. Runtime GRU dùng model TFLite float16; HDF5 chỉ được giữ để đối chiếu.

Từ 2026-09-06, cả hai launch co-carry tắt stationary HOLD của predictor
(`hold.enabled=false`) cho GRU và SVGP. EE đứng yên không còn reset lịch sử hay
thay prediction bằng vị trí hiện tại. Khi người nhả lực, robot vẫn có thể đi
theo reference dự đoán; **lực bằng 0 không phải lệnh dừng**. Dùng Stop Run khi
muốn dừng. HOLD an toàn do force stale, force limit và các kiểm tra readiness
ở controller không bị tắt. Profile camera cũ không thay đổi.

Predictor đã sửa rate gate để duy trì nhịp 15 Hz khi input 15 Hz có jitter nhỏ.
Streamer co-carry bật `--continuous-cartesian-smoothing`, bỏ nhánh snap tới
target gần khi đang đảo chiều; vị trí tiếp tục được tích phân từ vận tốc có
giới hạn. Không tăng giới hạn Cartesian/joint, không đổi K và không sửa model.
Các thay đổi này xử lý lỗi nhịp/smoother đã tái hiện, chưa bảo đảm hết giật trên
robot thật; joint clipping độc lập và chất lượng mạng force vẫn cần đánh giá.

Sau trial mô phỏng GRU `20260906_105435`, launch simulation bật thêm
`prediction_reference_tau_sec=0.4` để giảm ripple của vòng phản hồi EE/model.
Đây là lọc bậc một của nominal **tại controller, trước khi cộng admittance
error**, không sửa/che raw output GRU trong predictor và CSV. Đường
`Limited x_d` hiển thị nominal đã qua khâu này rồi qua giới hạn khoảng cách.
Khâu này có đánh đổi độ trễ; 0.4 s là tham số thử nghiệm, không phải bảo đảm
ổn định cho mọi model/quỹ đạo. Ground Truth và MJM LEADER không bị lọc.
K, giới hạn tốc độ, force watchdog giữ nguyên. Sau khi người dùng xác nhận
trial `20260906_111811`, **cả real và simulation mặc định 0.4 s**. Launch real
hỗ trợ cùng arg `prediction_reference_tau_sec`; không cần truyền arg để bật.
Độ mượt phần cứng thật vẫn cần xác nhận trực tiếp, không suy ra từ mock.

Chạy mô phỏng với bản giảm ripple (mặc định GRU):

```bash
ros2 launch cocarry_admittance_control cocarry_admittance_sim_gui.launch.py \
  prediction_reference_tau_sec:=0.4
```

Đối chứng hành vi trước sửa: dừng launch rồi chạy lại với
`prediction_reference_tau_sec:=0.0`. Không chạy hai launch cùng domain.

Trong lần thử đầu, giảm speed override trên pendant, kiểm tra riêng từng hướng
X+/X-/Y+/Y-/Z+/Z-, và luôn sẵn sàng nhấn E-stop. Calibrate Axia khi không chạm
vào thanh sắt. Controller sẽ tự capture pose/orientation, calibrate lại gốc
`robot_ee`, xóa buffer predictor rồi mới chuyển từ PREPARING sang RUNNING.

CSV mới được ghi trong `~/cocarry_ws/cocarry_logs` với prefix
`cocarry_admittance_3d_`. Ngoài các tín hiệu co-carry, logger lưu position,
velocity và effort thô của sáu khớp; khi phép đổi đơn vị được bật, logger còn
lưu joint torque ước lượng, `f_robot_x/y/z`, trạng thái hiệu chuẩn và các chỉ
số DLS/singularity.

Sensorless force dùng quy ước `tau_robot = J^T W_robot`, không tự đổi dấu lực.
Moment được giải nội bộ để không làm sai nghiệm lực nhưng không được publish và
không đi vào controller. Từ 2026-09-07, `robot_effort_unit_mode` mặc định rỗng:
launch dùng `effort_unit_mode` trong YAML, chỉ override khi truyền arg rõ ràng.
YAML hiện dùng `torque_nm`, theo hợp đồng đơn vị của MotoROS2 chính thức, nhưng
firmware/scale trên robot này chưa được kiểm chứng; vẫn `calibration_confirmed=false`.
Không tự nhân giới hạn URDF dựa trên độ lớn effort. Khi cần so sánh giả thiết
normalized, phải chọn rõ `robot_effort_unit_mode:=normalized_rated_torque`.

Trong trial hợp tác bình thường, logger ghi `f_robot_x/y/z` từ mẫu ước lượng
hợp lệ về mặt số học, **trước deadband** (`f_robot_output_stage=pre_deadband`).
Đây là dữ liệu chưa hiệu chuẩn, không phải lực tương tác đã xác nhận. Topic
`/sensorless_force` vẫn giữ deadband và ngưỡng loại mẫu 500 N như cũ.
CSV bổ sung `f_robot_unfiltered_x/y/z`, mode, frame, tuổi mẫu, đúng joint/effort
nguồn, scale và bias. Khi vượt ngưỡng, chỉ cột unfiltered giữ số chẩn đoán;
`f_robot_x/y/z` trống và status `INVALID`. Mẫu quá 0.25 s bị đánh dấu `STALE`
và không lặp lại lực cũ. `NO_SAMPLE` nghĩa chưa nhận `/sensorless_force/sample`.
Estimator và logger phải cùng được restart sau build. Không sửa CSV cũ.

Đường lực lấy Jacobian tại `tool0`, biểu diễn XYZ trong `base_link`; không tự
đổi dấu theo vận tốc robot. Lực phanh có thể ngược chiều chuyển động. Hiệu chuẩn
sau này phải đối chiếu tải/lực chuẩn và quy ước tác dụng lực, không ép dấu trùng
chiều vận tốc.

Kiểm tra thu dữ liệu, không Enable/Start robot:

```bash
ros2 launch cocarry_admittance_control cocarry_admittance_real_gui.launch.py \
  test_mode:=true robot_effort_unit_mode:=torque_nm \
  robot_force_calibrated:=false
```

Kiểm tra live (terminal đã source workspace và `ROS_DOMAIN_ID=10`):

```bash
ros2 param get /sensorless_force_node effort_unit_mode
ros2 topic echo /sensorless_force/sample --once
```

`test_mode` không chạy controller nên logger trial không tự có các tick reference
để tạo CSV; dùng nó để kiểm tra topic. Trong trial thật người vận hành Start Run
và bật logger như quy trình UI hiện tại thì các cột lực sẽ được ghi.

`normalized_rated_torque` chỉ nhân effort với các giới hạn torque trong URDF,
do đó là giả thiết sơ bộ để đối chiếu Axia, chưa phải hiệu chuẩn. Chỉ đặt
`robot_force_calibrated:=true` sau khi hệ số, bias/gravity/payload và dấu trên
cả X+/X-/Y+/Y-/Z+/Z- đã được xác minh. Cột
`f_robot_ready_for_role_selection` phải là `true` trước khi lực này được phép
tham gia chọn FOLLOWER/LEADER.

## Mô phỏng Force Sensor thật với robot ảo trong RViz

Launch mô phỏng này độc lập với pipeline camera cũ. Axia thật gửi UDP từ máy
thứ hai, còn HC10DTP, MotoROS2 services và `/joint_states` đều là fake hardware.
Mặc định launch dùng `ROS_DOMAIN_ID=42`, tách khỏi robot thật ở domain 10.

Trước khi chạy, dừng mọi launch đang giữ UDP port 50000. Không chạy
`start_microros.sh` và không bật Servo robot thật. Máy 2 Ubuntu vẫn chạy Axia
driver và gửi tới IP của máy `hungnb` như khi test sensor.

```bash
cd ~/cocarry_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  hc10dtp_simulation cocarry_admittance_control
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch cocarry_admittance_control \
  cocarry_admittance_sim_gui.launch.py
```

`Ground Truth` là mode mặc định và giữ `x_d` ở pose lúc bấm Start Run. Backend
mặc định là GRU; muốn thử SVGP M100 `.npz`, thêm `prediction_model:=svgp` vào
lệnh launch.
Sau đó chọn nút mang tên backend (`SVGP`/`GRU`) trước khi bấm Start Run. Muốn
thử hybrid, di chuyển robot tới đích, bấm `Set Goal / Capture Target`, đưa robot
về pose bắt đầu, chọn `<backend>+MJM`, rồi mới `Start Run`. UI đổi target EE tuyệt đối
thành displacement tương đối tại Start Run. FOLLOWER dùng `M=1`, `K=5`, critical
`D=4.4721`; LEADER bỏ qua Admittance. Không còn launch argument `force_only` và
`fixed_nominal`.

Trình tự trên hai cửa sổ UI:

1. Chờ robot ảo về home và MotoROS2 mock sẵn sàng.
2. Force Dashboard: giữ Roll 0, Pitch 0, Yaw -90; Calibrate khi không chạm.
3. Để kiểm tra lần đầu, tắt Calib Mode để trả deadband về 4 N và chọn bộ lọc
   Alpha 0.2 hoặc 0.5.
4. Predictor UI: bấm `Enable Robot`, chờ queue mode sẵn sàng, rồi `Start Run`.
5. Tác động từng hướng X+/X-/Y+/Y-/Z+/Z- dưới 10 N và quan sát RViz.
6. Bấm `Stop Run`, sau đó `Disable Robot` trước khi thoát launch.

Trong profile Admittance, Force Dashboard chỉ vẽ `F_human` XYZ ở 20 Hz; dữ
liệu ROS cho controller vẫn theo tốc độ UDP. Dashboard điều khiển cũng ẩn hàng
đồ thị Camera, `Calibrate Camera` và `Capture Init Pose`, nhưng pipeline camera
cũ vẫn giữ giao diện đầy đủ nhờ profile mặc định riêng.

Log mô phỏng được tách vào `~/cocarry_ws/cocarry_logs/simulation` với prefix
`cocarry_admittance_sim_3d_`.

Các tham số hữu ích:

```bash
# Không mở RViz (dùng cho chẩn đoán/headless)
ros2 launch cocarry_admittance_control \
  cocarry_admittance_sim_gui.launch.py use_rviz:=false \
  launch_dashboard:=false launch_sensor_ui:=false

# Nếu Axia UI đã được chạy riêng trên cùng ROS domain
ros2 launch cocarry_admittance_control \
  cocarry_admittance_sim_gui.launch.py launch_sensor_ui:=false
```

Mọi terminal dùng để `ros2 topic`, `ros2 node` hoặc `ros2 service` trong bài
test mô phỏng cũng phải `export ROS_DOMAIN_ID=42`.
