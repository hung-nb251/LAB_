# Co-carrying 3D bằng Admittance Control

Pipeline này độc lập với pipeline co-carrying dùng camera và với package
`codrawing_control`. Nó dùng `/joint_states` để tạo chuỗi `robot_ee`, mô hình dự
đoán tạo `x_d`, và lực Axia `/axia/human_force` tạo sai lệch admittance trên đủ
ba trục XYZ. Không có camera hoặc `coord_transform/transform_node` trong launch.

Luật điều khiển:

`M(xr_ddot-xd_ddot) + D(xr_dot-xd_dot) + K(xr-xd) = Fh`

Với cấu hình hiện tại: `M=[1,1,1]`, `K=[10,10,10]` và D được tính tự động theo
`D_i=2*sqrt(M_i*K_i)=[6.3246,6.3246,6.3246]`. Tần số là 15 Hz, giới hạn vận tốc
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
  enxec9a0c1fc063 --ip <IP_WIFI_HUNGNB> --port 50000 --hz 100
```

Terminal 2 trên máy `hungnb`:

```bash
cd ~/cocarry_ws
source install/setup.bash
ros2 launch cocarry_admittance_control cocarry_admittance_real_gui.launch.py
```

Launch mặc định dùng SVGP. Để dùng GRU robot-EE đã train, thêm
`prediction_model:=gru`; tên hai nút prediction trên UI sẽ đổi thành `GRU` và
`GRU+MJM`:

```bash
ros2 launch cocarry_admittance_control \
  cocarry_admittance_real_gui.launch.py prediction_model:=gru
```

Trong lần thử đầu, giảm speed override trên pendant, kiểm tra riêng từng hướng
X+/X-/Y+/Y-/Z+/Z-, và luôn sẵn sàng nhấn E-stop. Calibrate Axia khi không chạm
vào thanh sắt. Controller sẽ tự capture pose/orientation, calibrate lại gốc
`robot_ee`, xóa buffer predictor rồi mới chuyển từ PREPARING sang RUNNING.

CSV mới được ghi trong `~/cocarry_ws/cocarry_logs` với prefix
`cocarry_admittance_3d_`; không ghi joint position, velocity hoặc effort.

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
mặc định là SVGP; muốn thử GRU, thêm `prediction_model:=gru` vào lệnh launch.
Sau đó chọn nút mang tên backend (`SVGP`/`GRU`) trước khi bấm Start Run. Muốn
thử hybrid, di chuyển robot tới đích, bấm `Set Goal / Capture Target`, đưa robot
về pose bắt đầu, chọn `<backend>+MJM`, rồi mới `Start Run`. UI đổi target EE tuyệt đối
thành displacement tương đối tại Start Run. FOLLOWER dùng `M=1`, `K=10`, critical
`D=6.3246`; LEADER bỏ qua Admittance. Không còn launch argument `force_only` và
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
