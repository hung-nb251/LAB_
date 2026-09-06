# CODEX.md — Quy ước làm việc cho dự án CoCarry

> Cập nhật: 2026-09-03  
> Workspace chính: `/home/hungnb/cocarry_ws`  
> Máy điều khiển Ubuntu có tên người dùng `hungnb`; không dùng đường dẫn
> `/home/duy` trong code, config hoặc hướng dẫn.

## 1. Mục đích của tài liệu

File này là tài liệu khởi đầu cho một cuộc trò chuyện Codex mới. Trước khi phân
tích hoặc sửa dự án, phải đọc toàn bộ file này rồi kiểm tra code/config hiện tại
để xác nhận các giá trị chưa thay đổi.

Thứ tự ưu tiên khi có mâu thuẫn:

1. Yêu cầu mới nhất, rõ ràng của người dùng.
2. Code, launch và config thực sự đang được chạy trong workspace.
3. File `CODEX.md` này.
4. `docs/`, `ARCHITECTURE.md`, `README.md` và lịch sử Git.

`README.md` và `ARCHITECTURE.md` còn chứa một số mô tả của pipeline camera cũ;
không được dùng chúng để ghi đè hành vi được thể hiện trong code hiện tại.

## 2. Cách làm việc bắt buộc

- Luôn chạy `git status --short` trước khi sửa. Worktree thường có nhiều thay
  đổi chưa commit của người dùng; không reset, checkout, xóa hoặc ghi đè thay
  đổi không thuộc nhiệm vụ.
- Với thay đổi ảnh hưởng điều khiển robot, trước tiên đọc code liên quan, giải
  thích kế hoạch và chỉ thực thi sau khi người dùng đồng ý nếu họ yêu cầu duyệt
  kế hoạch trước.
- Không tự ý bật Servo, Enable Robot, Start Run hoặc gửi lệnh chuyển động tới
  robot thật. Người vận hành trực tiếp thực hiện các thao tác này.
- Thử logic thuần và mô phỏng trước robot thật khi có thể.
- Không nới workspace, joint limits, force limits, timeout, tracking-error
  threshold hoặc tắt `--fail-closed` nếu chưa giải thích rủi ro và được người
  dùng xác nhận.
- Không đổi frame, chiều trục, dấu lực, pose Home, tool offset, khối lượng tải
  hoặc góc gá cảm biến chỉ dựa trên suy đoán. Nếu dữ liệu thực nghiệm chưa đủ,
  hỏi lại người dùng.
- Sau khi sửa Python: chạy `python3 -m py_compile` cho file liên quan. Sau khi
  sửa package ROS: chạy test phù hợp và `colcon build --symlink-install
  --packages-select ...` ở mức tối thiểu cần thiết.
- Không commit hoặc push nếu người dùng chưa yêu cầu.
- Log và model là dữ liệu thực nghiệm: không xóa, đổi tên hoặc ghi đè. Model mới
  phải lưu vào thư mục mới và có metadata.

## 3. Trạng thái nhiệm vụ hiện tại

Mục tiêu hiện tại là **human–robot co-carrying trong không gian 3D bằng
Admittance Control**, dùng lực ATI Axia và chuỗi vị trí End-Effector của robot.
Pipeline này không dùng camera để điều khiển.

Ba pipeline phải được giữ độc lập:

1. Pipeline hiện tại: `cocarry_admittance_control` — Axia + robot EE + 3D.
2. Pipeline camera cũ: `hrc_bringup` — RealSense/Kinect; phải giữ nguyên để có
   thể dùng lại.
3. Pipeline `codrawing_control` — co-drawing XY; hiện không phải mục tiêu chính
   nhưng không được xóa hoặc trộn vào co-carrying.

Không chạy đồng thời hai pipeline có thể publish
`/cartesian_streamer/target_pose` hoặc cùng điều khiển robot thật.

## 4. Kiến trúc co-carrying 3D hiện tại

```text
/joint_states
  -> ee_tracker_node.py (FK)
  -> /hand_position, source=robot_ee
  -> trajectory_predictor
  -> /ml/predicted_position = x_d tương đối

Axia trên PC cảm biến
  -> UDP 50000
  -> axia_sensor_ui.py
  -> /axia/human_force trong base_link

x_d + F_h
  -> cocarry_admittance_controller
  -> /cartesian_streamer/target_pose
  -> cartesian_streamer_hc10dtp.py
  -> IK + QueueTrajPoint -> HC10DTP
```

Các file chính:

- `src/cocarry_admittance_control/config/cocarry_admittance_params.yaml`
- `src/cocarry_admittance_control/cocarry_admittance_control/admittance.py`
- `src/cocarry_admittance_control/cocarry_admittance_control/admittance_controller.py`
- `src/cocarry_admittance_control/launch/cocarry_admittance_real_gui.launch.py`
- `src/cocarry_admittance_control/launch/cocarry_admittance_sim_gui.launch.py`
- `src/hc10dtp_bringup/scripts/ee_tracker_node.py`
- `src/hc10dtp_bringup/scripts/cartesian_streamer_hc10dtp.py`
- `src/trajectory_predictor/trajectory_predictor/predictor_node.py`
- `src/trajectory_predictor/trajectory_predictor/inference_worker.py`
- `axia_sensor_ui.py`

### Luật điều khiển

```text
M (ẍr - ẍd) + D (ẋr - ẋd) + K (xr - xd) = Fh
e = xr - xd
M ë + D ė + K e = Fh
xr = xd + e
```

Robot chỉ nhận target Cartesian position; streamer giải IK. Không cần gửi
Cartesian velocity và không có `command_force` vì hệ thống không điều khiển lực
trực tiếp xuống robot.

Profile co-carrying 3D chuẩn trong config:

- `M = [1, 1, 1] kg`
- `K = [10, 10, 10] N/m`
- Critical damping: `D_i = 2*sqrt(M_i*K_i) = 6.32455532`
- Control/stream rate: `15 Hz`
- Max virtual/Cartesian velocity: `0.15 m/s`
- Max acceleration: `0.50 m/s²`
- Max command lead so với EE feedback: `0.03 m`
- Force limit mỗi trục: `15 N`; norm: `20 N`
- Force watchdog hai tầng: trên `0.20 s` giữ EE tại feedback hiện tại; trên
  `0.50 s` mới hard-fault và disable. Không được tăng ngưỡng hard-fault nếu
  chưa đánh giá lại trên robot thật.

Z là bậc tự do điều khiển trong co-carrying 3D, không dùng `--lock-z`. Z vẫn
chịu workspace, IK và joint limits. Controller dùng tọa độ EE; biên dưới EE đã
cộng chiều dài thanh để đầu thanh không đi dưới `0.05 m`:

```text
X: [-1.4, 1.4] m
Y: [-0.5, 1.3] m
EE Z: [0.2314, 1.50] m
Tip Z tối thiểu: 0.05 m
```

Orientation được chụp tại Start Run và giữ cố định. Control, prediction, UI,
Capture Target và log đều dùng robot EE; không cộng/trừ tool offset trong đường
điều khiển. Chiều dài thanh `148 + 8 + 25.4 = 181.4 mm` chỉ còn được dùng để
tính biên an toàn Z: `minimum_ee_z = minimum_tip_z + 0.1814`.

### Chọn nguồn `x_d` bằng Trajectory Mode

Không còn launch option `force_only` hoặc `fixed_nominal`. Admittance dùng
`M=1`, `K=5`, critical `D=4.4721` trong Ground Truth và pha FOLLOWER. Giới hạn
lực hiện tại là 20 N trên từng trục và 30 N cho chuẩn lực tổng:

- `Ground Truth` là mặc định: giữ `x_d` cố định tại robot EE pose được capture
  lúc `Start Run`. Lực người tạo admittance error `e`, do đó `x_r=x_d+e`.
  Khi người nhả lực, `e -> 0` do `K > 0` và robot trở lại pose capture này.
  "Home" trong ngữ cảnh trial là pose capture tại Start Run, không mặc định là
  joint Home dùng bởi nút Go Home.
- `SVGP`/`Prediction`: lấy `x_d = capture_ee + predicted_relative`, trong đó
  dự đoán đến từ SVGP dùng chuỗi robot EE tương đối. AI nominal hoạt động cả
  khi lực nhỏ hoặc bằng 0 để giảm effort của người. Lực người vẫn luôn đi qua
  Admittance tạo `e`, vì vậy người có thể kéo reference lệch khỏi dự đoán để
  giành lại quyền điều khiển. Không gate SVGP bằng độ lớn lực. Nominal AI chỉ
  bị giới hạn hình học cách actual EE tối đa `0.05 m` để bảo vệ IK/tracking;
  raw SVGP chỉ được log để chẩn đoán offline, không hiển thị trên UI.
- `SVGP+MJM`: bắt đầu ở FOLLOWER như trên; sau `mjm.t_switch` tạm thời chuyển
  sang LEADER. MJM bắt đầu từ robot EE thực tế và gửi point trực tiếp; LEADER
  bỏ qua Admittance và không cộng `e`.
- Chọn mode trên UI trước `Start Run`. Đổi mode khi PREPARING/RUNNING vẫn gây
  safety fault; phải Stop Run rồi mới đổi mode.

Robot EE đầu vào SVGP được `ee_position_tracker` lấy mẫu đều tại `15 Hz`, độc
lập với `/joint_states` 50/100 Hz. Điều này bắt buộc để window 10 mẫu có cùng
nhịp thời gian với dữ liệu train. Tham số chung nằm tại namespace
`ee_position_tracker.publish_rate_hz` trong
`cocarry_admittance_params.yaml` và được dùng bởi cả launch thật lẫn mô phỏng.

UI co-carry chỉ vẽ `Limited x_d` từ `/cocarry/nominal_position` và `Actual EE`.
Raw SVGP vẫn được lưu trong CSV để chẩn đoán offline nhưng không hiển thị trên
UI. Hai đường dùng timestamp thật với trục X là thời gian dương tính từ
`Start Run`; không ghép theo sample index vì Actual EE và controller khác tần
số. Mỗi trial tự xóa buffer đồ thị cũ.

### Kiến trúc FOLLOWER/LEADER — đã triển khai từng phần

Nguồn tham khảo thiết kế:

- `/home/hungnb/simulation_hri/images/architecture.png`
- `/home/hungnb/simulation_hri/outer_loop.py`
- `/home/hungnb/simulation_hri/inner_loop.py`

Sơ đồ trên là kiến trúc mô phỏng cũ, không được sao chép nguyên trạng sang robot
thật. Các hiệu chỉnh đã được người dùng xác nhận:

- Ký hiệu `X_h`/chuỗi tay người từ camera trong sơ đồ cũ phải được thay bằng
  chuỗi vị trí `robot_ee` trong hệ hiện tại.
- Số điểm/horizon của Goal Classification trong sơ đồ và simulation chỉ là tham
  khảo; phải xác nhận lại trước khi triển khai.
- Robot thật không có khối PD Cartesian như `inner_loop.py`. Hệ thật chỉ gửi
  từng Cartesian position target cho streamer; streamer giải IK và gửi trajectory
  point xuống robot. Không thiết kế đầu ra Cartesian velocity/force chỉ vì sơ đồ
  cũ có `x_dot_d` hoặc `F_r`.

Kiến trúc đích có hai luật tạo reference loại trừ lẫn nhau:

```text
FOLLOWER:
  robot_ee history -> SVGP -> nominal x_d
  F_h -> Admittance, M*e_ddot + D*e_dot + K*e = F_h
  Cartesian target gửi robot: x_r = x_d + e

LEADER:
  robot_ee history -> Goal Classification -> predicted goal
  current robot position + predicted goal + t_f -> MJM
  Cartesian target gửi robot trực tiếp: x_r = x_MJM
  BỎ QUA Admittance Control; không cộng e vào x_MJM
```

Quy tắc chuyển `FOLLOWER -> LEADER`:

- Vì target cuối của FOLLOWER đã là `x_r = x_d,SVGP + e`, điểm đầu MJM phải neo
  tại trạng thái robot ở thời điểm chuyển (`x_MJM(0) = x_r(t_switch)`, ưu tiên
  actual/current robot pose hoặc một cơ chế bumpless-transfer đã được xác nhận),
  không dùng riêng raw/filtered SVGP `x_d` và không cộng lại `e` trong LEADER.
- Đây phù hợp với nhánh `MJM_METHOD == CURRENT` của simulation `outer_loop.py`,
  nơi `x_0_mjm = current_x_robot.copy()` khi vừa vào LEADER.
- Trong LEADER, lực người không đi qua Admittance để sinh lệnh chuyển động. Có
  thể vẫn dùng lực làm tín hiệu giám sát an toàn/xung đột để trả quyền về FOLLOWER,
  nhưng tiêu chí thực tế chưa được chốt.
- Chuyển `LEADER -> FOLLOWER` cũng cần bumpless transfer/blending và khởi tạo lại
  state Admittance phù hợp với pose hiện tại; không được bật lại state `e` cũ một
  cách đột ngột.

Phần đã triển khai: UI Capture Target theo robot EE, đổi goal tuyệt đối thành
displacement tương đối tại Start Run, MJM 15 Hz bắt đầu từ robot EE đo được,
gắn nhãn output `mjm`, và controller bypass Admittance trong LEADER. Force
timeout/force limit, workspace, command-lead, pose timeout và streamer readiness
vẫn hoạt động trong LEADER. Yêu cầu Capture Target này chỉ áp dụng cho profile
co-carry không-camera; UI camera cũ tiếp tục dùng nút/goal và fallback riêng.

Predictor co-carry hiện do controller sở hữu chuỗi Start/Stop; việc chỉ chọn nút
SVGP trên UI không tự chạy worker. Mỗi Start/Stop reset toàn bộ buffer, filter,
velocity và HOLD state, đồng thời tăng prediction epoch để loại response cũ.
Từ 2026-09-06, co-carry đặt `hold.enabled=false` cho cả GRU/SVGP: EE đứng yên
không còn ép predictor sang HOLD hoặc reset lịch sử; nhả lực không đồng nghĩa
với yêu cầu dừng robot. HOLD an toàn khi dữ liệu lực stale ở controller vẫn giữ
nguyên. Camera profile giữ HOLD mặc định bật và cơ chế auto-toggle cũ qua
parameter `trajectory_mode_auto_toggle=true`; co-carry override thành `false`.
Rate gate predictor dùng deadline monotonic với dung sai jitter để không bỏ
mỗi mẫu đến sớm một chút khi input và inference cùng 15 Hz. Hai launch co-carry
bật `--continuous-cartesian-smoothing`: tích phân vận tốc qua đoạn đảo chiều,
không snap thẳng tới target gần. Giới hạn vận tốc/gia tốc và joint limits không
đổi; profile camera giữ hành vi smoother cũ. Chưa thay cơ chế joint clipping
độc lập, mạng force sensor hay model GRU trong đợt sửa này.

Sau trial GRU simulation `20260906_105435`, đã tái hiện ripple trong vòng
robot-EE -> prediction -> nominal -> Admittance -> robot-EE mà không cần mạng
hay IK. Output GRU raw vẫn giữ nguyên. Launch simulation hiện override
`prediction_reference_tau_sec=0.4`: lọc bậc một nominal tại controller trước
khi cộng `e`, rồi mới áp lead/workspace limits. Đây là điều hòa reference có
đánh đổi độ trễ, không phải sửa đồ thị hay chứng minh model chính xác hơn.
Ground Truth và MJM LEADER bypass khâu này. State reset tại Start/realign và
đóng băng cùng force HOLD. Sau khi người dùng xác nhận trial 11:18:11 và yêu
cầu áp dụng sang robot thật, YAML và cả hai launch mặc định `0.4`. Real và sim
đều hỗ trợ launch arg `prediction_reference_tau_sec:=0.0` để đối chứng legacy.
Không nới limits, không tự Enable/Start robot thật. Cùng cấu hình nominal không
bảo đảm phần cứng thật có cùng đáp ứng với mock; cần người vận hành xác nhận.

Phần chưa triển khai: Goal Classification, disagreement/role arbitration hoàn
chỉnh, horizon chính thức, điều kiện trả quyền do conflict và blend LEADER ->
FOLLOWER. Hiện FOLLOWER -> LEADER vẫn dùng `mjm.t_switch = 5 s` làm trigger tạm.
Phải trình kế hoạch và xin người dùng đồng ý trước khi triển khai các phần còn
lại.

## 5. Force Sensor và calibration

Trạng thái phần cứng đã chốt:

- Chỉ dùng `Fx, Fy, Fz`; torque không dùng cho control hoặc UI.
- Thanh sắt hiện vẫn lắp hướng xuống đất, chưa gắn thêm tải 2 kg.
- Payload dùng để bù trọng lực hiện tại: `0.33 kg`.
- Góc bù gá mặc định: Roll X `0°`, Pitch Y `0°`, Yaw Z `-90°`.
- Deadband vận hành: radial deadband `4 N`, không phải deadband riêng từng trục.
- Calib Mode chỉ đặt deadband về `0 N` để quan sát/calibrate hướng; nó không tự
  thay thế thao tác bấm `Calibrate F/T Sensor`.
- Filter mặc định: median size 5 + EMA alpha `0.1`. Khi test hướng nên tạm dùng
  Raw `alpha=1.0` và Calib Mode; sau đó trả lại filter/deadband vận hành.
- UI chỉ vẽ `F_human` XYZ ở 20 Hz. ROS vẫn publish lực theo tốc độ UDP; không
  hiển thị Raw force để giảm tải giao diện.

Quy trình calibrate:

1. Phải có `/joint_states` và TF `base_link <-> axia_sensor_link`.
2. Đặt robot ở pose sẽ vận hành, giữ thanh sắt đúng cách lắp, không chạm và
   không treo lực ngoài.
3. Xác nhận Roll/Pitch/Yaw đúng; thay đổi bất kỳ góc nào sẽ làm calibration cũ
   mất hiệu lực.
4. Bấm `Calibrate F/T Sensor`; node thu 100 mẫu và tính bias sau khi trừ trọng
   lực theo pose hiện tại.
5. Kiểm tra không tải và sau khi nhả. Nếu residual lớn/giữ ở mức cao, kiểm tra
   gá cơ khí, cáp, UDP/EtherCAT, filter và calibrate lại; không che lỗi bằng cách
   tăng deadband tùy tiện.
6. Test lần lượt X+/X-/Y+/Y-/Z+/Z- ở lực nhỏ. Chiều dương của `F_human` phải
   khớp các trục robot hiển thị trên UI/RViz.

Quan sát gần nhất với Yaw `-90°` cho đáp ứng đúng chiều tổng thể:

- X+: Fx dương; X-: Fx âm.
- Y+: Fy dương; Y-: Fy âm.
- Z+: Fz dương; Z-: Fz âm.

Controller từ chối Start Run nếu Axia chưa calibrated hoặc dữ liệu lực/pose
không còn fresh. Start Run tự capture pose/orientation, calibrate gốc robot EE,
xóa/reset predictor rồi mới vào RUNNING. Vì vậy model robot EE nhận tọa độ tương
đối đúng với lúc train; không cần người dùng lưu joint positions làm mốc.

Protocol thực nghiệm có thể calibrate lại F/T trước mỗi trial, nhưng chỉ khi
thanh/gá đang ổn định, không chạm môi trường và thật sự không chịu lực ngoài.
Không tare ở trạng thái còn preload sau khi kéo vì như vậy sẽ xóa một lực vật lý
thật và làm sai zero của trial kế tiếp.

## 6. Kết nối phần cứng và ROS domain

- Robot thật: `ROS_DOMAIN_ID=10`.
- Mô phỏng: `ROS_DOMAIN_ID=42` để cô lập khỏi robot thật. Đây là chủ ý an toàn,
  không phải lỗi cấu hình.
- Axia truyền 6 float qua UDP port `50000`, nhưng PC điều khiển chỉ sử dụng ba
  thành phần force.

Topology ưu tiên là hai máy:

- PC 2 Ubuntu, user `binhdangnguyen`, đọc Axia bằng `axia_sensor_driver.py` qua
  interface `enxf8e43b7aeaf2` và gửi UDP tới IP Wi-Fi máy `hungnb`.
- PC Ubuntu chạy robot/MotoROS2 và `axia_sensor_ui.py` nhận UDP.
- Khi PC 2 đã gửi dữ liệu, không chạy thêm `run_sensor_driver.sh` trên Ubuntu.

### Thiết lập máy 2 Ubuntu để đọc Axia

Hai đường mạng có vai trò khác nhau, không được cấu hình lẫn nhau:

- `enxf8e43b7aeaf2` là cổng Ethernet trên hub USB-C nối trực tiếp với Axia.
  EtherCAT dùng frame Ethernet tầng 2 nên cổng này không cần IP `10.136.x.x`.
- Wi-Fi dùng để hai laptop ping nhau và truyền gói UDP lực. Các địa chỉ đã từng
  dùng là máy `hungnb`: `10.136.12.182/20`, máy `binhdangnguyen`:
  `10.136.4.243/20`. Đây là IP DHCP, phải kiểm tra lại ở mỗi buổi thí nghiệm.
- Driver trên máy 2 không phải ROS node nên máy 2 không cần đặt
  `ROS_DOMAIN_ID`. Domain `10` hoặc `42` chỉ áp dụng cho các terminal ROS trên
  máy điều khiển `hungnb`.

Kiểm tra IP Wi-Fi hiện tại và kết nối hai chiều. Trên mỗi máy chạy:

```bash
hostname -I
ip -4 addr show
ping -c 4 <IP_WIFI_MAY_CON_LAI>
```

Hai IP `/20` ở trên cùng thuộc mạng `10.136.0.0/20`, vì vậy có thể liên lạc
trực tiếp nếu Wi-Fi không bật client isolation. Không dùng IP của
`enxf8e43b7aeaf2` làm địa chỉ nhận UDP.

Chuẩn bị máy 2 lần đầu:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip openssh-server ethtool tcpdump
sudo systemctl enable --now ssh
mkdir -p ~/axia_driver
python3 -m venv ~/axia_driver/.venv
~/axia_driver/.venv/bin/python -m pip install --upgrade pip
~/axia_driver/.venv/bin/python -m pip install pysoem
```

Từ máy `hungnb`, chép driver sang máy 2 bằng IP Wi-Fi hiện tại:

```bash
scp ~/cocarry_ws/axia_sensor_driver.py \
  binhdangnguyen@<IP_WIFI_MAY_2>:/home/binhdangnguyen/axia_driver/
```

Nếu `scp` timeout, kiểm tra `ping`, `sudo systemctl status ssh` trên máy 2 và,
nếu UFW đang active, cho phép profile `OpenSSH`. Không ghi mật khẩu SSH vào
script hoặc tài liệu. Sau khi chép, kiểm tra trên máy 2:

```bash
ls -lh ~/axia_driver/axia_sensor_driver.py
~/axia_driver/.venv/bin/python -m py_compile \
  ~/axia_driver/axia_sensor_driver.py
~/axia_driver/.venv/bin/python -c "import pysoem; print('pysoem OK')"
```

Cắm Axia vào hub USB-C của máy 2, sau đó kiểm tra link:

```bash
ip link show enxf8e43b7aeaf2
sudo nmcli device set enxf8e43b7aeaf2 managed no
sudo ip addr flush dev enxf8e43b7aeaf2
sudo ip link set dev enxf8e43b7aeaf2 up
cat /sys/class/net/enxf8e43b7aeaf2/carrier
```

Giá trị `carrier` phải là `1`. Đặt cảm biến và thanh sắt ở trạng thái ổn định,
không chịu lực ngoài, rồi chạy driver trên máy 2. Thay IP dưới đây bằng IP
Wi-Fi hiện tại của máy `hungnb`:

```bash
sudo /home/binhdangnguyen/axia_driver/.venv/bin/python \
  /home/binhdangnguyen/axia_driver/axia_sensor_driver.py \
  enxf8e43b7aeaf2 --ip <IP_WIFI_HUNGNB> --port 50000 --hz 100
```

Ví dụ chỉ khi IP chưa đổi: `--ip 10.136.12.182`. Phải dùng đúng Python trong
`.venv` sau `sudo`; nếu gọi `sudo python3` thì thường sẽ báo thiếu `pysoem`.
Driver thực hiện hardware tare khi khởi động, vì vậy tuyệt đối không chạm hoặc
tạo preload trong lúc khởi động. `run_sensor_driver.sh` hiện chứa đường dẫn của
user `hungnb`, không chạy nguyên trạng trên máy 2; dùng lệnh Python đầy đủ ở
trên.

Trên máy `hungnb`, launch hệ thống theo mục 7 nhưng chưa Enable/Start robot.
Sau đó xác nhận UDP và ROS:

```bash
sudo tcpdump -ni any udp port 50000
ros2 topic echo /axia/connected --once
ros2 topic echo /axia/calibrated --once
ros2 topic hz /axia/human_force
```

`/axia/connected` phải là `true`; `/axia/calibrated` chỉ thành `true` sau khi
bấm `Calibrate F/T Sensor`. Dừng `tcpdump` và `topic hz` bằng `Ctrl+C`. Nếu UFW
trên máy `hungnb` đang active và không thấy packet, chỉ mở UDP từ đúng máy 2:

```bash
sudo ufw allow from <IP_WIFI_MAY_2> to any port 50000 proto udp
```

Chẩn đoán nhanh:

- Không có EtherCAT OP hoặc `carrier=0`: kiểm tra đúng interface, hub, cáp và
  nguồn Axia; chưa liên quan đến ROS domain.
- Máy 2 báo đang gửi nhưng `tcpdump` máy `hungnb` không thấy UDP: sai IP đích,
  Wi-Fi/client isolation hoặc firewall.
- Có UDP nhưng UI không có lực: kiểm tra tiến trình nào đang giữ port bằng
  `sudo ss -lunp | rg ':50000'`; chỉ một `axia_sensor_ui.py` được bind port.
- `/axia/udp_gap_ms` lớn hoặc force stale: kiểm tra Wi-Fi, tải CPU và EMI trước
  khi tăng watchdog.
- Sau khi driver/mạng reconnect, calibration cũ cố ý bị vô hiệu. Phải đặt cảm
  biến không tải và Calibrate lại trước `Start Run`.

Thứ tự an toàn cho robot thật: khởi động micro-ROS trên máy `hungnb`; launch UI
nhưng chưa Enable; chạy driver Axia trên máy 2; xác nhận connected/fresh và
Calibrate; sau đó mới Enable Robot và Start Run. Khi kết thúc, Stop Run và
Disable Robot trước, rồi mới `Ctrl+C` launch và driver Axia.

Đã từng quan sát khi cắm Axia EtherCAT và robot vào cùng laptop: bật Servo có
thể làm driver/micro-ROS mất kết nối và pendant báo nhóm lỗi `8011[...]` kèm
`8017`. Đây chưa được chứng minh là lỗi phần mềm; phải coi là vấn đề có thể liên
quan USB/Ethernet, EMI hoặc ground loop. Việc vừa sạc laptop có thể làm đường
ground xấu hơn. Không thử lại topology một máy với robot chuyển động nếu chưa có
kế hoạch cách ly điện, kiểm tra nguồn/cáp và người giám sát an toàn.

## 7. Lệnh vận hành chuẩn

### Build

```bash
cd ~/cocarry_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Robot thật, Axia gửi từ máy thứ hai

Terminal Ubuntu 1:

```bash
cd ~/cocarry_ws
./start_microros.sh
```

Terminal Ubuntu 2:

```bash
cd ~/cocarry_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch cocarry_admittance_control \
  cocarry_admittance_real_gui.launch.py
```

Chỉ kiểm tra joint states/TF/force/UI, không chạy controller thật:

```bash
ros2 launch cocarry_admittance_control \
  cocarry_admittance_real_gui.launch.py \
  test_mode:=true use_rviz:=false
```

Launch trên đã tạo `/joint_states` thông qua bringup robot; không khởi chạy một
terminal MoveIt riêng lần nữa trừ khi code launch đã thay đổi.

Nếu dùng Axia ngay trên Ubuntu thay vì PC 2, thêm một terminal chạy
`./run_sensor_driver.sh`, nhưng đây không phải topology ưu tiên do sự cố phần
cứng nói trên.

### Mô phỏng Axia thật + robot ảo trong RViz

Không chạy `start_microros.sh`, không bật Servo robot thật. PC 2 vẫn gửi Axia
UDP đến Ubuntu.

```bash
cd ~/cocarry_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cocarry_admittance_control \
  cocarry_admittance_sim_gui.launch.py
```

Launch tự đặt domain 42. Các terminal chẩn đoán riêng phải đặt
`export ROS_DOMAIN_ID=42`.

Launch dùng Ground Truth và backend GRU mặc định. Muốn test GRU làm nominal
reference, chọn nút `GRU` trên UI trước `Start Run`. Muốn test SVGP M100
NumPy, chọn backend ngay khi launch; UI sẽ đổi hai nút thành `SVGP` và
`SVGP+MJM`:

```bash
ros2 launch cocarry_admittance_control \
  cocarry_admittance_sim_gui.launch.py prediction_model:=svgp
```

Trình tự UI an toàn: chờ joint states/TF -> calibrate Axia không tải -> Enable
Robot -> chờ streamer ready -> Start Run -> Stop Run -> Disable Robot.

## 8. Predictor robot EE

Hai launch co-carry thật/mô phỏng hỗ trợ `prediction_model:=svgp|gru`; GRU là
mặc định để ưu tiên runtime ổn định và nhẹ. Mỗi backend có profile riêng về model
directory, window, số feature và cách tính velocity. Có thể override riêng bằng
`svgp_model_dir:=...`, `gru_model_dir:=...`, hoặc dùng `model_dir:=...` để thay
directory của backend đang chọn.

### SVGP

Model mới đã train ngày 2026-09-05:

```text
/home/hungnb/cocarry_ws/pHRI_Models/svgp_robot_ee_h5_m100_relative
```

Model này đã train, đã kiểm tra load/inference và được cả hai launch dùng khi
chọn `prediction_model:=svgp`:

```text
/home/hungnb/cocarry_ws/pHRI_Models/svgp_robot_ee_h5_m100_relative
```

Runtime dùng trực tiếp `svgp_model.npz`; `svgp_model.pkl` chỉ được giữ làm
fallback/đối chứng GPflow.

Cấu hình model mới:

- Source: `robot_ee_x`, `robot_ee_y`, `robot_ee_z`.
- Chỉ dùng log `GROUND_TRUTH`.
- Mỗi quỹ đạo được resample 15 Hz và trừ pose robot EE đầu tiên.
- Window 10 bước, horizon 5 bước, dự đoán trước khoảng 0.333 s.
- SVGP Matern52, 100 inducing points.
- Train 80 files/13,017 windows; validation 13 files; test 7 files.
- Best epoch 128.
- Test RMSE X/Y/Z: `3.16 / 8.42 / 4.91 mm`; mean 3D error `8.50 mm`.

Artifact:

- `svgp_model.npz` là runtime mặc định, dùng NumPy với alpha đã tính trước.
- `svgp_model.pkl` được giữ làm fallback/đối chứng GPflow.
- `scaler_x.pkl`
- `scaler_y.pkl`
- `metadata.json`

Worker dùng `SVGPNumpyRunner` cho `.npz`; output đã đối chiếu với GPflow sai
khác dưới `5e-13`. Inference trực tiếp qua worker đo khoảng `0.08--0.13 ms` trên
máy hiện tại, so với khoảng `15 ms` của `.pkl`. Nếu một model directory cũ
không có `.npz`, worker tự fallback sang file `.pkl` cùng tên gốc.

Script train nằm ngoài ROS workspace tại:

```text
/home/hungnb/simulation_hri/train_svgp_pHRI.py
```

Dataset train tương ứng:

```text
/home/hungnb/simulation_hri/cocarry_logs
```

### GRU joint XYZ

Model GRU robot-EE hoàn tất ngày 2026-09-04 và đã được chuyển sang HDF5 tương
thích TensorFlow 2.15 của máy runtime:

```text
/home/hungnb/cocarry_ws/pHRI_Models/gru_robot_ee_h5_relative_v1
```

Cấu hình runtime khớp notebook train: một GRU joint cho XYZ, window 20, horizon
5, input `[x,y,z,dx,dy,dz]` ở 15 Hz. `dx,dy,dz` là displacement chính xác giữa
hai mẫu liên tiếp, không chia 16 và không EMA. Mẫu đầu mỗi trial dùng velocity
bằng 0. GRU xuất raw prediction, không đi qua proximity/rate/EMA output filter;
controller vẫn giữ prediction-lead, workspace, IK và tracking safety limits.
Runtime dùng `gru_joint_Ts5_float16.tflite` (float16 weights, float32 I/O), còn
file HDF5 được giữ làm đối chứng. Artifact cũng gồm `scaler_x.pkl`,
`scaler_y.pkl` và metadata/manifests. Kết quả test của lần train được chọn:
RMSE X/Y/Z `2.929 / 7.814 / 5.525 mm`, mean 3D `8.532 mm`, p95 `18.727 mm`.

`pHRI_Models/` đang bị Git ignore. Phải sao lưu model riêng nếu cần chuyển máy.

## 9. Logging

Co-carrying 3D thật ghi vào:

```text
/home/hungnb/cocarry_ws/cocarry_logs
```

Mô phỏng ghi vào:

```text
/home/hungnb/cocarry_ws/cocarry_logs/simulation
```

Logger co-carrying lưu các nhóm chính:

- actual robot EE XYZ;
- predicted relative `x_d` và nominal absolute `x_d`;
- admittance error và reference `x_r`;
- `f_human_x/y/z` và `f_robot_x/y/z`;
- inference time, model, role và timestamp lực.
- raw SVGP relative, `force_age_ms`, `prediction_age_ms` và `udp_gap_ms` để phân
  biệt model inference, HOLD và dropout cảm biến.

UI co-carry vẽ riêng bốn tín hiệu trên cùng hệ `base_link`: raw SVGP nominal,
nominal đã filter/bound, reference gửi robot và actual robot EE. Nhãn `Model`
hiển thị `svgp`, `hold` hoặc `mjm`; không được diễn giải đường Reference–Actual
bám sát là độ chính xác dự đoán SVGP.

Không thay các trường lực robot bằng joint position/velocity/effort. Co-drawing
nếu dùng lại phải ghi riêng trong `codrawing_logs/`.

## 10. Safety và lỗi cần nhớ

- Người vận hành phải sẵn sàng E-stop và đặt speed override thấp cho lần chạy
  đầu sau mỗi thay đổi.
- Streamer dùng 15 Hz, previous joint seed, soft joint limits, workspace và
  `--fail-closed`.
- Profile mô phỏng co-carry đặt giới hạn R/B/T `0.08 rad/s` qua launch argument
  `wrist_joint_velocity_limit`; J1/J2/J3 vẫn là `0.20 rad/s`. Profile robot thật
  không truyền override này nên R/B/T vẫn giữ giới hạn mặc định `0.08 rad/s`.
- `No kinematics solver` hoặc IK fail liên tiếp có thể khiến target không được
  chấp nhận. Không được gửi nghiệm clamp tùy tiện; kiểm tra frame, orientation,
  reachability, seed và joint limits.
- Tracking error hiện dừng an toàn khi actual EE lệch accepted queued EE quá
  `0.050 m` trong 5 lần kiểm tra sau grace period 1 s. Một lỗi đã thấy chủ yếu
  đến từ trục Z. Không tăng threshold để che lỗi trước khi tìm nguyên nhân.
- Workspace clamp không đảm bảo pose đạt được: orientation cố định và joint
  limits vẫn có thể khiến IK vô nghiệm.
- Khi force timeout, pose timeout, mất readiness, quá lực hoặc mất calibration,
  controller phải fault và disable; không bypass watchdog để tiếp tục chạy.
- Khoảng mất lực `0.20--0.50 s` chỉ được phép giữ actual EE và đóng băng
  Admittance. Khi dữ liệu trở lại phải realign state tại feedback hiện tại;
  trên `0.50 s` vẫn fault. Trong LEADER, tín hiệu `/cocarry/control_hold` còn
  phải tạm dừng chỉ số MJM để quỹ đạo không chạy ngầm trong lúc giữ pose.
- Streamer chỉ cập nhật pose Cartesian nội bộ sau khi nghiệm IK đã hợp lệ. Nếu
  một bước IK fail, phải giữ pose hợp lệ trước đó; không nội suy tiếp từ pose ảo.

## 11. Checklist khi bắt đầu cuộc trò chuyện mới

1. Đọc `CODEX.md` này.
2. Chạy `git status --short`; xác định thay đổi sẵn có của người dùng.
3. Xác định pipeline được hỏi: camera cũ, co-drawing hay co-carrying 3D.
4. Đọc launch, config và node đúng pipeline; không chỉ dựa vào README.
5. Xác nhận đang làm simulation hay robot thật, domain 42 hay 10.
6. Nếu liên quan lực, xác nhận cách lắp tool, payload, góc bù và trạng thái Axia
   chưa thay đổi.
7. Nếu liên quan predictor, xác nhận model directory thực sự được launch sử
   dụng; không nhầm artifact mới với default.
8. Trình bày nguyên nhân/kế hoạch trước thay đổi có rủi ro và kiểm thử theo mức
   độ: unit -> load/inference -> simulation -> robot thật.
