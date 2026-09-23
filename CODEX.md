# CODEX.md — Quy ước làm việc cho dự án CoCarry

> Cập nhật: 2026-09-23 · Workspace: `/home/hungnb/cocarry_ws` · Máy điều khiển
> Ubuntu, user `hungnb` (không dùng đường dẫn `/home/duy`).
>
> Các mục 1–11 là **trạng thái hiện hành**, đã đối chiếu với code/launch ngày
> 23/09. Mục 12 là nhật ký quyết định theo ngày. Bản nguyên văn trước lần dọn
> này ở `docs/codex_archive_20260923_vi.md` (chỉ để tra lịch sử, nhiều giá trị
> trong đó đã lỗi thời).

## 0. Việc đang dở — đọc trước tiên

1. **Hybrid: "phải bấm LEADER ngay sau Target"** — đồng nghiệp báo, chưa tái
   hiện được, chưa có log nút *Hybrid* nào. Việc tiếp theo và cách đọc kết quả
   ở mục 12, ngày 23/09.
2. **LEADER bị từ chối im lặng** khi cầu nối không khả thi (~0,2% thời điểm).
   Chưa sửa. Mục 12, 23/09.
3. **Bấm FOLLOWER giữa chừng MJM** — chưa có dữ liệu kiểm chứng.
4. **Goal Classification và conflict/role arbitration tự động** — chưa triển
   khai (mục 3). Phải trình kế hoạch trước khi làm.
5. Tùy chọn: giới hạn mức reference LEADER tiến lên mỗi tick sau một lần tick
   bị treo (bảo vệ lớp hai cho lỗi đã sửa 23/09).

## 1. Mục đích của tài liệu

File này là tài liệu khởi đầu cho một cuộc trò chuyện mới. Trước khi phân tích
hoặc sửa dự án, đọc mục 0–11 rồi kiểm tra code/config hiện tại để xác nhận các
giá trị chưa thay đổi.

Thứ tự ưu tiên khi có mâu thuẫn:

1. Yêu cầu mới nhất, rõ ràng của người dùng.
2. Code, launch và config thực sự đang được chạy trong workspace.
3. File `CODEX.md` này.
4. `docs/`, `ARCHITECTURE.md`, `README.md` và lịch sử Git.

`README.md` và `ARCHITECTURE.md` còn mô tả pipeline camera cũ; không dùng
chúng để ghi đè hành vi thể hiện trong code hiện tại.

## 2. Cách làm việc bắt buộc

- Luôn chạy `git status --short` trước khi sửa. Worktree thường có nhiều thay
  đổi chưa commit của người dùng; không reset, checkout, xóa hoặc ghi đè thay
  đổi không thuộc nhiệm vụ.
- Với thay đổi ảnh hưởng điều khiển robot, đọc code liên quan, giải thích kế
  hoạch và chỉ thực thi khi người dùng đồng ý.
- Không tự ý bật Servo, Enable Robot, Start Run hoặc gửi lệnh chuyển động tới
  robot thật. Người vận hành trực tiếp làm các thao tác này.
- Thử logic thuần và mô phỏng trước robot thật khi có thể — nhưng nhớ giới hạn
  của mô phỏng ở mục 10.
- Không nới workspace, joint limits, force limits, timeout, tracking-error
  threshold hoặc tắt `--fail-closed` nếu chưa giải thích rủi ro và được người
  dùng xác nhận.
- Không đổi frame, chiều trục, dấu lực, pose Home, tool offset, khối lượng tải
  hoặc góc gá cảm biến chỉ dựa trên suy đoán. Thiếu dữ liệu thì hỏi.
- **Đổi một biến mỗi lần** khi thử trên robot thật. So sánh deadband ngày 21/09
  đã vô giá trị vì trùng với một biến gây nhiễu khác (mục 12, 22/09).
- Sau khi sửa Python: `python3 -m py_compile <file>`. Sau khi sửa package ROS:
  chạy test phù hợp và `colcon build --symlink-install --packages-select ...`.
  Module Python trong `install/` có thể là bản copy; phải build lại mới có hiệu
  lực, rồi relaunch.
- Dọn tiến trình mô phỏng bằng cách giết đúng **process group** đã tạo. **Không**
  dùng `pkill -f` theo tên node: launch thật dùng cùng tên node và cùng máy.
- Không commit hoặc push nếu người dùng chưa yêu cầu.
- Log và model là dữ liệu thực nghiệm: không xóa, đổi tên hoặc ghi đè. Model mới
  lưu vào thư mục mới và có metadata.
- Script Python thao tác trực tiếp đặt trong `scripts/`; mã package ROS trong
  `src/`. Test tự động trong `tests/` hoặc `<package>/test/`. Không thêm bản nháp
  kiểu `test_ik2.py`, `refactor_*.py` ở root; thử nghiệm ngắn đã được thay bằng
  implementation/test thì xóa sau khi đối chiếu.

## 3. Trạng thái hiện tại

Mục tiêu: **human–robot co-carrying trong không gian 3D bằng Admittance
Control**, dùng lực ATI Axia và chuỗi vị trí End-Effector của robot. Pipeline
này không dùng camera để điều khiển.

Ba pipeline phải giữ độc lập, không chạy đồng thời hai pipeline cùng publish
`/cartesian_streamer/target_pose`:

1. `cocarry_admittance_control` — Axia + robot EE + 3D. **Pipeline hiện tại.**
2. `hrc_bringup` — camera RealSense/Kinect cũ; giữ nguyên để dùng lại.
3. `codrawing_control` — co-drawing XY; không phải mục tiêu chính nhưng không
   được xóa hoặc trộn vào co-carrying.

**Đã triển khai và đã chạy trên robot thật:**

- FOLLOWER: nominal từ GRU (mặc định) hoặc SVGP + lực người qua Admittance.
- Ground Truth: nominal cố định tại pose capture lúc Start Run.
- LEADER: MJM do controller sở hữu, bỏ qua Admittance; vào LEADER qua cầu nối
  C2/C1 (`soft_handoff.Bridge`); tới đích thì tự về FOLLOWER qua warmup 10 mẫu
  và blend 0,6 s. Đã chạy nhiều lần qua nút *GRU+MJM Test*.
- Căn pha nhịp controller với luồng prediction (33,3 ms), ngày 21/09.
- Cấu hình vận hành đã xác nhận ngày 22/09: nhanh hơn 37%, không ripple trong
  169 s buông tay (p ≈ 2,5%). Chi tiết mục 12.

**Đã triển khai, chưa có log robot thật:** nút *Hybrid* (chọn Target, người
quyết định lúc nào LEADER). Đã kiểm thử trên lớp controller production bằng
harness test, không phải trên robot.

**Chưa triển khai:** Goal Classification, conflict/role arbitration tự động,
horizon chính thức. Chỉ số bất đồng **phải** định nghĩa là lực người so với
chuyển động robot *định* thực hiện (vận tốc reference, hướng GRU/MJM), không phải
lực so lực — `F_robot` không khả thi trên cơ cấu hiện tại (mục 12, 19–21/09).

## 4. Kiến trúc co-carrying 3D

```text
/joint_states
  -> ee_tracker_node.py (FK, lấy mẫu đều 15 Hz)
  -> /hand_position, source=robot_ee
  -> trajectory_predictor (GRU mặc định)
  -> /ml/predicted_position = x_d tương đối so với pose capture

Axia -> UDP 50000 -> scripts/axia_sensor_ui.py
  (median3_adaptive -> bù trọng lực -> radial deadband 1,5 N)
  -> /axia/human_force trong base_link

x_d + F_h -> cocarry_admittance_controller (15 Hz)
  nominal: capture_ee + prediction -> PredictionReference (tau, lead) -> kẹp 50 mm
  FOLLOWER: x_r = nominal + e,  M ë + D ė + K e = F_h
  LEADER:   x_r = x_MJM (bỏ qua Admittance)
  -> kẹp command lead -> /cartesian_streamer/target_pose
  -> cartesian_streamer_hc10dtp.py (IK + QueueTrajPoint) -> HC10DTP
```

Robot chỉ nhận target Cartesian position; streamer giải IK. Không gửi Cartesian
velocity và không có `command_force`.

Các file chính: `config/cocarry_admittance_params.yaml`; trong
`cocarry_admittance_control/`: `admittance.py`, `admittance_controller.py`,
`manual_hybrid.py`, `soft_handoff.py`, `prediction_reference.py`; hai launch
`cocarry_admittance_{real,sim}_gui.launch.py`; trong `hc10dtp_bringup/scripts/`:
`ee_tracker_node.py`, `cartesian_streamer_hc10dtp.py`; predictor
`trajectory_predictor/{predictor_node,inference_worker,inference_schedule}.py`;
UI `predictor_ui/ui_node.py`; lực `scripts/axia_sensor_ui.py`.

### Tham số hiệu lực

**Launch override YAML.** Giá trị trong YAML chỉ là nền; robot thật và mô phỏng
dùng giá trị do launch truyền vào. Kiểm tra bằng
`ros2 launch cocarry_admittance_control cocarry_admittance_real_gui.launch.py --show-args`.

| Tham số | Robot thật | Mô phỏng | YAML nền |
|---|---|---|---|
| `M` / `K` / `D` | 1 / 5 / 4,472 (critical) | như thật | như thật |
| Nhịp điều khiển | 15 Hz | 15 Hz | 15 Hz |
| `command_lead_m` | **0,055** | 0,04 | 0,03 |
| `max_tracking_error_m` (streamer) | **0,065** | 0,050 | code: 0,050 |
| `cartesian_velocity_mps` | **0,25** | 0,18 | 0,15 |
| `cartesian_acceleration_mps2` | **1,00** | 0,65 | 0,50 |
| Cartesian jerk (streamer) | 10 m/s³ | 10 (mặc định code) | — |
| `joint_velocity_limit` J1/J2/J3/J5/J6 | **0,60 rad/s** | 0,60 | — |
| J4/R | 0,08 rad/s | 0,08 | — |
| `prediction_reference_tau_sec` | 0,5 | 0,5 | 0,5 |
| `prediction_reference_lead_sec` | 0,15 (trần 20 mm) | 0,15 | — |
| `prediction_max_nominal_lead_m` | 0,05 | 0,05 | 0,05 |
| `prediction_phase_align_sec` | 0,0333 | 0,0333 | 0,0333 |
| Prebuffer queue | 3 | 3 (mặc định code) | — |

`cartesian_velocity_mps` điều khiển đồng thời `--max-vel` của streamer và
`max_virtual_velocity_mps` của controller; tương tự cho acceleration.
`command_lead_m` và `max_tracking_error_m` là **một cặp an toàn** (mục 10).

### Workspace và tool

EE trong `base_link`: `X ∈ [-1,4; 1,4]`, `Y ∈ [-0,5; 1,3]`, `Z ∈ [0,0314; 1,5]`
m. Tool hiện là **handle ngang**; công thức cộng chiều dài thanh dọc
(`0,05 + 0,1814 = 0,2314`) đã bỏ và không còn dùng trong controller. Z là bậc tự
do điều khiển (không `--lock-z`). Orientation chụp tại Start Run và giữ cố định.
Control, prediction, UI, Capture Target và log đều dùng tọa độ robot EE, không
cộng/trừ tool offset.

### Chế độ trên UI

Chọn trước Start Run; đổi mode khi PREPARING/RUNNING gây safety fault.

| Nút (profile co-carry) | Profile | Hành vi |
|---|---|---|
| **Ground Truth** (mặc định) | `ground_truth` | `x_d` cố định tại pose capture; nhả lực thì `e → 0` và robot về pose đó. "Home" của trial là pose capture, không phải joint Home của nút Go Home |
| **GRU** (hoặc **SVGP**) | `svgp` | FOLLOWER thuần: `x_d = capture_ee + prediction`. Nominal chạy cả khi lực bằng 0 để giảm effort; lực người luôn qua Admittance nên người vẫn kéo lệch được |
| **Hybrid** | `svgp_mjm` | Như GRU, cộng chọn Target 1/2 khi RUNNING và bấm LEADER khi người muốn |
| **GRU+MJM Test** | `svgp_mjm_test` | Target phải chọn **trước** Start Run; controller **tự bật LEADER sau 5 s**, một chặng, khóa đổi Target khi đang chạy |

Nhãn nút đổi theo backend (`prediction_model:=svgp` hiện `SVGP`,
`SVGP+MJM Test`).

### FOLLOWER / LEADER

- **FOLLOWER:** `robot_ee history -> predictor -> nominal x_d`;
  `F_h -> Admittance`; gửi `x_r = x_d + e`.
- **LEADER:** MJM từ trạng thái robot lúc chuyển tới Target đã chọn; gửi
  `x_r = x_MJM`, **không cộng `e`**. Lực người không sinh chuyển động nhưng các
  watchdog lực/pose/workspace/lead/streamer vẫn chạy.
- **FOLLOWER → LEADER:** `Bridge()` dựng cầu nối từ `(p, v, a)` đã phát sang
  chặng MJM; ưu tiên C2, nếu gia tốc đầu không khả thi thì C1 giữ p/v; không tìm
  được thì từ chối. `bridge_continuity` ghi lựa chọn.
- **LEADER → FOLLOWER:** tự động khi tới đích (cách đích ≤ 1 cm, tốc độ
  < 0,02 m/s, giữ 0,5 s; quá thời lượng MJM + 5 s mà chưa tới thì fault) hoặc khi
  bấm FOLLOWER. Reset Admittance tại pose hiện tại, chờ **10 mẫu** prediction
  mới (`FOLLOWER_FORCE_WARMUP`, predictor đã tạm dừng trong LEADER và reset
  window theo token), rồi **blend 0,6 s** (`FOLLOWER_REENTRY_BLEND`) về nominal
  mới. Anti-windup trong blend hiệu chỉnh cả hai nhánh Admittance.
- Target lưu dạng tuyệt đối trong `base_link` tại
  `config/hybrid_targets_domain_<id>.json`; chỉ Capture/Reset khi stopped, Reset
  có xác nhận. Tới đích hoặc bấm FOLLOWER **không** tự đổi Target.
- Predictor co-carry đặt `mjm.manual_control=true`: controller sở hữu MJM và vai
  trò; timer `mjm.t_switch` cũ của predictor không dùng cho co-carry.
- `control_phase` phân biệt `FOLLOWER_PREDICTION`, `FOLLOWER_FORCE_WARMUP`,
  `FOLLOWER_REENTRY_BLEND`, `LEADER_BRIDGE`, `LEADER_MJM` và force HOLD.
- MJM hai điểm nội suy `p0 + h(s)(p1 − p0)`: **đường trong không gian là đoạn
  thẳng**; dạng S là tiến độ theo thời gian. Đường cong cần planner khác.

Thiết kế đích còn lại (chưa làm): LEADER lấy đích từ Goal Classification thay vì
Target chọn tay, và trả quyền về FOLLOWER theo tiêu chí conflict. Sơ đồ tham khảo
`/home/hungnb/simulation_hri/{images/architecture.png,outer_loop.py,inner_loop.py}`
là mô phỏng cũ, không sao chép nguyên trạng: thay `X_h` camera bằng chuỗi
`robot_ee`; robot thật không có khối PD Cartesian như `inner_loop.py`; horizon
Goal Classification phải xác nhận lại.

### Nominal và các khâu điều hòa

- Predictor do controller sở hữu Start/Stop; chọn nút trên UI không tự chạy
  worker. Mỗi Start/Stop reset buffer, filter, velocity, tăng prediction epoch.
- `hold.enabled=false` cho co-carry: EE đứng yên không ép predictor sang HOLD;
  nhả lực không có nghĩa là muốn robot dừng. Force-stale HOLD ở controller vẫn
  bật. Camera profile giữ HOLD và `trajectory_mode_auto_toggle=true`.
- `PredictionReference`: lọc bậc một nominal với `tau` rồi bù lead
  `(lead_sec + tuổi mẫu) × vận tốc`, kẹp 20 mm, rồi mới áp kẹp 50 mm quanh EE.
  Ground Truth và LEADER bỏ qua khâu này. `tau` là nút chỉnh ripple chính: tau
  0,2 làm reference rung gấp đôi (mục 12, 22/09).
- Căn pha: mỗi lượt, trong PREPARING, timer controller được đặt lại để tick rơi
  **33,3 ms** sau lúc prediction tới (`prediction_age_ms` ≈ 33–35 ms thay vì rơi
  ngẫu nhiên trong [0; 66,7] ms theo lần launch). Tắt bằng
  `prediction_phase_align_enabled:=false` để đối chứng.
- Streamer: `--continuous-cartesian-smoothing` (tích phân vận tốc qua đoạn đảo
  chiều, không snap tới target gần), `joint_coordination:=synchronized` (các khớp
  cùng tiến theo một tỉ lệ giới hạn vận tốc; một khớp chạm trần thì throttle cả
  tay).
- Soft deadzone Fz: ramp 0 → 2 N **chỉ khi** nhận diện chuyển động X−, rồi ramp
  về 0 sau hysteresis/dwell; Z thuần giữ nguyên độ nhạy. Đặt
  `additional_z_deadzone_n: 0.0` để tắt. `intent_threshold_n` (radial deadzone
  thứ hai ở controller) đang `0.0` — nếu bật sẽ cộng dồn với deadband Axia.

## 5. Force Sensor và calibration

- Chỉ dùng `Fx, Fy, Fz`; torque không dùng cho control hoặc UI.
- Tool: **handle ngang**. Payload bù trọng lực `PAYLOAD_MASS_KG = 1.126 kg`, fit
  từ `handle_mass_check_20260908_{164919,165059,165215}` (hai cặp pose
  1,122/1,131 kg; residual 0,42–0,51 N). Không tự đổi góc gá, Tool Data Yaskawa,
  workspace. Sau khi khởi động lại Axia UI phải calibrate bias không tải.
- Góc bù gá: Roll X `0°`, Pitch Y `0°`, Yaw Z `-90°`.
- **Deadband: radial `1,5 N`** (`DEADBAND_N` trong `scripts/axia_sensor_ui.py`),
  dùng chung cho cả ba pipeline. Đây là deadband **kiểu trừ**:
  `F_out = F · (|F| − dz) / |F|`, nên hạ ngưỡng vừa giảm mức lực bắt đầu có tác
  dụng vừa **cộng thêm `(4 − dz)` N độ lợi** so với mốc 4 N cũ ở mọi mức lực vượt
  ngưỡng. Các audit trước 21/09 giả định 4 N. So sánh 4 / 2,5 / 1,5 N ngày 21/09
  bị nhiễu bởi `prediction_age` và không dùng được (mục 12).
- **Bộ lọc runtime: `median3_adaptive`** (median-3 + cutoff thích nghi 1,5–15 Hz,
  reset khi NaN, timestamp không tăng hoặc UDP gap > 0,2 s), đặt trước bù trọng
  lực. Không còn lựa chọn bộ lọc trên UI; lớp `ForceFilter` median-5 + EMA chỉ
  giữ cho replay/test. `/axia/raw_wrench` và giới hạn lực an toàn không đổi.
- **Calib Mode** chỉ đặt deadband về 0 N để quan sát hướng; không thay cho bấm
  `Calibrate F/T Sensor`.
- UI chỉ vẽ `F_human` XYZ ở 20 Hz; ROS vẫn publish theo tốc độ UDP.

Quy trình calibrate:

1. Có `/joint_states` và TF `base_link <-> axia_sensor_link`.
2. Robot ở pose sẽ vận hành, handle lắp đúng, không chạm, không treo lực ngoài.
3. Xác nhận Roll/Pitch/Yaw; đổi góc nào thì calibration cũ mất hiệu lực.
4. Bấm `Calibrate F/T Sensor`: thu 100 mẫu, tính bias sau khi trừ trọng lực theo
   pose hiện tại.
5. Kiểm tra không tải và sau khi nhả. Residual lớn thì kiểm tra gá, cáp,
   UDP/EtherCAT, calibrate lại — không che bằng cách tăng deadband. Với deadband
   1,5 N, residual 0,42–0,51 N còn biên khoảng 3 lần; nếu robot trôi khi không
   chạm thì calibrate lại.
6. Test X±/Y±/Z± ở lực nhỏ; chiều dương `F_human` khớp trục robot. Với Yaw −90°:
   X+ → Fx dương, Y+ → Fy dương, Z+ → Fz dương.

Controller từ chối Start Run nếu Axia chưa calibrated hoặc lực/pose không còn
fresh. Start Run tự capture pose/orientation, calibrate gốc robot EE, reset
predictor rồi mới vào RUNNING. Chỉ tare lại khi handle ổn định và thật sự không
chịu lực; tare lúc còn preload sẽ xóa một lực thật và làm sai zero lượt sau.

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

### Mapping cổng trên máy `hungnb` đã xác minh 2026-09-09

Khi cắm robot và Axia qua hub hiện tại, hai USB-Ethernet được nhận như sau:

- `enxec9a0c1fc063` — Naxiang SZNX LAN 100M (`35b5:3500`), profile
  `robot-link`, IP robot-side `192.168.1.100/24`, robot YRC1000 là
  `192.168.1.53`. Không dùng cổng này cho Axia.
- `enxf8e43b7aeaf2` — ASIX AX88179 (`0b95:1790`), cổng EtherCAT của Axia.
  Cổng này phải có `carrier=1`; không cần đặt IP cho EtherCAT.

Tên `enx...` có thể đổi nếu thay hub/cổng USB. Luôn xác minh bằng
`udevadm info`/`lsusb` trước khi chạy driver, không chỉ sao chép tên interface
cũ.

Nếu nối Force Sensor trực tiếp vào chính máy `hungnb` (không qua PC 2), chạy
trong terminal người dùng:

```bash
cd ~/cocarry_ws
./run_sensor_driver.sh --iface enxf8e43b7aeaf2
```

Lệnh này cần mật khẩu `sudo` vì pysoem mở raw EtherCAT socket. Driver sẽ vào
SAFEOP/OP, đọc hệ số counts/F và counts/T, hardware-tare lúc khởi động rồi
phát UDP tới `127.0.0.1:50000`. Trong lúc driver khởi động tuyệt đối không chạm
handle. Giữ terminal này chạy; mở terminal khác để launch `real_gui` hoặc
`sim_gui`, sau đó kiểm tra `/axia/connected`, `/axia/calibrated` và
`/axia/human_force`. Chỉ chạy một driver giữ port EtherCAT/UDP tại một thời
điểm. Dừng bằng `Ctrl+C`; script sẽ đóng master và hạ interface Axia.

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
scp ~/cocarry_ws/scripts/axia_sensor_driver.py \
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

### Robot thật

Terminal 1 — micro-ROS:

```bash
cd ~/cocarry_ws && ./start_microros.sh
```

Terminal 2 — launch co-carry (domain 10):

```bash
cd ~/cocarry_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch cocarry_admittance_control cocarry_admittance_real_gui.launch.py
```

Driver Axia: chạy trên PC 2 (mục 6), **hoặc** trên chính máy `hungnb` ở
terminal 3 bằng `./run_sensor_driver.sh --iface enxf8e43b7aeaf2` (script trỏ tới
`scripts/axia_sensor_driver.py`). Chỉ một driver giữ port EtherCAT/UDP tại một
thời điểm.

Launch arg hay dùng (mặc định = cấu hình vận hành ở mục 4, không truyền thì
không đổi gì): `prediction_model:=gru|svgp`, `logging_profile:=compact|diagnostic|calibration`,
`prediction_reference_tau_sec`, `command_lead_m` + `max_tracking_error_m`
(luôn đi cùng nhau), `cartesian_velocity_mps`, `cartesian_acceleration_mps2`,
`joint_velocity_limit`, `use_rviz`.

Chỉ kiểm tra joint states/TF/force/UI, **không** chạy controller và streamer:

```bash
ros2 launch cocarry_admittance_control cocarry_admittance_real_gui.launch.py \
  test_mode:=true use_rviz:=false
```

Có **hai thứ tên "test" khác nhau**: launch arg `test_mode:=true` tắt
controller/streamer; còn nút UI *GRU+MJM Test* là chế độ tự bật LEADER sau 5 s.
Đừng nhầm.

Launch đã tạo `/joint_states` qua bringup robot; không mở thêm terminal MoveIt.

### Mô phỏng (Axia thật + robot ảo)

Không chạy `start_microros.sh`, không bật Servo robot thật.

```bash
cd ~/cocarry_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch cocarry_admittance_control cocarry_admittance_sim_gui.launch.py
```

Launch tự đặt domain 42; terminal chẩn đoán riêng phải `export ROS_DOMAIN_ID=42`.
Mặc định Ground Truth, backend GRU. Muốn SVGP thì thêm `prediction_model:=svgp`.
Launch sim có thêm `launch_sensor_ui`, `launch_dashboard`, `use_rviz` (tắt cả ba
để chạy không giao diện), và cùng bộ arg tốc độ/lead/tracking với mặc định riêng
của sim. **Nếu session robot thật đang chạy thì phải `launch_sensor_ui:=false`**,
vì Axia UI giữ UDP 50000 bất kể ROS domain.

Trình tự UI an toàn: chờ joint states/TF → calibrate Axia không tải → Enable
Robot → chờ streamer ready → chọn mode → Start Run → Stop Run → Disable Robot.

## 8. Predictor robot EE

Hai launch hỗ trợ `prediction_model:=svgp|gru`; **GRU là mặc định**. Mỗi backend
có profile riêng về model directory, window, số feature và cách tính velocity.
Override bằng `svgp_model_dir:=...`, `gru_model_dir:=...` hoặc `model_dir:=...`.

**Dữ liệu train đến từ pipeline camera cũ.** Cả GRU và SVGP chỉ học từ log nhãn
`GROUND_TRUTH` ngày 16/06 — đó là chế độ điều khiển bằng camera, **không** phải
chế độ Ground Truth hiện tại của pipeline Axia. So sánh vùng tọa độ train với vùng
robot chạy hiện nay vì vậy là bằng chứng yếu hơn vẻ ngoài: khác cả cảm biến lẫn
tác vụ. Mỗi quỹ đạo train chỉ dài 7,7–15,6 s và 0,78–1,29 m.

### GRU joint XYZ (mặc định)

`/home/hungnb/cocarry_ws/pHRI_Models/gru_robot_ee_h5_relative_v1`, chuyển sang
HDF5 tương thích TensorFlow 2.15. Một GRU cho XYZ, window 20, horizon 5, input
`[x,y,z,dx,dy,dz]` ở 15 Hz; `dx,dy,dz` là displacement giữa hai mẫu liên tiếp
(không chia 16, không EMA); mẫu đầu dùng velocity 0. Xuất raw prediction, không
qua filter đầu ra; điều hòa nằm ở controller (mục 4). Runtime dùng
`gru_joint_Ts5_float16.tflite` (float16 weights, float32 I/O); HDF5 giữ làm đối
chứng. Test RMSE X/Y/Z `2,929 / 7,814 / 5,525 mm`, mean 3D `8,532 mm`, p95
`18,727 mm`. Inference ~0,48 ms.

### SVGP

`/home/hungnb/cocarry_ws/pHRI_Models/svgp_robot_ee_h5_m100_relative` (train
05/09). Source `robot_ee_x/y/z`, resample 15 Hz, trừ pose EE đầu tiên; window 10,
horizon 5 (~0,333 s); Matern52, 100 inducing points; train 80 file / 13 017
window, val 13, test 7; best epoch 128. Test RMSE X/Y/Z `3,16 / 8,42 / 4,91 mm`,
mean 3D `8,50 mm`. Runtime `svgp_model.npz` qua `SVGPNumpyRunner` (khớp GPflow
sai khác < 5e-13, ~0,1 ms); `svgp_model.pkl` là fallback. Script train:
`/home/hungnb/simulation_hri/train_svgp_pHRI.py`; dataset
`/home/hungnb/simulation_hri/cocarry_logs`.

`pHRI_Models/` bị Git ignore — phải sao lưu riêng khi chuyển máy.

## 9. Logging và công cụ phân tích

Robot thật ghi vào `~/cocarry_ws/cocarry_logs`, mô phỏng vào
`~/cocarry_ws/cocarry_logs/simulation`, tiền tố `cocarry_admittance_3d_`.

| Profile | Nội dung |
|---|---|
| `compact` (mặc định) | Một CSV 30 cột: EE thực, prediction đã lọc, nominal, reference `x_r`, lực Axia trước/sau deadzone, trọng số deadzone Z, model, role, trạng thái/pha, `status_reason`, bốn tuổi dữ liệu |
| `diagnostic` | CSV 61 cột: thêm joint position/velocity, raw prediction, `inference_ms`, `motion_diagnostics_json` (queue send/ack/due, tracking error, joint_scale, IK), event JSON |
| `calibration` | Như trên, thêm sidecar `.calibration.jsonl` raw Axia/M310 |

- **Tracking error, dữ liệu khớp và IK chỉ có ở `diagnostic`.** Mọi thử nghiệm
  liên quan tốc độ, lead hoặc giới hạn khớp phải bật profile này.
- **Output của node không được lưu.** `~/.ros/log/<launch>/launch.log` không
  chứa log của controller (kiểm tra 23/09: không một dòng nào); cảnh báo như
  `rejected: ...` chỉ còn lại trong cột `status_reason` của CSV. Muốn giữ thì
  chuyển hướng output terminal ra file.
- Nhãn `status_reason` hay gặp: `start_run`, `leader_queued`, `manual_leader`,
  `test_timer_leader` (chỉ gán khi ở nút *GRU+MJM Test*), `reached`,
  `rejected: ...`, `fault: ...`.
- UI co-carry vẽ **hai** đường trên `base_link`: `Limited x_d` (từ
  `/cocarry/nominal_position`) và `Actual EE`, trục X là thời gian thật kể từ
  Start Run, mỗi trial tự xóa buffer. Đường Reference–Actual bám sát **không**
  nói lên độ chính xác dự đoán.
- Raw CSV/JSONL là dữ liệu thí nghiệm, không xóa hoặc ghi đè. Co-drawing ghi
  riêng vào `codrawing_logs/`.

Công cụ phân tích trong `scripts/`:

- `report_ripple_episodes.py [tiền tố]` — số **đợt** dao động trên thời gian
  buông tay (không đếm lần đảo chiều, vì một đợt nhả nhiều lần đảo chiều tại
  chỗ), cộng `refAcc` cho rung liên tục. Mức nền lịch sử: 1 đợt / 46 s buông
  tay; Ground Truth: 0. Cần ≥ 100 s buông tay để kết luận.
- `compare_axia_deadband_ab.py` — so sánh theo nhóm, tính sai phân riêng trong
  từng đoạn liên tục để tránh gia tốc giả khi lọc theo pha.

Khi so sánh cấu hình, luôn **chuẩn hóa theo tốc độ** (gia tốc/v², jerk/v³) và đo
tốc độ **lúc lead bão hòa** — tốc độ median lẫn thời gian nghỉ của người.

## 10. Safety và giới hạn

- Người vận hành sẵn sàng E-stop và đặt speed override thấp cho lượt đầu sau mỗi
  thay đổi.
- Khi force timeout, pose timeout, mất readiness, quá lực hoặc mất calibration,
  controller phải fault và disable; không bypass watchdog.

### Giới hạn khớp (streamer)

| Khớp | Soft limit (rad) | IK dùng tới (trừ margin 3°) | Vận tốc thật | Guard liên tục IK |
|---|---|---|---|---|
| J1/S | 0,00 … 3,14 | 0,052 … 3,088 | 0,60 rad/s | 0,07 rad/tick |
| J2/L | −0,80 … 1,50 | −0,748 … **1,448** | 0,60 | 0,07 |
| J3/U | −2,00 … 1,50 | −1,948 … **1,448** | 0,60 | 0,07 |
| J4/R | −2,50 … 2,50 | −2,448 … 2,448 | 0,08 | 0,09 |
| J5/B | −2,09 … 0,52 | −2,038 … 0,468 | 0,60 | 0,09 |
| J6/T | −2,50 … 2,50 | −2,448 … 2,448 | 0,60 | 0,09 |

- J2/J3 trên nâng lên 1,50 rad ngày 15/09 để phủ hai target thật (J3 cần
  ~1,068/1,141 rad); với margin 3° IK dùng tới **1,448 rad (~82,9°)**. Vẫn theo dõi
  nhánh khuỷu, clearance và tracking.
- **Guard liên tục IK** `MAX_JOINT_DELTA_PER_AXIS` = 0,07 rad/tick (1,05 rad/s ở
  15 Hz) cho J1–J3, 0,09 cho J4–J6. Nó kiểm tra nghiệm IK **thô trước** giới hạn
  vận tốc và **gây SAFETY STOP chứ không cắt tốc độ**. Vận tốc khớp 0,60 rad/s dùng
  ~57% guard ≈ 0,25 m/s Cartesian — hai trần đang khớp nhau; nới vận tốc khớp quá
  ~0,85 rad/s thì cú nhảy nhánh IK dễ chạm guard.
- `joint_coordination:=synchronized`: một khớp chạm trần vận tốc sẽ throttle cả
  tay (`joint_scale` < 1). J3/U là khớp chạm trần trước tiên.
- Robot thật từng rung đáng ngại ở B/T 0,12 rad/s trong đợt nâng giới hạn đầu
  tháng 9; mỗi lần nâng giới hạn khớp phải chạy riêng một lượt kiểm tra rung.

### Tracking, lead, watchdog

- **Tracking watchdog:** actual EE lệch pose queue đến hạn (căn theo
  `time_from_start`) quá ngưỡng trong **5 lần liên tiếp** sau grace 1 s → safety
  stop. Ngưỡng thật **0,065 m**, sim và mặc định code 0,050 m.
- **Tracking tỉ lệ với lead, không với tốc độ:** đo được `≈ 0,70 × lead`, xấu nhất
  `≈ 0,92 × lead`. Vì vậy `command_lead_m` và `max_tracking_error_m` **không bao
  giờ nới riêng lẻ**; `test_phase_alignment.py` khóa ràng buộc ngưỡng ≥ 1,15 ×
  0,92 × lead và ≤ 1,6 × 0,92 × lead.
- **Lực:** trần 20 N mỗi trục, 30 N chuẩn tổng. Mất lực 0,20–0,50 s chỉ giữ actual
  EE và đóng băng Admittance, khi dữ liệu trở lại thì realign tại feedback hiện
  tại; trên 0,50 s thì fault. Trong LEADER, `/cocarry/control_hold` còn tạm dừng
  chỉ số MJM để quỹ đạo không chạy ngầm.
- **Pose:** giữ reference khi jitter > 0,25 s, fault khi mất > 0,50 s.
- **Mọi watchdog nằm trong `_control_tick`.** Một tick bị treo nghĩa là không
  watchdog nào chạy; tick chuyển FOLLOWER → LEADER từng treo tới 434 ms (87%
  ngưỡng 0,50 s) trước khi `Bridge()` được tối ưu ngày 23/09. Không đưa tính
  toán nặng vào tick.
- Workspace clamp không bảo đảm pose đạt được: orientation cố định và joint limits
  vẫn có thể khiến IK vô nghiệm. `No kinematics solver` hoặc IK fail liên tiếp
  (3 lần) thì dừng; không gửi nghiệm clamp tùy tiện. Streamer chỉ cập nhật pose
  Cartesian nội bộ khi nghiệm IK hợp lệ.
- Đẩy liên tục một hướng sẽ dồn robot tới góc workspace nơi J3 chạm giới hạn và
  IK fail; tốc độ càng cao càng tới nhanh.

### Giới hạn của mô phỏng

Robot mock bám lệnh **hoàn hảo** (tracking ~6 mm bất kể lead). Vì vậy mô phỏng
**không** kiểm chứng được tracking error, trần tốc độ thật hay độ mượt; nó chỉ
dùng được cho logic, IK/giới hạn khớp và sức khỏe queue. Ngày 22/09 cả cấu hình
đang chạy tốt trên robot thật cũng bị `IK branch jump` trong sim.

## 11. Checklist khi bắt đầu cuộc trò chuyện mới

1. Đọc mục 0–11 của `CODEX.md`.
2. `git status --short`; xác định thay đổi sẵn có của người dùng.
3. Xác định pipeline: camera cũ, co-drawing hay co-carrying 3D.
4. Đọc launch, config và node đúng pipeline; không chỉ dựa vào README.
   `--show-args` để xem giá trị launch thực sự dùng.
5. Xác nhận simulation hay robot thật, domain 42 hay 10, và session robot thật có
   đang chạy không trước khi khởi động hay dọn bất kỳ tiến trình nào.
6. Liên quan lực: xác nhận tool, payload, góc bù, deadband và trạng thái Axia.
7. Liên quan predictor: xác nhận model directory mà launch thực sự dùng.
8. Trình bày nguyên nhân/kế hoạch trước thay đổi có rủi ro; kiểm thử theo mức
   unit → load/inference → simulation → robot thật.

## 12. Nhật ký quyết định và kết quả thực nghiệm

Mới nhất trước. Mỗi mục ghi quyết định, số liệu làm căn cứ và tài liệu
chi tiết. Trạng thái hiện hành luôn nằm ở mục 0–11; nếu mâu thuẫn thì mục
0–11 đúng.

### 23/09 — UI replay lực Axia từ CSV

- Tạo `scripts/axia_force_replay_ui.py` để xem lại lực đã ghi, chạy bằng
  `python3 scripts/axia_force_replay_ui.py <trial.csv>`. Với log co-carry,
  chương trình tự lấy `f_human_x/y/z` (N) và `ros_timestamp_ns`; CSV khác có
  thể khai báo cột và đơn vị thời gian qua CLI. Replay dùng khoảng cách
  timestamp thực, giữ cả khoảng trống và mẫu trùng timestamp; phát mặc định
  1×, hỗ trợ tạm dừng, tua, phát lại và chỉnh tốc độ 0,1–10×.
- Đây là công cụ xem log offline: không mở UDP, không publish ROS, không tare,
  lọc hay bù trọng lực lại. CSV trial lấy mẫu theo controller khoảng 15 Hz,
  nên không thể tái tạo luồng Axia gốc khoảng 100 Hz nếu log không lưu nó.

### Hybrid: "phải bấm LEADER ngay sau Target" — CHƯA tái hiện được (23/09)

**Thiết kế mong muốn:** bấm Target 1/2 để robot biết trước đích, rồi *người*
quyết định lúc nào bấm LEADER (ý nghĩa conflict/confidence-aware). Đồng nghiệp
báo: phải bấm LEADER ngay sau Target thì mới vào LEADER được. **Chưa giải
quyết, chưa có log Hybrid nào để đối chiếu.**

Hai nút trên UI co-carry (profile không camera), đừng nhầm:

| Nhãn UI | trajectory profile | Hành vi |
|---|---|---|
| **Hybrid** | `svgp_mjm`, `test_mode=False` | Chọn Target khi RUNNING; LEADER khi người bấm; đổi Target được sau mỗi chặng |
| **GRU+MJM Test** | `svgp_mjm_test`, `test_mode=True` | Target phải chọn **trước** Start Run; **tự bật LEADER sau 5 s**; khóa đổi Target khi đang chạy; một chặng |

Đã kiểm tra:

- **Code controller không có timeout cho lựa chọn Target.** `ManualHybrid.select`
  chỉ bị xóa bởi `reset_run` (Start Run, Stop, fault, đổi mode). Cổng UI
  `manual_leader_rejection` chỉ đòi đủ hai Target và đã chọn.
- **Mô phỏng trên lớp controller production** (harness của
  `test_manual_hybrid_controller.py`): chọn Target 1 → đợi 0/5/10/30 s → bấm
  LEADER → vào LEADER ở mọi trường hợp. Đã thêm
  `test_manual_leader_accepted_long_after_target_selection` để khóa hành vi này.
- **Mọi lần vào LEADER trong log 21–22/09 (29 lần) đều là `test_timer_leader`**,
  nhãn chỉ được gán khi `test_mode=True`, tức toàn bộ đến từ nút *GRU+MJM Test*.
  Không có một lượt *Hybrid* nào trong dữ liệu, nên báo cáo của đồng nghiệp
  **chưa được đối chiếu với log thật**.

Một lỗi thật, hiếm, có thể liên quan: nếu bấm LEADER đúng lúc robot đang tăng
tốc mạnh, `Bridge()` không tìm được cầu nối, `ValueError` bị bắt trong
`_activate_manual_leader_on_tick`, `_manual_leader_pending` đã về `False` nên
**yêu cầu bị bỏ im lặng**; chỉ còn `status_reason = 'rejected: No bounded soft
bridge; slow down before requesting LEADER'`. Phát lại 44 870 trạng thái
FOLLOWER thật qua `ManualHybrid.lead()`: bị từ chối **0,2%**, chủ yếu khi
reference > 0,25 m/s; probe cũng bắt được ở pha tăng tốc từ đứng yên. Chưa sửa;
hai hướng là giữ yêu cầu ~1 s và thử lại mỗi tick, hoặc báo rõ trên UI. Lưu ý
chế độ Test cố ý ghi "one attempt only, no surprise retry".

**Việc tiếp theo:** chạy một lượt nút **Hybrid** với
`logging_profile:=diagnostic`: chọn Target, mang vật đi >10 s, rồi bấm LEADER.
Nếu không vào LEADER, xem `status_reason` quanh thời điểm bấm: `rejected: ...`
là lỗi Bridge ở trên; `leader_queued` mà không chuyển thành LEADER là một lỗi
khác chưa biết; không thấy `leader_queued` thì lệnh không tới controller (xem
UI). Chiều ngược lại — bấm FOLLOWER giữa chừng MJM — cũng chưa có dữ liệu.

### Cú giật khi chuyển FOLLOWER → LEADER — đã sửa 23/09

`soft_handoff.Bridge()` chạy **đồng bộ bên trong `_control_tick`** lúc bấm
LEADER, là tìm kiếm lưới 224 ứng viên × 401 điểm (chạy lại lần nữa nếu phải rơi
xuống C1). Đo trên 29 lần chuyển ngày 21–22/09: **26/29 tick chuyển bị treo**,
median 107,6 ms so với 66,7 ms, tệ nhất 434 ms (87% ngưỡng hard-fault lực
500 ms). Sau khi treo, rclpy bắn bù tick ngay (0,5–33 ms sau) và reference cầu
nối — tính theo đồng hồ thật — nhảy vọt qua cả khoảng bị lỡ: lượt `180537`
đo được v_ref 3,35 m/s trong 0,8 ms. Quay màn hình chỉ khuếch đại (tranh CPU),
không phải nguyên nhân gốc.

Đã tối ưu `_fit` bằng cơ sở đa thức tính sẵn và nhân ma trận, **không đổi thuật
toán**: `Bridge()` từ median 36,2 / p95 64,3 / max 70,7 ms xuống
**10,7 / 16,6 / 20,2 ms**. `test_soft_handoff.py` giữ nguyên văn bản gốc làm
oracle và khẳng định kết quả trùng khít trên 200 đầu vào ngẫu nhiên phủ cả C2,
C1 và FAIL. Chưa làm: giới hạn mức reference LEADER tiến lên mỗi tick sau một
lần treo (bảo vệ lớp hai, thay đổi hành vi thật nên để riêng).

Chiều LEADER → FOLLOWER **không** bị treo (26/26 tick ≤ 69,4 ms). Có một cú
cắt vận tốc reference trong pha warmup khi người đang đẩy lúc chuyển, nhưng
đó là hành vi chung của việc tăng tốc từ đứng yên vào trần command-lead, gặp cả
trong FOLLOWER thường (1094 lần khởi động: a_ref đỉnh median 1,72, p90 2,92
m/s²), không phải lỗi riêng của chuyển pha. Cả 26 lần đều là tự thoát khi tới
target; **chưa có dữ liệu cho trường hợp bấm FOLLOWER giữa chừng**.

### Thử nới trần tốc độ chiều 22/09 — không có lợi, giữ cấu hình lead 55

Launch thật có thêm năm arg (mặc định = cấu hình đang dùng, không truyền thì
không đổi gì): `command_lead_m` 0.055, `max_tracking_error_m` 0.065,
`cartesian_velocity_mps` 0.25, `cartesian_acceleration_mps2` 1.00,
`joint_velocity_limit` 0.60 (J1/J2/J3/J5/J6; J4 cố định 0.08). Launch sim có
cùng các arg với mặc định riêng của sim.

Kết quả trên robot thật, tốc độ đo **chỉ lúc lead đang bão hòa** (lúc người
thật sự đẩy) để khỏi lẫn thời gian nghỉ:

| Cấu hình | v khi bão hòa | gia tốc/v² |
|---|---|---|
| **A: lead 55, vel .25, khớp .60, tau .5** | **0,133 m/s** | **17,1** |
| chỉ vel .33 + khớp .80 | 0,122 | 19,1 |
| lead 70 + trk 80 + vel .33 + khớp .80, tau .5 | 0,130–0,135 | 18,6–19,3 |
| như trên + tau .8 | 0,134 | 21,3 |

**Tốc độ duy trì không tăng** dù lead 55 → 70; mô hình `v ≈ lead/0,3 s` gãy
trong khoảng này. Đỉnh thì có tăng (v_max 0,209 → 0,266), nên lý giải hợp lý
nhất là **ở lead 55 người đã là thứ đặt nhịp**. Độ mượt chuẩn hóa xấu đi ở mọi
cấu hình mới; tau 0,8 giảm độ rung reference (4,07–4,76 → 3,58 trên v) đúng như
người vận hành cảm nhận, nhưng không về được mức 3,05 của A. Tracking p95 ở lead
70 là 58,7–66,0 mm (biên 17,5% so với 22% của A). **Kết luận: dùng mặc định A.**

Trần cứng nếu sau này vẫn muốn nhanh hơn: `MAX_JOINT_DELTA_PER_AXIS =
[0.07, 0.07, 0.07, 0.09, 0.09, 0.09]` rad/tick trong
`cartesian_streamer_hc10dtp.py` (= 1,05 rad/s cho J1–J3 ở 15 Hz). Guard này
kiểm tra nghiệm IK **thô, trước** giới hạn vận tốc, và **gây SAFETY STOP chứ
không cắt tốc độ**. Khớp 0,60 rad/s ≈ 57% guard ≈ 0,25 m/s Cartesian — hai trần
hiện đang khớp nhau, nâng một mình không có tác dụng.

Mô phỏng **không** trả lời được câu hỏi trần tốc độ: robot mock bám hoàn hảo
(tracking ~6 mm bất kể lead) và cả cấu hình A — vốn chạy tốt trên robot thật —
cũng bị `IK branch jump` trong sim với kịch bản đẩy một hướng liên tục. Dọn tiến
trình sim phải giết theo process group; **không** dùng `pkill -f` theo tên node
vì launch thật dùng cùng tên node.

### Cấu hình vận hành mới ngày 22/09 — lead 55 mm, tracking 65 mm

Mặc định của `cocarry_admittance_real_gui.launch.py` đổi từ `0.04/0.050` sang
`command_lead_m:=0.055` và `max_tracking_error_m:=0.065`. Đo trên lượt `121935`
(82 s, profile diagnostic):

| | lead 40 / ngưỡng 50 | lead 55 / ngưỡng 65 |
|---|---|---|
| Tốc độ EE median | 0,095 m/s | **0,130 m/s** |
| Tracking median | 28,3 mm | 38,5 mm |
| Tracking p95 | 35,1 mm | 44,3 mm |
| Tracking max | 45,0 mm | 50,6 mm |
| Biên tới ngưỡng | 5,0 mm (10%) | **14,4 mm (22%)** |

Ba điều phải nhớ:

- **Lead quyết định tốc độ**, không phải `max_virtual_velocity` hay
  `prediction_reference_tau`. Quan hệ đo được là `v ≈ lead / 0,3 s`, và trần
  0,095 m/s suốt hai ngày trước đó chính là do lead 40 mm.
- **Tracking error tỉ lệ với lead, không với tốc độ**: đo được
  `tracking ≈ 0,70 × lead`, xấu nhất `0,92 × lead`. Vì vậy hai tham số này là
  **một cặp an toàn** và không bao giờ được nới riêng lẻ. Có test khóa ràng
  buộc này trong `test_phase_alignment.py`.
- Gia tốc và jerk **chuẩn hóa theo tốc độ giảm** (gia tốc/v² từ 32–39 xuống
  23–26; jerk/v³ từ 8 800–11 800 xuống 4 500–5 400). Robot nhanh hơn mà đường
  đi sạch hơn, không phải đánh đổi.

**Ripple đã hết, và lần này có ý nghĩa thống kê.** Tám lượt ở cấu hình mới
(12:12–12:19 và 16:09–16:24) cho **169 giây buông tay, 0 đợt dao động**. Mức nền
lịch sử 1 đợt/46 giây kỳ vọng 3,7 đợt, nên xác suất được 0 một cách tình cờ chỉ
khoảng **2,5%**. Lưu ý diễn giải: so với mức nền đó thì cấu hình khác nhiều thứ
cùng lúc (deadband 1,5 N, tau 0,5, căn pha prediction, lead 55), nên **không
quy công cho riêng lead được**.

Nút thắt tiếp theo nếu muốn nhanh hơn, theo đúng thứ tự chạm trần (đo trên lượt
`121935`, v = 0,130 m/s, **khi giới hạn khớp còn 0,50 rad/s, J6 0,40**; nay là
0,60):

| Khớp | Trần | median | p95 | p99 | max |
|---|---|---|---|---|---|
| J3/U | 0,50 | 0,27 | **0,69** | **0,79** | **0,97** |
| J6/T | 0,40 | 0,21 | 0,45 | 0,48 | 0,51 |
| J5/B | 0,50 | 0,14 | 0,40 | 0,53 | 0,68 |
| J2/L | 0,50 | 0,22 | 0,39 | 0,44 | 0,51 |
| J1/S | 0,50 | 0,17 | 0,36 | 0,39 | 0,40 |
| J4/R | 0,08 | 0,00 | 0,04 | 0,07 | 0,07 |

**J3/U là khớp duy nhất sắp chạm trần** (p99 0,79, max 0,97); `joint_scale` đã
bắt đầu throttle ở 0,2% số tick, thấp nhất 0,929. Các khớp còn lại đều rộng
rãi. Vì vậy nới lead thêm sẽ đụng J3 trước, rồi tới
`max_virtual_velocity_mps = 0.25`, chứ không phải đụng ngưỡng tracking.

`--max-tracking-error` giờ là CLI arg của streamer (mặc định trong code vẫn
`0.050`); giá trị vận hành do launch truyền vào.

### 21–22/09 — Ripple: chuỗi nguyên nhân đã tìm ra

Triệu chứng: các **đợt** dao động ~1 Hz, mỗi đợt ~4 s, gần như chỉ khi người buông
tay (81% số lần đảo chiều xảy ra lúc lực bằng 0, mật độ cao gấp 9 lần).

Đã loại trừ, có số liệu: GRU chạy ngoài vùng train (tương quan yếu), bão hòa
command lead (tương quan ≈ 0), giới hạn vận tốc khớp (không khớp nào chạm trần ở
0,095 m/s), hàng đợi streamer (queue lag 21–39 ms, không xu hướng), lực quán tính
payload (~0,1 N), điều kiện động học, và vị trí trong workspace (đếm theo đợt thì
không tập trung — đếm theo lần đảo chiều từng cho một bản đồ giả).

Hai phát hiện thật:

1. **`prediction_age` là xổ số theo lần launch.** Tracker, predictor và
   controller là ba lưới 15 Hz độc lập nên tuổi mẫu là một hằng số ngẫu nhiên
   trong [0; 66,7] ms, chốt lúc launch. Ở ~64 ms ripple nhiều hơn, ở ~5 ms thì
   16% tick dùng lại mẫu cũ. Đã khóa về 33,3 ms (mục 4). **Nhưng khóa pha không
   hết ripple**: sau đó vẫn có lượt sạch và lượt ripple trong cùng một launch.
2. **Bộ dao động là vòng GRU ↔ robot.** GRU ăn lại vị trí do chính nó điều khiển;
   cánh tay người là bộ giảm chấn. Ground Truth (không có GRU trong vòng): 176 s
   buông tay, **0 đợt**. GRU: **1 đợt / 46 s buông tay**, xác suất tăng theo độ dài
   đoạn buông tay (≥ 8 s: 45%). Đây là một quá trình ngẫu nhiên, không phải trạng
   thái hỏng — nên nó có cảm giác như xổ số.

`prediction_reference_tau` là lever đúng: tau 0,2 làm `refAcc` 0,76 so với
0,36–0,40 ở tau 0,4–0,8, đúng chiều mô hình độ lợi vòng dự đoán. Kết quả cuối ở
cấu hình vận hành hiện tại: **169 s buông tay, 0 đợt** (p ≈ 2,5%).

Ghi chú:

- "Phương án A" (bù lead theo tuổi mẫu trong `PredictionReference`) vẫn nằm trong
  code. Nó **không** sửa được ripple (nó chữa sai số bám, không chữa biên pha),
  nhưng không gây hại; chưa đánh giá riêng.
- So sánh deadband 4 / 2,5 / 1,5 N ngày 21/09 **bị nhiễu hoàn toàn**: nhóm 4 N
  tình cờ rơi `prediction_age` 5,4 ms, nhóm 1,5 N rơi 53–62 ms. Đừng dùng lại kết
  luận "1,5 N gây dao động". `docs/axia_deadband_ab_protocol_20260921_vi.md` là
  protocol viết trước khi phát hiện nhiễu.
- Kế hoạch và thiết kế khóa pha: `docs/prediction_phase_alignment_plan_20260921_vi.md`.

### 19–21/09 — `F_robot`: kết luận không khả thi trên cơ cấu hiện tại

- Handle bắt cứng vào flange **xuyên qua chính Axia** (plate → Axia → spacer →
  thanh). Ở một mặt cắt cứng chỉ có một lực tương tác; theo định luật III "lực
  robot" bằng âm "lực người", nên mọi cách tính đều sụp về `cos ≈ 1`.
- M310–M315 / M320–M325 là **ước lượng ngoại lực** — cùng đại lượng Axia đo —
  chỉ ~4,8–4,9 Hz so với Axia 100 Hz, sáu kênh lệch nhau ~150 ms. Bỏ khỏi đường
  đo lực, dùng Axia. Hai trial LEADER 21/09 (`104650`, `104823`), người cố ý đẩy
  ngược robot: `cos(F_register, F_Axia)` âm 0/39 mẫu, trong khi
  `cos(v_EE, F_Axia)` âm 33/39.
- `joint_states.effort` là servo torque feedback theo **Nm** (MotoROS2 nhân 1e-6,
  không scale thêm theo rated). Nó chứa trọng lực, quán tính, Coriolis, ma sát và
  ngoại lực — không phải `F_robot`. `effort = 0` khi Servo tắt, không phải lỗi.
- `sensorless_force_node` (M310) **vẫn được launch thật khởi chạy** ở chế độ
  shadow: `calibration_confirmed=false`, `role_valid=false`, không dùng cho điều
  khiển. Candidate calibration T1 (`cocarry_logs/hc_force_calibration/20260919_t1_combined_v3/candidate_frozen.json`)
  chưa bao giờ deploy và giờ không còn lý do deploy.
- Tool Data Yaskawa khai sai (W = 2,350 kg, CoG ≈ 0 trong khi thanh vươn 160 mm);
  chỉ còn ảnh hưởng safety/PFL. **Không tự sửa.**
- Muốn chỉ số `φ` có nghĩa như mô phỏng `simulation_hri/inner_loop.py` thì phải đổi
  cơ cấu sang một vật được cùng nâng, không bắt cứng vào flange — quyết định phần
  cứng.
- Tài liệu: `docs/f_robot_source_reassessment_20260919_vi.md`,
  `docs/f_robot_mregister_external_evidence_20260921_vi.md`,
  `docs/hc_force_t1_calibration_result_20260919_vi.md`,
  `docs/hc_force_p0_unified_calibration_20260919_vi.md`,
  `docs/hc_force_baseline_audit_20260918_vi.md`,
  `docs/m310_reader_rate_audit_20260919_vi.md`. Runbook thu dữ liệu calibration
  (lệnh `hc_force_trial_logger.py`): `docs/hc_force_calibration_session_runbook_vi.md`.

### 19/09 — Bộ lọc Axia (P1) và băng thông pipeline (P2)

- P1: benchmark 8 bộ lọc causal trên 105 389 mẫu raw; chọn `median3_adaptive` và
  tích hợp trước bù trọng lực. RMS lúc nghỉ 0,0172 → 0,0094 N; t90 bước 20 → 40 ms
  (EMA 0,1 cũ mất ~230 ms). Loại tốt xung một mẫu, còn lọt phần lớn burst 2–3 mẫu.
  `docs/axia_filter_p1_benchmark_20260919_vi.md`.
- P2: prebuffer 2 so với 3 không cải thiện tốc độ (vẫn ~0,095 m/s); giữ 3. Trần
  0,095 m/s khi đó hóa ra là do command lead 40 mm (phát hiện 22/09).
  `docs/p2_pipeline_bandwidth_audit_20260919_vi.md`,
  `docs/cocarry_progress_next_steps_20260918_vi.md`.

### 15/09 — Phối hợp khớp

`joint_coordination:=synchronized`; smoother cập nhật theo FK/vận tốc đã ACK;
tracking nội suy tuyến tính giữa các FK endpoint; sửa đồng hồ queue khi BUSY/retry
(không rollback thời gian điểm đang pending); thêm `motion_diagnostics_json`; nâng
J2/J3 trên lên 1,50 rad. Thuật toán này không bảo đảm giới hạn gia tốc/jerk riêng
từng khớp. `docs/motion_coordination_20260915_vi.md`.

### 10–11/09 — Hybrid thủ công ban đầu

Chọn Target 1/2 độc lập với LEADER/FOLLOWER, cầu nối C2/C1, warmup/blend khi quay
về FOLLOWER, chặn NaN/Inf ở đầu vào và reference, `control_phase`, soft deadzone Fz
cho X−. Tất cả đã gộp vào mục 4. `docs/manual_hybrid_test_20260910_vi.md`.

### Trước 10/09

Các profile tốc độ khớp đầu tháng 9 (`[0.30,0.30,0.35,0.08,0.30,0.25]`, Cartesian
0,15–0,22 m/s, lead 0,03–0,05), tool thanh dọc với payload tạm 3,856 kg, thiết lập
hybrid theo timer `mjm.t_switch` — **đều đã bị thay thế**. Tra trong
`docs/codex_archive_20260923_vi.md` nếu cần.
