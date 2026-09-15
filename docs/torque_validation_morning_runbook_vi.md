# Hướng dẫn buổi đo torque validation tại một pose

Áp dụng cho code kiểm tra ngày 07/09/2026, HC10DTP + YRC1000micro,
MotoROS2 v0.2.1, controller YBS3.05.00A, Axia trên máy thứ hai.

Mục tiêu buổi đầu: ghi đồng thời effort sáu khớp và lực/moment Axia để
kiểm tra tín hiệu khi có tác động và khi nhả tay. Chưa tạo hệ số calibration
và chưa đưa F_robot vào chọn FOLLOWER/LEADER.

**Robot giữ nguyên pose trong mỗi lượt ghi. X+/X−/Y+/Y− là hướng lực tay
tác dụng lên handle, không phải lệnh cho robot chạy theo các trục.**

## 1. Chuẩn bị trước khi mở terminal

- Dành khoảng 30–45 phút cho kết nối, kiểm tra và hai lượt đo ngắn.
- Dùng topology hai máy đã hoạt động: robot nối máy 1 (`hungnb`);
  Axia nối cổng EtherCAT máy 2 (`binhdangnguyen`); hai máy truyền UDP qua
  LAN đã thiết lập. Nếu LAN chưa hoạt động, xác minh địa chỉ trước khi chạy.
  Đo sớm giúp giảm một số nguồn nhiễu nhưng không thay thế kiểm tra dữ liệu.
- Robot và thanh/gá/sensor giữ cách lắp hiện tại. Không tự đổi tool offset,
  payload, chiều trục, torque scale hoặc các giới hạn để thực hiện bài đo.
- Chọn trước một pose quen thuộc, có khoảng trống thao tác, tránh chạm bàn,
  tránh căng cáp. Nếu cần di chuyển đến pose này, người vận hành thực hiện
  theo quy trình pendant của lab **trước khi bắt đầu ghi**.
- Chưa bật Servo khi đang chuẩn bị kết nối. Không chạy trial admittance,
  prediction hoặc một launch điều khiển khác đồng thời.
- Chỉ thực hiện tiếp xúc khi quy trình an toàn của lab cho phép; giữ khả năng
  dừng robot. Không chèn/giữ cưỡng bức công tắc enabling trên pendant.
  Nếu không thể vừa giữ enabling đúng quy trình vừa thao tác một mình,
  cần người hỗ trợ thay vì thay đổi chế độ an toàn.

Tên terminal trong tài liệu:

| Máy | Terminal | Chức năng | Giữ chạy? |
| --- | --- | --- | --- |
| Máy 1 | T1 | micro-ROS agent | Có |
| Máy 1 | T2 | ROS/TF và UI Axia ở test mode | Có |
| Máy 1 | T3 | Kiểm tra và gọi bắt đầu/dừng logger | Dùng nhiều lần |
| Máy 1 | T4 | Torque validation logger | Có |
| Máy 2 | S1 | Kiểm tra LAN rồi chạy driver Axia | Có khi driver chạy |

## 2. T3 trên máy 1: xác nhận môi trường và IP

```bash
cd /home/hungnb/cocarry_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10

ip -4 -br addr
ip route
ros2 pkg executables hc10dtp_bringup | rg torque_validation_logger
ros2 pkg prefix industrial_msgs
```

Lệnh executables phải có `torque_validation_logger.py`; industrial_msgs phải
trả về đường dẫn package. Nếu thiếu, dừng để xử lý môi trường/build trước.

Phân biệt ba đường kết nối:

- `192.168.1.100/24` là IP máy 1 trên đường robot theo cấu hình đã dùng;
  robot là `192.168.1.53`.
- `192.168.50.1` là IP máy 1 trên đường LAN hai máy **nếu vẫn được gán**.
  Chỉ dùng nó trong lệnh driver bên dưới khi thấy trong `ip -4 -br addr`.
- Cổng EtherCAT của máy 2 nối Axia không cần một IP để trao đổi EtherCAT.

Kiểm tra robot:

```bash
ping -c 10 -W 1 192.168.1.53
```

Nếu không liên lạc được, kiểm tra nguồn/cáp/interface; chưa chuyển sang đo.
Không tự sửa cấu hình mạng đang hoạt động theo tên hub cũ.

## 3. T1 trên máy 1: micro-ROS agent

Chỉ chạy một agent; nếu agent đã hoạt động thì giữ terminal cũ.

```bash
cd /home/hungnb/cocarry_ws
./start_microros.sh
```

Giữ T1 chạy. Quay lại T3:

```bash
ros2 node list
ros2 topic echo /joint_states --once
```

Phải có `/motoman_ros2` và đủ sáu tên joint từ `joint_1_s` đến `joint_6_t`.
Effort bằng 0 khi Servo OFF không chứng minh lỗi. Không dùng dữ liệu Servo OFF
làm baseline lực của lượt Servo ON.

## 4. T2 trên máy 1: khởi động test mode

```bash
cd /home/hungnb/cocarry_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10

ros2 launch cocarry_admittance_control \
  cocarry_admittance_real_gui.launch.py \
  test_mode:=true \
  use_rviz:=false
```

Giữ T2 chạy. Launch này tạo TF và mở UI Axia; trong code hiện tại test mode
tắt streamer, admittance controller và move_group. Predictor/UI phụ vẫn có
thể xuất hiện. Không bấm Enable Robot, Go Home hoặc Start Run trên UI.

Không chạy thêm `python3 axia_sensor_ui.py`: T2 đã mở nó và chỉ được có một
tiến trình nhận UDP port 50000. Nếu đã mở một phiên cũ, kết thúc phiên cũ theo
đúng quy trình rồi mở lại để có publisher `/axia/raw_wrench` mới.

Tại T3 kiểm tra:

```bash
ros2 node list | rg 'cartesian_streamer|cocarry_admittance_controller|move_group'
```

Với phiên test độc lập, lệnh này không nên có kết quả. Nếu có, tìm và kết thúc
pipeline điều khiển cũ trước khi tiếp tục. Không bật một launch mới để che lỗi.

## 5. S1 trên máy 2: LAN và driver Axia

Không cần source ROS hoặc đặt ROS_DOMAIN_ID trên máy 2 vì driver gửi UDP.

```bash
ip -br link
ip -4 -br addr
ip route get 192.168.50.1
ping -c 20 -W 1 192.168.50.1
```

Các lệnh dùng `192.168.50.1` chỉ áp dụng khi T3 đã xác nhận đó là IP LAN máy 1.
Route phải đi qua cổng Ethernet nối **hai máy**, không qua Wi-Fi hay cổng Axia.
Nếu IP đã đổi, thay đúng IP trong cả ping và `--ip` bên dưới.

Trước khi chạy driver, không chạm handle: driver có hardware tare lúc khởi
động và sau khi khởi tạo lại kết nối.

Lệnh dưới dùng cổng Axia đã xác nhận trước đây `enxf8e43b7aeaf2`.
Chỉ giữ tên này khi nó thực sự là cổng nối Axia trên máy 2:

```bash
sudo /home/binhdangnguyen/axia_driver/.venv/bin/python \
  /home/binhdangnguyen/axia_driver/axia_sensor_driver.py \
  enxf8e43b7aeaf2 --ip 192.168.50.1 --port 50000 --hz 100
```

Giữ S1 chạy. Không chạy thêm driver khác cùng sensor. Chờ driver vào trạng
thái EtherCAT OP và gửi dữ liệu. Nếu lỗi không tìm thấy interface, thiếu
pysoem hoặc không vào OP, xử lý trước; không bắt đầu ghi.

## 6. T3: kiểm tra Axia và TF

Chạy từng lệnh; `hz` và `tf2_echo` chạy liên tục, bấm Ctrl+C để kết thúc riêng
lệnh đó trước khi nhập lệnh kế tiếp.

```bash
ros2 topic echo /axia/connected --once
ros2 topic info /axia/raw_wrench
ros2 topic echo /axia/raw_wrench --once
ros2 topic hz /axia/raw_wrench
```

Kỳ vọng: connected=true, một publisher raw_wrench, có đủ force XYZ và torque
XYZ; tần số gần tốc độ driver 100 Hz. Giá trị rate là kiểm tra sơ bộ, không
chứng minh đồng bộ thời gian hay không mất packet.

```bash
ros2 run tf2_ros tf2_echo base_link axia_sensor_link
```

Chờ TF xuất hiện và ổn định rồi Ctrl+C. Nếu liên tục báo frame không tồn tại,
không chuyển sang thu. Không tự thêm một static TF đoán để bỏ qua lỗi.

## 7. Pendant và UI: chuẩn bị trạng thái đo

1. Đặt robot ở pose đã chọn bằng thao tác pendant được lab cho phép nếu chưa
   thực hiện. Sau đó ngừng mọi lệnh di chuyển.
2. Người vận hành bật Servo ON đúng quy trình. Giữ robot đứng yên; chờ các
   luồng dữ liệu ổn định. Nếu bật Servo gây reconnect, chờ phục hồi hoàn toàn
   trước khi calibrate và thu.
3. Tại T3 kiểm tra:

   ```bash
   ros2 topic echo /robot_status --once
   ros2 topic echo /joint_states --once
   ```

   `drives_powered.val` cần là 1, `in_motion.val` là 0, `in_error.val` là 0.
   `motion_possible=0` không tự động có nghĩa torque feedback hỏng.
   Có đủ effort sáu khớp; không yêu cầu mỗi khớp đều khác 0.
4. Không chạm handle, không để cáp căng kéo nó. Trên UI Axia bấm
   **Calibrate F/T Sensor**, chờ thông báo hoàn tất.
5. Xác nhận tại T3:

   ```bash
   ros2 topic echo /axia/calibrated --once
   ```

   Phải là true. Ghi lại Roll/Pitch/Yaw đang hiển thị; mặc định dự án là
   0°, 0°, −90°. Không đổi góc chỉ để làm đường lực trông hợp lý.

Deadband 4 N không ảnh hưởng raw_wrench và không ảnh hưởng bước lấy bias trước
deadband. Có thể giữ nguyên khi calibrate. Để dễ quan sát mức lực nhỏ và việc
trở về zero, **sau calibration có thể bật Calib Mode** (deadband về 0), ghi
lại lựa chọn này. Calib Mode không phải nút calibrate lại. Giữ filter hiện tại
và ghi lại cấu hình; phân tích validation dùng raw nên không cần đổi filter.
Không suy ra lực thực 5 N từ đường UI đã áp deadband 4 N.

## 8. T4: mở logger, T3: thử ghi 5 giây

T4:

```bash
cd /home/hungnb/cocarry_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10

ros2 run hc10dtp_bringup torque_validation_logger.py \
  --ros-args -p file_prefix:=P0_validation
```

Chờ `Torque validation logger ready`. Node chỉ bắt đầu ghi khi gọi service.
Tại T3, khi không chạm handle:

```bash
ros2 service call /torque_validation_logger/toggle \
  std_srvs/srv/SetBool "{data: true}"
```

Chờ khoảng 5 giây rồi:

```bash
ros2 service call /torque_validation_logger/toggle \
  std_srvs/srv/SetBool "{data: false}"

ls -lt /home/hungnb/cocarry_ws/cocarry_logs/torque_validation/
```

Phải có file mới; T4 báo `Wrote ... rows`. Giữ file thử, ghi chú nó là precheck.
Đây là logger độc lập: nút Start Run hoặc logger trial trên UI không thay thế
service `/torque_validation_logger/toggle`.

## 9. T3: kiểm tra CSV vừa lưu

Chạy đoạn này sau file thử và sau mỗi lượt chính. Nó chỉ đọc file mới nhất
có prefix P0_validation, không sửa dữ liệu:

```bash
python3 - <<'PY'
import csv
import math
from pathlib import Path

folder = Path('/home/hungnb/cocarry_ws/cocarry_logs/torque_validation')
paths = list(folder.glob('P0_validation_*.csv'))
if not paths:
    raise SystemExit('CHUA CO FILE: kiem tra T4 va service toggle.')
path = max(paths, key=lambda p: p.stat().st_mtime_ns)
with path.open() as f:
    rows = list(csv.DictReader(f))
print('File:', path)
print('Rows:', len(rows))
if not rows:
    raise SystemExit('FILE RONG: chua thu duoc joint_states.')
def finite(value):
    try:
        return math.isfinite(float(value))
    except (ValueError, TypeError):
        return False
groups = {
    'joint position': [f'joint_position_j{i}' for i in range(1, 7)],
    'joint effort': [f'joint_effort_j{i}' for i in range(1, 7)],
    'Axia 6D': ['axia_raw_' + k for k in ('fx', 'fy', 'fz', 'tx', 'ty', 'tz')],
    'TF': ['base_to_sensor_' + k for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')],
}
for name, fields in groups.items():
    count = sum(all(finite(r.get(k)) for k in fields) for r in rows)
    print(name, ':', count, '/', len(rows), 'rows finite')
for field in ('axia_raw_fresh', 'axia_connected', 'axia_calibrated'):
    print(field, ':', sum(r.get(field) == 'True' for r in rows), '/', len(rows))
stamps = {r['axia_raw_timestamp_ns'] for r in rows if r['axia_raw_timestamp_ns']}
print('Distinct Axia timestamps:', len(stamps))
PY
```

Kỳ vọng mọi nhóm có dữ liệu hữu hạn, nhiều timestamp Axia khác nhau và các
trạng thái phần lớn/hoàn toàn true sau khi ổn định. Nếu thiếu cả nhóm Axia/TF,
hoặc timestamp Axia đứng im, dừng thu chính để xử lý. Kiểm tra này chỉ xác nhận
thu dữ liệu, chưa xác nhận lực đúng hay các mẫu đồng bộ chính xác.

## 10. Lượt đo chính P0 lần 1

Giữ T1/T2/T4/S1 chạy. Robot ở đúng pose, Servo ON và đứng yên.

1. T3 gọi toggle true như mục 8; đợi phản hồi success và ghi tên file.
2. **15 giây đầu:** không chạm handle. Đây là baseline Servo ON.
3. Tiếp cận handle, tăng lực nhẹ từ từ theo X+, giữ khoảng 2–3 giây.
   Mức khoảng 5 N chỉ là ví dụ lực nhỏ khi phù hợp điều kiện lab; không cố
   tăng lực để ép robot dịch chuyển hoặc để đạt con số trên UI.
4. Nhả hoàn toàn, chờ **5–10 giây**. Quan sát lực có trở về mức trước kéo.
5. Lặp theo X−, Y+, Y−. Z+/Z− chỉ làm nếu thuận tiện và an toàn; không cần
   đủ sáu hướng ngay lần đầu. Hướng chéo vẫn có giá trị vì Axia đo vector thực.
6. **10 giây cuối:** không chạm handle.
7. T3 gọi toggle false; chờ T4 báo đã ghi file và kiểm tra CSV theo mục 9.

Trong lượt: không jog pendant, không bấm Start Run/Enable Robot/Go Home,
không đổi filter/góc gá, không calibrate lại. Không kê/chạm tay vào một bộ phận
khác của robot để tạo điểm tựa vì sẽ xuất hiện lực ngoài không đi qua Axia.

Nếu robot chuyển động rõ rệt, có alarm hoặc không duy trì được trạng thái
Servo theo quy trình, dừng tác động và xử lý trạng thái máy trước. Đánh dấu
lượt này bị gián đoạn; không coi nó là phép đo tĩnh hợp lệ.

## 11. Khi nhả vẫn còn 1–2 N

- Tiếp tục ghi và chờ 5–10 giây; không tare để xóa phần lệch.
- Ghi rõ sau hướng nào xảy ra, mức còn lại và mất bao lâu mới trở về baseline.
- Nếu UI giảm chậm nhưng raw trở về baseline, bộ lọc là một khả năng.
- Nếu raw cũng lệch, cần kiểm tra cáp/gá, thay đổi pose, preload, drift hoặc
  hysteresis. Giữ dữ liệu để phân tích, chưa dùng để xác định scale torque.
- Lệch 1–2 N sau lực thử 5 N là đáng kể. Không tăng deadband để đánh dấu đạt.
- Nếu Axia reconnect/hardware tare trong lượt: dừng ghi, ghi chú sự kiện,
  chờ ổn định rồi calibrate lại và bắt đầu **file mới** từ baseline.

## 12. Lượt P0 lần 2 và ghi chú bắt buộc

Sau khi nhả và ổn định, giữ nguyên pose và cấu hình, lặp mục 10 vào file mới.
Đây là lượt kiểm tra độc lập về khả năng lặp lại tại P0. Chưa cần đổi pose
hay thu hàng chục pose trong buổi đầu. Nếu zero vẫn lệch sau lượt 1, ưu tiên
ghi và tìm nguyên nhân thay vì calibrate lại liên tục để tạo hai lượt đẹp.

Logger hiện chưa có nhãn pha hay metadata góc gá/Servo, nên ghi bằng giấy
hoặc một file ghi chú riêng, mỗi CSV một dòng mô tả:

| Thông tin | Nội dung cần ghi |
| --- | --- |
| Tên CSV | Tên đầy đủ; precheck / P0 lần 1 / P0 lần 2 |
| Pose | P0; sáu joint lưu trong CSV, không cần nhập lại |
| Thứ tự tác động | Ví dụ X+, X−, Y+, Y−; ghi hướng nào bỏ qua |
| Mốc pha | 15 giây đầu không chạm, 10 giây cuối không chạm; ghi mốc khác nếu đổi |
| Axia | Roll/Pitch/Yaw thực tế; Calib Mode ON/OFF; filter đang dùng |
| Cơ khí | Handle/gá giữ nguyên; vị trí tay trên thanh; cáp có căng không |
| Robot | Servo ON giữ suốt lượt hay có gián đoạn; robot có dịch chuyển không |
| Bất thường | Lực còn dư, hướng gây lệch, reconnect, alarm hoặc nhả chưa hoàn toàn |
| Mạng | IP đích UDP, LAN hay Wi-Fi; driver có báo lỗi không |

Phần góc gá đặc biệt quan trọng: raw_wrench chưa áp góc bù gá −90° của UI;
TF logger lưu là TF nominal. Không lấy raw nhân thẳng với TF rồi kết luận
sai chiều torque. Phân tích offline phải dùng đúng góc gá và dịch điểm moment
về cùng điểm quy chiếu với Jacobian.

## 13. Kết thúc đúng thứ tự

1. Nhả handle. T3 gọi toggle false nếu còn ghi; chờ T4 báo `Wrote ... rows`.
2. Kiểm tra file hiện trên ổ đĩa. Logger hiện giữ mẫu trong RAM đến khi dừng;
   không tắt nguồn máy/đóng cưỡng bức tiến trình trước khi lưu.
3. Người vận hành tắt Servo theo quy trình pendant.
4. Ctrl+C ở T4 để thoát logger.
5. Ctrl+C ở T2 để đóng launch/UI.
6. Ctrl+C ở S1 để dừng driver Axia.
7. Ở T1, script `start_microros.sh` cần **Ctrl+C hai lần trong vòng 2 giây**
   để thoát; một lần chỉ restart agent. Chỉ thực hiện sau khi robot đã dừng.

Giữ nguyên mọi CSV, kể cả lượt lỗi. Sau buổi đo, cung cấp tên hai CSV chính
và ghi chú trên để phân tích Δeffort so với JᵀΔwrench. Chưa chuyển
`calibration_confirmed` sang true và chưa sửa scale/dấu lực sau buổi thu này.

## 14. Các giới hạn của bộ thu hiện tại

- Logger ghép wrench nhận gần nhất vào mỗi joint state, không phải đồng bộ
  phần cứng. Timestamp Axia là lúc nhận trên máy 1; gói UDP chưa có timestamp
  gốc từ sensor. TF cũng lấy mẫu mới nhất. Vì vậy buổi đầu dùng pose tĩnh và
  đoạn giữ lực, chưa đánh giá lực trong chuyển động nhanh.
- Raw Axia là trước filter/gravity/deadband của UI, nhưng sau hardware tare
  và quy đổi counts của driver. Nó không phải tín hiệu tuyệt đối trước tare.
- TF nominal, góc bù gá, thứ tự các thành phần wrench và quy ước dấu phải được
  xử lý nhất quán ở bước phân tích.
- Một pose chỉ đủ kiểm tra tại pose đó; không chứng minh calibration đúng
  trên toàn workspace hoặc khi đổi Home.

Nếu cần dừng để khắc phục, hãy ghi lại terminal nào lỗi và thông báo lỗi;
không cần tiếp tục đủ các bước để có một file hình thức.
