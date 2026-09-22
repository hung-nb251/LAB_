# Quy trình session CoG và calibration F_robot

## 1. Mục tiêu và cấu trúc dữ liệu

Mỗi đợt đo dùng một `session` duy nhất. Logger tự tạo cấu trúc:

```text
cocarry_logs/hc_force_calibration/<SESSION>/
├── 01_static_cog/
│   ├── pose_01_home/<timestamp>_cog_p01/
│   ├── pose_02_pitch_pos/<timestamp>_cog_p02/
│   └── ...
└── 02_dynamic_force/
    ├── home/<timestamp>_home_x/
    └── target1/<timestamp>_target1_x/
```

Mỗi trial vẫn có `metadata.json` và `events.jsonl`. Không đổi tên hoặc gom file
sau khi đo. Dùng cùng chính tả `SESSION` trong mọi lệnh.

Ví dụ session:

```bash
export HC_CALIB_SESSION=20260916_tool0_cog_v1
export HC_CALIB_ROOT=/home/hungnb/cocarry_ws/cocarry_logs/hc_force_calibration
```

Hai biến này phải được khai báo lại trong mỗi terminal logger mới. Không dùng
một session cũ sau khi đổi tool, tải, mounting, Torque Origin hoặc Axia tare.

## 2. Điều kiện chung

- Cân và ghi khối lượng tổng của cụm sau flange.
- Giữ Tool 0, mounting, cáp và tải không đổi.
- Tare/calibrate Axia một lần tại Home trước pose đầu; không tare lại từng pose.
- Nếu driver Axia reconnect, dừng và mở session mới vì hardware tare có thể đổi.
- Deadband UI có thể giữ nguyên; phép fit dùng `/axia/raw_wrench` 6D.
- Không có người chạm tool trong bài CoG tĩnh.
- Logger chỉ đọc dữ liệu; người vận hành tự jog/Enable/Start/Stop theo quy trình
  robot thật đang được phép sử dụng.

## 3. Các terminal nền

Terminal 1, micro-ROS:

```bash
cd /home/hungnb/cocarry_ws
./start_microros.sh
```

Terminal 2, pipeline robot thật:

```bash
cd /home/hungnb/cocarry_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch cocarry_admittance_control cocarry_admittance_real_gui.launch.py
```

Terminal 3 chạy duy nhất driver Force Sensor theo cấu hình một máy/hai máy hiện
hành. Không mở thêm `axia_sensor_ui.py`, vì launch ở Terminal 2 đã mở UI này.

Terminal 4 dùng cho `hc_force_trial_logger.py`. Terminal 5 chạy
`hc_force_marker_cli.py` và gửi marker qua `/hc_force_trial/marker`. Logger vẫn
cho phép nhập marker trực tiếp ở Terminal 4 để dự phòng.

## 4. Thu 8–12 pose tĩnh cho CoG

Chọn tối thiểu 8, khuyến nghị 10 pose với orientation khác nhau. Thay đổi roll
và pitch của tool đủ rõ, đồng thời giữ robot xa singularity, joint limit và biên
workspace. Việc chỉ đổi XYZ nhưng giữ nguyên orientation không cung cấp đủ
thông tin trọng lực để nhận dạng CoG.

Gợi ý nhãn, không phải lệnh robot tự động:

```text
pose_01_home
pose_02_pitch_pos
pose_03_pitch_neg
pose_04_roll_pos
pose_05_roll_neg
pose_06_pitch_pos_roll_pos
pose_07_pitch_pos_roll_neg
pose_08_pitch_neg_roll_pos
pose_09_pitch_neg_roll_neg
pose_10_validation
```

Tại mỗi pose, mở một logger riêng. Ví dụ pose đầu:

```bash
cd /home/hungnb/cocarry_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10
export HC_CALIB_SESSION=20260916_tool0_cog_v1
export HC_CALIB_ROOT=/home/hungnb/cocarry_ws/cocarry_logs/hc_force_calibration

python3 scripts/hc_force_trial_logger.py \
  --session "$HC_CALIB_SESSION" \
  --category static_cog \
  --trial cog_p01 \
  --mode static \
  --pose pose_01_home \
  --tool-number 0 \
  --group all \
  --timeout 5.0 \
  --scan-gap 0.10 \
  --output "$HC_CALIB_ROOT" \
  --notes 'mass_total_kg=TODO; one_tare_at_home; CalibMode_OFF; deadband_4N; no_contact'
```

Trong logger:

1. Nhập `settling`, Enter ngay khi robot đã đứng ở pose.
2. Không chạm tool; chờ Axia ổn định 5–10 phút. Logger ghi cả quá trình drift.
3. Nhập `stable`, Enter; giữ thêm 60–120 giây.
4. Nhập `q`, Enter.
5. Đổi `--trial` và `--pose` cho pose tiếp theo; không đổi `--session`.

Không cần chờ bằng mắt rồi mới bật logger. Việc ghi cả pha `settling` cho phép
xác định thời gian hội tụ bằng số liệu thay vì cảm giác.

## 5. Thu lực động theo pose và trục

Thực hiện sau khi fit CoG sơ bộ, cập nhật Tool Data và mở một session calibration
mới cho cấu hình Tool Data đó. Mỗi pose/trục dùng một file riêng để nếu một bài
lỗi thì không làm mất các trục còn lại.

Ví dụ X tại Home:

```bash
python3 scripts/hc_force_trial_logger.py \
  --session "$HC_CALIB_SESSION" \
  --category dynamic_force \
  --trial home_x \
  --mode ground_truth \
  --pose home \
  --tool-number 0 \
  --group all \
  --timeout 5.0 \
  --scan-gap 0.10 \
  --output "$HC_CALIB_ROOT" \
  --notes 'ToolData_updated; same_tare_session; GroundTruth; axis=X; three_repeats'
```

Marker cho X:

```text
pre_start
baseline
xplus_1
release_xplus_1
xminus_1
release_xminus_1
xplus_2
release_xplus_2
xminus_2
release_xminus_2
xplus_3
release_xplus_3
xminus_3
release_xminus_3
baseline_end
stopped
q
```

Ở Terminal 5, sau khi Terminal 4 đã chạy logger:

```bash
cd /home/hungnb/cocarry_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10
python3 scripts/hc_force_marker_cli.py
```

Nhập các marker trên ở Terminal 5. Mỗi marker phải hiện `SENT:` tại Terminal 5
và `MARKER:` tại Terminal 4 rồi mới thực hiện thao tác tương ứng.

Với Y/Z, đổi `--trial`, `axis=` trong notes và dùng `yplus/yminus` hoặc
`zplus/zminus`. Lặp tại `home`, `target1`, và pose validation đã chọn. Mỗi lần
tác động giữ tương đối ổn định 3–5 giây; mỗi release giữ 10–15 giây.

## 6. Kiểm tra ngay sau mỗi trial

Logger in đường dẫn `LOG:` khi bắt đầu và `Saved` khi kết thúc. Kiểm tra session:

```bash
find "$HC_CALIB_ROOT/$HC_CALIB_SESSION" -name metadata.json -printf '%h\n' | sort
```

Kiểm tra timeout trong trial vừa lưu:

```bash
rg '"kind": "register_(timeout|error)"' \
  "$HC_CALIB_ROOT/$HC_CALIB_SESSION"
```

Không xóa trial lỗi. Ghi trial mới với tên khác để giữ truy vết.

## 7. Dữ liệu tối thiểu trước khi fit

- Khối lượng tổng và mô tả thứ tự lắp cơ khí.
- Ít nhất 8 pose tĩnh; mỗi pose có pha `stable` tối thiểu 60 giây.
- Axia raw, joint state và M310–M345 không timeout kéo dài.
- Dynamic X/Y/Z tại Home và Target 1; pose thứ ba giữ để validation.
- Cùng Tool Data/tare trong mỗi session, có ghi rõ mọi lần reconnect.
