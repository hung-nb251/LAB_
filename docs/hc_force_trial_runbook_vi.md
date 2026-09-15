# Thu log lực HC: pilot tĩnh và Ground Truth

## Phạm vi và dữ liệu

Dùng `hc_force_trial_logger.py` ở gốc workspace. Logger chỉ gọi `/read_mregister`
và subscribe; không bật servo, chạy robot, tare hay sửa tham số.
Giữ nguyên URDF +61.2°, Axia offset −90°, Tool 0 và Tool Data hiện tại.
Home/Target 1 đã đủ cho pilot hôm nay; Target 2 là bài mở rộng sau.
Chưa coi kết quả là hiệu chuẩn tuyệt đối: mass/CoG chưa xác nhận, transform
điểm quy chiếu wrench chưa đo đủ. Software tare Axia khác Torque Origin HC.

Mỗi lần chạy tạo thư mục mới trong `cocarry_logs/hc_force_trials/`:
- `metadata.json`: trial/mode/pose, tool do người vận hành khai báo, notes,
  domain và bản sao source/config. Source không chứng minh giá trị UI runtime.
- `events.jsonl`: Axia raw 6D, human force, joints đầy đủ cùng source timestamp,
  TF, connected/calibrated, trạng thái run, marker, từng response thanh ghi.
- M310–315: Nm; M320–322: N; M323–325: Nm theo `(raw-10000)*0.1`.
  M330–345 giữ raw. Response lỗi không có giá trị quy đổi.
- Mỗi response có scan_id, địa chỉ, success/code, thời điểm gửi/nhận và latency.
  Không coi một lượt quét là mẫu đồng thời. Marker giữa scan được giữ cả ở
  thời điểm gửi và nhận. Log flush từng dòng, Ctrl+C/q lưu phần đã thu.

Raw Axia là sáu số UDP trước xử lý UI, nhưng có thể đã được hardware tare
trong driver lúc khởi động/reconnect. Human force có thể chịu deadband UI;
không lấy human_force gần zero làm chứng nhận accuracy. Không so trực tiếp
hai vector ở frame khác nhau. Không thể suy nguyên nhân service treo chỉ từ
việc đọc lại được; logger ghi latency để có bằng chứng.

## Chuẩn bị chung

Dùng phiên MotoROS2/Axia đang chạy, không mở thêm driver/UI trùng. Robot domain 10.
Xác nhận connected/calibrated và TF; sau reconnect phải đánh dấu phiên tare mới.
Ghi notes: thời điểm/pose hardware tare và software tare (không nhớ ghi unknown),
filter, deadband UI thực tế, mass/CoG Tool Data, mounting, điểm người cầm,
servo state, có thay tải hay không. Chụp UI làm bằng chứng runtime.

Terminal mới:
```bash
cd /home/hungnb/cocarry_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10
```

Không chạy vòng service call thủ công cùng logger. Khi logger báo joint_age hoặc
axia_age MISSING/stale, chưa bắt đầu tương tác. `/axia/connected=true` không
thay thế việc kiểm tra dữ liệu thật. Xem health mỗi 5 giây.

## Bài S1: lực tĩnh tại Home

Người vận hành giữ robot ở Home bằng chế độ phù hợp, Servo ON, không có
admittance/Start Run đang hoạt động. Bật logger:
```bash
python3 hc_force_trial_logger.py --trial S1_home --mode static --pose home \
  --tool-number 0 --group all \
  --notes 'before_tool_data_update; mass=2.350kg; CoG=(0,0,0.001)mm; bổ sung tare/filter/deadband thực tế'
```

Trong chính terminal logger, nhập tên pha rồi Enter ngay trước thao tác.
Có thể nhờ người thứ hai đánh dấu để người tác động giữ đúng hướng/điểm tiếp xúc.

| Nhãn pha | Thao tác | Thời lượng |
|---|---|---|
| baseline | Không tiếp xúc, chờ ổn định | 20 s |
| xplus_1 | Tác động nhẹ theo base +X ở điểm cầm cố định | 5 s hoặc dài hơn để đủ ≥3 scan hoàn tất |
| release_plus_1 | Nhả hoàn toàn | 10 s |
| xminus_1 | Tác động nhẹ theo base −X | Như +X |
| release_minus_1 | Nhả hoàn toàn | 10 s |
| xplus_2 … release_minus_3 | Lặp cả chu kỳ thêm hai lần | Như trên |
| baseline_end | Không tiếp xúc | 20 s |

Sau đó nhập `q`. Nếu lực khó giữ ổn định lâu, không ép kéo dài: ghi nhận thiếu
mẫu và ưu tiên quét `--group wrench` trong file thử tiếp theo. Không tự tăng lực
để vượt deadband; raw không có deadband UI. Có thể thêm mức lực thứ hai trong
phạm vi vận hành đã xác nhận, nhãn `level2_xplus_1`… Không có lực chuẩn thì chỉ
gọi là hai mức tương đối, không gán số N theo cảm giác.

Phân tích S1 trước bài động: force/joints fresh; không đổi tare; đủ mẫu từng pha;
Axia và M-register thay đổi lặp lại khi tác động/nhả; joint pose gần tĩnh.
Dùng chênh lệch với baseline cùng pose để khảo sát đáp ứng, không ghi offset vào
controller. Nếu dấu/frame chưa xác định, giữ kết luận ở mức tương quan.
Gửi đường dẫn thư mục S1 để phân tích, không cần chép tay từng thanh ghi.

## Bài G1: Ground Truth X±

Sau S1 nhất quán, dùng pipeline Ground Truth hiện có. Không chạy hai launch điều
khiển. Người vận hành chọn Ground Truth khi stopped, Enable/Start theo quy trình
hiện hành. Logger không làm các thao tác này.

```bash
python3 hc_force_trial_logger.py --trial G1_home_x --mode ground_truth --pose home \
  --tool-number 0 --group wrench \
  --notes 'before_tool_data_update; same tare as S1 nếu đúng; ghi file CSV Ground Truth và cấu hình UI'
```

1. Nhập `pre_start`, ghi 10 s; người vận hành Start Run.
2. Nhập `baseline`, nhả tay 15 s.
3. Nhập `xplus_1`, tác động nhẹ để robot đi +X trong vùng đã chạy, khoảng 3–5 s.
4. Nhập `release_plus_1`, nhả và chờ robot ổn định 10–15 s.
5. Nhập `xminus_1`, thực hiện chiều −X; `release_minus_1`, nhả 10–15 s.
6. Lặp chu kỳ tới lần 3, dùng nhãn tương ứng.
7. Nhập `baseline_end`, ghi 15 s; người vận hành Stop Run, nhập `stopped`, sau 5 s nhập `q`.

Ground Truth có K>0: nhả tay có thể làm robot quay về pose capture tại Start Run;
pose capture không tự động là joint Home. Các pha release cũng là dữ liệu chuyển động.
Giữ log CSV Ground Truth thông thường cùng JSONL này. Nếu cần khảo sát torque
joint/channels khi động, chạy file G2 riêng `--group all` với cùng kịch bản;
chấp nhận rate mỗi kênh thấp hơn, không gọi G1/G2 là đồng thời.

Timeout M-register: logger tự ngừng gửi mới và vẫn ghi topic. Không tiếp tục bài
động chỉ để chờ thanh ghi. Người vận hành dừng trial theo quy trình hiện hành.
Khi response cũ về, nhập `resume` để tiếp tục đo tĩnh trước. Nếu response không
về, ghi lỗi và kết thúc log; không tự restart controller hay reset robot.

## Đánh giá dữ liệu sau bài

Tính latency và khoảng cách mẫu theo từng địa chỉ, số response lỗi/late, độ dài
scan, dropout topic, thay đổi calibrated/connected. Chỉ phân tích động ở thang
thời gian được rate thực tế hỗ trợ; không hứa 100 Hz cho M-register bằng service.
Giữ timestamp từng kênh, loại scan cắt qua marker khỏi trung bình pha tĩnh.
So baseline trước/sau và đáp ứng lực lặp lại. Dữ liệu dùng fit và kiểm chứng phải
khác lần lặp/pose. Mass/CoG và transform chưa đủ thì chưa tuyên bố calibrated.
