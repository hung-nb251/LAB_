# Phân công và hướng dẫn thu F_robot quanh Target 1

Ngày lập: 19/09/2026. Người vận hành: một người tại lab.

## 1. Mục tiêu và trạng thái bản hướng dẫn

Hai nhánh làm song song:

1. **Codex:** calibration F_robot cục bộ quanh Target 1, tận dụng dữ liệu cũ và
   một bài di chuyển tự do có baseline đầu/cuối.
2. **Claude:** cải thiện tốc độ/chất lượng đọc M310–M315, ưu tiên timestamp rõ
   ràng và một vector đủ sáu kênh; mục tiêu ban đầu >=15 vector hợp lệ/s.

Người vận hành chỉ cần thực hiện một lượt phát triển khoảng 2–3 phút và, sau
khi model được đóng băng, một lượt kiểm chứng độc lập tương tự. Đây là lượng
thu khởi đầu, không phải bảo đảm đủ calibration. Nếu cần thêm, chỉ bổ sung
đúng phần thiếu được phân tích chỉ ra. Không yêu cầu thu lại 12 pose hoặc T2.

Phạm vi kết luận là vùng T1 thực sự đã đo, tool/tải/gá tương ứng. M310 là
estimated external joint torque; khớp Axia chưa chứng minh được lực chủ động
hay ý định độc lập của robot. Trong đợt này F_robot tiếp tục shadow,
`calibration_confirmed=false`, `role_valid=false`.

**Tài liệu này là phân công và runbook, không phải biên bản đã kiểm thử hoặc
đã triển khai bộ đọc/model mới.** Các lệnh dưới dựa trên source hiện tại.
Trước buổi thu, Codex đối chiếu source/install thực dùng, kiểm tra logger và
chốt cấu hình với Claude. Source hiện có cả candidate smoother P2 chưa được
xác nhận trên robot thật; cần ghi rõ phiên bản điều khiển được dùng và hoàn
tất bước kiểm tra cần thiết trước khi coi lượt thu là dữ liệu chuẩn.

Tham chiếu: [P0](hc_force_p0_unified_calibration_20260919_vi.md),
[baseline audit](hc_force_baseline_audit_20260918_vi.md),
[runbook session cũ](hc_force_calibration_session_runbook_vi.md),
[bàn giao P2](p2_handoff_next_session_vi.md), [CODEX.md](../CODEX.md).

## 2. Phân công và thứ tự bàn giao

| Bên | Công việc trước khi thu | Công việc sau khi thu | Sản phẩm bàn giao |
|---|---|---|---|
| Codex | Kiểm tra chuỗi tính lực/frame/baseline; đánh giá dữ liệu cũ quanh T1; kiểm tra logger tích hợp và sidecar; chuẩn bị parser cho dữ liệu mới | Kiểm tra chất lượng ngay sau lượt; fit/validation theo trial/session; đóng băng model; đánh giá test mới và replay runtime | Báo cáo calibration T1, manifest, metric, model có version và phạm vi áp dụng |
| Claude | Đo reader hiện tại khi Stop/Disable; tìm source controller; benchmark scan gap; chuẩn bị candidate scalar/batch/topic và test lỗi | Đánh giá rate/jitter/timeout khi thu; review độc lập protocol và kết quả calibration | Báo cáo tốc độ, candidate reader, schema/timestamp, cấu hình đã đo và cách quay về reader cũ |
| Người vận hành | Xác nhận cấu hình cơ khí/mạng, khởi động terminal, calibrate Axia không tải, đưa robot đến vùng T1 bằng quy trình sẵn có | Thực hiện bài ngắn, Stop/Disable, xác nhận hai đoạn không tiếp xúc, gửi đường dẫn log và ghi chú | Một lượt development; sau đó một lượt test độc lập |

Thứ tự: Codex và Claude chuẩn bị song song -> thống nhất reader/schema và
cấu hình thu -> lượt development -> kiểm tra/fit/review -> đóng băng model và
tiêu chí -> lượt test mới -> quyết định có candidate shadow tốt hơn hay không.

Nếu batch cần source controller chưa có, Claude báo giới hạn scalar đã đo;
Codex vẫn có thể dùng bài chậm/có dừng với reader hiện tại. Không đổi reader
hoặc filter giữa một lượt. Nếu test dùng cấu hình khác, phải đánh giá ảnh
hưởng thay đổi đó, không gộp như điều kiện giống nhau.

### 2.1. Ranh giới sửa file

- Codex sở hữu parser/audit/calibration, `cocarry_logger.py` và tài liệu thu.
- Claude sở hữu phần acquisition trong `mregister_force_node.py`, benchmark
  reader và source/interface controller khi tìm được. Giữ nguyên phép đổi
  đơn vị, estimator, baseline và interface cũ trừ khi đã phối hợp với Codex.
- YAML/launch/interface JSON là phần giao nhau: Claude đề xuất thay đổi;
  Codex tích hợp sau khi chốt schema. Không cùng sửa một file đồng thời.
- Workspace đang có nhiều thay đổi chưa commit. Mỗi bên kiểm tra `git status
  --short`, lưu diff/hash liên quan; không reset/checkout hay commit thay bên kia.
- Thư mục output mới cho mỗi lần phân tích; không ghi đè log/model cũ.

### 2.2. Yêu cầu kỹ thuật cho Claude

1. Đo vector hoàn chỉnh/s, median/P95 thời gian service, span sáu kênh,
   jitter, timeout, duplicate và freshness joint state. Phân biệt tốc độ
   publish với tốc độ cập nhật giá trị thật ở controller.
2. Benchmark cấu hình hiện tại và candidate giảm scan gap khi Stop/Disable.
   `scan_gap_sec` hiện được đọc vào `_scan_gap` tại khởi tạo; chưa có callback
   cập nhật. Không dùng `ros2 param set` rồi mặc định tốc độ đã thay đổi.
3. Reader hiện tại lưu torque chưa hiệu chỉnh theo Nm, timestamp chung và
   `register_scan_span_ms`; chưa lưu raw integer và timestamp từng register.
   Bổ sung raw integer, địa chỉ, request/response timestamp từng kênh, scan ID,
   backend/version, lỗi đọc và thời gian monotonic cho đo latency. Giữ các
   trường JSON cũ để logger/estimator vẫn dùng được.
4. Tìm source/build custom `/read_mregister`. Nếu có, chuẩn bị batch/topic
   sáu kênh; ghi rõ timestamp controller hay host, quy ước clock và mức đồng
   thời thực. Một response batch không tự chứng minh atomic sampling.
5. Chỉ một reader truy cập register. Không thử song song sáu request hoặc
   chạy benchmark client cạnh node tích hợp. Thiết kế xử lý request trễ,
   timeout và restart không chồng request còn pending.
6. Kiểm tra offline/fake server trước, benchmark controller dừng sau; bàn
   giao lệnh chính xác để chạy candidate và quay về baseline. Chưa tự cài
   firmware/controller package hoặc khởi động lại controller.
7. Mục tiêu >=15 Hz phải đi kèm số đo lỗi/jitter/span và tải hệ thống. Không
   hạ timeout hoặc nới giới hạn điều khiển chỉ để đạt con số tần số.

### 2.3. Yêu cầu kỹ thuật cho Codex

- Kiểm tra Axia raw/processed, TF, bias, payload, filter thực tế; ghép theo
  nguồn thời gian, không tối ưu lag trên test.
- Chuẩn bị parser cho `*.calibration.jsonl`: P0 cũ dùng `events.jsonl`, nên
  không giả định script cũ tự nhận sidecar mới.
- Ghép file chính và baseline cuối bằng mã lượt đo, khử trùng pre-roll,
  giữ marker và báo lỗi/mẫu bị loại. Không suy no-contact từ lực bằng 0.
- Snapshot model, code/config và thiết lập runtime; ghi riêng vùng joint/EE
  thực sự quan sát được, điều kiện Jacobian và độ phủ hướng lực.
- Fit model đơn giản phù hợp dữ liệu cục bộ trước; so runtime cùng reference.
  Baseline cuối chỉ dùng chẩn đoán drift/đánh giá, không làm đầu vào tương lai
  cho dự đoán trước đó hoặc để fit model trên test.
- Báo RMSE vector/từng trục, sai số góc và tỷ lệ sai dấu trên lực đủ lớn,
  tỷ lệ mẫu được đánh giá, residual không tải, latency và INVALID/STALE.
  Chốt metric/ngưỡng trước test; test thất bại rồi dùng để sửa model thì cần
  test mới. Shadow không tự cấp quyền dùng lực cho chọn vai.

## 3. Chuẩn bị tại lab

Giữ topology đã được phép vận hành: máy `hungnb` điều khiển robot, máy cảm
biến gửi Axia qua UDP. Một người tại lab vẫn có thể dùng hai máy. CODEX.md
ghi nhận sự cố khi robot và Axia cùng nối một laptop; tài liệu này không
chuyển topology sang một máy.

Người vận hành xác nhận tool/tải/gá và tare không đổi trong session. Nếu
driver reconnect/tare lại, mở session mới. Ghi speed override trên pendant.
Không đổi Tool Data, Torque Origin, frame, force/workspace/joint limits.
Mọi Enable/Start/jog do người vận hành thực hiện theo quy trình đã có.

Dùng bốn terminal chức năng: **T1, T2, T4 trên máy hungnb; T3 trên máy cảm
biến**. Nếu hệ thống đang chạy đúng cấu hình, giữ terminal hiện có, không
khởi động thêm bản thứ hai. Chỉ dùng domain 10 cho robot thật.

### Terminal 1 — micro-ROS, máy hungnb

```bash
cd /home/hungnb/cocarry_ws
./start_microros.sh
```

Giữ terminal chạy. Chưa Enable/Start.

### Terminal 2 — pipeline và UI, máy hungnb

Chọn mã session mới; ví dụ dưới là session development. Nếu thu ngày khác,
đổi ngày. Không dùng lại mã session khi đã đổi tare/tool/reader.

```bash
cd /home/hungnb/cocarry_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10
export HC_T1_SESSION=20260919_t1_local_dev_v1
ros2 launch cocarry_admittance_control cocarry_admittance_real_gui.launch.py \
  use_rviz:=false \
  log_dir:=/home/hungnb/cocarry_ws/cocarry_logs/hc_force_calibration/$HC_T1_SESSION
```

Launch hiện đã mở Axia UI, logger và reader M310. Không mở thêm
`axia_sensor_ui.py`, `hc_force_trial_logger.py` hay một reader register khác.
Lệnh này dùng reader đã cài hiện tại, không phải lệnh kích hoạt batch tương lai.
Sau bàn giao của Claude, Codex cập nhật đúng lệnh nếu cần launch arg mới.

### Terminal 3 — driver Axia, máy cảm biến

Nếu driver đang ổn định, giữ nguyên; restart sẽ làm thay đổi hardware tare.
Nếu cần khởi động, dùng interface EtherCAT/IP Wi-Fi hiện tại đã xác minh,
không sao chép IP cũ. Trên máy hungnb có thể xem IP bằng `ip -4 -br addr`.

Lệnh dưới theo cấu hình máy cảm biến `binhdangnguyen` đã ghi trong CODEX.md:

```bash
ip -br link
read -r -p 'IP Wi-Fi hiện tại của máy hungnb: ' HC_T1_PC_IP
read -r -p 'Interface EtherCAT Axia đã xác minh: ' HC_T1_AXIA_IFACE
sudo /home/binhdangnguyen/axia_driver/.venv/bin/python \
  /home/binhdangnguyen/axia_driver/axia_sensor_driver.py \
  "$HC_T1_AXIA_IFACE" --ip "$HC_T1_PC_IP" --port 50000 --hz 100
```

Không chạm handle khi driver tare. Nếu đường dẫn/interface máy cảm biến
khác, xác minh cấu hình máy đó trước khi thay lệnh. Không chạy thêm driver
trên hungnb. Giữ terminal chạy đến khi robot đã Stop/Disable cuối buổi.

### Terminal 4 — kiểm tra và marker, máy hungnb

```bash
cd /home/hungnb/cocarry_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10
export HC_T1_TRIAL=t1_dev_01
ros2 topic echo /axia/connected --once
```

Chờ joint states/TF và Axia connected. Trên Axia UI, calibrate bias khi robot
đứng yên, không tiếp xúc, theo quy trình hiện có; giữ cấu hình filter/deadband
đã chốt suốt session. Không bắt buộc đổi sang Calib Mode/Raw UI vì sidecar
ghi raw riêng. Sau đó kiểm tra:

```bash
ros2 topic echo /axia/calibrated --once
ros2 topic echo /axia/calibration_sample --once
ros2 topic echo /sensorless_force/sample --once
```

Axia cần `calibrated: true`, `tf_valid: true`; M310 có đủ sáu
`mregister_values_nm`, joint position fresh và không timeout kéo dài.
Trước Start, trạng thái `BASELINE:...` là bình thường. `/sensorless_force/valid`
vẫn false trong shadow, không phải yêu cầu phải bật true mới được ghi.
Codex kiểm tra stream lặp/freshness; một mẫu `--once` không chứng nhận cả stream.

## 4. Một lượt development quanh T1

### A. Đưa robot tới vùng bắt đầu

Người vận hành đưa robot đến vùng gần T1 bằng quy trình jog/điều khiển đã được
phép. Kết thúc thao tác đó bằng Stop/Disable và chờ đứng yên. Không có lệnh
terminal tự động đi tới T1 trong tài liệu này.

Chọn **Ground Truth** trên UI khi stopped. Trong mode này, pose Start Run
là tâm reference; khi nhả tay trong RUNNING, robot có xu hướng trở về tâm
này do K>0. Vì vậy chỉ nhả để đánh dấu baseline cuối sau Stop/Disable.
Không đổi mode hay bấm LEADER trong bài thu.

### B. Ghi baseline đầu — robot đứng yên, chưa Start

Tại T4 bật logger và đợi service trả kết quả:

```bash
ros2 service call /logger/toggle std_srvs/srv/SetBool '{data: true}'
ros2 topic pub --once /hc_force_trial/marker std_msgs/msg/String \
  "{data: 'trial_begin:${HC_T1_TRIAL}'}"
ros2 topic pub --once /hc_force_trial/marker std_msgs/msg/String \
  "{data: 'no_contact_begin:${HC_T1_TRIAL}:start'}"
```

Chỉ phát marker no-contact khi đã nhả tay, robot đứng yên và không chạm
môi trường. Giữ 15 giây. Tại T4:

```bash
ros2 topic pub --once /hc_force_trial/marker std_msgs/msg/String \
  "{data: 'no_contact_end:${HC_T1_TRIAL}:start'}"
ros2 topic echo /sensorless_force/status --once
```

Codex kiểm tra baseline window chứa mẫu mới tại pose này; cấu hình hiện tại
cần tối thiểu 12 scan và cửa sổ 30 scan. Nên chờ đủ 30 scan mới tại pose bắt
đầu để thay các mẫu từ pose trước; nếu reader chậm, kéo dài hơn 15 giây.
Nhận dạng stationary không tự xác nhận no-contact, nên vẫn cần bạn nhả tay.

Service có thể báo success dù mở sidecar lỗi trong code hiện tại. Nếu T2 có
`Calibration sidecar setup failed` hoặc `Calibration sidecar write failed`,
dừng chuẩn bị và để Codex kiểm tra trước khi thu.

### C. Di chuyển nhẹ quanh T1

Phát marker cuối cùng tại T4 trước khi chuyển sang vận hành:

```bash
ros2 topic pub --once /hc_force_trial/marker std_msgs/msg/String \
  "{data: 'interaction_pending:${HC_T1_TRIAL}'}"
```

1. Người vận hành Enable Robot, chờ ready và robot đứng yên. Giữ không tiếp
   xúc trước Start để baseline M310 được chốt đúng; nếu pose thay đổi thì
   chờ đủ cửa sổ baseline mới.
2. Bấm Start Run, chờ RUNNING. UI sẽ gọi bật logger; vì đã bật ở bước B nên
   logger tiếp tục file đang mở. T2 phải có thông báo `Frozen M310 baseline`.
3. Tác động handle cho robot đi qua lại trong vùng T1 đã vận hành được,
   khoảng 2–3 phút. Có đổi chiều, có lên/xuống nếu thuận tay và đủ clearance;
   thỉnh thoảng dừng 3–5 giây ở các vị trí khác nhau. Các đoạn còn cầm handle
   vẫn là contact, không tự xem là baseline.
4. Không cần gõ marker từng hướng. Không thao tác terminal trong khi phải
   giữ/giám sát robot. Nếu chỉ di chuyển được một mặt phẳng, ghi nhận điều đó;
   Codex sẽ giới hạn kết luận hoặc đề nghị bổ sung một đoạn ngắn.
5. Bấm Stop Run, xác nhận robot dừng và Disable. Dùng E-stop theo quy trình
   nếu có chuyển động bất thường; không cố hoàn tất thời lượng thu.

UI hiện tự tắt logger khi Stop Run và gửi yêu cầu disable streamer. Cần xác
nhận trạng thái dừng thực tế. Không nhầm Stop Run với chỉ gửi marker `stopped`.

### D. Ghi baseline cuối vào file phụ

Sau Stop/Disable, xác nhận logger cũ đã đóng. Lệnh dưới đóng idempotent rồi
mở file mới; chờ ít nhất 2 giây từ lần mở file trước để tránh trùng tên giây:

```bash
ros2 service call /logger/toggle std_srvs/srv/SetBool '{data: false}'
ros2 service call /logger/toggle std_srvs/srv/SetBool '{data: true}'
ros2 topic pub --once /hc_force_trial/marker std_msgs/msg/String \
  "{data: 'post_baseline_for:${HC_T1_TRIAL}'}"
ros2 topic pub --once /hc_force_trial/marker std_msgs/msg/String \
  "{data: 'no_contact_begin:${HC_T1_TRIAL}:end'}"
```

Giữ không tiếp xúc 15 giây, robot đứng yên. Sau đó:

```bash
ros2 topic pub --once /hc_force_trial/marker std_msgs/msg/String \
  "{data: 'no_contact_end:${HC_T1_TRIAL}:end'}"
ros2 service call /logger/toggle std_srvs/srv/SetBool '{data: false}'
```

File baseline cuối có thể chỉ có `.csv.calibration.jsonl`, không có CSV chính
vì controller không phát reference tick. Đây là hành vi dự kiến. Dòng
`No 3D co-carrying samples to write` không tự có nghĩa sidecar bị mất.
Codex ghép hai phần bằng mã `${HC_T1_TRIAL}` và timestamp, không đổi tên file.

Không dùng marker `q` để đóng logger tích hợp: `q` chỉ là marker với logger
này. Không cần terminal `hc_force_marker_cli.py`; các lệnh T4 đã đủ.

## 5. Kiểm tra và bàn giao sau lượt

Tại T4, thay đường dẫn dưới nếu đã chọn session khác:

```bash
ls -lt /home/hungnb/cocarry_ws/cocarry_logs/hc_force_calibration/20260919_t1_local_dev_v1
```

Gửi Codex tên thư mục session, mã lượt, speed override và các bất thường
(reconnect, fault, thay tare, thiếu hướng, robot/cảm biến bị chạm). Chưa cần
tự chạy script calibration. Codex kiểm tra JSONL parse được, raw/TF/marker,
đủ scan mới, file baseline cuối, cấu hình thực tế và trùng pre-roll.

Claude đọc rate/jitter/span/error từ cùng dữ liệu, không khởi chạy client
đọc register thứ hai. Nếu cần benchmark khác, thực hiện ở pha Stop/Disable
riêng trước hoặc sau lượt, và giữ cấu hình thu nhất quán trong mỗi lượt.

Hai kết quả phải trả cho người vận hành: (1) lượt vừa thu dùng được hay không,
(2) có cần bổ sung đúng một đoạn nào không. Không yêu cầu lặp cả bài chỉ vì
model chưa đạt nếu chưa xác định nguyên nhân.

## 6. Lượt test độc lập

Chỉ thu sau khi Codex đóng băng model, reference/preprocessing, quality mask,
phạm vi áp dụng và tiêu chí đánh giá, rồi Claude review. Giữ cùng bài thao
tác nhưng thực hiện mới; không tái sử dụng các đoạn development làm test.

Khi hệ thống Stop/Disable, đóng logger và đổi thư mục log bằng cách relaunch
T2 theo quy trình, ví dụ session `20260920_t1_local_test_v1` (dùng ngày thật).
Tại T4 đặt `export HC_T1_TRIAL=t1_test_01`. Giữ driver Axia nếu ổn định; nếu
restart UI thì calibrate lại bias không tải theo yêu cầu, ghi trạng thái đó
trong metadata. Lặp mục 4. Test có baseline đầu riêng, chỉ baseline đó được
phép làm tare đầu vào; không refit bằng dữ liệu test.

Nếu cần dừng buổi lab: Stop Run -> xác nhận Disable -> đóng logger bằng T4 ->
Ctrl+C T2, T3 rồi T1. Giữ nguyên log đã tạo, kể cả trial lỗi.

## 7. Nội dung giao cho Claude

Có thể gửi nguyên đoạn sau cùng đường dẫn tài liệu này:

> Bạn phụ trách tăng tốc và cải thiện chất lượng đọc M310–M315 cho CoCarry.
> Đọc CODEX.md, báo cáo P0 ngày 19/09, tiến độ ngày 18/09 và bản phân công
> docs/hc_force_t1_work_assignment_runbook_20260919_vi.md. Codex phụ trách
> calibration/estimator/logger; tôi trực tiếp vận hành robot. Hãy kiểm tra
> reader hiện tại, tìm source server /read_mregister và chuẩn bị candidate
> cùng benchmark để đo khi robot Stop/Disable. Mục tiêu >=15 vector hợp lệ/s,
> kèm raw integer, timestamp từng register hoặc timestamp batch có định nghĩa
> rõ, scan ID/span/jitter/error và tương thích schema logger hiện tại. Không
> chạy thêm client cạnh reader hiện có, không tự Enable/Start hoặc thay cấu
> hình điều khiển. Không tự deploy/restart controller. Phối hợp trước khi sửa
> YAML/launch/schema chung; không đổi phép tính calibration. Bàn giao code,
> test offline, số đo nếu đã được thực hiện, lệnh benchmark/khởi động/rollback
> cụ thể, và các giới hạn nếu thiếu source controller. Phân biệt công việc đã
> kiểm chứng với đề xuất. Sau đó review độc lập kết quả calibration T1 của Codex.

## 8. Những hành vi source đã đối chiếu khi viết

- `cocarry_admittance_real_gui.launch.py`: có `log_dir`, tự mở reader/logger/UI.
- `mregister_force_node.py`: chốt baseline khi `/run_status` lên true; đọc
  scalar tuần tự; scan gap chỉ nạp lúc init; shadow luôn `role_valid=false`.
- `cocarry_logger.py`: `/logger/toggle`, marker topic, pre-roll 10 giây,
  sidecar cả khi không có CSV; marker không điều khiển đóng/mở logger.
- `predictor_ui/ui_node.py`: Start/Stop gọi logger toggle; Stop yêu cầu disable.
- `axia_sensor_ui.py`: có raw wrench và calibration sample chứa TF/filter/bias.

Các đối chiếu trên là đọc source, chưa thay thế test logger/ROS trước buổi thu.
