# Kiểm tra giật/lag Prediction — 2026-09-05

Phạm vi: pipeline co-carry 3D, từ Axia/UDP, EE tracker, predictor/worker,
Admittance, Cartesian smoother, IK, queue robot đến logger và mock simulation.
Đã đọc CODEX.md và kiểm tra worktree. Không thay đổi code vận hành, model,
tham số, mạng hay gửi lệnh robot. Báo cáo phân biệt lỗi tái hiện offline và
dấu hiệu thực nghiệm; chưa phải xác nhận một nguyên nhân duy nhất trên robot.

## Dữ liệu và giới hạn

- Rà các CSV ngày 05/09 trong `cocarry_logs` và `cocarry_logs/simulation`.
- Các trial thật 17:38:44, 17:39:48, 17:40:52 có source `svgp`/`hold`, role
  `OFF`, không phải GRU hoặc LEADER. Không thấy trial thật mang source `gru`
  trong các CSV ngày này. GRU được kiểm tra trên log simulation và artifact.
- GRU simulation 14:46:42: TFLite, window 20, 6 features, delta_position,
  output filter disabled; wrist limits 0.08 rad/s theo startup log.
- Profile hiện tại mặc định GRU TFLite; SVGP được chọn riêng dùng M100 NPZ.
  Các kết quả SVGP buổi chiều không mặc nhiên đại diện cho artifact mặc định
  hiện tại. Trong những trial thật nói trên inference SVGP khoảng 17–21 ms
  median, khác backend NPZ mới.
- Installed Python packages trỏ qua egg-link/build symlink về source hiện tại.
- Khi kiểm tra live, máy không có Ethernet robot trong danh sách interface;
  Wi-Fi đang là 192.168.1.15/24. Không thể đo lại đường robot của buổi chiều.
  Counter Wi-Fi hiện tại không có RX/TX errors; không chứng minh mạng cũ tốt.

## 1. Rate gate tự bỏ request dù inference nhanh — đã tái hiện

Nguồn: `src/trajectory_predictor/trajectory_predictor/predictor_node.py:597`.
Tracker publish 15 Hz và predictor đặt inference_rate_hz=15. Mỗi callback:

```python
if now - last_request < 1 / 15:
    return
last_request = now
```

Callback đến sớm hơn chu kỳ chỉ một chút sẽ bị bỏ. Mẫu kế tiếp cách lần gửi
trước khoảng 133 ms mới được nhận. History vẫn nhận các mẫu trước khi gate;
lỗi này làm thưa output, không tự đổi khoảng thời gian các mẫu trong history.

- Tái hiện 15 Hz với jitter Gaussian 0.1 ms: chỉ 9.47 request/s.
- GRU 14:46:42: số lần vector raw prediction thay đổi quan sát được 9.47/s;
  median khoảng cách cập nhật 132.60 ms, p95 134.04 ms.
- GRU 09:37:34: 9.49/s, median 131.91 ms.
- Chỉ số đếm thay đổi từ CSV là quan sát xấp xỉ, không phải đếm trực tiếp
  mọi publish. Tái hiện rate gate và pattern khoảng 133 ms hỗ trợ cùng cơ chế.
- GRU 14:46:42 inference median 0.47 ms, p95 0.75 ms; prediction_age median
  44.81 ms, p95 111.52 ms. Số age này chỉ đo từ lúc controller nhận prediction,
  không bao gồm tuổi input, IPC trước đó hay thời gian xếp hàng robot.

Đây là lỗi cần ưu tiên sửa: một sample đồng bộ tạo một request khi worker rảnh,
hoặc deadline không trôi theo jitter với cơ chế chỉ giữ request mới nhất.

## 2. Smoother có nhánh nhảy thẳng tới đích — đã tái hiện bằng method hiện tại

Nguồn: `src/hc10dtp_bringup/scripts/cartesian_streamer_hc10dtp.py:1309`.
Khi distance <20 mm và velocity đang ngược vector lỗi, code gán position bằng
target và đặt velocity/acceleration về zero. Nhánh này có thể xảy ra lúc
prediction đổi hướng; phép gán bỏ qua bước tích phân vừa bị giới hạn.

Tách `_smooth_pose` bằng AST, chạy với clock/message giả, không khởi tạo ROS:

- Vị trí hiện tại 0; target +19 mm; vận tốc trước -0.1 m/s; dt=66.67 ms.
- Method trả về position +19 mm trong một tick; giới hạn 0.15 m/s chỉ tương
  ứng 10 mm/tick. Đây là bước nhảy của Cartesian trung gian.
- Khâu giới hạn joint phía sau vẫn hoạt động; không suy ra robot thật đã
  chuyển đúng 19 mm/tick. Nhưng tính liên tục của smoother đã bị phá.

Cần sửa cách phanh/đổi hướng, kiểm tra cả position, velocity, acceleration ở
đầu ra thực tế của smoother và sau khi queue chấp nhận.

## 3. Cắt vận tốc từng joint phá phối hợp giữ orientation — có tái hiện và log

Nguồn: `cartesian_streamer_hc10dtp.py:1452`. IK tìm q_target giữ orientation,
sau đó từng thành phần vận tốc bị clip riêng ở 0.20/0.08 rad/s. Kết quả q_cmd
không còn là cùng một mức tiến trên vector q_target-q_previous.

Tái hiện từ pose đầu trial thật 17:39:48, dt=1/15 s, solver hiện tại:

- Target X+8 mm giữ orientation yêu cầu tốc độ J1≈-0.308, J6≈+0.301 rad/s.
  Clip riêng J1 và J6 tạo sai góc khoảng 0.432° ngay trong bước này.
- Target Z+8 mm giữ orientation: clip tạo sai góc khoảng 0.902°.

FK của 6 joint đo được trong CSV, so với orientation mẫu đầu mỗi trial:

| Trial thật | Sai góc p95 | Sai góc lớn nhất |
|---|---:|---:|
| Ground Truth 17:37:09 | 4.73° | 5.63° |
| SVGP 17:38:44 | 5.47° | 6.79° |
| SVGP 17:39:48 | 8.45° | 8.55° |
| SVGP 17:40:52 | 8.44° | 8.68° |

Đó là sai góc từ encoder/FK, không phải đo quang học độc lập. Không thể phân
bổ toàn bộ sai góc cho một hàm chỉ từ CSV; tuy nhiên cơ chế clip được chứng
minh và sai orientation thực tế đáng kể dù mục tiêu được giữ cố định.
Tracking watchdog hiện kiểm tra sai số position, không có ngưỡng orientation.

Cần đánh giá tiến đồng bộ các joint, ví dụ common scaling dưới cùng giới hạn
vận tốc hiện có, kèm kiểm tra FK/orientation và động học. Chưa nới R/B/T.
ACK đang cập nhật position smoother theo FK(q_accepted), nhưng không đồng bộ
velocity/acceleration smoother với chuyển động đã clip: một nguồn dao động
tiềm năng khác cần replay.

## 4. Force dropout thật gây giữ/nhả robot — xác nhận ở trial 17:38:44

Log controller `cocarry_logs/python3_14801_1788604560095.log` ghi **8 lần**
`Force stream stale` và **8 lần** phục hồi trong khoảng CSV 17:38:44.
Force age khi giữ khoảng 205–233 ms. Code reset velocity Admittance khi giữ
và realign sau phục hồi, nên có các nhịp dừng/khởi động lại rõ ràng.

Log Axia `python3_14809_1788604560294.log` ghi UDP gap 225.7, 231.7, 230.9,
238.1, 229.5 ms trong trial. UI chỉ coi disconnect sau 3 s, controller giữ
sau 0.20 s: UI vẫn connected hoàn toàn có thể đồng thời robot bị khựng.

Hai trial 17:39:48 và 17:40:52 không có stale episodes trong khoảng CSV;
dropout không giải thích mọi trial. `udp_gap_ms` trong CSV lưu giá trị mới
nhất và có thể bỏ lỡ spike; lấy p95/max CSV thấp để loại trừ mạng là sai.

Đường lực là Axia → EtherCAT PC2 → UDP/Wi-Fi → PC robot. Log chưa phân biệt
được hub Axia, Wi-Fi, delay xử lý hay scheduler. Protocol 24 byte không có
sequence/acquisition timestamp nên không đo được tuổi dữ liệu gốc.

Đường queue robot trong các cửa sổ hoạt động ghi tick/send/ACK gần 15 Hz,
busy/retry/reject=0, local IK trung bình ~1 ms và ik_fails=0. `inter_ack_ms`
~66.7 là khoảng cách ACK, không phải RPC latency. Mẫu rate đầu sau Enable
có thể tính cả khoảng dừng giữa trial; không coi nó là network latency.
Chưa có bằng chứng hub robot gây nghẽn trong các cửa sổ này.

## 5. HOLD và tuổi prediction — lỗi tiềm năng, không phải GRU train bị hỏng

Nguồn: `predictor_node.py:565`, `:398`, `:600`.

- HOLD chuyển ngay output từ prediction sang pose đo được lúc vào HOLD;
  target đó giữ cố định đến khi nhả, không liên tục bằng actual mới nhất.
- Khi nhả (>10 mm), xóa buffer rồi lặp mẫu hiện tại để đủ window. Với GRU,
  lặp cả delta của mẫu mới cho các mẫu giả tạo history vị trí không đổi
  nhưng delta khác zero. Train padding lặp mẫu đầu trial có delta=0.
- Kiểm tra tổng quát trên 120 điểm replay 14:46: xóa history rồi lặp feature
  hiện tại làm output khác full history median 13.66 mm, p95 24.05 mm,
  max 30.48 mm. Đây là thử nhiễu history nhân tạo, không phải thống kê
  bước nhảy ở 120 sự kiện HOLD thật. Log trial này chỉ có một lần nhả HOLD.
- Vào HOLD không tăng epoch và reader không kiểm tra hold_detector.active:
  response đang bay có thể publish đè output HOLD. Chưa chứng minh race
  đã xảy ra trong log đã lưu.
- Worker không có in-flight/latest-only backpressure; input/sequence timestamp
  không đi cùng response. Response cũ cùng epoch có thể được đóng dấu publish
  mới và controller coi là fresh. GRU nhẹ nên chưa thấy backlog compute.

## 6. Model GRU hiện tại: nhanh, artifact gần tương đương; độ chính xác có hạn

Chạy riêng artifact hiện tại, 120 cửa sổ đủ 20 mẫu từ EE simulation 14:46:42,
resample 15 Hz; CPU TensorFlow 2.15, một thread theo runtime:

| Phép đo | Kết quả |
|---|---:|
| TFLite core median / p95 / max | 0.193 / 0.312 / 0.474 ms |
| JSON worker roundtrip median / p95 / max, 100 lần sau warm-up | 0.677 / 1.255 / 2.150 ms |
| Khác TFLite–HDF5 trung bình / max theo chuẩn 3D | 0.084 / 0.115 mm |
| Sai số GRU trước 5 mẫu, trung bình / p95 | 31.50 / 61.66 mm |
| Baseline giữ vị trí hiện tại, trung bình | 38.85 mm |

Replay model trên log là open-loop, lấy origin mẫu CSV đầu và nội suy theo
timestamp logger; không dựng lại chính xác phase input/request online.
Không được dùng nó như sai số online chính xác hoặc bảo đảm closed-loop.
Tốc độ được đo trên máy hiện tại không có tải vận hành robot/RViz.

GRU có sai số đáng kể trên quỹ đạo này, nhưng không thấy conversion TFLite
phá model hay inference làm nghẽn 15 Hz. Train/runtime đều relative XYZ và
delta/sample, không thấy cộng origin hai lần. OOD không đủ để kết luận mọi
hiện tượng giật/cứng. Notebook mentor không tự trừ origin; hệ tọa độ CSV gốc
cần đọc riêng nếu muốn khẳng định dữ liệu train của mentor tuyệt đối/tương đối.

## 7. Tham số và logger làm thay đổi cách diễn giải

- M=1, K=5, D≈4.472 dùng chung GT/Prediction; không tự tăng K khi chọn model.
- Max virtual acceleration 0.5 m/s² chỉ giới hạn e; x_d mới được cộng trực
  tiếp sau step nên thay đổi của x_d không chịu giới hạn này. Bound 50 mm
  của nominal là khoảng cách tới actual, không phải giới hạn tốc độ/jerk.
- Command lead 30 mm tác động lên tổng x_d+e và ghi lại state e, đồng thời
  loại velocity hướng ra ngoài. Khi saturate, hệ không còn đáp ứng tuyến tính
  đơn giản M e_ddot + D e_dot + K e = F như mô tả chưa có limiter.
  Trial Prediction chạm gần 30 mm khoảng 57–64%, GT khoảng 34%. Đây là dấu
  hiệu giới hạn hoạt động nhiều, chưa đủ để kết luận phải tăng khoảng lead.
- Force filter median5 + EMA0.1 có memory đáng kể: ở 100 Hz riêng EMA có
  mean sample age 9 mẫu≈90 ms. Force_age không đo độ trễ của filter.
  Kèm deadband4N nên lực sửa hướng không tác động tức thời như cảm giác
  một cơ cấu compliance cơ khí. Tham số này dùng chung GT/Prediction.
- Logger chụp `_latest` của các topic độc lập khi callback reference chạy.
  Controller publish error SAU reference. Không đảm bảo N,E,R cùng tick;
  `_latest` còn được giữ qua Start. Sai `R != N+E` hoặc hàng đầu có e cũ
  chưa chứng minh điều khiển cộng sai hoặc giữ state cũ qua trial.

## 8. Các đường dự phòng và mô phỏng

- BUSY retry rollback cumulative time dù giữ nguyên timestamp của point
  resend; ACK không restore cumulative tương ứng. Sau BUSY có nguy cơ
  timestamp tiếp theo lặp/giảm. Không thấy BUSY trong các trial liên quan.
- Timeout500ms chỉ clear inflight, không cancel future cũ/token request;
  callback muộn có thể ghi state ACK cũ. Chưa có bằng chứng kích hoạt.
- EE tracker stamp lại mẫu mới dù joint cache không đổi; không có freshness
  tại tracker. Controller/streamer có watchdog riêng nhưng chẩn đoán tuổi
  input predictor vẫn thiếu.
- Mock MotoROS2 luôn nhận điểm khi active, chuyển cumulative time thành dt
  rồi publish single-point JTC; dt<=1ms còn tự thay thành66ms. Nó không mô
  phỏng queue BUSY, ACK/dropout, thời gian controller hay cơ học tiếp xúc.
  RViz mượt không xác nhận robot thật mượt.
- LEADER bỏ Admittance là hành vi đã được chốt; các trial đối chiếu ở đây
  không phải LEADER, nên không dùng nhánh đó giải thích chúng.

## Thứ tự xử lý được đề xuất

1. Sửa rate gate, đánh dấu timestamp/sequence từ input tới output; bounded
   latest-only request; ghi snapshot controller đồng bộ theo tick.
2. Sửa nhánh snap của smoother và kiểm tra phối hợp giới hạn joint/Cartesian
   state, giữ các giới hạn vận tốc hiện có; thử offline trước simulation.
3. Làm handoff HOLD không bước nhảy, bảo vệ response đang bay; giữ history
   đo được hoặc xây padding nhất quán với feature delta.
4. Kiểm tra đường lực theo timestamp/sequence và jitter; giữ watchdog,
   tìm nguyên nhân 8 lần stale thay vì nới timeout để che lỗi.
5. Sau đó mới A/B chất lượng model/gain trên cùng dữ liệu và điều kiện.

Phép thử đối chứng đúng: với cùng force và cùng x_d=capture cố định, đường
Prediction đã chuẩn bị phải cho cùng reference với GT (cùng initial state,
watchdog, tick). Ép x_d=actual không tương đương Ground Truth vì làm điểm cân
bằng di chuyển theo actual; phép thử đề xuất trước đó không cô lập được lỗi.
Đánh giá closed-loop cần replay/plant nhất quán rồi người vận hành xác nhận
trên robot; chưa cần train lại GRU để sửa các lỗi lập lịch và ghép lệnh trên.

Script chẩn đoán tạm: `/tmp/cocarry_prediction_audit_Kb9PVb/check.py`.
Chạy lại: `python3 /tmp/cocarry_prediction_audit_Kb9PVb/check.py`.
Script chỉ load model, đọc log, chạy worker pipe và AST smoother bằng stub;
không ROS publisher/service, không thay model/config. /tmp có thể bị dọn khi reboot.

## Cập nhật triển khai 2026-09-06

Theo yêu cầu tiếp theo của người dùng, đã thay đổi code/config (phần báo cáo
ở trên mô tả trạng thái trước sửa):

- Rate gate dùng deadline monotonic, dung sai tối đa 5 ms, reset tại Start/Stop;
  tránh bỏ mẫu 15 Hz chỉ vì callback đến sớm một chút. Không phát bù hàng loạt
  các deadline bị lỡ. Đây chưa phải cơ chế bounded/latest-only worker queue.
- Co-carry tắt stationary HOLD bằng `hold.enabled=false` cho cả hai backend,
  không reset history khi EE đứng yên/di chuyển lại. Camera giữ mặc định bật.
  Force-stale HOLD ở controller giữ nguyên; lực bằng 0 không phải lệnh dừng.
- Hai launch co-carry bật `--continuous-cartesian-smoothing`: bỏ snap ở target
  gần/đảo chiều, vẫn tích phân velocity có giới hạn và cap desired speed theo
  khoảng cách/dt để hội tụ. Camera giữ smoother cũ. Không khẳng định jerk luôn
  bị chặn: nhánh bỏ jerk limit dưới 20 mm đã có trước và chưa được thay đổi.
- Giữ K=5, Cartesian 0.15 m/s và 0.50 m/s², joint S/L/U=0.20 và R/B/T=0.08
  rad/s. Chưa đổi independent joint clipping, đồng bộ state sau ACK, mạng lực,
  model/training GRU, timestamp/sequence input-output hay snapshot logger.

Kiểm tra sau sửa:

- 57 pytest đạt (predictor, bringup, admittance), gồm jitter, đứng yên rồi di
  chuyển, Stop/reset, target đảo chiều và giới hạn velocity/acceleration từ
  position finite difference. Ba package build thành công.
- Probe ROS localhost domain 42 với worker thật, hybrid tắt, input giả 15 Hz
  đứng yên 3 s rồi ramp 0.005 m/s; tổng 7 s/backend, không gửi lệnh robot:
  GRU 15.0006 Hz, khoảng cách output p50/p95=66.70/68.00 ms;
  SVGP M100 NPZ 15.0014 Hz, p50/p95=66.64/67.99 ms.
  Mỗi backend 104 output, không source HOLD, giữ history 20/10 mẫu; sau Stop
  không tiếp tục publish khi đã drain message đang truyền.
- Headless simulation khởi động được với flag mới, GRU READY, FK cross-check
  đạt, K/limits đúng; không Enable/Start Run. Timeout SIGINT sau 12 s gây một
  số traceback shutdown/KeyboardInterrupt; chưa sửa phần shutdown này.
- Chưa xác nhận độ mượt closed-loop trên robot thật. RViz/mock không kiểm tra
  được tiếp xúc, joint coordination sau clipping và mạng thực tế.

Probe tạm: `/tmp/cocarry_prediction_fix_PhJtJd/probe.py` (có thể mất khi reboot).
