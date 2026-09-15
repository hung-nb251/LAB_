# Hybrid thủ công: chọn đích độc lập với vai trò

## Phạm vi

Co-carry robot EE: UI `Hybrid` thay tên `GRU+MJM`. Backend GRU mặc định;
backend SVGP vẫn có thể chọn qua launch như trước. Không dùng GMM/conflict tự động.
Camera legacy giữ Hybrid theo predictor cũ. Không thay tốc độ, workspace, K,
force limits hoặc các watchdog trong đợt triển khai này.

Controller sở hữu trạng thái và MJM; predictor co-carry đặt
`mjm.manual_control=true`, không tự chuyển LEADER sau t_switch. Trong LEADER,
predictor vẫn nhận EE nhưng tạm ngừng gửi yêu cầu inference; kết quả đang chờ
bị vô hiệu hóa bằng epoch. Khi nhả LEADER, xóa lịch sử và lấy mẫu EE mới,
không đổi gốc tọa độ capture của lượt chạy.

## Cập nhật logic 2026-09-11

- Giữ warmup **10 mẫu mới**, blend **0,3 s**, GRU window 20 (padding mẫu đầu).
  0,3 s là thời gian hòa trộn trọng số, không phải cam kết robot bám x_d hoàn toàn
  trong 0,3 s: còn bộ lọc reference tau 0,4 s và các giới hạn chuyển động.
- Khi blend bị giới hạn, áp dụng cùng độ hiệu chỉnh vị trí cho cả hai nhánh
  admittance; loại thành phần vận tốc tích phân hướng ra khỏi giới hạn.
  Không ghi sai error hỗn hợp vào riêng nhánh GRU.
- Nối FOLLOWER → LEADER ưu tiên C2. Nếu không khả thi do gia tốc đầu,
  thử C1 (giữ vị trí/vận tốc, gia tốc đầu đoạn nối bằng 0); vẫn kiểm tra
  workspace và giới hạn vận tốc/gia tốc. Nếu cả hai không đạt, từ chối LEADER.
  C1 không bảo đảm liên tục gia tốc tại thời điểm bàn giao.
- NaN/Inf trong lực, EE hoặc prediction đang được sử dụng gây FAULT;
  chặn reference không hữu hạn trước khi phát lệnh. Stop/Fault xóa trạng thái
  Hybrid cũ kể cả khi đã là FOLLOWER.
- Prediction chưa đủ mẫu chỉ phục vụ raw diagnostic và báo tiến độ qua
  `/ml/reentry_prediction`, không phát vào `/ml/predicted_position`.
- Status/sự kiện bổ sung `control_phase`, `bridge_continuity` để phân biệt
  force-only warmup, blend, prediction, MJM và HOLD.
- Lực -2 N trong probe được phát trực tiếp vào topic lực controller, **sau**
  xử lý sensor; không tương đương tác động raw -2 N với deadband Axia bật.

Kiểm chứng sau sửa: **108 tests đạt**, build thành công 3 package
`trajectory_predictor`, `predictor_ui`, `cocarry_admittance_control`.
ROS integration (GRU thật + robot giả, domain 64, localhost) đạt hai chặng:
đến đích → FOLLOWER → warmup/blend → ACTIVE; và force-stale recovery → hủy
LEADER → lực ngược → ACTIVE. Probe kết thúc Stop Run và disable robot giả;
phiên launch đã dừng. Log `/tmp/cocarry_hybrid_fixes_xcJYkt/probe.log`, CSV
`/tmp/cocarry_hybrid_fixes_xcJYkt/cocarry_admittance_sim_3d_20260911_000204.csv`.
Đây là kiểm chứng chức năng mô phỏng, không chứng minh độ mượt/an toàn của
robot thật trong mọi tình huống; cần thử thực nghiệm có giám sát.

## Cách sử dụng

1. Khởi động mô phỏng như trước:

   ```bash
   cd /home/hungnb/cocarry_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch cocarry_admittance_control cocarry_admittance_sim_gui.launch.py
   ```

2. Dùng quy trình vận hành đã có để đưa EE đến vị trí đích 1. Stop Run, đợi
   robot dừng; bấm **Lưu đích 1**. Lặp lại với đích 2. Việc lưu không gửi motion.
   Có thể dùng Ground Truth để đưa robot đến đích rồi Stop; không cần Hybrid.
3. Hai đích đã lưu sẽ khóa ghi đè. **Reset Target** chỉ hoạt động khi không
   PREPARING/RUNNING, có xác nhận xóa cả hai đích.
4. Chọn **Hybrid**, đợi controller xác nhận. Calibrate Axia và Enable Robot
   theo quy trình đang dùng. Người vận hành bấm Start Run.
5. Đợi RUNNING. Mỗi Start mới luôn FOLLOWER và **chưa chọn đích**.
6. Chọn **Target 1** hoặc **Target 2**: chỉ cung cấp ý định, không chuyển vai trò.
7. Bấm **LEADER**: bắt đầu MJM từ EE thực đến đích đang chọn, giữ orientation
   đã capture tại Start. LEADER không cộng admittance error.
8. Đến đích: tự FOLLOWER. Không tự đổi đích và không tự bắt đầu chặng khác.
9. Muốn đổi đích trong LEADER: **FOLLOWER → chọn Target mới → LEADER**.
   Nút Target bị khóa khi LEADER; bấm FOLLOWER không tự chuyển sang đích khác.
10. Stop Run/fault không tính là đến đích. Start lại xóa lựa chọn, giữ tọa độ.

FOLLOWER không phải Stop: GRU + admittance vẫn có thể tạo chuyển động khi nhả lực.
Việc chọn Target không thêm lệnh chuyển động, nhưng không dừng chuyển động FOLLOWER
đang có. Hai nút vai trò chỉ dùng trong Hybrid RUNNING.

## Lưu tọa độ

Mặc định lưu tuyệt đối XYZ, frame `base_link`, không lưu orientation riêng:

- Robot thật, domain 10: `config/hybrid_targets_domain_10.json` trong workspace.
- Mô phỏng, domain 42: `config/hybrid_targets_domain_42.json`.
- Đổi home không làm dịch hai đích. Đổi frame hoặc bố trí thí nghiệm cần đặt lại.
- Có thể override `hybrid_target_file:=/duong/dan/targets.json` trên cả hai launch.
- File ghi atomic; file hỏng/sai frame/đích ngoài workspace làm controller từ chối
  khởi động thay vì âm thầm dùng tọa độ khác. Không tự sửa file trong lúc chạy.
- Hai đích cách nhau hơn 2 lần tolerance. Capture yêu cầu pose mới và robot chậm
  hơn ngưỡng arrival speed. Kiểm tra này không thay thế kiểm tra đường đi/IK.

## Ngưỡng kiểm thử ban đầu

Controller parameters (có thể cấu hình trước khi launch, chưa phải chuẩn an toàn):

| Parameter | Mặc định |
|---|---:|
| hybrid_arrival_tolerance_m | 0.01 m |
| hybrid_arrival_speed_mps | 0.02 m/s |
| hybrid_arrival_dwell_sec | 0.5 s |
| hybrid_arrival_grace_sec | 5 s sau thời lượng MJM |
| hybrid_reentry_warmup_samples | 10 mẫu (~0.66 s tại 15 Hz) |
| hybrid_reentry_blend_sec | 0.3 s |

Arrival dùng EE thực + tốc độ từ chênh lệch pose + dwell, chỉ ghi một lần cho
mỗi lựa chọn/chặng. Có thể xác nhận arrival trước khi MJM hết thời lượng nếu
EE đã thỏa cả ba điều kiện. FOLLOWER đến đích cũng ghi sự kiện, không đổi vai trò.
Chọn lại cùng Target không tạo lại sự kiện; LEADER khởi tạo chặng mới khi đang
ở ngoài vùng đích. Nếu đang trong vùng đích, báo `already_at_target`, không chạy MJM.

MJM quintic có v0=a0=vf=af=0. Thời lượng dùng Fitts (a=3.8697, b=0.7492,
w=0.3) và tăng nếu cần để giữ đỉnh vận tốc/gia tốc dưới giới hạn Cartesian
cấu hình. Không bảo đảm liên tục vận tốc ở lúc bàn giao, không chứng minh
đường đi luôn có nghiệm IK hoặc không va chạm.

Lực stale: giữ actual và không tăng tiến độ MJM. Timeout/quá lực/mất readiness/
pose timeout vẫn fault. Hết thời lượng + grace mà chưa đến đích cũng fault.
Khi trả FOLLOWER: giữ actual chờ prediction mới có timestamp sau bàn giao,
reset conditioner và error để reference đầu tiên bằng actual; timeout vẫn fault.

## Giao diện phần mềm và log

`/cocarry/hybrid_command`: String JSON, UI gửi action `mode`, `save`, `reset`,
`select`, `leader`, `follower`. Controller kiểm tra điều kiện, không tin trạng
thái enable của nút UI. Ví dụ `{ "action": "select", "target": 2 }` chỉ chọn đích.
`mode` chứa `enabled` boolean; `save` chứa `target` và lấy EE trong controller,
không nhận tọa độ tùy ý từ UI.

`/cocarry/hybrid_status`: JSON chứa role, selected/active target, tọa độ đích,
leg, reason, controller_state, timestamp, thời lượng/tiến độ và khoảng cách.
Nguồn intent/role là `manual`, không tạo confidence giả.

CSV thêm `hybrid_status_json`; cột role trong Hybrid dùng vai trò controller.
File cạnh CSV `<trial>.csv.hybrid_events.json` ghi các thay đổi trạng thái khi
logger đang bật. Logger vẫn ghép topic bất đồng bộ; status/reference không phải
một mẫu đồng bộ phần cứng. Event Stop ở đúng lúc tắt logger có thể phụ thuộc
thứ tự callback; controller ROS log vẫn là nguồn bổ sung khi chẩn đoán.

## Kiểm thử ngày 10/09

- **Pycompile & Build**:
  - Build thành công ba package: `cocarry_admittance_control`, `predictor_ui`, `trajectory_predictor` và `hc10dtp_bringup`.
- **Unit & Logic Tests**:
  - Đạt **99/99 tests pass** trên ba package chính (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/cocarry_admittance_control/test src/predictor_ui/test src/trajectory_predictor/test` trong 1.11s).
  - Đạt **17/17 tests pass** trên `src/hc10dtp_bringup/test`.
  - Cần cờ `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` do plugin `launch_testing` của ROS 2 Humble không tương thích pytest 9 trên hệ thống.
  - Các module kiểm thử bao quát toàn bộ logic cốt lõi:
    - Soft switching FOLLOWER → LEADER: khớp vận tốc tham chiếu tức thời ($v_0$) qua đa thức bậc 5 (`soft_handoff.py`), loại bỏ giật vận tốc khi bắt đầu chặng MJM.
    - Soft reentry LEADER → FOLLOWER: cơ chế warmup làm ấm dữ liệu (`WAIT` với `force_follower=True` để robot bám lực Admittance tự do $K=0$) kết hợp hòa trộn quintic (`BLEND` → `ACTIVE`).
    - **Tối ưu hóa độ nhạy Reentry (cập nhật)**:
      - Giảm số mẫu làm ấm từ 20 mẫu (~1.33s) xuống **10 mẫu** (~0.66s tại 15 Hz) qua tham số ROS `hybrid_reentry_warmup_samples`.
      - Rút ngắn thời gian hòa trộn `hybrid_reentry_blend_sec` từ 0.5s xuống **0.3s**.
      - Giảm tổng thời gian chuyển giao từ ~1.8s xuống dưới **1.0s**, giúp người vận hành nhận trợ lực dự đoán từ AI sớm hơn đáng kể mà vẫn triệt tiêu hoàn toàn cú giật do dữ liệu cũ.
    - Đồng bộ token giữa controller và predictor qua `/cocarry/hybrid_status` và `/cocarry/predicted_trajectory`.
    - Lựa chọn độc lập giữa Target (ý định) và Role (vai trò), chống bấm lặp, lưu/reset tọa độ atomic, dwell arrival và stale force timeout.
- **Tích hợp ROS Simulation (End-to-End)**:
  - Chạy mô phỏng tích hợp tự động bằng kịch bản probe (`/tmp/cocarry_manual_hybrid_test_run/probe.py`) trên domain cô lập `ROS_DOMAIN_ID=64`, `ROS_LOCALHOST_ONLY=1` với mock phần cứng HC10DTP (`motoros2_mock`), mô hình GRU TFLite thật và UI offscreen:
    1. *Khởi tạo*: Bắt tọa độ hiện tại lưu Target 1; điều khiển Cartesian streamer tiến +35 mm và lưu Target 2 (khoảng cách 0.0350 m $\ge$ 0.02 m).
    2. *Bắt đầu chạy (Start Run)*: Xác nhận trạng thái ban đầu là `FOLLOWER`, `selected=None`.
    3. *Chặng 1 (Leg 1 - Đến đích tự động)*: Chọn Target 1 → bấm `LEADER` → robot chạy quỹ đạo MJM về Target 1 → tự động chuyển về `FOLLOWER` (`reason: reached`, `force_follower=True`, `reentry_phase: WAIT`) → thu thập đủ 10 mẫu GRU mới sau nhả quyền (`samples=10/10`) → chuyển `BLEND` (0.3s) → chuyển `ACTIVE` (`samples=15/10`, `force_follower=False`).
    4. *Chặng 2 (Leg 2 - Dừng lực & Hủy thủ công)*: Chọn Target 2 → bấm `LEADER` → dừng cấp lực 0.3s để kiểm tra giữ vị trí an toàn (`resumed_after_force_hold`) → bấm hủy thủ công `FOLLOWER` (`reason: skipped_by_user`, `force_follower=True`, `reentry_phase: WAIT`) → tác dụng lực cản người mô phỏng (-2.0 N trục X), robot di chuyển theo lực (-0.1630 m) mượt mà không bị giật → tự động kích hoạt `BLEND` tại mẫu 10 và hoàn tất `ACTIVE` tại mẫu 15.
    5. *Kết thúc*: Tắt logger, Stop Run, kiểm tra log CSV (`cocarry_admittance_sim_3d_20260910_232304.csv`) và file sự kiện `.csv.hybrid_events.json` (ghi nhận đầy đủ 16 bước chuyển trạng thái với `reentry_required_samples=10`).
- Khi Ctrl+C hoặc kết thúc launch, một số tiến trình nền ROS có thể in traceback shutdown/KeyboardInterrupt; các kiểm thử xác nhận robot đã Stop, tắt logger an toàn trước khi đóng môi trường.

Trước robot thật phải restart các node sau build, đặt hai đích thật riêng,
kiểm tra khoảng trống và khả năng IK dọc đường, thử ngắn có người vận hành
giám sát. Không tự bật Servo/Enable/Start hoặc tăng tốc trong bài kiểm thử này.
