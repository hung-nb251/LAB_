# Lưu trữ CODEX.md trước lần dọn 23/09

Bản nguyên văn của `CODEX.md` ngay trước khi viết lại cấu trúc ngày
2026-09-23. Giữ để tra cứu lịch sử quyết định và số liệu chi tiết; **không phải
nguồn sự thật hiện hành** — nhiều giá trị ở đây đã lỗi thời (xem `CODEX.md`).

---

# CODEX.md — Quy ước làm việc cho dự án CoCarry

## Hybrid: "phải bấm LEADER ngay sau Target" — CHƯA tái hiện được (23/09)

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

## Cú giật khi chuyển FOLLOWER → LEADER — đã sửa 23/09

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

## Thử nới trần tốc độ chiều 22/09 — không có lợi, giữ cấu hình lead 55

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

## Cấu hình vận hành mới ngày 22/09 — lead 55 mm, tracking 65 mm

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
`121935`, v = 0,130 m/s):

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

## Cập nhật profile tốc độ khớp ngày 22/09

Launch robot thật và simulation hiện đặt giới hạn `[J1,J2,J3,J5,J6]` ở
`0.60 rad/s`; J4/R vẫn giữ `0.08 rad/s`. Cartesian velocity, acceleration,
jerk, command lead, tracking threshold và các cơ chế `--fail-closed` không đổi.
Các giới hạn này chỉ là trần của streamer; vận tốc thực tế vẫn có thể thấp hơn
do IK, joint coordination, workspace hoặc các bộ giới hạn an toàn khác.

## Quy ước logging gọn từ 21/09

- `cocarry_admittance_real_gui.launch.py` và bản simulation mặc định dùng
  `logging_profile:=compact`: mỗi trial chỉ tạo **một CSV 30 cột**, không tạo
  `.hybrid_events.json` hay `.calibration.jsonl`.
- CSV compact chỉ giữ các biên cần phân tích: prediction đã lọc, nominal,
  reference, EE thực; lực Axia trước/sau deadzone; role/trạng thái và bốn chỉ
  số tuổi dữ liệu. Các trường suy ra được, joint/M310 và JSON lặp theo từng
  dòng bị loại.
- Chỉ dùng `logging_profile:=diagnostic` khi debug controller; profile này giữ
  CSV 61 cột và event JSON. Chỉ dùng `logging_profile:=calibration` khi thật sự
  cần raw Axia/M310; profile này mới tạo thêm `.calibration.jsonl`.
- ROS 2 runtime log trở lại thư mục mặc định `~/.ros/log`, không trộn các file
  process `.log` với dữ liệu thí nghiệm trong `cocarry_logs/`.
- Raw CSV/JSONL lịch sử là dữ liệu thí nghiệm, không tự động xóa hoặc ghi đè.

## Quy ước script Python ở workspace root

- Toàn bộ script Python thao tác trực tiếp đã được gom vào `scripts/`; mã
  package ROS vẫn nằm trong `src/` theo cấu trúc colcon.
- `scripts/` giữ entry point phần cứng/runtime, script train model, công cụ plot
  còn dùng và các script offline cần để tái lập báo cáo calibration/audit.
- Test tự động phải đặt trong `tests/` hoặc `<package>/test/`. Không thêm các
  file `test_ik2.py`, `test_ik3.py` hay bản nháp `refactor_*.py` ở root.
- Các thử nghiệm ngắn đã được thay bằng implementation/test trong `src/` phải
  xóa sau khi đối chiếu; không giữ nhiều bản đánh số của cùng một thử nghiệm.

## Xác nhận bằng hai trial LEADER ngày 21/09 — M-register là external force chậm

- Hai lượt `104650` và `104823` có pha LEADER mà người vận hành cố ý đẩy ngược
  hướng robot. Chỉ đếm scan M310 duy nhất và dùng ngưỡng norm lực >=4 N:
  `cos(F_robot_register,F_ext_Axia)` âm **0/39** mẫu, median theo lượt
  +0,976/+0,971; `cos(v_EE,F_ext)` âm **33/39** mẫu.
- Ở nửa sau LEADER, khi tác động đã ổn định, cosine hai lực vẫn dương 20/20,
  còn cosine vận tốc–lực âm 20/20; median lần lượt −0,864 và −0,842 theo trial.
  Đạo hàm reference cho cùng dấu âm 16/16 mẫu đủ điều kiện.
- Axia khoảng 100,02 Hz; M310 khoảng 4,82 Hz, chậm hơn 20,7 lần. Sáu register
  M310–M315 còn trải trên median 154 ms thay vì là một vector đồng thời.
- Kết luận: trường runtime đang gọi `F_robot` từ M-register là external
  interaction force estimate chậm hơn, không phải robot intent/internal force.
  Không dùng nó cho conflict/role. Báo cáo và artifact tái lập:
  `docs/f_robot_mregister_external_evidence_20260921_vi.md`.

## Kết luận F_robot cuối 19/09 — M310/M320 bị loại, effort chưa đủ, F_robot không khả thi

**Tóm tắt:** không có nguồn nào trên cấu hình phần cứng hiện tại cho ra `F_robot`
như một lực độc lập với `F_ext`. Chỉ số bất đồng phải định nghĩa lại thành
**lực người so với chuyển động robot định thực hiện**, không phải lực so lực.

### M310–M315 và M320–M325: loại khỏi đường đo lực

- Chúng là **ước lượng ngoại lực**, tức đo cùng đại lượng mà Axia đo. Không
  phải nguồn độc lập.
- Đo trên lượt `195619`: Axia **100,00 Hz** (chu kỳ 10,00 ms, P95 10,45 ms,
  sáu kênh đồng thời) so với M310 **4,94 Hz** (chu kỳ 202 ms, sáu kênh trải
  144,8 ms). Chênh 20 lần, và độ lệch kênh gây sai vector torque 3,04 Nm
  trung vị / 8,16 Nm P95 khi chuyển động.
- Axia đã calibrate xong (payload 1,126 kg, residual 0,42–0,51 N) và không phụ
  thuộc Tool Data của controller.
- **Kết luận: dùng Axia, bỏ M310/M320 khỏi đường đo ngoại lực.** Chúng không
  thêm thông tin nào, chỉ thêm sai số và công việc.
- Giữ lại một ghi chú cho tương lai: cảm biến torque ở khớp là nguồn **duy
  nhất** thấy được va chạm ở thân/khuỷu, nơi Axia không thấy. Chưa cần dùng.

### Bằng chứng M310 không mang tín hiệu xung đột

- Lượt `200407` có pha LEADER, robot thật sự đi ngược lực người 20% số mẫu
  (`cos(F_ext, v_reference)` và `cos(F_ext, v_EE)` đều âm 20%). `Φ = cos(F_robot,
  F_ext)` vẫn **0% âm**, trung vị +0,994 — cao hơn cả pha FOLLOWER. Cỡ mẫu pha
  LEADER chỉ 5, là chỉ dấu chứ chưa phải chứng minh thống kê.
- Lượt `193546`: 85/185 mẫu người đi ngược GRU, Φ cũng 0% âm; AUC tách chỉ
  0,738; ngưỡng `PHI_ANGLE=90°` cho tỷ lệ phát hiện **0,0%**.
- Sai số còn lại của model nằm ở **độ lớn theo trục** (Z hụt ~31%, X hụt ~32%,
  Y vượt, cùng dấu ở hai lượt độc lập), mà cosine thì chuẩn hoá mất độ lớn.

### `joint_states.effort` — torque feedback theo Nm, nhưng chưa phải `F_robot`

- MotoROS2 lấy tín hiệu bằng `mpSvsGetVelTrqFb`, yêu cầu đơn vị
  `TRQ_NEWTON_METER`, rồi nhân giá trị API với `1e-6` trước khi publish.
  ROS API công bố `JointState.effort` của khớp quay theo **Nm**. Theo
  Discussion #509, không nhân thêm rated torque hoặc gear ratio.
- Đây là **servo torque feedback** của actuator, khác với estimated external
  torque M310–M315. Tuy nhiên cần kiểm chứng trên đúng binary/controller đang
  cài rằng giá trị feedback có thể dùng như joint-side torque trong phương
  trình động lực học. Nó gồm ảnh hưởng của trọng lực, quán tính, Coriolis,
  ma sát, điều khiển bám và ngoại lực; không phải trực tiếp `F_robot`.
- `cocarry_logs/torque_validation/` có 27 CSV ngày 08–10/09 với các cột
  `joint_effort_j1..j6`; 12 file có giá trị khác 0. Dải 0,01–0,2 đã quan sát
  phải giữ nguyên cách hiểu là **Nm do MotoROS2 publish**. Không được diễn
  giải lại thành 1–20% rated chỉ vì trị số nhỏ.
- `effort = 0` trong các phiên gần đây là do **Servo tắt**, không phải lỗi
  cấu hình. Đã xác nhận với người vận hành.
- `HC10DTP_RATED_TORQUE_NM` trong `sensorless_force_math.py` lấy từ URDF chỉ
  là artifact chẩn đoán cũ; **không dùng** nó để scale `joint_states.effort`.
- Toàn bộ file có effort đều ở **cùng một pose** (J2=+0,028, J3=−0,711 rad) và
  không file nào có cột M310, nên dữ liệu cũ chưa đủ kiểm tra gravity/dynamics,
  quy ước dấu hoặc độ tin cậy của torque feedback theo nhiều pose.

### Vì sao F_robot không khả thi trên cấu hình này

Ba nghĩa khác nhau, ba kết cục:

1. **Lực robot tại điểm tiếp xúc** — không tồn tại tách biệt. Thanh cầm bắt
   cứng vào flange **xuyên qua chính Axia** (`hc_tool_geometry_frames_20260918_vi.md`:
   plate → Axia → spacer → thanh). Ở một mặt cắt cứng chỉ có một lực tương
   tác; định luật III Newton làm "lực robot" bằng âm "lực người". Axia đã đo
   nó. Đây là lý do mọi đường đi đều sụp về `cos ≈ 1`.
2. **Torque nội tại của tay máy** — inverse dynamics có thể dự đoán torque cần
   cho chuyển động, hoặc dùng residual torque để ước lượng ngoại lực. Nhưng
   `τ_actuator − J(q)ᵀF_ext` chỉ còn lại tổng `M(q)qdd + C(q,qdot)qdot + g(q)
   + τ_friction`; nó không phải một Cartesian `F_robot` duy nhất. Ánh xạ tổng
   này qua Jacobian cần mô hình động lực học/ma sát và định nghĩa rõ đại lượng
   đích. Ở dữ liệu co-carry chậm hiện tại, phần quán tính mang thông tin chuyển
   động nhỏ hơn nhiễu nên chưa nhận dạng đáng tin cậy.
3. **Ý định robot dạng vector** — có sẵn chính xác trong phần mềm (vận tốc
   reference, hướng GRU/MJM), ở nhịp controller, không cần cảm biến hay
   calibration. Nhưng nó **không phải lực**.

Khoảng cách gốc với mô phỏng: `simulation_hri/inner_loop.py` giả định người và
robot cùng đẩy **một vật thể tự do có động lực học riêng**, nên `f_h` và `f_r`
là hai lực độc lập thật. Cơ cấu hiện tại không có vật thể đó. Muốn `φ` có nghĩa
đúng như mô phỏng thì phải đổi cơ cấu sang một tấm/hộp được cùng nâng, không
bắt cứng vào flange — đó là quyết định phần cứng.

### Việc còn để ngỏ

- Nếu cần lực vật lý thật cho mục đích khác (kiểm tra mô hình động lực học,
  phát hiện bất thường cơ khí), phép thử rẻ nhất là 5–8 pose tĩnh không tiếp
  xúc, Servo bật, ghi đồng thời `effort` + Axia + q, xem `τ_actuator` có bám
  **dạng** trọng lực theo pose không. Chưa cần bảng rated cho phép thử đó.
- Tool Data hiện khai báo sai: W=2,350 kg nhưng Xg=Yg=0, Zg=0,001 mm,
  Ix=Iy=Iz=0 (ảnh `images/20240912_135149.png`), trong khi thanh vươn 160 mm.
  Nếu bỏ M310/M320 thì nó chỉ còn ảnh hưởng safety/PFL, không còn là việc của
  nhánh calibration. **Không tự sửa.**
- Mass và CoG nhận dạng được bằng thực nghiệm và **tự động bao gồm ốc vít, cáp**
  — không cần cân từng chi tiết. Inertia thì không nhận dạng được từ pose tĩnh,
  nhưng ở gia tốc 0,03 m/s² nó gần như không ảnh hưởng.
- Chi tiết, số liệu và đính chính: `docs/f_robot_source_reassessment_20260919_vi.md`.

## Calibration T1 tối 19/09 — có candidate hợp nhất, vẫn chưa đổi runtime

- Lượt `180049` (XY, 125 s RUNNING, 612 scan) và lượt Z± `183108`+`183502`
  (`t1_dev_z_02`, 430 scan dùng được) đã được hợp nhất. Lượt Z lấp đúng phần
  thiếu: từ 13 mẫu Z+ / 0 Z− lên 54 Z+ / 37 Z−. Y± trong lượt Z bằng 0.
- Candidate hợp nhất `full` ridge 0,01 tốt hơn runtime trên **mọi** đối chứng:
  holdout 18:00 2,922→2,536 N; holdout Z 2,661→2,245 N; macro 10 trial T1
  lịch sử 2,792→**2,015 N**. Bản 18:24 chỉ fit XY từng làm macro lịch sử xấu
  đi (3,278 N) và không đạt kiểm tra tiến cử trên lượt Z; việc bổ sung Z±
  đã đảo ngược kết luận đó.
- Singular values correction gain `[1,445 … 0,468]` — hiệu chỉnh vừa phải
  quanh model hiện tại, không phải gain lớn 22,04 của thử nghiệm trước.
- **Vẫn không deploy.** `decision.json` giữ `KEEP_CURRENT_RUNTIME_SHADOW;
  REQUIRE_NEW_SESSION_TEST`: mọi split lịch sử và cả hai khối nội bộ trong
  ngày đều đã được xem, nên không còn tập nào là test độc lập. Lượt Z sau khi
  dùng để fit đã mất vai trò kiểm tra tiến cử. F_robot giữ shadow,
  `calibration_confirmed=false`, `role_valid=false`.
- Candidate đóng băng: `cocarry_logs/hc_force_calibration/20260919_t1_combined_v3/
  candidate_frozen.json`. Cùng schema với model runtime nên nạp được bằng
  tham số `calibration_file`; **không tự đổi tham số đó**.
- Giới hạn dữ liệu: `183502` ghi liên tục >15 phút thay vì 15 s và chỉ có một
  marker `post_baseline_for`, thiếu `no_contact_begin/end`; logger được đóng
  lúc 18:52 theo yêu cầu người dùng. Baseline cuối chỉ dùng chẩn đoán.
  `t1_z_analysis_v4` đã đọc file khi còn đang ghi nên hash không còn khớp;
  bản dùng cho kết quả là `t1_z_analysis_v5`.
- Từ 18:53 Claude tiếp quản nhánh calibration theo yêu cầu người dùng, tiếp
  tục `calibrate_hc_t1_combined.py` mà Codex đang chạy dở tại `t1_combined_v2`
  (thư mục rỗng, giữ nguyên). **Không còn lớp review độc lập giữa hai nhánh.**
- Báo cáo đầy đủ: `docs/hc_force_t1_calibration_result_20260919_vi.md` mục 3b.
  Tốc độ/chất lượng reader: `docs/m310_reader_rate_audit_20260919_vi.md`.

## Cập nhật P2 ngày 19/09 — A/B prebuffer và candidate sửa jerk

- Lượt `125048` xác nhận đúng `command_lead_m=0.04`, `prebuffer=2`. So với hai
  lượt mốc `prebuffer=3`, tốc độ lệnh trung vị vẫn khoảng 0,095 m/s, queue lag
  theo thời gian vẫn khoảng 0,40 s; BUSY/retry/reject và IK fail đều bằng 0.
  Giảm prebuffer chỉ hạ tracking error trung vị khoảng 1 mm, không cải thiện tốc
  độ có ý nghĩa. Artifact: `cocarry_logs/20260919_p2_lead004_prebuffer2_v1/`.
- Launch thật đã trở về baseline an toàn `command_lead_m=0.04`, `prebuffer=3`.
  Không tăng velocity, acceleration, jerk, tracking threshold hay force limit.
- Candidate source cho co-carry continuous giữ jerk limit cả khi `dist<20 mm`
  và reconcile velocity/acceleration từ ACK qua các bound thay vì ghi đè bằng
  sai phân bậc hai thô. Camera/demo non-continuous giữ hành vi cũ. Candidate đã
  qua test offline nhưng **chưa chạy robot thật**; không được kết luận nó làm
  robot nhanh hơn. Lượt thật kế tiếp chỉ dùng để kiểm tra jerk/độ mượt, tracking,
  overshoot và queue health với cấu hình baseline.
- Báo cáo đầy đủ và số mốc: `docs/p2_pipeline_bandwidth_audit_20260919_vi.md`.

## Cập nhật P1 ngày 19/09 — benchmark và tích hợp shadow runtime

- Đã benchmark 8 filter causal trên 74 đoạn/105.389 mẫu raw, từ 44 log.
  Chọn preset bằng validation 17/09, giữ 18/09 để đối chứng.
- Candidate `median3_adaptive` (median-3 + cutoff thích nghi 1,5–15 Hz)
  giảm RMS nghỉ từ khoảng 0,01717 xuống 0,00937 N so với Raw UI median-5;
  t90 bước tổng hợp tăng 20→40 ms, còn EMA 0,1 mất khoảng 230 ms.
- Candidate loại tốt xung một mẫu nhưng để lọt phần lớn burst 2–3 mẫu.
  Đã tích hợp `median3_adaptive` vào `axia_sensor_ui.py` trước gravity
  compensation; `/axia/raw_wrench` và safety/force limits không đổi. Chưa dùng
  candidate cho F_robot hoặc role selection. Filter reset khi NaN, timestamp
  không tăng hoặc UDP gap >0.20 s.
- Audit 782.750 raw samples không thấy xung đơn theo tiêu chí đã công khai;
  không coi đó là bằng chứng không có burst hoặc lỗi bù trọng lực/TF.
- Kết quả, giới hạn, cấu hình candidate và bước shadow:
  `docs/axia_filter_p1_benchmark_20260919_vi.md`. Có thể tiếp tục P2 offline.

## Cập nhật P0 ngày 19/09 — đã audit hợp nhất, chưa thay model runtime

- Đã kiểm kê 77 event logs (61 trong kho calibration + 16 lịch sử), hợp nhất
  12 pose tĩnh, 54 baseline và 25 trial/3.993 mẫu lực hợp lệ ngày 15–18/09.
- Baseline đủ rank số học 6 nhưng có một hướng yếu; condition chuẩn hóa
  khoảng 101 ở 12 pose và 197 ở tập hợp nhất. Không còn diễn giải rank=2
  của tập con cũ là kết luận về toàn bộ dữ liệu.
- Candidate P0 train+r3 đạt r4 RMSE 3,134 N so với runtime 2,306 N trên cùng
  protocol; lượt dài 17:22 là 8,771 so với 7,144 N. Chưa vượt model hiện tại,
  không deploy hoặc bật calibrated/role_valid. F_robot tiếp tục shadow.
- Phần thiếu rõ nhất: lực X±/Y− trong vùng T2 có M310 đồng thời, baseline
  động có marker không tiếp xúc, test session mới có raw/sidecar đầy đủ.
  Không thu lại diện rộng 12 pose. Chi tiết, code tái lập và artifacts:
  `docs/hc_force_p0_unified_calibration_20260919_vi.md`.

## Trạng thái cuối ngày 18/09 — calibration và băng thông hệ thống

- Không thu lại diện rộng baseline tĩnh/động. Dự án đã có 61 log event, 12 pose
  tĩnh không tiếp xúc, các lượt GT/GRU/GRU+MJM và bảy lượt GRU M310 mới. Audit
  kế tiếp phải hợp nhất toàn bộ dữ liệu; chỉ thu đúng pose/hướng còn thiếu sau
  khi kiểm tra rank, condition và độ phủ.
- `F_robot` từ M310–M315 hiện là shadow trong `base_link`, tính bằng nghiệm
  regularized `J(q)^T W=tau`. Replay runtime khớp công thức offline, nhưng sai
  số Axia còn khoảng 6–7 N ở các lượt dài; calibration chưa hoàn chỉnh và chưa
  được dùng cho Admittance hoặc role selection.
- Chế độ Axia ghi `Raw/alpha=1.0` vẫn qua median-5 nên chưa phải raw thật. Việc
  tiếp theo là benchmark offline bộ lọc causal loại xung + bám nhanh/thích nghi
  theo noise, latency và sai số hướng, rồi mới chạy shadow.
- Độ chậm chuyển động có thể do nhiều khâu nối tiếp: prediction-reference
  tau=0.4 s, Admittance limits, Cartesian jerk/accel/velocity, IK/queue và
  command-lead. Phải đo delay/limiter từng tầng và mô phỏng A/B; không hạ tau
  và tăng jerk đồng thời trên robot thật.
- Minimum Jerk hai điểm hiện nội suy `p0+h(s)(p1-p0)`: đường trong không gian
  phải thẳng; dạng S là tiến độ vị trí theo thời gian. Kiểm chứng bằng đồ thị
  along-path velocity/acceleration/jerk. Đường cong không gian cần planner khác.
- M310–M315 hiện chỉ khoảng 5 Hz vì sáu service scalar tuần tự. Giảm scan gap
  chỉ là benchmark; giải pháp đích là controller-side batch/topic sáu register
  với một timestamp, mục tiêu ban đầu >=15 Hz sau khi kiểm tra tải khi robot
  Stop/Disable.
- Tài liệu đầy đủ, thứ tự P0–P5 và tiêu chí đánh giá:
  `docs/cocarry_progress_next_steps_20260918_vi.md`.

## Calibration M310 ngày 18/09 — trạng thái sau audit runtime

- F_robot CSV hiện lấy từ M310–M315, baseline Home + torque/pose candidate +
  nghiệm regularized `J(q)^T W=tau`, chỉ shadow. Chưa calibrated hoàn chỉnh.
- Axia của launch hiện tại do `axia_sensor_ui.py` publish trong base_link,
  gồm bù góc gá sẵn có; không xoay thêm chỉ vì thấy frame sensor trong node
  `gravity_compensator.py` legacy không được launch ở pipeline này.
- Hai lượt runtime mới cho sai số khoảng 7 N trên reference suy ngược deadband.
  Mô hình baseline riêng chưa vượt candidate cũ nhất quán; chưa triển khai.
- CSV chính giữ 61 cột, chỉ ba cột `f_robot_x/y/z`, torque_J1_Nm..J6_Nm.
  Logger bổ sung `<csv>.calibration.jsonl` với raw M310, Axia raw/processed,
  baseline, q/q0, rotation, model snapshot và marker; không đọc register lần hai.
- Hướng dẫn thu phần dữ liệu còn thiếu và kết quả kiểm chứng:
  `docs/hc_force_baseline_audit_20260918_vi.md`. Người vận hành chạy robot;
  không tự Enable/Start, không thay Tool Data, tare/gá hoặc giới hạn điều khiển.

## Cập nhật phối hợp khớp 2026-09-15

- Theo yêu cầu người dùng, profile vận tốc launch thật hiện là
  `[0.50,0.50,0.50,0.08,0.50,0.40] rad/s`; profile thử nghiệm hiện tại
  Cartesian vmax=0.35 m/s, amax=1.5 m/s², jerk=15 m/s³,
  reference tau=0.40 s, command lead=0.04 m.
- Hai launch co-carry bật `joint_coordination:=synchronized`: khớp cùng tiến
  theo một tỷ lệ giới hạn vận tốc, cập nhật smoother theo FK/vận tốc đã ACK;
  tracking nội suy tuyến tính giữa FK endpoint. Giữ threshold 50 mm/5 mẫu.
- Nominal giữ tau=0.4 s, thêm `prediction_reference_lead_sec:=0.15`, bù theo
  vận tốc lọc tối đa 20 mm. Đặt lead=0 để đối chứng bộ lọc cũ. GT/MJM bypass.
- Đã sửa đồng hồ queue khi BUSY/retry; không rollback thời gian của điểm đang
  pending. CSV thêm `motion_diagnostics_json` với timestamp queue và tracking.
- Source/install hiện upper J2/J3=1.50 rad; margin IK 3° còn khoảng 1.4476 rad.
  Đây là cấu hình thử nghiệm robot thật được người dùng yêu cầu ngày 15/09.
- Chi tiết, giới hạn kiểm chứng và lệnh A/B:
  `docs/motion_coordination_20260915_vi.md`. Chưa xác nhận độ êm/tốc độ trên
  robot thật; thuật toán mới không bảo đảm gia tốc/jerk riêng từng joint.

## Cập nhật Hybrid 2026-09-11 (ưu tiên hơn mô tả lịch sử bên dưới)

- Giữ warmup 10 mẫu mới / blend 0,3 s và các cấu hình tốc độ, lực, workspace hiện tại.
- Anti-windup khi blend hiệu chỉnh cả hai nhánh admittance, không ghi error
  hỗn hợp vào riêng nhánh prediction. Stop/Fault reset cả trạng thái FOLLOWER cũ.
- Nối sang LEADER ưu tiên C2, fallback C1 giữ p/v nếu gia tốc đầu không khả thi;
  vẫn kiểm tra giới hạn, không âm thầm tăng tốc độ. `bridge_continuity` cho biết lựa chọn.
- Chặn NaN/Inf trên đầu vào được sử dụng và reference đầu ra. Predictor tạm
  ngừng inference trong manual LEADER, reset window theo token khi nhả quyền;
  kết quả warmup chưa đủ mẫu không phát lên topic prediction chính.
- Sau hai trial thật `20260911_100145/100806`, soft upper limit J2/L được tăng
  từ `1.20` lên `1.30 rad`; margin IK 3° vẫn giữ, nên cận IK an toàn thực tế là
  khoảng `1.2476 rad`. Thay đổi này xử lý vùng target xa theo bán kính ngang;
  không mở rộng cận dưới workspace X/Z.
- UI Hybrid cho phép bấm LEADER trong RUNNING/FOLLOWER để báo rõ target còn
  thiếu hoặc chưa được chọn ở thanh trạng thái và terminal. GRU+MJM Test cũng
  báo đúng điều kiện target còn thiếu khi từ chối Start Run.
- Status và hybrid event log có `control_phase` để phân biệt WAIT/BLEND/ACTIVE,
  LEADER bridge/MJM và force HOLD. Chi tiết tại `docs/manual_hybrid_test_20260910_vi.md`.
- Kiểm chứng lần này: 108 tests đạt; build 3 package thành công. ROS integration
  với GRU thật + robot giả, domain 64/localhost: đến đích, force-stale recovery,
  hủy LEADER, lực ngược và reentry đều đạt. Không chạy robot thật.
  Log: `/tmp/cocarry_hybrid_fixes_xcJYkt/` (tạm thời, có thể mất sau reboot).
- Từ 2026-09-12, controller ramp soft deadzone Fz từ 0 lên tối đa 2 N chỉ khi
  nhận diện chuyển động X−, rồi ramp về 0 sau hysteresis/dwell. Z thuần không
  có X− giữ nguyên độ nhạy. Đặt `additional_z_deadzone_n: 0.0` để hoàn nguyên;
  X/Y, dữ liệu Axia gốc và force safety limits không đổi. CSV ghi thêm
  `f_effective_x/y/z` và `z_deadzone_weight`.

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
- Sau khi sửa Python: chạy `python3 -m py_compile scripts/<file>.py` cho script
  trong `scripts/` (hoặc đường dẫn tương ứng trong `src/`). Sau khi
  sửa package ROS: chạy test phù hợp và `colcon build --symlink-install
  --packages-select ...` ở mức tối thiểu cần thiết.
- Không commit hoặc push nếu người dùng chưa yêu cầu.
- Log và model là dữ liệu thực nghiệm: không xóa, đổi tên hoặc ghi đè. Model mới
  phải lưu vào thư mục mới và có metadata.

## 3. Trạng thái nhiệm vụ hiện tại

### Cập nhật Hybrid thủ công 2026-09-10 (ưu tiên hơn mô tả Hybrid cũ bên dưới)

Đã triển khai chọn Target 1/2 độc lập với LEADER/FOLLOWER cho co-carry.
Xem `docs/manual_hybrid_test_20260910_vi.md`. Start mới FOLLOWER/chưa chọn đích;
chỉ bấm LEADER mới chạy MJM. Đến đích hoặc bấm FOLLOWER không tự đổi Target.
Muốn đổi đích khi LEADER phải FOLLOWER trước. Capture hai đích khi stopped,
persist absolute base_link tại `config/hybrid_targets_domain_<id>.json`;
Reset Target có xác nhận, chỉ khi stopped. Controller sở hữu MJM/role,
predictor co-carry đặt `mjm.manual_control=true` để vô hiệu timer cũ.
Camera legacy giữ nguyên. CSV thêm hybrid_status_json và file events cạnh CSV.
Đã kiểm thử logic và ROS fake hardware, chưa kiểm thử chuyển động robot thật.
Không đổi các giới hạn vận hành trong nhiệm vụ này.

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
  -> scripts/axia_sensor_ui.py
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
- `scripts/axia_sensor_ui.py`

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
- `K = [5, 5, 5] N/m`
- Critical damping: `D_i = 2*sqrt(M_i*K_i) = 4.47213595`
- Control/stream rate: `15 Hz`
- Max virtual/Cartesian velocity: `0.18 m/s`
- Max acceleration: `0.65 m/s²`
- Max command lead so với EE feedback: `0.04 m`
- Force limit mỗi trục: `20 N`; norm: `30 N`
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

Sau trial simulation `20260908_180431`, profile tốc độ được chuyển sang robot
thật theo từng cấp: J3/U tăng từ `0.20` lên `0.30 rad/s`; R/B/T thật vẫn giữ
`0.08 rad/s`. B/T `0.20 rad/s` tiếp tục chỉ là override mặc định của simulation
vì fake hardware không mô phỏng rung cơ khí/PFL và robot thật từng rung đáng
ngại ở B/T `0.12 rad/s`. Cartesian velocity/acceleration vẫn là
`0.15 m/s`/`0.50 m/s²`, command lead vẫn `0.03 m`, và `--fail-closed` giữ nguyên.

Sau ba trial simulation `20260909_103339/103430/103608` không có IK fail,
người dùng cho phép áp cùng profile lên robot thật: S/L/J3 `0.30 rad/s`,
R `0.08`, B/T `0.15`, Cartesian/virtual velocity `0.18 m/s`, acceleration
`0.65 m/s²`, command lead `0.04 m`. `--fail-closed`, workspace, force watchdog
và joint margins vẫn giữ nguyên.

Sau các trial thật và phép đo pendant High/Top ngày 2026-09-09, profile robot
thật hiện dùng `[S,L,U,R,B,T] = [0.30,0.30,0.35,0.08,0.30,0.25] rad/s`,
Cartesian/virtual velocity `0.22 m/s`, acceleration `0.80 m/s²` và command lead
`0.05 m`. Simulation giữ profile riêng. Tracking threshold vẫn là `0.050 m`.

Sau trial GRU simulation `20260906_105435`, đã tái hiện ripple trong vòng
robot-EE -> prediction -> nominal -> Admittance -> robot-EE mà không cần mạng
hay IK. Output GRU raw vẫn giữ nguyên. Launch simulation hiện override
`prediction_reference_tau_sec=0.5`: lọc bậc một nominal tại controller trước
khi cộng `e`, rồi mới áp lead/workspace limits. Đây là điều hòa reference có
đánh đổi độ trễ, không phải sửa đồ thị hay chứng minh model chính xác hơn.
Ground Truth và MJM LEADER bypass khâu này. State reset tại Start/realign và
đóng băng cùng force HOLD. Real và simulation hiện cùng dùng mặc định `0.5`;
có thể truyền `0.0` để đối chứng legacy.
Real và sim đều hỗ trợ launch arg `prediction_reference_tau_sec:=0.0` để đối
chứng legacy.
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
- Từ 2026-09-08, người dùng thay handle ngang rồi bỏ bớt tải; payload bù
  trọng lực Axia trong `axia_sensor_ui.py` là `1.126 kg` (thay giá trị tạm
  `3.856 kg`). Người dùng xác nhận không chạm handle và cho phép áp dụng fit
  P0/P1/P0 của các CSV `handle_mass_check_20260908_164919/165059/165215`.
  Hai cặp pose cho 1.122/1.131 kg; residual 0.42--0.51 N và P0 return 0.113 N.
  Không tự đổi góc gá, Tool Data Yaskawa, giới hạn workspace hay đánh dấu
  F_robot calibrated. Sau khi khởi động lại Axia UI cần calibrate bias không
  tải và kiểm tra lực ở P0/P1 trước vận hành.
- Góc bù gá mặc định: Roll X `0°`, Pitch Y `0°`, Yaw Z `-90°`.
- Deadband vận hành: radial deadband `2.5 N`, không phải deadband riêng từng
  trục. Giá trị nằm ở hằng số `DEADBAND_N` trong `scripts/axia_sensor_ui.py` và
  dùng chung cho cả ba pipeline (co-carry, co-drawing, camera). Các báo
  cáo/audit trước 21/09 giả định `4 N`; không đọc lại chúng theo ngưỡng mới.
- Deadband này là **kiểu trừ**: `F_out = F * (|F| - dz) / |F|`. Hạ ngưỡng vừa
  giảm mức lực bắt đầu có tác dụng, vừa **cộng thêm `(4 - dz)` N độ lợi** vào
  mọi mức lực vượt ngưỡng. Không coi nó chỉ là một nút chỉnh độ nhạy.
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
export ROS_DOMAIN_ID=10
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

Terminal 3:

```bash
cd ~/cocarry_ws
./run_sensor_driver.sh --iface enxf8e43b7aeaf2
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

Logger co-carrying mặc định (`logging_profile:=compact`) lưu các nhóm chính:

- actual robot EE XYZ;
- predicted relative đã lọc, nominal absolute và reference `x_r`;
- `f_human_x/y/z`, lực effective sau deadzone và trọng số deadzone Z;
- model, role, controller state/phase, timestamp và tuổi dữ liệu.

Raw prediction, joint/M310, motion diagnostics và full hybrid JSON chỉ có trong
profile `diagnostic`; raw Axia/M310 sidecar chỉ có trong profile `calibration`.

UI co-carry vẽ riêng bốn tín hiệu trên cùng hệ `base_link`: raw SVGP nominal,
nominal đã filter/bound, reference gửi robot và actual robot EE. Nhãn `Model`
hiển thị `svgp`, `hold` hoặc `mjm`; không được diễn giải đường Reference–Actual
bám sát là độ chính xác dự đoán SVGP.

Không thay các trường lực robot bằng joint position/velocity/effort. Co-drawing
nếu dùng lại phải ghi riêng trong `codrawing_logs/`.

Thu lực robot cập nhật 2026-09-07: ba trial thật cuối 06/09 có đủ sáu effort,
nhưng status luôn RAW_ONLY. Launch trước đây ghi đè YAML bằng raw_only; đã sửa
`robot_effort_unit_mode` mặc định rỗng để dùng YAML. YAML nay chọn `torque_nm`
theo API MotoROS2 chính thức, chỉ là giả định chẩn đoán cho firmware đang dùng;
`calibration_confirmed=false`, không tự dùng tỷ lệ torque URDF.
Estimator/logger nhận joint_states với sensor-data QoS. Topic JSON schema 1
`/sensorless_force/sample` ghép lực, timestamp, chất lượng, mode/frame và joint
nguồn thành một mẫu. CSV mặc định ghi F_robot trước deadband để không xóa tín
hiệu nhỏ phục vụ calibration (`robot_force_log_unfiltered=true`); topic wrench
cũ vẫn giữ deadband/force sanity limit. INVALID/STALE không lặp lực cũ. Các cột
unfiltered lưu cả ước lượng bị loại vì norm>500 N nhưng không dùng cho role.
F_robot chỉ diagnostic, base_link/tool0, không đổi Admittance/GRU/gains/safety
và không tự đổi dấu theo hướng vận tốc. Phải xác minh đơn vị/scale/gravity rồi
đối chiếu lực chuẩn trước khi đánh dấu calibrated. Test-mode không tạo tick
reference của controller nên dùng để kiểm tra topic, không hứa tự có CSV trial.

## 10. Safety và lỗi cần nhớ

- Người vận hành phải sẵn sàng E-stop và đặt speed override thấp cho lần chạy
  đầu sau mỗi thay đổi.
- Streamer dùng 15 Hz, previous joint seed, soft joint limits, workspace và
  `--fail-closed`.
- Profile robot thật hiện dùng giới hạn `[S,L,U,R,B,T] =
  [0.30,0.30,0.35,0.08,0.30,0.25] rad/s`; simulation giữ profile riêng.
- Từ 2026-09-15, giới hạn mềm trên J2 và J3 của streamer là `1.50 rad` (~85.9°),
  thay cho `1.05 rad`; margin 3° vẫn giữ nên IK chỉ dùng tới khoảng `1.198 rad`
  (~68.6°). Thay đổi áp dụng cả real/sim, không đổi X/Y workspace hay các joint
  khác. Nó được mở để bao phủ hai target thật đã cần J3 khoảng 1.068/1.141 rad;
  vẫn phải theo dõi nhánh khuỷu, clearance và tracking trên robot thật.
- `No kinematics solver` hoặc IK fail liên tiếp có thể khiến target không được
  chấp nhận. Không được gửi nghiệm clamp tùy tiện; kiểm tra frame, orientation,
  reachability, seed và joint limits.
- Tracking error dừng an toàn khi actual EE lệch pose queue đến hạn thực thi quá
  `0.050 m` trong 5 lần kiểm tra sau grace period 1 s. Watchdog căn pose theo
  `time_from_start`, không so feedback với điểm mới nhất còn nằm phía trước.
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

python3 scripts/hc_force_trial_logger.py \
    --session "$HC_CALIB_SESSION" \
    --category static_cog \
    --trial cog_p013 \
    --mode static \
    --pose pose_013 \
    --tool-number 0 \
    --group all \
    --timeout 5.0 \
    --scan-gap 0.10 \
    --output "$HC_CALIB_ROOT" \
    --notes 'mass_total_kg=TODO; one_tare_at_home; CalibMode_OFF; deadband_4N; no_contact'
