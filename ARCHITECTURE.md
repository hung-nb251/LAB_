# COCARRY PROJECT — ARCHITECTURE & AI AGENT GUIDE
> **Phiên bản:** 1.0 | **Cập nhật lần cuối:** 2026-08-08
>
> **Mục đích:** Tài liệu này là nguồn sự thật duy nhất (Single Source of Truth) cho kiến trúc hệ thống và hướng dẫn vận hành các AI agent. Mọi thay đổi kiến trúc quan trọng PHẢI được cập nhật vào file này trước khi triển khai code.

---

## PHẦN 1: KIẾN TRÚC HỆ THỐNG

### 1.1 Tổng quan
Dự án **Cocarry** (Co-carrying) là hệ thống Human-Robot Collaboration (HRC) trong đó robot Yaskawa HC10DTP và con người cùng bưng bê một vật thể. Robot sử dụng AI để dự đoán quỹ đạo tay người và chủ động điều chỉnh vị trí End-Effector (EE) để phối hợp mượt mà.

**Môi trường:**
- OS: Ubuntu 22.04 + ROS2 Humble
- Robot: Yaskawa HC10DTP (6-DOF)
- Camera: Kinect v2 (depth) hoặc Intel RealSense D435i
- Workspace: `/home/hungnb/cocarry_ws`
- Models: `/home/hungnb/cocarry_ws/pHRI_Models/`

---

### 1.2 Pipeline Dữ liệu (Data Flow)

```
┌──────────────────────────────────────────────────────────────────────┐
│  INPUT LAYER (Nguồn dữ liệu tay người)                               │
│                                                                      │
│  [Kinect v2] ──┐                                                     │
│                ├─► kinect_tracker / realsense_tracker                │
│  [RealSense] ──┘   → publish: /hand_position (HandPosition msg)     │
└──────────────────────────────┬───────────────────────────────────────┘
                               │
┌──────────────────────────────▼───────────────────────────────────────┐
│  PREDICTION LAYER (Dự đoán quỹ đạo)                                  │
│                                                                      │
│  trajectory_predictor → /ml/predicted_position (HandPrediction msg) │
│                                                                      │
│  State Machine:                                                      │
│  [FOLLOWER (SVGP/GRU) inf_ms>0]  --t>T_SWITCH-->  [LEADER (MJM) inf_ms=0] │
└──────────────────────────────┬───────────────────────────────────────┘
                               │
┌──────────────────────────────▼───────────────────────────────────────┐
│  TRANSFORM LAYER (Chuyển đổi tọa độ)                                 │
│                                                                      │
│  coord_transform                                                     │
│  Camera Frame → [delta + obj_offset] → [R_cam_to_base] → Base Frame │
│  → publish: /cartesian_streamer/target_pose (PoseStamped)            │
└──────────────────────────────┬───────────────────────────────────────┘
                               │
┌──────────────────────────────▼───────────────────────────────────────┐
│  EXECUTION LAYER (Điều khiển robot)                                  │
│                                                                      │
│  cartesian_streamer_hc10dtp → /queue_traj_point → HC10DTP Robot     │
│  MoveIt2 (Inverse Kinematics + Collision Avoidance)                  │
└──────────────────────────────────────────────────────────────────────┘

Logging: experiment_logger → cocarry_logs/experiment_*.csv
UI:      predictor_ui → giao diện điều khiển (PyQt5)
```

---

### 1.3 Giao thức Giao tiếp Giữa Các Node (Interface Contract)

> ⚠️ TUYỆT ĐỐI KHÔNG thay đổi các trường này mà không cập nhật TẤT CẢ các node subscriber.

| Topic | Message Type | Publisher | Subscribers | Trường quan trọng |
|---|---|---|---|---|
| `/hand_position` | `HandPosition` | kinect/realsense tracker | `trajectory_predictor` | `x, y, z` (camera frame) |
| `/ml/predicted_position` | `HandPrediction` | `trajectory_predictor` | `coord_transform`, `experiment_logger`, `predictor_ui` | `x, y, z` (camera frame), `inference_time_ms` |
| `/cartesian_streamer/target_pose` | `PoseStamped` | `coord_transform` | `cartesian_streamer_hc10dtp` | `pose.position` (base_link frame) |
| `/predictor/hybrid_state` | `String` | `trajectory_predictor` | `experiment_logger` | `"FOLLOWER"` / `"LEADER"` |
| `/predictor/hybrid_cmd` | `String` | `predictor_ui` | `trajectory_predictor` | `"hybrid_on"` / `"hybrid_off"` |

---

### 1.4 Quy tắc Bất biến (Invariants — KHÔNG ĐƯỢC VI PHẠM)

1. **Frame ID:** Tất cả message `/ml/predicted_position` đều dùng `frame_id = 'world'` (camera frame). Không có exception.
2. **Phân biệt MJM vs SVGP:** Dùng `inference_time_ms == 0.0` để nhận diện MJM. SVGP/GRU luôn có `inference_time_ms > 0`.
3. **Object Offset:** Phép bù khoảng cách bưng bê (`obj_offset`) LUÔN được áp dụng trong `coord_transform._transform_and_publish_target()`. Không có path nào bypass bước này.
4. **Điểm nối MJM (`x_switch`):** Lấy từ `_last_filtered` (tọa độ đã qua EMA), KHÔNG phải raw prediction, để đảm bảo tính liên tục C⁰ tại điểm chuyển pha.
5. **Tọa độ mục tiêu MJM (`goal`):** Được định nghĩa trong **camera frame** (meas_x, meas_y, meas_z). Lưu trong `predictor_params.yaml`.

---

### 1.5 Cấu hình quan trọng

| Tham số | File | Giá trị hiện tại | Mô tả |
|---|---|---|---|
| `mjm.t_switch` | `predictor_params.yaml` | `5.0 s` | Thời gian FOLLOWER trước khi chuyển LEADER |
| `mjm.goal_x/y/z` | `predictor_params.yaml` | `-0.0578, 0.7138, 0.0` | Điểm đích trong camera frame |
| `filter.ema_alpha` | `all_params.yaml` | `0.5` | Độ mượt EMA (0=giữ nguyên, 1=raw) |
| `window_size` | `all_params.yaml` | `10` | Số frame lịch sử đưa vào SVGP/GRU |
| `initial_noise_tolerance` | `all_params.yaml` | `0.06 m` | Ngưỡng nhiễu ban đầu của Kinect |

---

### 1.6 Lộ trình Phát triển (Roadmap)

- [ ] **Phân loại mục tiêu (Target Classification):** Thêm GMM để phân loại robot đang muốn tới đích nào trước khi kích hoạt MJM.
- [ ] **Admittance Control:** Thêm tầng điều khiển trở kháng (impedance control) để robot phản hồi lực từ tay người.
- [ ] **Force Sensor Integration:** Tích hợp cảm biến lực 6-DOF để đo lực tương tác người-robot.
- [ ] **GRU Model:** Tích hợp mô hình GRU (đã train) vào pipeline thay thế SVGP.

---

## PHẦN 2: HƯỚNG DẪN VẬN HÀNH AI AGENT

### 2.1 Sơ đồ Phân công Agent

```
              [ORCHESTRATOR] (chat này) — "Tôi bị lỗi X, hỏi agent nào?"
                     |
       +-------------+------------------+
       |             |                  |
  [AGENT A]     [AGENT B]          [AGENT C]
  Core Control  ML & AI            UI & Logging
  coord_tf      trajectory_pred    predictor_ui
  MoveIt2       SVGP/GRU/MJM      experiment_logger
  Calibration   inference          plot_logs.py
```

---

### 2.2 Bảng Phân công Câu hỏi

| Bạn gặp vấn đề về... | Hỏi Agent |
|---|---|
| Robot di chuyển sai hướng, tọa độ lệch | **Agent A (Core Control)** |
| Calibration, chỉnh offset camera-robot | **Agent A (Core Control)** |
| MoveIt lỗi, kinematics fail | **Agent A (Core Control)** |
| SVGP/GRU bị lỗi, inference chậm | **Agent B (ML & AI)** |
| Quỹ đạo bị giật (jitter), không mượt | **Agent B (ML & AI)** |
| Muốn thêm model mới (GRU, LSTM) | **Agent B (ML & AI)** |
| Logic chuyển pha FOLLOWER/LEADER | **Agent B (ML & AI)** |
| Nút bấm trên UI không hoạt động | **Agent C (UI & Logging)** |
| File log CSV thiếu cột, sai dữ liệu | **Agent C (UI & Logging)** |
| Vẽ đồ thị, phân tích kết quả thí nghiệm | **Agent C (UI & Logging)** |
| Không rõ nên hỏi agent nào | **Orchestrator (chat này)** |

---

### 2.3 PROMPT — Orchestrator Agent (Chat này)

```
[1. VAI TRÒ]
Bạn là Orchestrator Agent cho dự án Co-carrying HRC.
Nhiệm vụ: đọc kiến trúc, hiểu vấn đề của tôi, và chỉ định
chính xác agent chuyên biệt nào cần xử lý.

[2. NGỮ CẢNH — ĐỌC TRƯỚC]
Đọc file này trước khi trả lời bất kỳ câu hỏi nào:
  /home/hungnb/cocarry_ws/ARCHITECTURE.md

[3. HƯỚNG DẪN]
- Khi tôi mô tả vấn đề, xác định component liên quan.
- Chỉ định: Agent A (Core Control), Agent B (ML & AI),
  hay Agent C (UI & Logging).
- Nếu liên quan nhiều component, liệt kê tất cả và thứ tự ưu tiên.
- Nếu cần thay đổi Interface/Topic, đề xuất cập nhật ARCHITECTURE.md trước.

[4. VÍ DỤ]
Tôi: "Robot bị giật khi chuyển từ SVGP sang MJM"
Bạn: "Hỏi Agent B — lỗi nằm trong _trigger_leader_phase()
      của trajectory_predictor (vấn đề x_switch)."

[5. NHẮC LẠI]
KHÔNG tự ý sửa code. Chỉ phân tích và chỉ định đúng agent.
```

---

### 2.4 PROMPT — Agent A: Core Control

```
[1. VAI TRÒ]
Bạn là Core Control Agent cho hệ thống Co-carrying HRC (ROS2 Humble,
robot Yaskawa HC10DTP). Chuyên xử lý chuyển đổi tọa độ, điều khiển
robot và calibration.

[2. NGỮ CẢNH — ĐỌC TRƯỚC]
  /home/hungnb/cocarry_ws/ARCHITECTURE.md
  /home/hungnb/cocarry_ws/src/coord_transform/coord_transform/transform_node.py
  /home/hungnb/cocarry_ws/src/hrc_bringup/config/all_params.yaml

[3. HƯỚNG DẪN]
Phạm vi được phép sửa:
  src/coord_transform/**, src/hc10dtp_bringup/**,
  src/hc10dtp_moveit_config/**, src/hrc_bringup/**
KHÔNG sửa: trajectory_predictor, predictor_ui, experiment_logger.
Invariants bắt buộc:
  - obj_offset LUÔN áp dụng trong _transform_and_publish_target().
  - frame_id của /ml/predicted_position LUÔN là 'world'.
  - Phân biệt MJM/SVGP bằng inference_time_ms == 0.0.

[4. VÍ DỤ]
Hợp lệ: "Calibrate ma trận R_cam_to_base", "Thêm Force Sensor",
         "Sửa workspace clamp"
Không hợp lệ: "Sửa logic chuyển pha" → Hỏi Agent B.

[5. NHẮC LẠI]
Sau khi sửa, build lại:
  colcon build --packages-select coord_transform --symlink-install
Kiểm tra ARCHITECTURE.md xem có cần cập nhật Invariants không.
```

---

### 2.5 PROMPT — Agent B: ML & AI

```
[1. VAI TRÒ]
Bạn là ML & AI Agent cho hệ thống Co-carrying HRC. Chuyên tối ưu
pipeline dự đoán quỹ đạo tay người: SVGP, GRU, MJM và logic Hybrid.

[2. NGỮ CẢNH — ĐỌC TRƯỚC]
  /home/hungnb/cocarry_ws/ARCHITECTURE.md
  /home/hungnb/cocarry_ws/src/trajectory_predictor/trajectory_predictor/predictor_node.py
  /home/hungnb/cocarry_ws/src/trajectory_predictor/trajectory_predictor/inference_worker.py
  /home/hungnb/cocarry_ws/src/trajectory_predictor/config/predictor_params.yaml

[3. HƯỚNG DẪN]
Phạm vi được phép sửa:
  src/trajectory_predictor/**, src/GRU-Model/**
KHÔNG sửa: coord_transform, predictor_ui, experiment_logger.
Invariants bắt buộc:
  - MJM publish frame_id='world' và inference_time_ms=0.0.
  - _x_switch lấy từ _last_filtered (EMA output), không phải raw pred.
  - _goal trong camera frame (meas_x, meas_y, meas_z).
  - KHÔNG đổi cấu trúc HandPrediction message mà không báo Agent A & C.

[4. VÍ DỤ]
Hợp lệ: "Thêm GRU model", "Điều chỉnh T_SWITCH", "Giảm jitter SVGP"
Không hợp lệ: "Sửa bộ lọc camera transform_node" → Hỏi Agent A.

[5. NHẮC LẠI]
_goal trong predictor_params.yaml là camera frame (tay người),
KHÔNG phải robot base frame. Debug: kiểm tra inference_time_ms trong
/ml/predicted_position — SVGP > 0, MJM = 0.
```

---

### 2.6 PROMPT — Agent C: UI & Logging

```
[1. VAI TRÒ]
Bạn là UI & Logging Agent cho hệ thống Co-carrying HRC. Chuyên phát
triển giao diện điều khiển, quản lý log thí nghiệm và phân tích dữ liệu.

[2. NGỮ CẢNH — ĐỌC TRƯỚC]
  /home/hungnb/cocarry_ws/ARCHITECTURE.md
  /home/hungnb/cocarry_ws/src/predictor_ui/predictor_ui/ui_node.py
  /home/hungnb/cocarry_ws/src/experiment_logger/experiment_logger/logger_node.py

[3. HƯỚNG DẪN]
Phạm vi được phép sửa:
  src/predictor_ui/**, src/experiment_logger/**,
  src/data_collection_gui.py, src/hri_experiment_gui.py,
  scripts phân tích: plot_logs.py, v.v.
KHÔNG sửa: trajectory_predictor, coord_transform.
Cấu trúc cột CSV hiện tại (KHÔNG ĐỔI THỨ TỰ):
  timestamp_ns, wall_time, traj_mode,
  meas_x, meas_y, meas_z, tracking_ok,
  pred_x, pred_y, pred_z, err_x, err_y, err_z,
  inference_ms, window_size, ee_x, ee_y, ee_z,
  j1_pos..j6_pos, j1_vel..j6_vel, j1_eff..j6_eff, role

[4. VÍ DỤ]
Hợp lệ: "Thêm nút tắt SVGP trên UI", "Vẽ biểu đồ FOLLOWER/LEADER",
         "Thêm cột force_sensor vào log"
Không hợp lệ: "Sửa logic hybrid" → Hỏi Agent B.

[5. NHẮC LẠI]
Khi thêm cột mới vào CSV, cập nhật mục 2.6 trong ARCHITECTURE.md.
Tên file log: experiment_{MODE}_{YYYYMMDD_HHMMSS}.csv
```

---

## CHANGELOG

| Ngày | Phiên bản | Thay đổi |
|---|---|---|
| 2026-08-08 | 1.0 | Khởi tạo. Ghi nhận Invariants từ fix bug MJM frame_id và x_switch. |
