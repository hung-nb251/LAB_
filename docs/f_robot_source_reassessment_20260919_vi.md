# Đánh giá lại nguồn tín hiệu cho F_robot — 19/09/2026

Tài liệu này sửa một giả định đã dẫn cả ngày 19/09 đi sai hướng: rằng
`M310–M315` có thể dùng làm `F_robot`. Dữ liệu tối 19/09 bác bỏ giả định đó,
và chỉ ra một nguồn khác **đã từng được ghi trong kho dữ liệu của dự án** nhưng
chưa được khai thác.

## 1. Kết luận

`M310–M315` là **ước lượng ngoại lực**. Nó đo cùng một lực tiếp xúc mà Axia
đo, nên về nguyên tắc không bao giờ đối hướng với `F_ext`. Không thể dùng nó
để tính chỉ số xung đột `Φ = cos(F_robot, F_ext)`.

`joint_states.effort`, lấy từ `mpSvsGetCelTrqFb` phía MotoROS2, là **tổng
torque truyền động**. Đây là đại lượng bổ sung, không phải thay thế. Hiệu
`τ_total − τ_external` là phần torque robot tự sinh ra để thắng trọng lực,
quán tính và để bám quỹ đạo — gần với "lực chủ động" hơn nhiều.

**Chưa có gì được kiểm chứng trên phần cứng theo hướng mới này.** Mục 5 là
đề xuất, không phải kết quả.

## 2. Bằng chứng M310 là ngoại lực

Lượt `20260919_200407` có pha FOLLOWER rồi pha LEADER, người vận hành cố ý
giằng lại trong pha LEADER. Kết quả trên các mẫu có cả hai norm ≥ 4 N:

| Pha | n | Φ trung vị | Φ min | Φ < 0 |
|---|---:|---:|---:|---:|
| FOLLOWER | 10 | +0,986 | +0,568 | 0,0% |
| LEADER | 5 | +0,994 | +0,707 | 0,0% |

Trong chính pha LEADER đó, `cos(F_ext, vận tốc reference)` và
`cos(F_ext, vận tốc EE thực)` đều âm **20%** số mẫu — tức robot **thật sự**
di chuyển ngược lực người. Φ vẫn không âm lần nào, trung vị còn cao hơn pha
FOLLOWER.

Cỡ mẫu rất nhỏ, 5 mẫu trong pha LEADER, nên đây là chỉ dấu chứ chưa phải
chứng minh thống kê. Nhưng nó khớp với định nghĩa thanh ghi và với lượt
`20260919_193546`, nơi 85/185 mẫu có người đi ngược GRU mà Φ vẫn 0% âm.

Cơ chế: controller đã trừ động lực học và mô-men truyền động của chính nó
trước khi xuất ra `M310–M315`. Lực robot dùng để kéo về đích nằm trong phần
bị trừ mất.

## 3. `joint_states.effort` — nguồn đã có nhưng chưa dùng

### 3.1. Dữ liệu đã ghi được

Kho `cocarry_logs/torque_validation/` có 27 CSV ngày 08–10/09 với sáu cột
`joint_effort_j1..j6`. Trong đó 12 file có giá trị khác 0, 11 file toàn 0, và
hai file có tỷ lệ trung gian 15% và 70%.

Ví dụ `P0_single_pc_20260910_111159.csv`, 4.889 mẫu hợp lệ:

| Khớp | Dải effort | Bước nhỏ nhất | Độ phân giải |
|---|---|---:|---:|
| J1 (S) | [0,005; 0,045] | 0,000280 | 0,028% rated |
| J2 (L) | [−0,155; −0,138] | 0,000280 | 0,028% rated |
| J3 (U) | [0,178; 0,202] | 0,000083 | 0,008% rated |
| J4 (R) | [−0,086; −0,048] | 0,000064 | 0,006% rated |
| J5 (B) | [0,018; 0,043] | 0,000064 | 0,006% rated |
| J6 (T) | [−0,027; 0,009] | 0,000064 | 0,006% rated |

Hai điều quan trọng:

- **Giá trị rất nhỏ**, cỡ 0,01–0,2, đúng như MotoROS2 Discussion #509 mô tả.
  Chúng là **tỷ lệ so với rated torque**, không phải Nm.
- **Độ phân giải mịn hơn pendant rất nhiều.** Servo Monitor hiển thị Torque
  Spec dưới dạng số nguyên phần trăm, tức bước 1%. Dữ liệu ROS có bước
  0,006–0,028%, tức mịn hơn khoảng 35–150 lần. Lo ngại "1% là quá thô" không
  áp dụng cho đường ROS.

### 3.2. Quy đổi sang Nm và vì sao đây là tổng torque

Nhân với `HC10DTP_RATED_TORQUE_NM`:

| Khớp | Nm ước lượng |
|---|---|
| J1 (S) | 1,9 … 16,7 |
| **J2 (L)** | **−64,2 … −57,2** |
| J3 (U) | 28,3 … 32,0 |
| J4 (R) | −3,5 … −2,0 |
| J5 (B) | 0,6 … 1,4 |
| J6 (T) | −0,8 … 0,3 |

J2 khoảng −60 Nm khi robot đứng yên không tiếp xúc là đúng bậc độ lớn của
mô-men vai đỡ cánh tay cộng cụm tải 2,35 kg. So sánh: `M310–M315` ở trạng
thái tương tự chỉ vài Nm và về gần 0 khi không tiếp xúc.

Hai đại lượng khác hẳn nhau về bản chất, đúng như
`hc_force_register_plan_20260912_vi.md` đã cảnh báo: *"Cùng Nm vẫn phải phân
biệt tổng actuator torque với external torque đã bù."*

### 3.3. Hệ số quy đổi chưa được xác nhận

`sensorless_force_math.py:29` ghi rõ trong chính comment của nó rằng
`HC10DTP_RATED_TORQUE_NM = [368,48; 414,54; 158,76; 41,16; 33,32; 31,36]` là
**effort limit lấy từ URDF**, "available for diagnostic comparison only; they
are not a calibration of the YRC1000 electrical signal".

Đây đúng là thứ mà kế hoạch register cấm dùng: *"không lấy URDF effort limit
làm rated torque"*. Vì vậy mọi con số Nm ở mục 3.2 chỉ dùng để so bậc độ lớn.
Cần bảng rated specification từng trục từ Yaskawa mới quy đổi được.

### 3.4. Vì sao hiện tại effort bằng 0

Kiểm tra `/joint_states` trong phiên tối 19/09 cho `effort` toàn số 0, trong
khi log ngày 08–10/09 có giá trị.

**Đã giải quyết.** Người vận hành xác nhận `effort` bằng 0 vì **Servo đang
tắt**. Điều này giải thích luôn mẫu hình xen kẽ trong buổi 08/09, nơi
`handle_mass_20260908_115138` có 0% mẫu khác 0 còn `handle_mass_20260908_115410`
ghi sau vài phút có 100%. Không phải lỗi cấu hình, không cần điều tra thêm.

## 4. Ảnh Torque Spec và Tool Data

Bốn ảnh trong `images/` chụp 12/09.

`20240912_122553.png` và `20240912_140635.png` là Servo Monitor ở **cùng một
pose** (feedback pulse lệch dưới 2 xung):

| | S | L | U | R | B | T |
|---|---:|---:|---:|---:|---:|---:|
| 12:25 | 0 | 13 | −23 | 22 | 0 | −4 |
| 14:06 | 0 | 13 | −22 | 23 | −10 | 4 |

S, L, U, R lặp lại tốt. Nhưng **B đổi từ 0 sang −10 và T đổi dấu từ −4 sang
+4** ở cùng pose. Giữa hai lần chụp có ảnh `20240912_135149.png` là màn hình
Tool Data, nên nhiều khả năng Tool Data đã được chỉnh trong khoảng đó. Không
kết luận chắc từ ba ảnh.

`20240912_125508.png` ở pose khác cho L = 49 thay vì 13, xác nhận Torque Spec
thay đổi mạnh theo pose — đúng đặc tính của **tổng** torque bị chi phối bởi
trọng lực.

### Vấn đề Tool Data

Ảnh `20240912_135149.png` cho Tool 0 `TOOL_SP`:

```
X = 35,355 mm    Y = −35,355 mm    Z = 130,000 mm    Rx = Ry = Rz = 0
W = 2,350 kg
Xg = 0,000 mm    Yg = 0,000 mm     Zg = 0,001 mm
Ix = Iy = Iz = 0,000 kg·m²
```

Khối lượng 2,350 kg khớp con số toàn cụm đã ghi trong dự án. Nhưng **trọng tâm
được khai báo trùng gốc tool và toàn bộ inertia bằng 0**, trong khi hình học
thực tế có thanh sắt vươn 160 mm khỏi tâm Axia
(`hc_tool_geometry_frames_20260918_vi.md`).

Controller dùng chính Tool Data này để bù trọng lực và quán tính khi tính
external torque. Trọng tâm và inertia sai làm phép bù đó sai theo pose. **Đây
là ứng viên đáng kể cho sai số gain có cấu trúc** quan sát được tối 19/09: Z
hụt khoảng 31% ở đỉnh, X hụt 32% ở đáy, Y thì vượt — cùng dấu ở hai lượt độc
lập.

Không tự sửa Tool Data. Thay đổi nó ảnh hưởng safety/PFL và làm mọi calibration
hiện có mất hiệu lực; cần người phụ trách quyết định.

## 5. Giả thuyết — KHÔNG phải kết quả, và chưa có nguồn

### 5.0. Đính chính về xuất xứ

Bản đầu của tài liệu này viết `τ_self = τ_total − τ_external` như một sự thật
đã xác lập. **Không có nguồn nào cho khẳng định đó.** Nó được suy ra từ phương
trình động lực học tay máy tiêu chuẩn:

```
τ_actuator = M(q)q̈ + C(q,q̇)q̇ + g(q) + τ_friction + J(q)ᵀ F_ext
```

Chuyển vế cho `τ_actuator − J(q)ᵀF_ext = M q̈ + C q̇ + g + τ_friction`. Phần
đại số này là sách giáo khoa. Phần **chưa xác lập** là hai tín hiệu đo được có
thoả các đồng nhất thức đó hay không:

1. **Hai cảm biến khác nhau, ở hai phía hộp giảm tốc.**
   `hc_force_register_plan_20260912_vi.md` ghi cảm biến torque của HC nằm ở
   **đầu ra hộp giảm tốc**, còn Torque Spec là ước lượng từ **dòng motor**.
   Hai phía lệch nhau theo tỷ số truyền và theo ma sát/hiệu suất hộp số — mà
   ma sát chính là số hạng `τ_friction` ở trên, không nhỏ với tay máy có hộp số.
2. **Quy ước dấu và phía quy chiếu** của cả hai đều chưa xác định.
3. **Đơn vị**: M310 giải mã ra Nm; effort là tỷ lệ so với một rated chưa biết.
4. Tác giả tài liệu này **chưa tự đọc MotoROS2 Discussion #509**; nội dung của
   nó ở đây là do người vận hành thuật lại.

Vì vậy mục 5 dưới đây là **giả thuyết cần kiểm chứng**, không phải thiết kế
đã được biện minh.

### 5.1. Giả thuyết

```
τ_self(q)  =  τ_total(effort × rated)  −  τ_external(M310–M315)
F_robot    =  [ (J Jᵀ + λ²I)⁻¹ J · (τ_self − τ_self_baseline(q)) ]₁₋₃
```

`τ_self` sẽ là phần torque robot tự sinh: trọng lực, quán tính, ma sát và nỗ
lực bám quỹ đạo. Trừ baseline không tiếp xúc theo pose để khử trọng lực tĩnh.

**Giới hạn khái niệm ngay cả khi mọi đơn vị đã đúng:** `τ_self` bị chi phối bởi
trọng lực. Phần mang "ý định" là số hạng quán tính để tăng tốc dọc quỹ đạo, mà
trong co-carry chậm thì nhỏ so với trọng lực. Tách được nó đòi hỏi mô hình
trọng lực chính xác — tức quay lại phụ thuộc Tool Data và inertia của link.

Thứ tự làm, mỗi bước dừng được:

0. **Kiểm tra tính khả cộng, làm trước tiên.** Thu đồng thời `effort` và
   `M310–M315` ở 5–8 pose tĩnh **không tiếp xúc**, trải rộng theo J2/J3. Ở
   trạng thái đó `τ_external` phải về gần 0 còn `τ_total` phải biến thiên mạnh
   theo pose đúng dạng trọng lực. Nếu hai kênh không thoả điều này thì phép trừ
   ở mục 5.1 vô nghĩa, và việc khai báo lại Tool Data hay tìm bảng rated **cũng
   không cứu được**. Đây là phép thử rẻ nhất và nó quyết định toàn bộ hướng đi.
   Hiện **chưa có log nào ghi đồng thời hai kênh**, nên không làm offline được.
1. ~~Xác định vì sao `effort` bằng 0~~ — **đã giải quyết**: người vận hành xác
   nhận do Servo được tắt. Không phải lỗi cấu hình.
2. Sau bước 0, mới đánh giá `τ_total` so với mô hình trọng lực để ước lượng
   rated theo cách độc lập.
3. Lấy bảng rated specification từ Yaskawa. Không dùng URDF effort limit.
4. Chỉ sau khi 1–3 đạt mới thu lượt LEADER có giằng co dài 30–45 giây và kiểm
   tra `Φ` tính theo `τ_self` có đổi dấu không.

### Đường thứ hai, rẻ hơn nhiều

`simulation_hri/inner_loop.py:105` định nghĩa lực robot là

```python
f_r = K_p @ (x_d - x_robot) + K_d @ (0 - dx_robot)
```

Đây là **output của luật điều khiển PD tính từ sai số bám reference**, không
phải đại lượng đo. Trên robot thật cả `x_d` và `x_robot` đều có sẵn chính xác
trong phần mềm, nên `f_r` tính được y hệt: không cảm biến, không calibration,
không trễ 145 ms, ở nhịp controller thay vì 5 Hz.

Dữ liệu tối 19/09 ủng hộ: trong pha LEADER, `cos(F_ext, vận tốc reference)`
âm 20% số mẫu — sai số bám reference **có** mang tín hiệu giằng co đúng lúc nó
xảy ra, còn Φ theo M310 thì không.

Nếu mục tiêu là conflict và role arbitration thì đường này trả lời được mà
không cần thêm một lượt robot nào. Đường ở đầu mục 5 đáng làm nếu cần lực vật
lý thật, không chỉ tín hiệu ý định.

## 6. Những gì vẫn đúng từ công việc trước

- `F_robot` từ M310 vẫn là ước lượng ngoại lực độc lập với Axia. Nó dùng được
  cho chẩn đoán và cho phát hiện va chạm ngoài ý muốn, chỉ không dùng được để
  đo xung đột ý định.
- Candidate `20260919_t1_combined_v3` vẫn đóng băng, chưa deploy,
  `calibration_confirmed=false`, `role_valid=false`. Nó hụt ngưỡng nghiệm thu
  1,3% và tệ hơn runtime 48,8% trên lượt `194241`, nên chưa sẵn sàng bất kể
  hướng đi nào được chọn.
- Các phép đo reader vẫn nguyên giá trị: 5,05 Hz, span sáu kênh 144,8 ms, sai
  lệch vector torque 3,04 Nm trung vị khi chuyển động. Xem
  `m310_reader_rate_audit_20260919_vi.md`.

## 7. Tái lập

```bash
cd /home/hungnb/cocarry_ws
OPENBLAS_NUM_THREADS=1 python3 scripts/plot_f_robot_vs_f_ext.py \
  cocarry_logs/cocarry_admittance_3d_20260919_200407.csv \
  --output <thu_muc_moi>
```

Dữ liệu effort lịch sử: `cocarry_logs/torque_validation/*.csv`, cột
`joint_effort_j1..j6`. **Toàn bộ các file có effort đều ở cùng một pose**
(J2 = +0,028, J3 = −0,711 rad), nên chúng không kiểm chứng được phụ thuộc pose
và không dùng để fit rated được. Không file nào có cột M310. Ảnh pendant: `images/20240912_*.png`. Node effort cũ:
`src/hc10dtp_bringup/scripts/sensorless_force_node.py`, tham số
`effort_unit_mode` với bốn chế độ `raw_only`, `torque_nm`,
`normalized_rated_torque`, `custom_scale`.
