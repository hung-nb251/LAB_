# Tổng kết tiến độ calibration F_robot

Cập nhật: 17/09/2026, sau trial Target 2 tĩnh lúc 18:05:09.

## 1. Kết luận hiện tại

**Đã có dữ liệu và bộ hệ số calibration cục bộ tại các pose cố định. Chưa xác nhận được một bộ calibration dùng liên tục khi robot di chuyển trên toàn tuyến Home–Target 1–Target 2.**

- Đã đọc được register, giải mã đơn vị và ghi đồng thời Axia, joint states, TF, trạng thái chạy cùng marker thao tác.
- Đã thu 12 pose tĩnh hợp lệ phục vụ khảo sát tải/CoG, các bài lực Ground Truth, các bài tác động lực khi robot đứng yên, và các lượt GRU/GRU+MJM.
- Đã fit ma trận lực 3×3 cục bộ tại Home, Target 1, gần Target 2 và đúng Target 2.
- Đã kiểm chứng một phần bằng lượt tác động khác hoặc giữ một pha ngoài tập fit; kết quả tốt hơn khi robot đứng tại pose đã đo.
- Chưa có mô hình baseline theo góc khớp được kiểm chứng cho chuyển động. Chưa chốt được độ trễ tổng của tín hiệu lực.
- Chưa triển khai bộ hệ số vào điều khiển thật hoặc bật tự chuyển vai bằng conflict. Không được hiểu các file `candidate` là calibration đã được xác nhận toàn hệ thống.

## 2. Mục tiêu và đại lượng đang sử dụng

Mục tiêu của người vận hành là dùng F_robot cùng F_ext để tính conflict trong outer loop, kết hợp confidence của bộ phân loại đích để chọn LEADER/FOLLOWER.

Chỉ số hướng lực đang xét:

\[
\phi(t)=\frac{F_{robot}^{T}F_{ext}}{\|F_{robot}\|\,\|F_{ext}\|}
\]

Đây là tích vô hướng hai lực, không phải đạo hàm F_ext. Ngưỡng quyết định có thể cấu hình theo người dùng, không bắt buộc bằng 0. Chưa chọn ngưỡng, thời gian duy trì hay hysteresis từ dữ liệu có nhãn đồng thuận/xung đột.

| Nguồn | Ý nghĩa theo tài liệu đã đối chiếu | Tình trạng sử dụng |
|---|---|---|
| M310–M315 | Estimated external joint torque | Đã thu trong các bài trước; chưa có nhánh Jacobian được kiểm chứng toàn hành trình |
| M320–M322 | Estimated external TCP force | Nguồn chính của các phép fit lực gần đây |
| M323–M325 | Estimated external TCP moment | Ghi cùng lực trong nhóm `wrench` |
| M330–M335, M340–M345 | Sensor channels CH1/CH2 | Giữ raw; chưa tự gán cùng scale với external wrench |
| Axia raw wrench | Wrench cảm biến trong frame sensor | Tham chiếu chính cho phân tích calibration hiện tại |
| Axia human force | Lực đã qua xử lý UI, trong base_link | Kiểm tra tính nhất quán và thao tác; không mặc định là raw không deadband |

Với M310–M315 và M320–M325, quy đổi đang dùng là `(raw - 10000) × 0.1`, đơn vị Nm hoặc N tương ứng. Service thực tế là `/read_mregister`.

**Giới hạn về định nghĩa:** register trên là ước lượng ngoại lực; phép fit khớp Axia chưa chứng minh đó là lực truyền động độc lập hay ý định chủ động của robot. Góc giữa hai nguồn đo lực không đồng nghĩa tỷ lệ phân loại conflict đúng. Ý nghĩa của `J^{-T} tau` cũng phụ thuộc loại torque đầu vào và frame/quy ước dấu.

## 3. Cấu hình và công cụ đã chuẩn bị

- Khối lượng toàn cụm do người vận hành cung cấp: khoảng **2,350 kg**.
- Kích thước báo cáo: flange→Axia 120 mm; Axia→tâm thanh ngoài 160 mm; tấm đệm 10 mm và 8 mm. Chưa dùng các khoảng cách này để khẳng định CoG/inertia toàn cụm.
- Giữ nguyên URDF và hiệu chỉnh RPY Axia theo yêu cầu. Phân tích dùng TF ghi trong log và hiệu chỉnh yaw −90° theo source hiện có; source snapshot không xác nhận được giá trị UI runtime.
- Phân tích động đang giả định tải phía Axia 1,126 kg theo source. Đây là tải cảm biến đỡ, khác với khối lượng toàn cụm 2,350 kg.
- Terminal 4 chạy [hc_force_trial_logger.py](../scripts/hc_force_trial_logger.py); Terminal 5 chạy [hc_force_marker_cli.py](../scripts/hc_force_marker_cli.py).
- Logger hỗ trợ nhãn `static`, `ground_truth`, `gru`, `gru+mjm`; chỉ đọc dữ liệu. `--group all` đọc 24 register; `--group wrench` đọc M320–M325.
- Marker giúp phân đoạn baseline/tác động/release. Khi marker gửi muộn, dùng joint states và CSV controller để xác định trạng thái thật.

## 4. Dữ liệu khi robot đứng yên

### 4.1. Pose tĩnh không tiếp xúc để khảo sát tải và CoG

Session: `cocarry_logs/hc_force_calibration/20260916_tool0_cog_v1/01_static_cog/`.

Có **12 pose hợp lệ**; pose 03 thiếu joint states, đã thay bằng pose 13. Đã tổng hợp raw wrench, góc khớp và register.

Phép fit sơ bộ `tau = r × F + bias` cho CoG trong frame sensor khoảng `[-6,61; -0,98; 8,24] mm`. Đây chỉ là kết quả mô hình sơ bộ, **không phải CoG toàn cụm đã xác nhận để nhập Tool Data**. Bias cảm biến, phạm vi orientation và phân bố khối lượng vẫn là giới hạn nhận dạng. Không suy được inertia đáng tin cậy từ các pose đứng yên.

Báo cáo cũ ghi độ phân tán hướng raw force khoảng 8,68°. Không được tự đồng nhất hướng raw có bias với hướng trọng lực đã xác minh bằng TF.

Tham khảo: [phân tích CoG](../cocarry_logs/hc_force_calibration/20260916_tool0_cog_v1/03_analysis/static_cog_analysis_vi.md).

### 4.2. Tác động lực khi robot giữ pose

Các trial này thường nằm trong thư mục `02_dynamic_force` do cách tổ chức session, nhưng **robot thực tế đứng yên**. Không phân loại chuyển động chỉ dựa vào tên thư mục.

Root A: `cocarry_logs/hc_force_calibration/20260917_tool0_force_v1/02_dynamic_force/`.

| Pose thật | Trial tương đối dưới root A | Nội dung và chất lượng |
|---|---|---|
| Target 1 | `target1/20260917_125927_479349_target1_static_force` | Sáu hướng X±/Y±/Z±, mỗi hướng 2 lượt; 989 scan hoàn chỉnh; không timeout |
| Home | `home/20260917_132544_477259_home_static_force_validation` | X−, Y±, Z+; 373 scan hoàn chỉnh |
| Home | `target1/20260917_133451_892918_target1_z` | Bổ sung X+ và một số Y+/Z+; tên thư mục sai pose, joint states xác nhận Home |
| Home | `home/20260917_142459_050508_home_static_force_validation` | Bổ sung Z−; 364 scan hoàn chỉnh |
| Gần Target 2 | `new_pose/20260917_143656_102660_new_pose_static_force_validation` | Giữ X±, Y+, Y− lượt 1; 665 scan trước timeout M320; loại Y− lượt 2 và phần sau lỗi khỏi fit cặp M/Axia |
| Gần Target 2 | `new_pose/20260917_145157_643170_new_pose_static_force_validation` | Bổ sung Y−, Z±; 546 scan hoàn chỉnh; không timeout |

Root B: `cocarry_logs/hc_force_calibration/20260917_tool0_gru_validation_v1/02_dynamic_force/`.

| Pose thật | Trial tương đối dưới root B | Nội dung và chất lượng |
|---|---|---|
| Đúng Target 2 | `target2/20260917_180509_330340_target2_static_force_validation` | Sáu hướng X±/Y±/Z±, mỗi hướng một lượt; 1.485 scan hoàn chỉnh; không timeout; biến thiên joint tối đa 0,000065 rad |

`new_pose` được người vận hành xác nhận là **gần Target 2**, nhưng không trùng pose Target 2 chính thức. Không gộp hai baseline thành một pose.

### 4.3. Kết quả calibration tĩnh

Công thức candidate:

\[
\hat F_{base}=A\,[F_M-b_M]
\]

`A` là ma trận 3×3 có xét ảnh hưởng chéo giữa trục; `b_M` là baseline không tiếp xúc tại pose đo. Axia raw cũng được trừ baseline tại cùng pose rồi đổi frame. Việc trừ baseline ở pose cố định loại phần bias/trọng lực không đổi nhưng không tạo ra mô hình bù khi đổi pose.

| Pose / cách kiểm chứng | RMSE vector | Góc trung vị | Góc P95 | Giới hạn |
|---|---:|---:|---:|---|
| Target 1, fit lượt 1 → kiểm tra lượt 2 | 2,83 N | 4,81° | 9,04° | Kiểm chứng khác lượt tại cùng pose |
| Home, fit lượt 1 → kiểm tra lượt 2 | 4,85 N | 4,49° | 7,01° | Tập lượt 2 chỉ có Z−; không đại diện đủ sáu hướng |
| Gần Target 2, fit lượt 1 → lượt 2 | 3,64 N | 6,03° | 15,13° | Riêng Z− P95 khoảng 22,9° |
| Đúng Target 2, fit và đánh giá cùng dữ liệu | 2,52 N | 5,62° | 16,20° | Chưa có lượt lặp độc lập |

Góc chỉ tính khi norm cả reference và estimate ≥4 N; đây là điều kiện phân tích, không phải ngưỡng conflict. RMSE dùng toàn bộ mẫu trong tập tương ứng. Không coi mẫu bị loại do lực nhỏ là mẫu đo đúng.

Tại đúng Target 2, giữ từng hướng ngoài tập fit cho thấy Y− và Z+ còn yếu: góc P95 khoảng **26,0° và 24,7°**. X− có sai số góc thấp nhưng RMSE vector lên khoảng **7,89 N**, cho thấy đúng hướng chưa đồng nghĩa đúng độ lớn.

Khi giữ nguyên một pose ngoài tập học, mô hình cũ dự đoán Home có góc P95 **54,56°**, dù đã có baseline tại Home. Vì vậy chưa thể coi các ma trận cục bộ là một calibration chung.

Kết quả: [calibration ba vùng](../cocarry_logs/hc_force_calibration/20260917_tool0_force_v1/03_analysis/vector_calibration/report_vi.md), [Target 2 chính thức](../cocarry_logs/hc_force_calibration/20260917_tool0_gru_validation_v1/03_analysis/target2_static_180509/report_vi.md), [hệ số Target 2](../cocarry_logs/hc_force_calibration/20260917_tool0_gru_validation_v1/03_analysis/target2_static_180509/target2_static_candidate.json).

## 5. Dữ liệu khi robot thực sự di chuyển

### 5.1. Ground Truth đã thu trước đó

Đã thu người tác động lực và robot di chuyển bằng admittance: Home X±, Y±, Z+ và Target 1 X±, Y±, Z+. Các nguồn đáng chú ý:

- Home X±: `cocarry_logs/hc_force_trials/20260915_141843_943366/`.
- Home Y±: `20260916_tool0_force_v1/02_dynamic_force/home/20260916_142750_583700_home_y/` dưới root calibration.
- Home Z+: `20260917_tool0_force_v1/02_dynamic_force/home/20260917_093158_901998_home_z/`.
- Target 1 X±: root A, `target1/20260917_101035_730907_target1_x` — 381 scan hoàn chỉnh, giữ dữ liệu.
- Target 1 Y±: root A, `target1/20260917_102540_949607_target1_y` — 324 scan hoàn chỉnh, thay trial Y lúc 10:16 bị timeout.
- Target 1 Z+: root A, `target1/20260917_104200_811194_target1_z` — giữ hai lượt đầu; timeout M321 ở scan 118, loại lượt 3 và phần sau khỏi fit cặp register/Axia.

Những bài này có giá trị khảo sát đáp ứng động, nhưng pose/baseline thay đổi trong lúc tác động và một số trial mất register. Không fit một gain đơn giản từ lực động rồi coi đó là calibration toàn workspace.

Quyết định giữ/loại chi tiết: [dynamic_trial_selection.json](../cocarry_logs/hc_force_calibration/20260917_tool0_force_v1/03_analysis/dynamic_trial_selection.json).

### 5.2. GRU và GRU+MJM ngày 17/09

Các đường dẫn dưới root B:

| Trial | Scan hoàn chỉnh / bắt đầu | Kết quả chuyển động và dữ liệu |
|---|---:|---|
| `home_to_target1/20260917_164311_423088_gru_home_to_target1` | 547/547 | Không timeout; có đoạn GRU chạy và đoạn đứng yên cuối |
| `home_to_target2/20260917_165901_639040_gru_home_to_target2` | 669/670 | Không timeout register; controller mất readiness khoảng 52,42 s; không xem đoạn sau lỗi là GRU chạy thành công |
| `home_to_target2/20260917_173750_951557_gru_mjm_home_to_target2` | 511/512 | MJM đã tới T2 rồi trở lại FOLLOWER; GRU kéo robot rời đích khoảng 124 mm trước khi dừng; baseline cuối thuộc pose khác |
| `home_to_target2/20260917_174621_964444_gru_mjm_home_to_target2` | 450/451 | Đã dừng đúng T2, sai số theo feedback khoảng 0,035 mm; baseline ổn định |

Scan cuối dở khi đóng logger không đồng nghĩa timeout. Các sai số vị trí rất nhỏ ở bảng là sai số theo feedback/FK so với target lưu, không phải phép đo độ chính xác cơ khí tuyệt đối.

Tốc độ đọc M320–M325 khoảng **4,8–4,9 Hz**; Axia khoảng **100 Hz**. Response service trung vị khoảng **20 ms**, P95 khoảng **30 ms**. M320–M322 trong cùng scan được đọc lệch nhau khoảng **50 ms** trung vị, không phải một vector lấy mẫu đồng thời.

Thử ma trận Home với baseline đầu lượt cố định:

| Lượt | RMSE vector | Góc trung vị | Góc P95 |
|---|---:|---:|---:|
| GRU → T1 lúc 16:43 | 8,73 N | 66,3° | 97,9° |
| GRU → T2 lúc 16:59, trước lỗi | 8,53 N | 27,6° | 62,6° |
| GRU+MJM lúc 17:37 | 8,06 N | 27,5° | 52,6° |
| GRU+MJM lúc 17:46 | 6,28 N | 30,6° | 50,7° |

Đây là phép thử cách bù cố định cũ trên mẫu RUNNING, không phải đánh giá một mô hình baseline theo pose đã hoàn chỉnh. Nội suy baseline đầu–cuối theo thời gian cũng chưa khắc phục đủ; phép này dùng thông tin tương lai nên chỉ có ý nghĩa chẩn đoán offline.

Ở các đoạn MJM ít ngoại lực, số mẫu human_force sau deadband đủ lớn rất ít. Riêng lượt 17:46 chỉ có 2 cặp mẫu human_force ≥4 N theo phép ghép đang dùng; raw trước deadband có nhiều mẫu hơn. Hai tập đó không được đồng nhất. MJM tới đích xác nhận được pose/baseline nhưng không thay bài tác động lực có hướng.

Chi tiết: [báo cáo validation động](../cocarry_logs/hc_force_calibration/20260917_tool0_gru_validation_v1/03_analysis/report_vi.md).

## 6. Vấn đề hiện tại

### Baseline phụ thuộc pose

Ví dụ cùng lượt 17:46:

- Home: `[21,8; −10,0; 11,1]` N.
- Target 2: `[11,3; −7,1; 6,6]` N.

Tại T2 tĩnh lúc 18:05, baseline là `[11,1; −6,2; 6,3]` N. Có sai khác giữa lần đo, nên không coi các vector này là hằng số tuyệt đối.

Một offset tại Home không đủ dùng khi đổi pose. **Đã có baseline ở các điểm chính nhưng chưa có hàm b(q) được kiểm chứng giữa các điểm đó.** Những thử nghiệm hồi quy sơ bộ chưa đủ cơ sở để xuất thành mô hình triển khai.

### Độ lớn, ảnh hưởng chéo và khả năng tổng quát hóa

Ma trận cục bộ cải thiện hướng lực, nhưng sai số thay đổi theo pose và hướng tác động. Chưa xác định được bao nhiêu sai số đến từ đáp ứng cảm biến, bù tải, drift, độ trễ hay phương pháp fit. Không kết luận mọi sai khác gain đều là đặc tính vật lý phụ thuộc pose.

### M321 từng đổi dấu bất thường

Lượt 16:59, tại đoạn cuối robot đứng yên: M321 đổi giữa khoảng +7,5 và −7,5 N, 54 lần đổi dấu/156 mẫu; Axia và joint feedback ổn định. Raw response thực sự có hai cụm giá trị, không phải do ma trận fit.

Hai lượt sau không tái diễn; lượt 17:46 đứng đúng T2 có M321 âm ổn định. Chưa xác định nguyên nhân lần trước. Không được ép dấu hoặc lấy trị tuyệt đối để che hiện tượng. Không có cơ sở quy lỗi chắc chắn cho GRU hoặc service chỉ từ các dữ liệu này.

### Độ trễ chưa được nhận dạng chắc chắn

20 ms của service không phải toàn bộ độ trễ cảm biến/ước lượng. Tương quan có thể bị lẫn biến đổi baseline, lực nhỏ, filtering và lệch thời điểm từng register. Chưa đủ bằng chứng để áp một bù trễ cố định, chẳng hạn 0,34 s.

### Giới hạn đánh giá conflict

Chưa có tập nhãn đồng thuận/chống hướng độc lập cùng tín hiệu ý định robot. Sai số góc giữa register và Axia chưa chứng minh quyết định LEADER/FOLLOWER đúng. Với lực nhỏ hoặc stale cần trạng thái không đủ thông tin; ngưỡng tùy người dùng không tự khắc phục được sai baseline hoặc sai dấu.

## 7. Việc tiếp theo, không yêu cầu thu lại toàn bộ

1. Giữ nguyên các log đã có và quy tắc loại đoạn lỗi. Lượt Target 2 18:05 dùng được, không cần thu lại vì chất lượng ghi.
2. Xây dựng và kiểm chứng bù baseline theo joint pose từ dữ liệu không tiếp xúc đã có; kiểm tra khác biệt giữa session và giới hạn vùng áp dụng.
3. So sánh mô hình gain đơn giản và ma trận có ảnh hưởng chéo trên các pha/lượt/pose được giữ ngoài tập fit. Không dùng cùng dữ liệu để vừa chọn mô hình vừa tuyên bố độ chính xác độc lập.
4. Ghép dữ liệu theo thời điểm từng register và đánh giá độ trễ trên đoạn lực đủ rõ. Chỉ thu bổ sung nếu xác định được phần thiếu cụ thể.
5. Sau khi đạt kiểm chứng offline, chạy estimator chỉ để ghi/quan sát trong phạm vi đã xác nhận; tiếp đó mới kiểm chứng logic conflict và ngưỡng người dùng. Chưa tự bật chuyển vai.

## 8. Mã và tài liệu để tiếp tục công việc

- [Kế hoạch register và nguồn tài liệu](hc_force_register_plan_20260912_vi.md).
- [Quy trình session và marker](hc_force_calibration_session_runbook_vi.md).
- [Đối chiếu định nghĩa conflict](hc_force_conflict_calibration_next_steps_20260917_vi.md).
- [Phân tích pose CoG](../scripts/analyze_hc_static_cog.py).
- [Fit ma trận lực tĩnh](../scripts/calibrate_hc_register_force.py).
- [Kiểm chứng GRU](../scripts/analyze_hc_gru_validation.py).
- [Phân tích Target 2 tĩnh](../scripts/analyze_hc_target2_static.py).

Các báo cáo lịch sử có kết luận theo dữ liệu tại thời điểm đó; bản này phân biệt rõ kết quả fit, kiểm chứng tại pose cố định và kiểm chứng khi chuyển động. Chưa có bằng chứng hoàn tất calibration toàn hành trình, CoG/inertia toàn cụm hoặc calibration an toàn của controller.
