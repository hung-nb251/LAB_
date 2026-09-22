# Tiến độ CoCarry và kế hoạch tiếp theo — 18/09/2026

## 1. Phạm vi và quyết định hiện tại

Ngày 18/09 tập trung vào hai nhóm việc: kiểm chứng phép ước lượng
`F_robot` từ M310–M315 và rà soát các nguyên nhân làm tín hiệu lực, phản hồi
chuyển động chậm hoặc chưa đúng kỳ vọng.

Không thu lại diện rộng baseline tĩnh hoặc các tuyến động đã có. Kho dữ liệu
hiện tại đã có 61 log `events.jsonl`, 12 pose tĩnh không tiếp xúc, các lượt
Ground Truth, GRU, GRU+MJM và bảy lượt GRU mới đến Target 1/2. Công việc tiếp
theo phải khai thác lại toàn bộ tập này trước; chỉ thu thử nghiệm robot tối
thiểu sau khi phân tích offline chỉ ra chính xác dữ liệu còn thiếu.

Không thay đổi force limit, workspace/joint limit, Tool Data, Torque Origin
hoặc đưa `F_robot` ước lượng vào điều khiển tại thời điểm cập nhật tài liệu này.

## 2. Công việc đã hoàn thành trong ngày

### 2.1. Hình học và frame

- Đã đối chiếu cụm CAD với lắp đặt thực tế, xác nhận tâm Axia là tâm vòng sáu
  lỗ, tâm flange là tâm vòng bốn lỗ và ký hiệu Y+ trên ảnh là Y+ của Axia.
- Đã làm rõ rằng lực Axia mà pipeline hiện tại publish đã được đổi sang
  `base_link`. Không tự thêm một phép quay yaw chỉ vì tên frame cảm biến.
- Đã ghi nhận hình học flange, tấm obround, Axia và thanh để phục vụ kiểm tra
  moment arm. Khối lượng và trọng tâm vẫn có thể bổ sung sau; chúng không ngăn
  việc audit thuật toán và dữ liệu hiện có.

### 2.2. Hai nhánh thanh ghi

- Đã so sánh nhánh M320–M325 và nhánh M310–M315 trên dữ liệu tĩnh/động đã có.
  M310–M315 được giữ làm nguồn torque khớp cho phép kiểm chứng
  `J(q)^T W = tau`; M320–M325 chưa được coi là ground truth thay cho Axia nếu
  chưa có định nghĩa chính thức và calibration độc lập.
- Đã thu bảy lượt GRU mới Home→Target 1/2 có M310–M315 và dùng chúng để fit,
  đối chứng candidate.
- Runtime hiện tính nghiệm regularized của `J(q)^T W=tau`, xuất thành phần lực
  `f_robot_x/y/z` trong `base_link`. Đây vẫn là kênh shadow để phân tích, chưa
  tham gia Admittance Control hay chọn vai trò.
- CSV chính đã được rút gọn còn ba cột lực robot và sáu cột torque khớp. Logger
  tạo thêm sidecar `<csv>.calibration.jsonl` chứa M310 raw, baseline, q/q0,
  model snapshot, Axia raw/processed và marker mà không đọc thanh ghi lần hai.

### 2.3. Kết quả audit calibration

- Replay raw M310 của lượt 17:22 khớp kết quả runtime tới khoảng
  `2.3e-7 N`; chưa thấy lỗi khác nhau giữa công thức offline và runtime.
- Candidate hiện tại đạt RMSE khoảng `2.16 N` trên T2-r4, nhưng khoảng
  `7.14 N` trên lượt runtime 17:22. Mô hình baseline riêng + gain 6×6 đạt lần
  lượt `2.49 N` và `6.28 N`; chưa cải thiện nhất quán để thay candidate.
- Lượt 17:42 còn khoảng `6.82 N` RMSE và sai lệch hướng lớn ở một số mẫu.
  Vì vậy calibration `F_robot` chưa hoàn chỉnh và chưa đủ tin cậy để đóng
  vòng điều khiển.
- Audit trước mới dùng hai đoạn `baseline_end` của bảy lượt mới nên kết luận
  rank=2 chỉ mô tả tập con đó. Nó không chứng minh toàn bộ dữ liệu baseline
  sẵn có thiếu hạng. Bước đúng tiếp theo là hợp nhất và chấm lại 12 pose tĩnh
  cùng các log cũ, không thu lại ngay.

### 2.4. Kiểm chứng phần mềm

- Các thay đổi về estimator/logger/audit đã qua 123 test và build package liên
  quan. Chưa chạy thêm robot thật sau các thay đổi phần mềm này.

## 3. Bốn vấn đề hệ thống mới

### 3.1. Bộ lọc Axia: nhiễu xung và độ trễ

Lựa chọn UI ghi là `Raw/Không lọc, alpha=1.0` hiện chưa phải raw thực sự.
`ForceFilter` vẫn dùng median cửa sổ 5 mẫu trước EMA; `alpha=1.0` chỉ bỏ tác
dụng EMA. Cấu hình lọc mạnh hơn tiếp tục median-5 rồi EMA, nên có thể triệt
nhiễu nhưng tạo trễ đáng kể đối với ý định con người.

Không nên chọn một alpha cố định bằng cảm giác. Cần dùng Axia raw trong sidecar
để replay các phương án causal, ưu tiên:

1. loại xung đơn lẻ bằng Hampel hoặc median cửa sổ ngắn;
2. lọc thích nghi/dual-path: bám nhanh khi lực thay đổi có chủ đích, lọc mạnh
   hơn khi đứng yên;
3. so sánh thêm One Euro hoặc low-pass bậc hai được đặt theo tần số cắt;
4. giữ deadband và force safety là hai khâu riêng, không dùng deadband để che
   chất lượng bộ lọc.

Chỉ chọn bộ lọc sau khi đo RMS/P95 khi không tải, tỷ lệ xung còn sót, độ trễ
10–90%, sai số hướng lực và độ trễ qua ngưỡng ý định. Candidate đầu tiên sẽ
chạy offline rồi shadow; chưa đưa thẳng vào robot.

### 3.2. Robot chậm nhưng tăng jerk lại mất mượt

Pipeline hiện có nhiều khâu cùng làm giảm băng thông:

```text
GRU + F_ext
  -> prediction reference: low-pass tau=0.4 s + velocity lead
  -> Admittance và giới hạn vận tốc/gia tốc
  -> target pose
  -> Cartesian streamer: tracking + giới hạn jerk/gia tốc/vận tốc
  -> IK, đồng bộ khớp, queue và command-lead limit
```

Hạ `reference_tau` đồng thời tăng jerk chỉ làm một khâu nhanh hơn trong khi
các khâu sau vẫn giới hạn tín hiệu, nên đầu ra có thể gắt mà robot không nhanh
tương ứng. Cần đo trễ từng tầng bằng timestamp và các trường raw prediction,
nominal/reference/actual cùng `motion_diagnostics_json` đã có.

Sau audit, thiết kế nên để một khâu chịu trách nhiệm chính về tạo profile mượt,
và các khâu sau giữ vai trò giới hạn an toàn. Các candidate cần replay/mô phỏng
A/B gồm:

- cấu hình hiện tại làm mốc;
- reference nhanh hơn nhưng có rate/acceleration bound rõ ràng;
- reference bậc hai critically damped hoặc jerk-limited, có velocity
  feed-forward/lead và tránh lọc lặp cùng streamer.

Tiêu chí gồm thời gian đạt 90% dịch chuyển, peak velocity, tracking error,
jerk tích phân/P95, số lần chạm command-lead/limit, lỗi IK và queue. Chỉ đổi
một tầng mỗi lần để xác định tác động thật.

### 3.3. Đường LEADER thẳng và Minimum Jerk

Code hiện dùng

```text
p(t) = p0 + (10s^3 - 15s^4 + 6s^5) (p1 - p0)
```

Do phần không gian luôn là vector `(p1-p0)`, quỹ đạo hình học từ Home đến
Target là đường thẳng. Dạng S của hàm bậc 5 nằm trên đồ thị tiến độ vị trí theo
thời gian; vận tốc có dạng chuông và gia tốc/jerk được làm liên tục ở biên.
Vì vậy quan sát đường thẳng trong không gian là đúng với lý thuyết, không phải
bằng chứng rằng smoothing đã xóa một đường cong.

Pha LEADER bỏ qua Admittance và prediction-reference thông thường, nhưng vẫn
qua giới hạn workspace/lead và Cartesian streamer. Những khâu này có thể làm
profile thời gian trễ hoặc méo. Cần vẽ tiến độ dọc đường, vận tốc, gia tốc,
jerk và cross-track error theo thời gian để kiểm chứng. Nếu yêu cầu thực sự là
đường cong trong không gian, phải bổ sung waypoint/Bezier/spline hoặc một path
planner khác; đó là yêu cầu hình học khác với Minimum Jerk hai điểm.

### 3.4. Tốc độ M310–M315

Node hiện gọi `/read_mregister` sáu lần tuần tự, mỗi lần đọc một địa chỉ, rồi
chờ `scan_gap_sec=0.05`. Với khoảng 20 ms/service, một vector đầy đủ chỉ đạt
xấp xỉ 5 Hz và sáu thành phần không có cùng thời điểm đo. Chỉ giảm scan gap về
0 có thể tăng một phần, nhưng giới hạn sáu vòng DDS/service vẫn còn và dự kiến
khó đạt nhịp điều khiển 15 Hz.

Giải pháp đích là một API batch phía controller: đọc M310–M315 trong một chu
kỳ/call, trả về vector sáu phần tử với một timestamp, hoặc publish trực tiếp
một topic vector. Việc này cần tìm lại source/build của MotoPlus service tùy
biến và kiểm tra API controller có đọc block liên tiếp hay phải lặp cục bộ.
Workspace hiện chỉ có interface scalar `ReadMRegister.srv`, chưa có source
server để sửa.

Không gọi song song mù sáu service scalar khi robot đang chạy: server/controller
có thể vẫn serialize request, tăng tải và trộn mẫu khác thời điểm. Thử nghiệm
đầu tiên phải thực hiện khi robot Stop/Disable:

1. đo rate, jitter và tải với `scan_gap=0` để có giới hạn thực nghiệm;
2. tìm source và quy trình build/deploy controller-side của service;
3. thiết kế `ReadMRegisters` hoặc topic batch; test trên controller dừng;
4. mục tiêu ban đầu là vector nguyên tử ít nhất 15 Hz; chỉ nâng 20–50 Hz nếu
   đo được controller và mạng còn đủ tải;
5. sau khi rate ổn định mới giảm timeout 0.40 s và đánh giá lại estimator.

MotoROS2 upstream hiện vẫn liệt kê đọc/ghi controller variables trong roadmap,
nên service đang dùng nhiều khả năng là phần mở rộng cục bộ, không thể giả định
upstream đã có sẵn API batch. Tham chiếu:
<https://github.com/Yaskawa-Global/motoros2>.

## 4. Thứ tự công việc tiếp theo

### P0 — Hợp nhất dữ liệu calibration đã có

- Lập manifest cho toàn bộ 61 log và gắn nhãn: static no-contact, dynamic
  no-contact, force interaction, Ground Truth, GRU, GRU+MJM.
- Đưa 12 pose tĩnh và các đoạn động hợp lệ vào cùng audit; kiểm tra rank,
  condition number, độ phủ joint/pose và sự ổn định theo session.
- Chia train/validation/test theo session và tuyến chạy, không chia ngẫu nhiên
  từng mẫu. So sánh baseline hằng, baseline phụ thuộc pose, gain 6×6 và model
  có regularization bằng RMSE, sai số hướng, residual không tải.
- Chỉ đề xuất thu thêm đúng pose/hướng lực mà phép kiểm tra observability chỉ
  ra là còn thiếu.

### P1 — Benchmark bộ lọc Axia offline

- Tách rõ raw thật, median output, EMA output và sau deadband trong replay.
- Chạy benchmark các candidate causal trên log có xung, không tải và thao tác
  có chủ đích. Chọn một candidate theo metric rồi chạy shadow.

### P2 — Audit băng thông toàn pipeline

- Dùng log hiện có dựng đồ thị raw GRU → nominal → reference → target → actual.
- Ước lượng delay/cutoff và thống kê lần chạm các limiter ở từng tầng.
- Replay/mô phỏng các candidate reference governor; chưa đổi đồng thời jerk,
  acceleration và tau trên robot thật.

### P3 — Kiểm chứng Minimum Jerk theo thời gian

- So sánh tiến độ dọc đường thực với quintic lý thuyết; đo thời gian, vận tốc,
  gia tốc, jerk và cross-track error.
- Chỉ mở nhánh thiết kế đường cong không gian nếu đó là yêu cầu nhiệm vụ mới.

### P4 — Tăng tốc đọc M310–M315

- Benchmark scalar service không có scan gap khi robot dừng.
- Tìm source custom MotoPlus/MotoROS2 và làm prototype batch six-register.
- Nếu source không còn, tiếp tục trao đổi với maintainer/Yaskawa dựa trên bài
  Discussion #509, nêu rõ nhu cầu vector nguyên tử >=15 Hz thay vì chỉ hỏi cách
  đọc từng thanh ghi.

### P5 — Thử robot tối thiểu sau khi có kết quả offline

Nếu P0–P4 chỉ ra candidate rõ ràng, thực hiện lần lượt một thử nghiệm filter có
kiểm soát, một lượt GRU T1, một lượt GRU T2 và một lượt LEADER. Không lặp lại
toàn bộ baseline tĩnh/động. Mỗi thay đổi phải có cấu hình mốc và tiêu chí dừng.

## 5. Trạng thái cuối ngày

`F_robot` từ M310–M315 đã có đường tính vật lý, dữ liệu raw và khả năng replay,
nhưng calibration chưa đủ chính xác để dùng trong điều khiển. Việc ưu tiên ngày
tiếp theo là phân tích dữ liệu đã có, đo băng thông từng tầng và thiết kế đọc
batch thanh ghi. Chưa có lý do để thu lại baseline diện rộng hoặc thay đổi ngay
các giới hạn làm robot mất mượt.
