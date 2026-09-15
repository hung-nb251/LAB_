# Kịch bản test Hybrid robot thật — chuẩn bị họp 2026-09-12

## 1. Phạm vi có thể chứng minh hiện tại

Runtime hiện tại đã có:

- lưu hai đích tuyệt đối Target 1/2 và chọn đích rõ ràng;
- FOLLOWER dùng GRU + Admittance;
- LEADER dùng MJM trực tiếp, không cộng Admittance;
- chuyển FOLLOWER → LEADER bằng nút LEADER;
- tự trả về FOLLOWER khi robot ở trong vùng đích 10 mm, tốc độ dưới 0,02 m/s
  liên tục 0,5 s;
- nút FOLLOWER để người vận hành hủy LEADER;
- sau khi nhả LEADER: chờ 10 mẫu GRU mới, blend 0,3 s rồi trở lại ACTIVE;
- force HOLD và các safety fault vẫn hoạt động.

Runtime chưa có Goal Classification, confidence score hoặc luật tự động trả
quyền vì conflict. `prediction_confidence` hiện được phát cố định bằng 1,0.
`F_robot` vẫn là diagnostic và chưa được phép quyết định vai trò. Vì vậy các
kịch bản conflict ở Mục 3 là **manual surrogate** để kiểm tra chuyển quyền và
thu dữ liệu, không phải bằng chứng rằng confidence-aware arbitration đã hoàn
thành.

## 2. Chuẩn bị chung trước mọi trial

1. Đóng launch cũ và launch lại để nạp J2 soft upper 1,30 rad và UI mới.
2. Đặt speed override thấp; người vận hành sẵn sàng E-stop.
3. Xác nhận joint states, TF, streamer ready, Axia connected/fresh và calibrate
   Axia khi tay cầm không chịu ngoại lực.
4. Khi robot stopped, đưa robot tới từng pose và lưu Target 1, Target 2. Đưa
   robot trở lại Home trước khi Start Run.
5. Target thấp phải được thử riêng ở tốc độ thấp trước. Không tăng tiếp joint
   limit hoặc workspace trong buổi test nếu chưa phân tích log.
6. Mỗi trial ghi lại: tên kịch bản, thời điểm Start/Stop, Target đang chọn,
   thời điểm bấm LEADER/FOLLOWER, hành động của người và kết quả quan sát.
7. Không cố giữ hoặc chống lại robot trong LEADER. Nếu cần tạo conflict, chỉ
   tạo tín hiệu nhẹ, ngắn, có người giám sát và bấm FOLLOWER để trả quyền.

## 3. Bộ kịch bản robot thật nên chạy chiều nay

### H1 — Baseline bắt buộc: Home → T1 → T2 → T1 → kết thúc

Mục tiêu: chứng minh toàn bộ vòng Hybrid qua ba chặng liên tiếp.

Trình tự:

1. Robot ở Home, chọn Hybrid, Start Run. Xác nhận `FOLLOWER`, chưa có target
   active.
2. Chọn Target 1, bấm LEADER. Không tác dụng lực trong LEADER.
3. Chờ robot tới T1 và tự về FOLLOWER. Chờ đủ `WAIT → BLEND → ACTIVE`.
4. Chọn Target 2, bấm LEADER. Chờ tới T2 và hoàn tất reentry như trên.
5. Chọn Target 1, bấm LEADER. Chờ tới T1 và hoàn tất reentry.
6. Stop Run, sau đó Disable Robot.

Đạt khi:

- đúng ba leg, active target lần lượt 1 → 2 → 1;
- mỗi lần đến đích có `reason=reached`, không cần bấm FOLLOWER;
- không có bước nhảy rõ rệt khi vào LEADER hoặc khi `WAIT → BLEND → ACTIVE`;
- không có IK fail, tracking fault, force fault hoặc target timeout;
- reference/actual đi liên tục và endpoint nằm trong tolerance đã cấu hình.

Nên chạy H1 ba lần để có một demo và hai lần kiểm tra tính lặp lại.

### H2 — Agreement: người và robot cùng hướng

Mục tiêu: tạo trường hợp conflict thấp, tương ứng confidence cao trong thiết kế
khối ngoài tương lai.

Trình tự:

1. Ở FOLLOWER, chọn T1.
2. Người kéo nhẹ theo hướng T1 trong một đoạn ngắn rồi nhả.
3. Khi chuyển động đã ổn định, bấm LEADER và để MJM hoàn thành.
4. Lặp lại theo hướng T2.

Đạt hiện tại khi chuyển LEADER êm, MJM đến đúng đích và tự trả FOLLOWER. Dữ liệu
cần trình bày là hướng `F_human`, vector tới target và role/control phase theo
thời gian. Với khối confidence-aware tương lai, đây phải là ca cho phép LEADER.

### H3 — Conflict trước khi trao LEADER: target chọn khác ý định người

Mục tiêu: minh họa disagreement khi target hệ thống và hướng lực người không
khớp nhau.

Trình tự:

1. Ở FOLLOWER, chọn T1 nhưng người kéo nhẹ, rõ ràng theo hướng T2.
2. Không bấm LEADER trong khoảng quan sát đã định trước; ghi lại lực và quỹ đạo.
3. Bỏ chọn ý định cũ bằng thao tác hợp lệ: vẫn ở FOLLOWER, chọn T2.
4. Bấm LEADER và cho robot đi tới T2.

Đạt hiện tại khi người vẫn điều khiển được trong FOLLOWER và việc chọn lại đích
không gây chuyển động bất ngờ. Khi trình mentor phải nói rõ quyết định đổi T1 →
T2 do người vận hành thực hiện. Luật tương lai mong muốn là confidence thấp hoặc
conflict cao phải giữ FOLLOWER và không tự trao LEADER.

### H4 — Conflict trong LEADER: người đổi ý giữa chặng

Mục tiêu: kiểm tra nhả quyền có kiểm soát khi robot đang dẫn.

Trình tự:

1. Chọn T1 và bấm LEADER.
2. Ở khoảng giữa quỹ đạo, người tạo tín hiệu phản đối nhẹ theo hướng ngược lại.
3. Người vận hành bấm FOLLOWER ngay; không tăng lực để cố thắng MJM.
4. Xác nhận robot nối êm sang force FOLLOWER.
5. Sau `WAIT → BLEND → ACTIVE`, chọn T2 rồi bấm LEADER.

Đạt hiện tại khi `reason=skipped_by_user`, role đổi một lần LEADER → FOLLOWER,
không giật tại handoff và người điều khiển lại được bằng lực. Trong hệ hiện tại,
lực ở LEADER chỉ được giám sát an toàn và không tự hủy MJM. Acceptance tương lai
là conflict duy trì đủ lâu phải tự tạo cùng hành vi trả quyền này.

### H5 — Conflict thoáng qua: chống role chattering

Mục tiêu: cung cấp ca kiểm thử hysteresis/debounce cho thiết kế confidence-aware.

Trình tự:

1. Chọn T1 và vào LEADER.
2. Tạo một xung lực phản đối rất ngắn rồi nhả; không bấm FOLLOWER.
3. Quan sát robot tiếp tục MJM và tới T1.
4. Chạy lại với phản đối duy trì; ở runtime hiện tại bấm FOLLOWER để kết thúc
   LEADER an toàn.

Điều có thể chứng minh hiện tại là safety không fault với xung nhỏ hợp lệ và
role không tự chatter. Acceptance tương lai: xung ngắn không đổi role; conflict
duy trì mới đổi một lần sang FOLLOWER. Ngưỡng lực và thời gian chưa được chốt,
không suy ra chúng từ một trial.

### H6 — Ambiguous/low-confidence intent ở FOLLOWER

Mục tiêu: tạo dữ liệu khi ý định chưa rõ và không nên trao quyền cho robot.

Trình tự:

1. Start ở FOLLOWER, chưa chọn target.
2. Người tạo chuyển động nhỏ xen kẽ giữa hướng T1 và T2, sau đó dừng.
3. Xác nhận bấm LEADER khi chưa chọn target bị UI từ chối và có thông báo.
4. Chọn target rõ ràng rồi mới bấm LEADER.

Đạt hiện tại khi không có chuyển vai trò ngoài ý muốn và guard của UI/controller
hoạt động. Acceptance tương lai: classifier giữ confidence thấp trong đoạn mơ
hồ, chỉ tăng sau khi có bằng chứng nhất quán và chỉ khi đó mới cho phép LEADER.

### H7 — Target thấp: endpoint stability

Mục tiêu: tách vấn đề ổn định tại Target thấp khỏi logic conflict.

Trình tự:

1. Chạy Home → Target thấp bằng LEADER, không tác dụng lực.
2. Sau khi role tự về FOLLOWER, tiếp tục không chạm tay cầm trong ít nhất 5 s.
3. Lặp lại một lần với người giữ lực rất nhẹ và ổn định sau khi reentry ACTIVE.
4. Stop ngay nếu có IK fail hoặc dao động tăng dần; không tăng J2 trong phiên.

Đạt khi endpoint được công nhận một lần, không lặp LEADER/FOLLOWER, không IK
fail và Actual EE không dao động tăng dần. Phân tích riêng các pha LEADER_MJM,
FOLLOWER_WAIT, FOLLOWER_BLEND và FOLLOWER_PREDICTION; không gộp chúng thành một
đoạn “đứng tại đích”.

### H8 — Safety HOLD và recovery (chỉ khi H1–H7 ổn định)

Mục tiêu: xác nhận mất force ngắn không làm MJM chạy ngầm và recovery không
nhảy reference.

Thực hiện theo cách ngắt dữ liệu cảm biến đã được nhóm phê duyệt, không tháo cáp
khi robot đang chuyển động nếu chưa có quy trình an toàn. Khoảng stale ngắn phải
HOLD tại actual EE; dữ liệu trở lại phải rebase và tiếp tục. Khoảng quá hard
timeout phải fault/disable. Đây là safety test, không phải confidence test.

## 4. Acceptance test cho confidence-aware outer loop tương lai

| Ca | Bằng chứng quan sát | Quyết định mong muốn |
|---|---|---|
| Agreement + confidence cao | lực/hướng chuyển động nhất quán với target dự đoán | cho FOLLOWER → LEADER sau dwell |
| Confidence thấp | hai goal gần ngang xác suất hoặc lịch sử chưa đủ | giữ FOLLOWER |
| Conflict trước LEADER | intent người ngược target robot | chặn LEADER |
| Conflict duy trì trong LEADER | disagreement liên tục vượt điều kiện đã chốt | trả về FOLLOWER một lần |
| Conflict thoáng qua | xung ngắn/nhiễu | giữ role, không chatter |
| Người đổi ý | confidence chuyển từ goal cũ sang goal mới | FOLLOWER trước, rồi mới chọn/đi goal mới |
| Input invalid/stale | confidence, force hoặc pose không fresh/finite | HOLD hoặc fault theo safety policy |

Trước khi gọi khối này là “đã test”, log phải có ít nhất: confidence cho từng
goal, predicted goal, conflict score, ngưỡng quyết định, thời gian dwell,
`role_source=confidence/conflict`, và lý do mỗi lần đổi role. Các trường này chưa
có trong CSV hiện tại.

## 5. Thứ tự chạy khuyến nghị trong một buổi

1. H1 một lần ở speed thấp.
2. H7 để xác nhận Target thấp và J2 mới.
3. H1 thêm hai lần nếu H7 không lỗi.
4. H2 agreement.
5. H3 conflict trước LEADER.
6. H4 đổi ý giữa LEADER.
7. H5 xung conflict ngắn.
8. H6 ambiguous intent.
9. H8 chỉ khi còn thời gian và đã thống nhất quy trình ngắt force.

Nếu bất kỳ trial nào fault, dừng chuỗi, ghi lại tên CSV và log ROS tương ứng,
không tiếp tục lặp bằng cách nới giới hạn.

## 6. Nội dung trình bày mentor

Nên chọn bốn đồ thị có cùng trục thời gian:

1. Actual EE, nominal GRU và reference robot theo XYZ.
2. `F_human` XYZ và norm lực.
3. role + `control_phase` + selected/active target.
4. Khoảng cách tới target, bridge/reentry phase và các safety flags.

Thông điệp có thể kết luận từ bản hiện tại:

- chuỗi ba leg Home → T1 → T2 → T1 hoạt động và tự nhận biết arrival;
- chuyển FOLLOWER/LEADER và reentry có log, guard và giới hạn an toàn;
- người có thể hủy LEADER và lấy lại quyền qua FOLLOWER;
- các kịch bản agreement/conflict đã tạo được dữ liệu để thiết kế outer loop.

Không nên kết luận rằng hệ thống đã tự nhận biết conflict hoặc confidence-aware
nếu quyết định vẫn do nút bấm của người vận hành.
