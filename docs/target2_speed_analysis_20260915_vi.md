# Phân tích Target 2 và tốc độ — sáng 15/09/2026

Phạm vi: phân tích log, kiểm tra IK offline và đối chiếu commit `7d7700ff`.
Không thay đổi cấu hình vận hành, URDF hoặc Axia; không gửi lệnh tới robot.

## 1. Dữ liệu và thời điểm dừng

Nguồn: `cocarry_logs/python3_5898_1789438342024.log`,
`cocarry_logs/python3_5900_1789438341836.log` và các CSV dưới đây.
Giờ địa phương UTC+7, tính từ timestamp trong log.

| CSV cocarry_admittance_3d_20260915_* | Dừng | Lý do |
|---|---|---|
| 091920.csv | 09:19:40.742 | Local IK failed 3 consecutive times |
| 092134.csv | 09:21:59.478 | Local IK failed 3 consecutive times |
| 093323.csv | 09:33:47.491 | Time-aligned queue tracking error 0.054 m > 0.050 m |

Cả ba CSV ghi `model=gru`, `hybrid.enabled=false`,
`control_phase=FOLLOWER_PREDICTION`, role CSV `OFF`.
Đây là chuyển động GRU trong vùng Target 2, không phải LEADER đang chạy MJM tới Target 2.

Có thêm lần dừng 09:32:43.947 trong trial 093157 do
`MotoROS2 point queue mode dropped during force-guided motion`.
Đó là sự kiện riêng; cần đối chiếu pendant/controller để biết vì sao mode mất.

## 2. Khả năng đạt tới Target 2

Cận mềm hiện tại J2=1.30 rad, J3=1.25 rad. Với margin 3°,
cận IK thực tế là J2=1.247640 rad, J3=1.197640 rad.

Kiểm tra offline bằng `LocalIKSolver`, dùng joint feedback làm seed,
orientation FK của mẫu đầu trial và reference của mẫu cuối trial:

| Trial | Reference cuối XYZ (m) | IK cận hiện tại | Nghiệm dùng cận URDF: J2/J3 (rad) |
|---|---|---|---|
| 091920 | (-0.512372, 1.057059, 0.258740) | Không hội tụ | 1.27492 / 1.17148 |
| 092134 | (-0.480419, 1.078599, 0.292001) | Không hội tụ | 1.29412 / 1.28344 |
| 093323 | (-0.479285, 1.029080, 0.285215) | Có nghiệm | 1.14938 / 0.93312 |

Thử riêng cận mềm J2=1.40 rad **chỉ trong phép tính offline**:
reference cuối 091920 có nghiệm; reference cuối 092134 vẫn không hội tụ
do nghiệm trên nhánh đang xét còn vượt J3. Không áp dụng giá trị này lên robot.

Target 2 lưu trong hybrid status là (-0.473192, 0.991108, 0.238881) m.
Với orientation đầu trial 091920, IK trong cận hiện tại có nghiệm
J2=1.10138, J3=0.72461 rad. Target đã lưu không tự nó đòi mở cận.
Các reference cuối hai lần lỗi có Y xa hơn Target 2 khoảng 66 và 87 mm.

Giới hạn suy luận: CSV không ghi chính xác pose sau smoothing ở từng lần IK fail.
Đây là kiểm tra khả năng đạt tới reference cuối, không phải replay đầy đủ ba
request thất bại. Nghiệm ngoài cận cho thấy nguyên nhân hình học rất đáng chú ý;
không chứng minh mọi nhánh IK đều vô nghiệm, cũng không chứng minh an toàn va chạm.
FK từ joint cuối khớp actual EE trong CSV ở độ chính xác số máy.

## 3. Tracking error

Ngay trước lỗi, cửa sổ runtime ghi tick/send/ack đều 15 Hz, BUSY=0,
retry=0, reject=0. Pose age tối đa cả trial 093323 khoảng 31.1 ms,
force age tối đa khoảng 19.9 ms. Không thấy bằng chứng mất pose hoặc
nghẽn service trong cửa sổ này.

Reference cuối cách actual EE khoảng 32.7 mm, nhưng watchdog báo 54 mm:
hai đại lượng khác nhau. Watchdog so actual với FK của điểm queue đã đến hạn,
không so với reference đầu vào hiện tại.

J3/J5 có lúc đạt khoảng 0.404/0.301 rad/s, sát profile lệnh 0.40/0.30.
J3 đổi hướng trong khoảng hơn một giây trước dừng (thấy trong joint positions).
Streamer hiện giới hạn vận tốc từng khớp riêng rồi tính lại vị trí; cách này
có thể làm lệch đường Cartesian và tạo thay đổi vận tốc khi khớp bão hòa.
Đây là yếu tố cần kiểm chứng, chưa đủ để kết luận là nguyên nhân duy nhất của 54 mm.

Lịch watchdog lấy epoch từ thời điểm ACK, không phải timestamp bắt đầu thực thi
được controller xác nhận. CSV chưa có điểm queue và lịch thực thi đầy đủ để tách
độ trễ vật lý khỏi sai lệch căn thời gian. Không nên tăng threshold để che lỗi.

## 4. Vì sao cảm giác chậm hơn bản camera

Commit `7d7700ff`, launch camera `hrc_bringup/cocarry_full.launch.py`:
streamer mặc định 15 Hz, vmax Cartesian 0.15 m/s, amax 0.50 m/s².
`coord_transform/config/transform_params.yaml` cấu hình EMA khi chuyển động
X/Y/Z=1.0/0.8/1.0; vẫn có median Y, outlier rejection và rate limit.

Hiện tại: 15 Hz, vmax 0.25 m/s, amax 1.00 m/s²;
profile khớp thực tế log xác nhận [0.30,0.30,0.40,0.08,0.30,0.25] rad/s.
Giới hạn Cartesian mới cao hơn, nhưng không đảm bảo tốc độ thực tế cao hơn.

Pipeline hiện tại lọc vị trí nominal với tau=0.4 s, giới hạn nominal lead
50 mm, reference sau admittance lead 40 mm, rồi smoothing và giới hạn khớp
ở streamer. Lọc tau=0.4 tại 15 Hz có alpha khoảng 0.154; riêng đáp ứng bậc thang
của bộ lọc cần khoảng 0.92 s để đạt 90%, nếu không có các giới hạn/reset khác.
Đây không phải số đo độ trễ toàn hệ thống.

Camera lấy chuyển động tay độc lập làm đầu vào; pipeline hiện tại lấy chuyển
động EE phản hồi. Trễ lọc nằm trong vòng tương tác này nên không thể so hai
pipeline chỉ bằng vmax. Tau này thuộc nhánh prediction; không đại diện cho
thời gian MJM trong manual LEADER.

Tốc độ từ sai phân actual EE trên mẫu CSV (chỉ xét mẫu >0.01 m/s):

| Trial | Median (m/s) | P95 (m/s) |
|---|---|---|
| 091920 | 0.097 | 0.135 |
| 092134 | 0.086 | 0.120 |
| 093323 | 0.105 | 0.142 |

Các số này phụ thuộc thao tác người và timestamp lấy mẫu, không phải benchmark
cùng đường đi với camera hoặc phép đo giới hạn tốc độ tối đa của robot.

## 5. Thứ tự triển khai đề xuất

1. Bổ sung telemetry: pose IK request khi fail, seed, khoảng cách tới cận,
   nghiệm trước/sau giới hạn vận tốc, scale tốc độ, thời gian gửi/ACK/due,
   expected/actual EE và tracking error. Phân biệt số IK fail tích lũy với
   số liên tiếp. Cần dữ liệu này trước khi kết luận lỗi tracking do timing.
2. Thêm bộ giới hạn reference theo khả năng IK: thử bước nhỏ hơn từ trạng thái
   hợp lệ, giới hạn chuyển động hướng ra ngoài khi gần cận, phản hồi reference
   đã chấp nhận về admittance để tránh tích lũy. Giữ kiểm tra fail-closed cho
   lỗi thực sự; không phát nghiệm clamp hoặc tự bỏ watchdog.
3. Phối hợp tốc độ khớp bằng time-scaling chung trên đường đã kiểm tra IK,
   kèm giới hạn gia tốc/jerk và phản hồi bão hòa về bộ sinh reference.
   Tránh sáu khớp tiến với sáu tỷ lệ khác nhau. Cách này không tự tăng tốc độ
   tối đa; mục tiêu là giảm méo đường và dừng/đuổi lặp lại khi một khớp bị giới hạn.
4. Giảm trễ prediction bằng bộ ước lượng trạng thái vị trí-vận tốc, dự báo
   reference trước một khoảng có giới hạn và dùng velocity feed-forward.
   Lọc phần sai số đo/dự đoán, giữ giới hạn v/a/jerk ở đầu ra và lọc mạnh khi
   đứng yên. Không lấy đạo hàm raw GRU để cộng bù trực tiếp; không bỏ lọc bằng
   tau=0. Cần thử offline để lựa chọn horizon/gain, chưa có tham số tối ưu được xác nhận.
5. So sánh A/B trên đường giữa workspace trước, sau đó gần Target 2: cùng pose,
   cùng hướng X±/Z±, giữ cùng giới hạn. Đánh giá thời gian di chuyển, lag,
   vận tốc/jerk, tracking P95/max, IK fail, clipping và dao động lúc đứng yên.

Ưu tiên xử lý tính khả thi và bám quỹ đạo trước khi tăng độ đáp ứng.
Chưa đề xuất tăng J2/J3 hoặc tracking threshold từ ba log này.
Giữ URDF, góc Axia và giới hạn lực; không dùng thay đổi deadband Z để xử lý
một giới hạn hình học tại Target 2.

## 6. Cập nhật sau góp ý về mở rộng giới hạn

Người dùng đồng ý hướng cải thiện reference/bám quỹ đạo và cho phép xem xét
mở rộng góc, vận tốc J1/J2/J3/J5/J6 khi cần; giữ J4 ở cấu hình hiện tại.
Phần này cập nhật đề xuất J2/J3 ở mục 5 sau khi kiểm tra thêm offline.

Kiểm tra 31 reference cuối mỗi CSV 091920, 092134, 093323, dùng seed từ
joint feedback cùng mẫu, orientation từ FK mẫu đầu trial, solver hiện tại:

| Trial | Không hội tụ với cận hiện tại | Không hội tụ với upper J2=J3=1.40 rad |
|---|---|---|
| 091920, 31 mẫu | 4 | 0 |
| 092134, 31 mẫu | 4 | 0 |
| 093323, 31 mẫu | 0 | 0 |

Margin 3° vẫn giữ, nên cận IK trên mới là 1.347640 rad. Các phép tính độc lập
theo mẫu này không mô phỏng queue, gia tốc, va chạm, độ võng hay PFL, không phải
93 điểm đã được kiểm chứng trên robot. Chưa thay cấu hình vận hành.

Đề xuất cấu hình thử theo hai bước tách biệt:

| Khớp | Cận góc mềm đề xuất (rad) | Vận tốc hiện tại (rad/s) | Vận tốc ứng viên bước sau (rad/s) |
|---|---|---|---|
| J1 | Giữ [0.00, 3.14], cho phép cấu hình riêng khi mở vùng thao tác | 0.30 | 0.35 |
| J2 | Tăng upper 1.30 → 1.50; lower giữ -0.80 | 0.30 | 0.35 |
| J3 | Tăng upper 1.25 → 1.50; chưa đổi lower | 0.40 | 0.45 |
| J4 | Giữ [-2.50, 2.50] | 0.08 | 0.08 |
| J5 | Giữ [-2.09, 0.52], cho phép cấu hình riêng khi cần | 0.30 | 0.35 |
| J6 | Giữ [-2.50, 2.50], cho phép cấu hình riêng khi cần | 0.25 | 0.30 |

Bước một đánh giá cận mới ở tốc độ cũ. Bước hai chỉ đánh giá profile vận tốc
ứng viên sau khi đã kiểm tra cơ chế phối hợp khớp và lỗi tracking. Các giá trị
vận tốc ứng viên là mức tăng thử nghiệm khoảng 12.5–20%, không phải kết quả tối
ưu hoặc tốc độ đã được xác nhận an toàn. Trần Cartesian vẫn xét riêng; mở tốc
độ khớp không đồng nghĩa robot luôn chạy nhanh hoặc được bỏ giới hạn Cartesian.

J4 vẫn tham gia IK để giữ orientation dù chuyển động nhỏ; giữ cận/vận tốc J4,
không cố định q4 về một hằng số.

Lưu ý cấu hình để triển khai: lower J3 mềm hiện tại -2.00 rad thấp hơn lower
J3 trong `LocalIKSolver.URDF_JOINT_LIMITS` là -pi/2. Không được lấy soft bounds
làm giới hạn cơ khí độc lập; bộ đọc profile cần lấy giao với bounds mô hình và
xác minh quy ước góc controller trước khi mở vùng đó. Không sửa URDF để hợp thức
hóa profile. Khác biệt này không ảnh hưởng 93 mẫu vùng J3 dương đã kiểm tra.

Nguồn hãng để phân biệt khả năng máy và profile ứng dụng:
[Yaskawa HC10DTP datasheet](https://www.motoman.com/getmedia/6bee5ac1-77ed-43da-aa0c-8d3e493d3aea/ds)
công bố tốc độ tối đa S/L=130°/s, U/R=180°/s, B/T=250°/s và nêu tốc độ
collaborative phụ thuộc đánh giá an toàn. Không sao chép bảng giới hạn góc hãng
trực tiếp vào ROS vì quy ước zero/range của mô hình hiện tại chưa trùng bảng.

Tốc độ mong muốn có thể thay đổi theo lực người qua admittance và chuyển động
dự đoán, trong một trần vận tốc/gia tốc đã cấu hình. Không tự nâng trần chỉ vì
người kéo mạnh hơn: gần singularity hoặc cận góc, cùng vận tốc EE có thể cần
vận tốc khớp lớn hơn nhiều. Vùng thao tác rộng cần được kiểm tra theo pose và
đường đi; không thể đạt được không gian vô hạn bằng cách mở soft limits.
