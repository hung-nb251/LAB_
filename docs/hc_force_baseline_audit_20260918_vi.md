# Kiểm chứng baseline và calibration ngày 18/09/2026

> **Cập nhật cuối ngày 18/09:** không thực hiện kế hoạch thu bổ sung diện rộng
> ở phần cuối tài liệu này. Rank=2 chỉ thuộc tập con `baseline_end` của bảy
> lượt GRU mới; dự án đã có 12 pose tĩnh không tiếp xúc và nhiều log động chưa
> được đưa vào audit này. Kế hoạch ưu tiên phân tích toàn bộ dữ liệu sẵn có và
> chỉ thu đúng phần observability còn thiếu được ghi tại
> `docs/cocarry_progress_next_steps_20260918_vi.md`.

## Kết luận

Chưa đủ bằng chứng để thay candidate runtime. Giữ output shadow trong
base_link. Không xác nhận calibration hoàn chỉnh hoặc lực chủ động của robot.

Script tái lập: `scripts/audit_hc_baseline_calibration.py --output <thu_muc_moi>`.
Kết quả đóng băng tại
`cocarry_logs/hc_force_calibration/20260918_baseline_audit_v2/audit.json`.

## Đã thực hiện

- Bỏ các bản lặp của cùng timestamp F_robot; ghép Axia về timestamp nguồn
  M310, không dịch lag để tối ưu kết quả và không ngoại suy qua khoảng thiếu.
- Với CSV cũ, chỉ suy ngược deadband trên hai đầu nội suy có lực khác 0,
  giả định radial deadband giữ 4 N. Không coi mẫu Axia=0 là không tiếp xúc.
- Tái tính runtime 17:22 từ raw register, baseline, q/q0 và candidate: sai
  khác lớn nhất 2.30e-7 N. Chưa thấy lỗi công thức runtime so với offline.
- Fit baseline-pose riêng bằng median các đoạn `baseline_end` được đánh dấu,
  sau 1 s ổn định, không RUNNING, biên độ joint <0.02 rad và Axia median <2 N.
  Đây là các đoạn không tiếp xúc theo protocol cũ, chưa phải baseline động.
- Fit gain torque sau trừ baseline-pose bằng r1/r2. Giữ r3 để validation;
  r4 là kiểm chứng lịch sử đã được xem trước đây, không gọi là test hoàn toàn mới.

| Mô hình | r3 RMSE | r4 RMSE | 17:22 RMSE |
|---|---:|---:|---:|
| Baseline riêng + gain đường chéo | 5.58 N | 5.61 N | 7.69 N |
| Baseline riêng + gain 6×6 | 2.26 N | 2.49 N | 6.28 N |
| Candidate đang chạy | — | 2.16 N | 7.14 N |

Candidate đang chạy được fit cả GT và r3, nên hàng này có tập training khác.
Các con số không chứng minh baseline riêng là nguyên nhân duy nhất của cải thiện.
RMSE 17:42 khi ghép timestamp nguồn là 6.82 N, góc trung vị/P95 là
22.96°/72.92° trên mẫu đủ lực. Kết quả phụ thuộc giả định deadband 4 N;
CSV không ghi giá trị deadband/filter thực tại từng thời điểm.

## Phần dữ liệu còn thiếu

Trong r1/r2 chỉ hai đoạn baseline_end vượt được tiêu chí trên, tức hai vector
pose độc lập để học ma trận baseline 6×6. Rank=2; không đủ nhận dạng baseline
toàn không gian khớp. Các lượt GRU trước chỉ có 19–22 mẫu scan RUNNING/lượt,
tương ứng khoảng 8–10 s, trong khi 17:22 và 17:42 dài 126 s và 113 s.
Chưa có baseline không tiếp xúc khi chuyển động được đánh dấu rõ.

CSV 17:42 có torque đã hiệu chỉnh nhưng thiếu raw M310, baseline và q0;
không đảo ngược model để tạo dữ liệu raw giả. Vì vậy chưa đánh giá được
candidate mới một cách độc lập trên lượt này. Cả hai CSV mới thiếu raw Axia
và reference trước deadband, nên không thể tái dựng chính xác phép đánh giá
raw như bảy trial cũ.

## Quy trình thu bổ sung cũ — chỉ dùng nếu audit hợp nhất chứng minh còn thiếu

Phần dưới được giữ lại làm runbook kỹ thuật, không phải kế hoạch mặc định cho
ngày tiếp theo. Logger đã giữ CSV 61 cột và tự tạo file cạnh CSV:
`<ten_csv>.calibration.jsonl`. File phụ chứa M310 raw, q/q0, baseline,
candidate snapshot, Axia raw 6D, Axia trước/sau deadband, rotation thực đã dùng,
mass/bias/deadband, joint states, controller status và marker. Có tối đa 10 s
pre-roll trước bật logger. Không thêm client đọc `/read_mregister`.
Axia diagnostics có `tf_valid`; chỉ dùng reference khi true và calibrated.

Khởi động lại launch sau khi robot đã Stop/Disable theo quy trình hiện có.
Không đổi tare, tải, Tool Data hoặc hướng gá giữa các đoạn cùng session.
Không dùng lại `hc_force_trial_logger.py` cùng node đọc M310 tích hợp.

Nếu cần ghi khi chưa Start Run, mở terminal quan sát:

```bash
cd /home/hungnb/cocarry_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=10
ros2 service call /logger/toggle std_srvs/srv/SetBool '{data: true}'
```

1. Thu không tiếp xúc tại Home, các pose trung gian trên hai tuyến đã vận hành,
   T1/T2 và Home sau khi trở về, mỗi pose 10–15 s. Chọn thêm pose trung gian
   đến khi độ phủ/rank cho vùng vận hành đủ; không coi số pose là bảo đảm rank.
   Mỗi đoạn đứng yên thật, người vận hành xác nhận không chạm rồi ghi marker:

```bash
ros2 topic pub --once /hc_force_trial/marker std_msgs/msg/String '{data: "no_contact_begin:home"}'
# Sau 10–15 s không tiếp xúc:
ros2 topic pub --once /hc_force_trial/marker std_msgs/msg/String '{data: "no_contact_end:home"}'
```

   Thay `home` bằng `t1_mid`, `t1`, `t2_mid`, `t2`, `home_return` cho từng đoạn.
   Không áp nhãn cho lúc người còn cầm hoặc robot còn ổn định.
2. Chỉ khi quy trình di chuyển đã có cho phép: ghi riêng đoạn chuyển động
   không tiếp xúc trên tuyến cũ, marker `no_contact_motion_begin:t1` và
   `no_contact_motion_end:t1`. Người vận hành điều khiển; không tạo đường chạy
   mới hoặc dùng GRU tự đi khi chưa có quy trình phù hợp.
3. Ở vài pose đã kiểm tra, thu lực nhẹ X±/Y±/Z± có nhả, nhiều mức lực,
   nhằm tách hướng lực khỏi pose. Giữ giới hạn vận hành hiện tại.
4. Đóng băng model sau validation. Thu một lượt mới T1 và một lượt mới T2
   chỉ để test; không refit hoặc chọn lag/ngưỡng bằng hai lượt test này.

Tắt ghi thủ công bằng:

```bash
ros2 service call /logger/toggle std_srvs/srv/SetBool '{data: false}'
```

Khi không có reference tick, CSV chính có thể không được tạo, nhưng file phụ
vẫn được ghi và đóng đúng. Đối với lượt GRU thường, Start/Stop logger như cũ.
Chưa thu robot thật trong lần chỉnh sửa phần mềm này.
