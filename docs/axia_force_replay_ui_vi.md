# Replay lực Axia từ CSV

Chạy từ workspace:

```bash
cd /home/hungnb/cocarry_ws
python3 scripts/axia_force_replay_ui.py cocarry_logs/cocarry_admittance_3d_20260922_175631.csv
```

UI tự phát ở tốc độ 1×, hiển thị ba đồ thị lực XYZ xếp dọc trên nền trắng,
theo ảnh tham khảo (Fx đỏ, Fy xanh dương, Fz xanh lá). Cửa sổ mặc định tối đa
1600 × 1000 px và được giới hạn theo màn hình hiện có. Đồ thị được đóng khung;
giao diện và trục dùng Times New Roman. Trục tung ghi F với chỉ số dưới x/y/z
và đơn vị N; trục hoành dưới cùng ghi `Time(s)`. Đường kẻ dọc cách nhau 2 giây.
UI còn hiển thị giá trị tức thời và chuẩn lực. Có nút tạm dừng/tiếp tục, phát
lại từ đầu, thanh tua và điều chỉnh tốc độ 0.1–10×. Mặc định đồ thị hiển thị
10 giây gần nhất; trục ngang là số giây tính từ dòng CSV đầu tiên. Hết file
thì dừng. Tên file còn ở thanh tiêu đề, các cột CSV không hiển thị trong UI.

```bash
python3 scripts/axia_force_replay_ui.py /duong/dan/trial.csv --paused --window-sec 30
python3 scripts/axia_force_replay_ui.py /duong/dan/trial.csv --speed 0.5
```

CSV compact và diagnostic của co-carry được đọc trực tiếp: lực lấy nguyên giá
trị `f_human_x/y/z` (N), thời gian lấy `ros_timestamp_ns` của từng dòng.
Đây là lực đã xử lý được lưu trong log; replay không lọc, tare hay bù trọng lực
lần nữa. Khoảng cách timestamp không đều, timestamp trùng và khoảng mất dữ
liệu được giữ nguyên. Trong khoảng trống, chỉ số lực giữ mẫu gần nhất.

Tốc độ 1× nghĩa là 10 giây dữ liệu mất 10 giây thời gian thực. Đồng hồ monotonic
giữ tiến độ, giao diện vẽ lại mỗi 20 ms và đưa vào đồ thị tất cả mẫu đã đến hạn.
Độ trễ hiển thị còn phụ thuộc bộ lập lịch và tải máy; đây không phải bộ phát
hard real time. CSV trial thường được ghi khoảng 15 Hz, dù Axia gửi khoảng
100 Hz; replay chỉ có thể hiển thị các mẫu thực sự có trong CSV.

Nếu muốn dùng timestamp của lực thay vì thời điểm ghi dòng:

```bash
python3 scripts/axia_force_replay_ui.py /duong/dan/trial.csv --time-column fh_timestamp_ns
```

Đối với CSV khác, chỉ rõ cột và đơn vị thời gian; lực phải có đơn vị N:

```bash
python3 scripts/axia_force_replay_ui.py /duong/dan/force.csv \
  --time-column timestamp --time-unit ms --force-columns Fx Fy Fz
```

Các đơn vị thời gian hỗ trợ: `s`, `ms`, `us`, `ns`. Timestamp phải hữu hạn và
không giảm; lực phải là số hữu hạn. Dữ liệu lỗi bị từ chối kèm số dòng để người
dùng sửa nguồn dữ liệu, không tự sắp xếp hay bỏ dòng. File được nạp khi mở UI,
không theo dõi những dòng được ghi thêm sau đó.

Ứng dụng chạy offline, không cần source ROS, không mở UDP và không publish
topic điều khiển. Phụ thuộc: Python 3, NumPy, PyQt5, pyqtgraph (cùng bộ thư viện
đồ thị của `axia_sensor_ui.py`).
