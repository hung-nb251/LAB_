# Workspace scripts

Các script Python thao tác trực tiếp, driver cảm biến, training, plotting và
offline audit/calibration được gom ở đây. Mã package ROS vẫn nằm trong `src/`.

Chạy script từ root workspace, ví dụ:

```bash
python3 scripts/axia_sensor_ui.py
python3 scripts/hc_force_trial_logger.py --help
python3 scripts/audit_hc_t1_sidecar.py --help
```

Các script audit/calibration chỉ đọc dữ liệu hiện có và ghi kết quả vào thư
mục output mới; không ghi đè raw log.
