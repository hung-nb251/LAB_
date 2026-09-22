#!/bin/bash
# Khởi động giao diện UDP của cảm biến bằng user thường.
# EtherCAT raw socket thuộc axia_sensor_driver.py và được chạy riêng bằng sudo.

# Đảm bảo chạy từ đúng thư mục (bất kể bạn gọi script từ đâu)
cd ~/cocarry_ws || { echo "[LỖI] Không tìm thấy ~/cocarry_ws"; exit 1; }
source install/setup.bash || { echo "[LỖI] Chưa build workspace! Chạy: colcon build"; exit 1; }

FASTRTPS_DEFAULT_PROFILES_FILE=/home/hungnb/cocarry_ws/fastdds_no_shm.xml \
LD_LIBRARY_PATH="$LD_LIBRARY_PATH" \
PYTHONPATH="$PYTHONPATH:/home/hungnb/.local/lib/python3.10/site-packages" \
python3 scripts/axia_sensor_ui.py
