#!/bin/bash
# Script khởi động Giao diện Cảm biến (Cần Sudo để truy cập mạng EtherCAT raw socket)
# Ép FastDDS dùng UDP để không bị mất kết nối với ROS2 của user thường.

# Đảm bảo chạy từ đúng thư mục (bất kể bạn gọi script từ đâu)
cd ~/cocarry_ws || { echo "[LỖI] Không tìm thấy ~/cocarry_ws"; exit 1; }
source install/setup.bash || { echo "[LỖI] Chưa build workspace! Chạy: colcon build"; exit 1; }

# Mở cổng mạng lên
echo "Bật cổng mạng Force Sensor..."
sudo ip link set dev enxec9a0c1fc063 up

# Đảm bảo khi bạn tắt UI (Ctrl+C), nó sẽ tự động bịt cổng mạng lại để không làm sập Robot
trap 'echo "Đóng cổng mạng Force Sensor..."; sudo ip link set dev enxec9a0c1fc063 down; exit 0' SIGINT SIGTERM

sudo -E FASTRTPS_DEFAULT_PROFILES_FILE=/home/hungnb/cocarry_ws/fastdds_no_shm.xml \
        LD_LIBRARY_PATH=$LD_LIBRARY_PATH \
        PYTHONPATH=$PYTHONPATH:/home/hungnb/.local/lib/python3.10/site-packages \
        python3 axia_sensor_ui.py enxec9a0c1fc063

# Đóng cổng mạng lại nếu code kết thúc bình thường
echo "Đóng cổng mạng Force Sensor..."
sudo ip link set dev enxec9a0c1fc063 down

