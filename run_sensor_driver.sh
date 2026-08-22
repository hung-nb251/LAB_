#!/bin/bash
# Script khởi động Hardware Driver của Force Sensor (Cần Sudo cho EtherCAT raw socket)
# Chạy TRƯỚC khi bật cocarry_real_gui.launch.py

cd ~/cocarry_ws || { echo "[LỖI] Không tìm thấy ~/cocarry_ws"; exit 1; }
source install/setup.bash || { echo "[LỖI] Chưa build workspace!"; exit 1; }

echo "Bật cổng mạng Force Sensor..."
sudo ip link set dev enxec9a0c1fc063 up

trap 'echo "Đóng cổng mạng..."; sudo ip link set dev enxec9a0c1fc063 down; exit 0' SIGINT SIGTERM

echo "Khởi động EtherCAT driver (headless)..."
sudo -E FASTRTPS_DEFAULT_PROFILES_FILE=/home/hungnb/cocarry_ws/fastdds_no_shm.xml \
        LD_LIBRARY_PATH=$LD_LIBRARY_PATH \
        PYTHONPATH=$PYTHONPATH:/home/hungnb/.local/lib/python3.10/site-packages \
        ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} \
        python3 axia_sensor_driver.py enxec9a0c1fc063

echo "Đóng cổng mạng Force Sensor..."
sudo ip link set dev enxec9a0c1fc063 down
