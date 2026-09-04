#!/bin/bash
# Script khởi động Hardware Driver của Force Sensor (Cần Sudo cho EtherCAT raw socket)
# ─────────────────────────────────────────────────────────────────────────────────────
# Chạy TRƯỚC khi bật cocarry_real_gui.launch.py trên PC 1.
#
# Cách dùng:
#   # Chế độ 1 máy (mặc định — gửi nội bộ localhost):
#   ./run_sensor_driver.sh
#
#   # Chế độ 2 máy — gửi UDP sang PC 1 qua Wifi (thay 192.168.x.x bằng IP thật của PC 1):
#   ./run_sensor_driver.sh 192.168.1.15
#
#   # Tuỳ chỉnh đầy đủ:
#   ./run_sensor_driver.sh 192.168.1.15 --hz 100 --port 50000

IFACE="enxec9a0c1fc063"
PC1_IP="${1:-}"           # Tham số 1: IP của PC 1 (để trống = chế độ 1 máy)

cd ~/cocarry_ws || { echo "[LỖI] Không tìm thấy ~/cocarry_ws"; exit 1; }
source install/setup.bash || { echo "[LỖI] Chưa build workspace!"; exit 1; }

echo "Bật cổng mạng Force Sensor..."
sudo ip link set dev $IFACE up

# ── Tối ưu USB adapter để giảm tải bus USB ────────────────────────────────────
# 1. Tắt autosuspend: Ngăn Linux tự ngắt điện adapter giữa chừng
ADAPTER_BUS_PATH=$(readlink -f /sys/class/net/$IFACE/device/../.. 2>/dev/null)
if [ -n "$ADAPTER_BUS_PATH" ]; then
    echo -1 | sudo tee "$ADAPTER_BUS_PATH/power/autosuspend_delay_ms" > /dev/null
    echo "on"  | sudo tee "$ADAPTER_BUS_PATH/power/control"           > /dev/null
    echo "[OK] USB autosuspend đã TẮT cho $IFACE"
else
    echo "[WARN] Không tìm được sysfs path — bỏ qua autosuspend fix"
fi

# 2. Tắt interrupt coalescing
if command -v ethtool &>/dev/null; then
    sudo ethtool -C $IFACE rx-usecs 0 tx-usecs 0 2>/dev/null \
        && echo "[OK] Interrupt coalescing đã TẮT cho $IFACE" \
        || echo "[WARN] ethtool coalescing không hỗ trợ — bỏ qua"
fi

# ── GHI CHÚ: Không dùng tc rate-limit hay nice nữa ───────────────────────────
# PC 2 chỉ chạy riêng driver cảm biến nên có đủ tài nguyên CPU và băng thông.
# Driver chạy với quyền ưu tiên mặc định (không hạ thấp bằng nice -n 15).
# Băng thông mạng EtherCAT không bị bóp bởi tc qdisc nữa.

# ── Xử lý khi Ctrl+C ──────────────────────────────────────────────────────────
trap 'echo "Dọn dẹp..."; sudo ip link set dev '$IFACE' down; exit 0' SIGINT SIGTERM

# ── Xây dựng lệnh chạy driver ─────────────────────────────────────────────────
DRIVER_CMD="python3 /home/hungnb/cocarry_ws/axia_sensor_driver.py $IFACE"

if [ -n "$PC1_IP" ]; then
    DRIVER_CMD="$DRIVER_CMD --ip $PC1_IP"
    echo "═══════════════════════════════════════════════════════════"
    echo "[CHẾ ĐỘ 2 MÁY] Dữ liệu sẽ được bắn sang PC 1: $PC1_IP:50000"
    echo "═══════════════════════════════════════════════════════════"
else
    echo "═══════════════════════════════════════════════════════════"
    echo "[CHẾ ĐỘ 1 MÁY] Dữ liệu gửi nội bộ localhost:50000"
    echo "═══════════════════════════════════════════════════════════"
fi

# Shift bỏ tham số $1 (IP), các tham số còn lại (--hz, --port...) truyền thẳng vào driver
shift 2>/dev/null
DRIVER_CMD="$DRIVER_CMD $@"

echo "Khởi động EtherCAT driver @ 100Hz (ưu tiên cao nhất — không throttle)..."
sudo \
    env FASTRTPS_DEFAULT_PROFILES_FILE=/home/hungnb/cocarry_ws/fastdds_no_shm.xml \
        LD_LIBRARY_PATH="$LD_LIBRARY_PATH" \
        PYTHONPATH="$PYTHONPATH:/home/hungnb/.local/lib/python3.10/site-packages" \
        ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
        $DRIVER_CMD

echo "Đóng cổng mạng Force Sensor..."
sudo ip link set dev $IFACE down


