#!/usr/bin/env python3
"""
axia_sensor_driver.py  — Hardware Driver (Chạy bằng SUDO, KHÔNG DÙNG ROS)
─────────────────────────────────────────────────────────────────────────
Node chuyên biệt chỉ làm 1 việc: Đọc dữ liệu thô từ ATI Axia80-M20
qua giao thức EtherCAT (pysoem, cần quyền CAP_NET_RAW / sudo).

Thay vì dùng ROS 2 phức tạp và bị Firewall chặn, node này dùng 
UDP Socket thuần túy để bắn dữ liệu siêu tốc độ sang tiến trình UI.

■ Truyền:
    127.0.0.1:50000 (UDP)
    Gói dữ liệu: 24 bytes (6 floats: Fx, Fy, Fz, Tx, Ty, Tz)

■ Cách chạy (qua run_sensor_driver.sh):
    sudo python3 axia_sensor_driver.py enxec9a0c1fc063
"""

import sys
import time
import struct
import socket
import threading
import pysoem

# ── Hằng số phần cứng ATI Axia80-M20 ─────────────────────────────────────────
RAW_FMT                   = '<6iII'
SDO_CALIB_INDEX           = 0x2021
SUBIDX_COUNTS_PER_FORCE   = 0x37
SUBIDX_COUNTS_PER_TORQUE  = 0x38
CONTROL_INDEX             = 0x7010
CONTROL_SUBIDX_1          = 0x01

# UDP Socket configuration
UDP_IP = "127.0.0.1"
UDP_PORT = 50000


def log_info(msg):
    print(f"[AxiaDriver] {msg}")

def log_error(msg):
    print(f"[AxiaDriver] [LỖI] {msg}", file=sys.stderr)


# ════════════════════════════════════════════════════════════════════════════
#  Vòng lặp EtherCAT 100 Hz
# ════════════════════════════════════════════════════════════════════════════

def ethercat_loop(iface: str, stop_event: threading.Event):
    master = None
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        master = pysoem.Master()
        master.open(iface)

        if master.config_init() <= 0:
            log_error('Không tìm thấy cảm biến. Kiểm tra dây EtherCAT!')
            return

        slave = master.slaves[0]
        master.config_map()

        # PRE_OP → SAFE_OP
        master.state = pysoem.SAFEOP_STATE
        master.write_state()
        master.state_check(pysoem.SAFEOP_STATE, 2_000_000)
        if master.state != pysoem.SAFEOP_STATE:
            log_error('Slave không vào được SAFEOP.')
            return

        # Đọc hệ số chuyển đổi từ SDO
        counts_per_force  = int.from_bytes(
            slave.sdo_read(SDO_CALIB_INDEX, SUBIDX_COUNTS_PER_FORCE), 'little')
        counts_per_torque = int.from_bytes(
            slave.sdo_read(SDO_CALIB_INDEX, SUBIDX_COUNTS_PER_TORQUE), 'little')
        log_info(f'counts/F={counts_per_force}, counts/T={counts_per_torque}')

        # SAFE_OP → OP
        master.state = pysoem.OP_STATE
        master.write_state()
        for _ in range(400):
            master.send_processdata()
            master.receive_processdata(50_000)
            master.state_check(pysoem.OP_STATE, 50_000)
            if master.state == pysoem.OP_STATE:
                break
        if master.state != pysoem.OP_STATE:
            log_error('Cảm biến không vào được OP state.')
            return

        # Hardware tare lúc khởi động
        slave.sdo_write(CONTROL_INDEX, CONTROL_SUBIDX_1, (1).to_bytes(4, 'little'))
        for _ in range(5):
            master.send_processdata()
            master.receive_processdata(2000)
            time.sleep(0.01)
        slave.sdo_write(CONTROL_INDEX, CONTROL_SUBIDX_1, (0).to_bytes(4, 'little'))

        log_info(f'EtherCAT OP. Bắt đầu bắn UDP tới {UDP_IP}:{UDP_PORT} @ 100Hz...')

        # Vòng lặp 100 Hz (BẢN CŨ CHỊU LÌ ĐÒN)
        while not stop_event.is_set():
            master.send_processdata()
            wkc = master.receive_processdata(2000)

            if wkc >= master.expected_wkc:
                data = slave.input
                if len(data) == struct.calcsize(RAW_FMT):
                    Fx, Fy, Fz, Tx, Ty, Tz, _s, _c = struct.unpack(RAW_FMT, data)
                    
                    # Giải mã sang N và Nm
                    fx = Fx / counts_per_force
                    fy = Fy / counts_per_force
                    fz = Fz / counts_per_force
                    tx = Tx / counts_per_torque
                    ty = Ty / counts_per_torque
                    tz = Tz / counts_per_torque

                    # Đóng gói 6 số float (4 bytes x 6 = 24 bytes) và gửi qua UDP
                    udp_payload = struct.pack('<6f', fx, fy, fz, tx, ty, tz)
                    sock.sendto(udp_payload, (UDP_IP, UDP_PORT))

            time.sleep(0.01)

    except Exception as e:
        log_error(f'EtherCAT lỗi: {e}')
    finally:
        if master is not None:
            try:
                master.state = pysoem.INIT_STATE
                master.write_state()
                master.close()
                log_info('EtherCAT đã đóng sạch sẽ.')
            except Exception:
                pass
        sock.close()


# ════════════════════════════════════════════════════════════════════════════
#  Điểm vào
# ════════════════════════════════════════════════════════════════════════════

def main():
    if len(sys.argv) < 2:
        print('Cách dùng: sudo python3 axia_sensor_driver.py <interface>')
        print('Ví dụ:     sudo python3 axia_sensor_driver.py enxec9a0c1fc063')
        sys.exit(1)

    iface = sys.argv[1]

    stop_event = threading.Event()
    eth_thread = threading.Thread(
        target=ethercat_loop, args=(iface, stop_event), daemon=True)
    eth_thread.start()

    log_info('Đang chạy (Bấm Ctrl+C để thoát)...')
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        eth_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
