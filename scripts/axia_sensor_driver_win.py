#!/usr/bin/env python3
r"""
axia_sensor_driver_win.py  — Hardware Driver dành riêng cho WINDOWS
─────────────────────────────────────────────────────────────────────────
Node chuyên biệt chỉ làm 1 việc: Đọc dữ liệu thô từ ATI Axia80-M20
qua giao thức EtherCAT (pysoem) trên môi trường WINDOWS.

YÊU CẦU TRÊN WINDOWS (PC 2):
1. Cài đặt Python 3.x
2. Cài đặt Npcap (hoặc WinPcap) để hỗ trợ chụp gói tin Raw Socket.
3. Cài thư viện: pip install pysoem

LƯU Ý VỀ TÊN CARD MẠNG (INTERFACE):
Trên Windows, tên card mạng không phải là "eth0" hay "enx...".
Nó là một chuỗi Device GUID, ví dụ: \Device\NPF_{2A3B4C5D...}
Bạn có thể tìm chuỗi này bằng cách chạy lệnh:
    python -m pysoem

■ Cách chạy trên PC 2 (Windows):
    python axia_sensor_driver_win.py \Device\NPF_{GUID_CUA_BAN} --ip 192.168.1.13
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

# UDP defaults
DEFAULT_UDP_IP   = '127.0.0.1' 
DEFAULT_UDP_PORT = 50000
DEFAULT_HZ       = 100.0


def log_info(msg):
    print(f"[AxiaDriver-Win] {msg}")

def log_error(msg):
    print(f"[AxiaDriver-Win] [LỖI] {msg}", file=sys.stderr)


# ════════════════════════════════════════════════════════════════════════════
#  Tiện ích Reconnect cho Windows
# ════════════════════════════════════════════════════════════════════════════

def _wait_for_link_up_win(stop_event: threading.Event, delay_s: float = 3.0) -> bool:
    """
    Trên Linux, chúng ta check file /sys/class/net/.../carrier.
    Nhưng trên Windows, cách dễ và ổn định nhất là delay mù (Blind Delay)
    vài giây rồi thử kết nối lại.
    """
    start = time.time()
    while not stop_event.is_set():
        if time.time() - start > delay_s:
            return True
        time.sleep(0.5)
    return False


def _init_ethercat(iface: str):
    master = pysoem.Master()
    master.open(iface)

    if master.config_init() <= 0:
        master.close()
        raise RuntimeError('Không tìm thấy cảm biến EtherCAT trên bus.')

    slave = master.slaves[0]
    master.config_map()

    master.state = pysoem.SAFEOP_STATE
    master.write_state()
    master.state_check(pysoem.SAFEOP_STATE, 2_000_000)
    if master.state != pysoem.SAFEOP_STATE:
        master.close()
        raise RuntimeError('Slave không vào được SAFEOP.')

    counts_per_force = int.from_bytes(
        slave.sdo_read(SDO_CALIB_INDEX, SUBIDX_COUNTS_PER_FORCE), 'little')
    counts_per_torque = int.from_bytes(
        slave.sdo_read(SDO_CALIB_INDEX, SUBIDX_COUNTS_PER_TORQUE), 'little')
    log_info(f'counts/F={counts_per_force}, counts/T={counts_per_torque}')

    master.state = pysoem.OP_STATE
    master.write_state()
    for _ in range(400):
        master.send_processdata()
        master.receive_processdata(50_000)
        master.state_check(pysoem.OP_STATE, 50_000)
        if master.state == pysoem.OP_STATE:
            break
    if master.state != pysoem.OP_STATE:
        master.close()
        raise RuntimeError('Cảm biến không vào được OP state.')

    slave.sdo_write(CONTROL_INDEX, CONTROL_SUBIDX_1, (1).to_bytes(4, 'little'))
    for _ in range(5):
        master.send_processdata()
        master.receive_processdata(2000)
        time.sleep(0.01)
    slave.sdo_write(CONTROL_INDEX, CONTROL_SUBIDX_1, (0).to_bytes(4, 'little'))

    return master, slave, counts_per_force, counts_per_torque


def _close_master_safe(master):
    if master is None:
        return
    try:
        master.state = pysoem.INIT_STATE
        master.write_state()
    except Exception:
        pass
    try:
        master.close()
    except Exception:
        pass


# ════════════════════════════════════════════════════════════════════════════
#  Vòng lặp EtherCAT 
# ════════════════════════════════════════════════════════════════════════════

def ethercat_loop(iface: str, hz: float, stop_event: threading.Event,
                  udp_ip: str, udp_port: int):
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    log_info(f'UDP target: {udp_ip}:{udp_port}')
    reconnect_count = 0
    MAX_RECONNECTS = 100

    try:
        while not stop_event.is_set() and reconnect_count <= MAX_RECONNECTS:
            master = None
            try:
                if reconnect_count > 0:
                    log_info(f'═══ RECONNECT #{reconnect_count} ═══')
                
                master, slave, counts_per_force, counts_per_torque = _init_ethercat(iface)
                log_info(f'EtherCAT OP. Bắt đầu bắn UDP tới {udp_ip}:{udp_port} @ {hz:.0f}Hz...')

                CYCLE_S = 1.0 / hz
                RECV_TIMEOUT_US = max(2_000, int(CYCLE_S * 500_000))
                consecutive_wkc_errors = 0
                MAX_WKC_ERRORS = 50

                next_deadline = time.perf_counter() + CYCLE_S
                while not stop_event.is_set():
                    master.send_processdata()
                    wkc = master.receive_processdata(RECV_TIMEOUT_US)

                    if wkc >= master.expected_wkc:
                        consecutive_wkc_errors = 0
                        data = slave.input
                        if len(data) == struct.calcsize(RAW_FMT):
                            Fx, Fy, Fz, Tx, Ty, Tz, _s, _c = struct.unpack(RAW_FMT, data)
                            fx = Fx / counts_per_force
                            fy = Fy / counts_per_force
                            fz = Fz / counts_per_force
                            tx = Tx / counts_per_torque
                            ty = Ty / counts_per_torque
                            tz = Tz / counts_per_torque
                            udp_payload = struct.pack('<6f', fx, fy, fz, tx, ty, tz)
                            sock.sendto(udp_payload, (udp_ip, udp_port))
                    else:
                        consecutive_wkc_errors += 1
                        if consecutive_wkc_errors >= MAX_WKC_ERRORS:
                            log_error(f'⚠ Mất kết nối EtherCAT ({MAX_WKC_ERRORS} lỗi)! Đang ngắt để thử lại...')
                            break

                    now = time.perf_counter()
                    sleep_s = next_deadline - now
                    if sleep_s > 0:
                        time.sleep(sleep_s)
                    next_deadline += CYCLE_S
                    if next_deadline < time.perf_counter():
                        next_deadline = time.perf_counter() + CYCLE_S

            except Exception as e:
                log_error(f'Lỗi EtherCAT: {e}')

            finally:
                _close_master_safe(master)
                master = None

            if stop_event.is_set():
                break

            reconnect_count += 1
            log_info("Đợi 4 giây trước khi thử kết nối lại...")
            if not _wait_for_link_up_win(stop_event, delay_s=4.0):
                break

        if reconnect_count > MAX_RECONNECTS:
            log_error("Dừng vì quá số lần reconnect.")

    finally:
        sock.close()
        log_info("Driver kết thúc.")


# ════════════════════════════════════════════════════════════════════════════
#  Điểm vào
# ════════════════════════════════════════════════════════════════════════════

def _parse_arg(argv, flag, default):
    if flag in argv:
        try:
            return argv[argv.index(flag) + 1]
        except IndexError:
            pass
    return default

def main():
    if len(sys.argv) < 2:
        print('Cách dùng trên Windows:')
        print('  python axia_sensor_driver_win.py \\Device\\NPF_{GUID_CUA_BAN} --ip 192.168.1.13')
        print('\\nĐể tìm GUID card mạng, hãy chạy lệnh: python -m pysoem')
        sys.exit(1)

    iface = sys.argv[1]
    argv  = sys.argv[1:]

    udp_ip   = _parse_arg(argv, '--ip', DEFAULT_UDP_IP)
    udp_port = int(_parse_arg(argv, '--port', str(DEFAULT_UDP_PORT)))
    hz       = float(_parse_arg(argv, '--hz', str(DEFAULT_HZ)))

    log_info(f'UDP target: {udp_ip}:{udp_port}')
    log_info(f'Interface : {iface}')
    
    stop_event = threading.Event()
    eth_thread = threading.Thread(
        target=ethercat_loop,
        args=(iface, hz, stop_event, udp_ip, udp_port),
        daemon=True)
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
