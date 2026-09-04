#!/usr/bin/env python3
"""
axia_sensor_driver.py  — Hardware Driver (Chạy bằng SUDO, KHÔNG DÙNG ROS)
─────────────────────────────────────────────────────────────────────────
Node chuyên biệt chỉ làm 1 việc: Đọc dữ liệu thô từ ATI Axia80-M20
qua giao thức EtherCAT (pysoem, cần quyền CAP_NET_RAW / sudo).

Chạy trên PC 2 (máy đo cảm biến), bắn dữ liệu qua UDP sang PC 1 (máy điều khiển).

■ Truyền:
    <ip_pc1>:50000 (UDP)
    Gói dữ liệu: 24 bytes (6 floats: Fx, Fy, Fz, Tx, Ty, Tz)

■ Cách chạy (qua run_sensor_driver.sh):
    # Chạy nội bộ cùng 1 máy (mặc định):
    sudo python3 axia_sensor_driver.py enxec9a0c1fc063

    # Chạy trên PC 2, bắn dữ liệu sang PC 1 qua Wifi:
    sudo python3 axia_sensor_driver.py enxec9a0c1fc063 --ip 192.168.1.15

    # Tuỳ chỉnh cổng và tần số:
    sudo python3 axia_sensor_driver.py enxec9a0c1fc063 --ip 192.168.1.15 --port 50000 --hz 100
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

# UDP defaults — có thể ghi đè qua tham số dòng lệnh --ip / --port
DEFAULT_UDP_IP   = '127.0.0.1'  # Localhost (1 máy); đổi sang IP PC1 khi dùng 2 máy
DEFAULT_UDP_PORT = 50000
DEFAULT_HZ       = 100.0        # 100Hz: lấy đủ dữ liệu mịn nhất


def log_info(msg):
    print(f"[AxiaDriver] {msg}")

def log_error(msg):
    print(f"[AxiaDriver] [LỖI] {msg}", file=sys.stderr)


# ════════════════════════════════════════════════════════════════════════════
#  Tiện ích kiểm tra carrier (link vật lý)
# ════════════════════════════════════════════════════════════════════════════

def _wait_for_link_up(iface: str, stop_event: threading.Event,
                      timeout: float = 30.0, poll_interval: float = 0.5) -> bool:
    """
    Chờ cho đến khi carrier (tín hiệu vật lý Ethernet) của interface lên lại.
    Trả về True nếu link up, False nếu timeout hoặc bị stop.
    """
    carrier_path = f'/sys/class/net/{iface}/carrier'
    operstate_path = f'/sys/class/net/{iface}/operstate'
    start = time.time()
    while not stop_event.is_set():
        try:
            with open(carrier_path, 'r') as f:
                carrier = f.read().strip()
            if carrier == '1':
                return True
        except (FileNotFoundError, OSError):
            # Interface tạm thời biến mất — chờ tiếp
            pass
        if time.time() - start > timeout:
            return False
        time.sleep(poll_interval)
    return False


def _init_ethercat(iface: str):
    """
    Khởi tạo EtherCAT master từ đầu: open → config_init → SAFEOP → đọc SDO → OP → tare.
    Trả về (master, slave, counts_per_force, counts_per_torque) hoặc raise Exception.
    """
    master = pysoem.Master()
    master.open(iface)

    if master.config_init() <= 0:
        master.close()
        raise RuntimeError('Không tìm thấy cảm biến EtherCAT trên bus.')

    slave = master.slaves[0]
    master.config_map()

    # PRE_OP → SAFE_OP
    master.state = pysoem.SAFEOP_STATE
    master.write_state()
    master.state_check(pysoem.SAFEOP_STATE, 2_000_000)
    if master.state != pysoem.SAFEOP_STATE:
        master.close()
        raise RuntimeError('Slave không vào được SAFEOP.')

    # Đọc hệ số chuyển đổi từ SDO
    counts_per_force = int.from_bytes(
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
        master.close()
        raise RuntimeError('Cảm biến không vào được OP state.')

    # Hardware tare lúc khởi động
    slave.sdo_write(CONTROL_INDEX, CONTROL_SUBIDX_1, (1).to_bytes(4, 'little'))
    for _ in range(5):
        master.send_processdata()
        master.receive_processdata(2000)
        time.sleep(0.01)
    slave.sdo_write(CONTROL_INDEX, CONTROL_SUBIDX_1, (0).to_bytes(4, 'little'))

    return master, slave, counts_per_force, counts_per_torque


def _close_master_safe(master):
    """Đóng EtherCAT master an toàn, bỏ qua mọi lỗi."""
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
#  Vòng lặp EtherCAT với AUTO-RECONNECT
# ════════════════════════════════════════════════════════════════════════════

def ethercat_loop(iface: str, hz: float, stop_event: threading.Event,
                  udp_ip: str = DEFAULT_UDP_IP, udp_port: int = DEFAULT_UDP_PORT):
    """
    Vòng lặp EtherCAT chính với khả năng tự động khôi phục kết nối.

    Chạy trên PC 2, bắn dữ liệu qua UDP tới udp_ip:udp_port (IP của PC 1).

    Khi nhiễu EMI từ servo motor làm carrier Ethernet rớt nhất thời,
    driver sẽ:
      1. Phát hiện mất kết nối (WKC errors liên tiếp)
      2. Đóng EtherCAT master sạch sẽ
      3. Chờ carrier (đèn LAN) lên lại (polling /sys/class/net/.../carrier)
      4. Khởi tạo lại toàn bộ EtherCAT state machine (INIT → SAFEOP → OP)
      5. Tiếp tục đọc dữ liệu cảm biến bình thường
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    log_info(f'UDP target: {udp_ip}:{udp_port}')
    reconnect_count = 0
    MAX_RECONNECTS = 100  # Giới hạn tổng số lần reconnect (tránh loop vô hạn)

    try:
        while not stop_event.is_set() and reconnect_count <= MAX_RECONNECTS:
            master = None
            try:
                # ── Bước 1: Khởi tạo EtherCAT ────────────────────────────
                if reconnect_count == 0:
                    log_info('Khởi tạo EtherCAT lần đầu...')
                else:
                    log_info(f'═══ RECONNECT #{reconnect_count} ═══ Đang khởi tạo lại EtherCAT...')

                master, slave, counts_per_force, counts_per_torque = _init_ethercat(iface)

                log_info(
                    f'EtherCAT OP. Bắt đầu bắn UDP tới {udp_ip}:{udp_port} @ {hz:.0f}Hz'
                    + (f' (reconnect #{reconnect_count})' if reconnect_count > 0 else '')
                    + '...'
                )

                # ── Bước 2: Vòng lặp đọc cảm biến ───────────────────────
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
                            log_error(
                                f'⚠ Mất kết nối EtherCAT ({MAX_WKC_ERRORS} lỗi liên tiếp)! '
                                f'Có thể do EMI khi Servo ON.')
                            break  # Thoát vòng lặp đọc → chuyển sang reconnect

                    # Ngủ đúng phần thời gian còn lại đến deadline tiếp theo
                    now = time.perf_counter()
                    sleep_s = next_deadline - now
                    if sleep_s > 0:
                        time.sleep(sleep_s)
                    next_deadline += CYCLE_S
                    if next_deadline < time.perf_counter():
                        next_deadline = time.perf_counter() + CYCLE_S

            except Exception as e:
                log_error(f'EtherCAT lỗi: {e}')

            finally:
                # ── Bước 3: Dọn dẹp master cũ ────────────────────────────
                _close_master_safe(master)
                master = None

            # Nếu bị stop thì thoát luôn
            if stop_event.is_set():
                break

            # ── Bước 4: Chờ link (carrier) lên lại ───────────────────────
            reconnect_count += 1
            log_info(f'Đang chờ đèn LAN (carrier) của {iface} sáng lại...')
            # Đợi 1s cho chip USB-LAN ổn định trước khi bắt đầu poll carrier
            time.sleep(1.0)

            if not _wait_for_link_up(iface, stop_event, timeout=30.0):
                log_error('TIMEOUT 30s: Carrier không lên lại. Kiểm tra cáp EtherCAT!')
                break

            log_info(f'✓ Carrier UP! Đợi thêm 2s để chip ổn định trước khi reconnect...')
            # Đợi thêm 2s sau khi carrier lên để chip USB-LAN ổn định hoàn toàn
            time.sleep(2.0)

        if reconnect_count > MAX_RECONNECTS:
            log_error(f'Đã vượt quá {MAX_RECONNECTS} lần reconnect. Dừng driver.')

    finally:
        sock.close()
        log_info(
            f'Driver kết thúc. Tổng số lần reconnect: {reconnect_count}.'
        )


# ════════════════════════════════════════════════════════════════════════════
#  Điểm vào
# ════════════════════════════════════════════════════════════════════════════

def _parse_arg(argv, flag, default):
    """Đọc giá trị của tham số --flag từ argv, trả về default nếu không có."""
    if flag in argv:
        try:
            return argv[argv.index(flag) + 1]
        except IndexError:
            pass
    return default


def main():
    if len(sys.argv) < 2:
        print('Cách dùng: sudo python3 axia_sensor_driver.py <interface> [--ip <addr>] [--port <port>] [--hz <rate>]')
        print('')
        print('  Ví dụ 1 — chạy nội bộ (1 máy):')
        print('    sudo python3 axia_sensor_driver.py enxec9a0c1fc063')
        print('')
        print('  Ví dụ 2 — chạy trên PC2, bắn dữ liệu sang PC1 qua Wifi:')
        print('    sudo python3 axia_sensor_driver.py enxec9a0c1fc063 --ip 192.168.1.15')
        print('')
        print('  Ví dụ 3 — tuỳ chỉnh đầy đủ:')
        print('    sudo python3 axia_sensor_driver.py enxec9a0c1fc063 --ip 192.168.1.15 --port 50000 --hz 100')
        sys.exit(1)

    iface = sys.argv[1]
    argv  = sys.argv[1:]

    # ── Đọc tham số --ip (địa chỉ PC 1, mặc định: localhost) ─────────────────
    udp_ip = _parse_arg(argv, '--ip', DEFAULT_UDP_IP)

    # ── Đọc tham số --port (mặc định: 50000) ─────────────────────────────────
    try:
        udp_port = int(_parse_arg(argv, '--port', str(DEFAULT_UDP_PORT)))
    except ValueError:
        print('[WARN] --port không hợp lệ, dùng mặc định 50000')
        udp_port = DEFAULT_UDP_PORT

    # ── Đọc tham số --hz (mặc định: 100Hz — PC riêng, không lo tải USB) ──────
    try:
        hz = float(_parse_arg(argv, '--hz', str(DEFAULT_HZ)))
        hz = max(10.0, min(hz, 200.0))  # Giới hạn an toàn 10–200Hz
    except ValueError:
        print('[WARN] --hz không hợp lệ, dùng mặc định 100Hz')
        hz = DEFAULT_HZ

    log_info(f'Polling rate : {hz:.0f} Hz (chu kỳ {1000/hz:.1f}ms)')
    log_info(f'UDP target   : {udp_ip}:{udp_port}')
    if udp_ip == '127.0.0.1':
        log_info('[Chế độ 1 máy] Dữ liệu được gửi nội bộ (localhost).')
    else:
        log_info(f'[Chế độ 2 máy] Dữ liệu sẽ được bắn sang PC1 tại {udp_ip}:{udp_port} qua Wifi.')

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
