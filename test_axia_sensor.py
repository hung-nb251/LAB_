"""
Test doc 6 gia tri Fx,Fy,Fz,Tx,Ty,Tz tu cam bien ATI Axia80-M20 qua EtherCAT (pysoem/SOEM).

Yeu cau:
- pip install pysoem
- Windows: cai Npcap (tick "Install Npcap in WinPcap API-compatible Mode") - https://npcap.com
- Chay voi quyen Administrator (Windows) / sudo (Linux)
- Cap mang noi TRUC TIEP tu PC toi cong EtherCAT IN cua sensor (khong qua switch thuong)

Chay:
  python test_axia_sensor.py --list      # liet ke adapter de lay dung iface
  python test_axia_sensor.py              # chay test doc lien tuc
"""

import struct
import sys
import time
import select

import pysoem

# TxPDO 0x1A00: Fx,Fy,Fz,Tx,Ty,Tz (DINT, 4 byte moi truc) + Status Code (UDINT) + Sample Counter (UDINT)
RAW_FMT = '<6iII'  # 24 + 4 + 4 = 32 byte, dung thu tu trong file ESI

SDO_CALIB_INDEX = 0x2021
SUBIDX_COUNTS_PER_FORCE = 0x37   # 55 - da doi chieu voi ESI, dung
SUBIDX_COUNTS_PER_TORQUE = 0x38  # 56 - da doi chieu voi ESI, dung

CONTROL_INDEX = 0x7010
CONTROL_SUBIDX_1 = 0x01


class ATISensor:
    def __init__(self, iface, state_timeout_us=50000):
        self.master = pysoem.Master()
        self.master.open(iface)

        if self.master.config_init() <= 0:
            self.master.close()
            raise RuntimeError(
                "Khong tim thay slave EtherCAT nao. Kiem tra: cap noi truc tiep, "
                "chay quyen Administrator/sudo, dung iface (dung --list de xem)."
            )

        self.slave = self.master.slaves[0]
        print(f"Tim thay slave: {self.slave.name}")

        self.master.config_map()

        # --- Chuyen SAFEOP roi moi sang OP, va XAC NHAN state thuc te ---
        self.master.state = pysoem.SAFEOP_STATE
        self.master.write_state()
        self.master.state_check(pysoem.SAFEOP_STATE, state_timeout_us)
        if self.master.state != pysoem.SAFEOP_STATE:
            self.master.close()
            raise RuntimeError("Slave khong vao duoc SAFEOP - kiem tra PDO mapping/day noi.")

        self.master.state = pysoem.OP_STATE
        self.master.write_state()

        reached_op = False
        for _ in range(40):
            self.master.send_processdata()
            self.master.receive_processdata(state_timeout_us)
            self.master.state_check(pysoem.OP_STATE, state_timeout_us)
            if self.master.state == pysoem.OP_STATE:
                reached_op = True
                break

        if not reached_op:
            self.master.close()
            raise RuntimeError("Slave khong vao duoc OP state sau nhieu lan thu.")

        print("Slave da vao trang thai OP.")

        self.counts_per_force = int.from_bytes(
            self.slave.sdo_read(SDO_CALIB_INDEX, SUBIDX_COUNTS_PER_FORCE), 'little')
        self.counts_per_torque = int.from_bytes(
            self.slave.sdo_read(SDO_CALIB_INDEX, SUBIDX_COUNTS_PER_TORQUE), 'little')

        if self.counts_per_force == 0 or self.counts_per_torque == 0:
            self.close()
            raise RuntimeError("Counts Per Force/Torque doc ve = 0, khong the quy doi don vi.")

        print(f"Counts Per Force = {self.counts_per_force}, Counts Per Torque = {self.counts_per_torque}")

    def read_force(self):
        self.master.send_processdata()
        wkc = self.master.receive_processdata(2000)
        if wkc < self.master.expected_wkc:
            raise RuntimeError(f"Working Counter thap ({wkc}/{self.master.expected_wkc}) - mat ket noi process data.")

        data = self.slave.input
        expected_len = struct.calcsize(RAW_FMT)
        if len(data) != expected_len:
            raise RuntimeError(f"Kich thuoc input PDO sai: nhan {len(data)} byte, can {expected_len} byte.")

        Fx, Fy, Fz, Tx, Ty, Tz, status, counter = struct.unpack(RAW_FMT, data)

        return {
            "Fx": Fx / self.counts_per_force,
            "Fy": Fy / self.counts_per_force,
            "Fz": Fz / self.counts_per_force,
            "Tx": Tx / self.counts_per_torque,
            "Ty": Ty / self.counts_per_torque,
            "Tz": Tz / self.counts_per_torque,
            "status": status,
            "counter": counter,
        }

    def tare(self):
        """Xung 1 -> 0 tren Control 1 (0x7010:01) de bias cam bien ve 0."""
        self.slave.sdo_write(CONTROL_INDEX, CONTROL_SUBIDX_1, (1).to_bytes(4, 'little'))
        # Duy tri process data trong luc cho 50ms de tranh timeout
        for _ in range(5):
            self.master.send_processdata()
            self.master.receive_processdata(2000)
            time.sleep(0.01)
        self.slave.sdo_write(CONTROL_INDEX, CONTROL_SUBIDX_1, (0).to_bytes(4, 'little'))

    def close(self):
        try:
            self.master.state = pysoem.INIT_STATE
            self.master.write_state()
        finally:
            self.master.close()


def list_adapters():
    print("Danh sach network adapter:")
    for adapter in pysoem.find_adapters():
        print(f"  {adapter.name}  -  {adapter.desc}")


def main():
    if len(sys.argv) == 1:
        print("Sử dụng: sudo python3 test_axia_sensor.py <interface_name>")
        print("Chạy 'sudo python3 test_axia_sensor.py --list' để xem danh sách giao diện mạng (thường là eth0, enp3s0...).")
        return

    if len(sys.argv) > 1 and sys.argv[1] == "--list":
        list_adapters()
        return

    # Lấy giao diện mạng từ đối số dòng lệnh (VD: eth0, enp3s0)
    iface = sys.argv[1]

    sensor = ATISensor(iface)
    try:
        print("Nhan Enter de tare (bias) cam bien ve 0 - dam bao khong co luc tac dong...")
        while True:
            sensor.master.send_processdata()
            sensor.master.receive_processdata(2000)
            
            # Non-blocking kiểm tra phím Enter trên Linux
            i, o, e = select.select([sys.stdin], [], [], 0.0)
            if i:
                sys.stdin.readline()
                break
                
            time.sleep(0.01)

        sensor.tare()
        print("Da tare.\n")

        print("Dang doc du lieu, Ctrl+C de dung.\n")
        last_print_time = time.time()
        while True:
            r = sensor.read_force()
            
            current_time = time.time()
            if current_time - last_print_time >= 0.1:
                print(
                    f"Fx={r['Fx']:+8.3f} N  Fy={r['Fy']:+8.3f} N  Fz={r['Fz']:+8.3f} N  "
                    f"Tx={r['Tx']:+8.4f} Nm  Ty={r['Ty']:+8.4f} Nm  Tz={r['Tz']:+8.4f} Nm  "
                    f"status=0x{r['status']:08x} counter={r['counter']}"
                )
                last_print_time = current_time
                
            time.sleep(0.01)  # Vong lap doc du lieu 100Hz de tranh Watchdog timeout

    except KeyboardInterrupt:
        print("\nDa dung test.")
    finally:
        sensor.close()


if __name__ == "__main__":
    main()
