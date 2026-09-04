# Hướng Dẫn Cài Đặt Axia Force Sensor Driver trên Windows (PC 2)

## Mục Tiêu
Tài liệu này giúp bạn cài đặt phần mềm đọc dữ liệu cảm biến lực ATI Axia80-M20 trên máy Windows và gửi dữ liệu sang máy Ubuntu của nhóm qua mạng Wifi hoặc LAN.

---

## Bước 0: Kiểm tra Trước Khi Bắt Đầu
Trước khi cài đặt, bạn cần có:
- [ ] Máy tính Windows đã cắm dây LAN với cảm biến lực ATI Axia80
- [ ] File `axia_sensor_driver_win.py` (lấy từ máy của người hướng dẫn)
- [ ] Biết địa chỉ IP Wifi của máy Ubuntu (PC 1) — hỏi người hướng dẫn

> [!IMPORTANT]
> Nếu kết nối 2 máy bằng **dây LAN trực tiếp** (không qua Wifi), địa chỉ IP cần dùng là `10.0.0.1`.
> Nếu kết nối qua **Wifi chung phòng Lab**, hỏi người hướng dẫn địa chỉ IP của họ (ví dụ: `192.168.1.13`).

---

## Bước 1: Cài Đặt Python 3

1. Vào trang [https://www.python.org/downloads/](https://www.python.org/downloads/)
2. Tải bản **Python 3.11** (hoặc mới hơn) và chạy file `.exe`
3. ⚠️ **Quan trọng:** Khi cửa sổ cài đặt hiện ra, nhớ **tích vào ô "Add Python to PATH"** ở dưới cùng trước khi bấm Install Now.
4. Sau khi cài xong, mở **Command Prompt** (bấm Windows + R, gõ `cmd`, Enter) và kiểm tra:
   ```cmd
   python --version
   ```
   Nếu hiện ra `Python 3.x.x` là thành công.

---

## Bước 2: Cài Đặt Npcap (Bắt buộc)

Npcap là phần mềm giúp Python có thể truy cập vào card mạng ở tầng thấp (cần thiết cho giao thức EtherCAT).

1. Vào trang [https://npcap.com/#download](https://npcap.com/#download)
2. Tải file **Npcap installer** (không phải Npcap SDK)
3. Chạy file cài đặt, khi thấy màn hình Options, **nhớ tích vào ô:**
   > ✅ **"Install Npcap in WinPcap API-compatible Mode"**
4. Bấm Install và chờ hoàn tất.

---

## Bước 3: Cài Đặt Thư Viện Python

Mở **Command Prompt** với quyền Administrator (click chuột phải vào cmd → "Run as administrator") và gõ:

```cmd
pip install pysoem
```

Chờ cho đến khi thấy dòng chữ `Successfully installed pysoem-...` là xong.

---

## Bước 4: Tìm Tên Card Mạng (Interface Name)

Trên Windows, tên card mạng không phải là `eth0` mà là một chuỗi mã rất dài. Bạn cần tìm đúng tên của card mạng đang cắm với cảm biến lực.

1. **Đảm bảo dây LAN của cảm biến đã được cắm vào máy.**
2. Mở **Command Prompt** với quyền Administrator và gõ:
   ```cmd
   python -m pysoem
   ```
3. Lệnh này sẽ in ra danh sách tất cả các card mạng, ví dụ:
   ```
   \Device\NPF_{1A2B3C4D-5E6F-7A8B-9C0D-1E2F3A4B5C6D}  (Realtek PCIe GbE ...)
   \Device\NPF_{9Z8Y7X6W-5V4U-3T2S-1R0Q-9P8O7N6M5L4K}  (USB Ethernet Adapter)
   ```
4. Bạn cần xác định dòng nào là card mạng đang cắm với cảm biến. Thường là dòng có chữ **USB Ethernet** hoặc **USB LAN** ở tên.
5. **Copy toàn bộ chuỗi** bắt đầu từ `\Device\NPF_{...}` của card đó lại, bạn sẽ cần nó ở bước tiếp theo.

---

## Bước 5: Copy File Driver và Chạy

1. Copy file `axia_sensor_driver_win.py` vào một thư mục dễ tìm, ví dụ: `C:\axia_driver\`
2. Mở **Command Prompt** với quyền Administrator, di chuyển vào thư mục đó:
   ```cmd
   cd C:\axia_driver
   ```
3. Chạy driver với lệnh bên dưới (thay thế các phần trong `{}` bằng thông tin thực của bạn):

   **Nếu kết nối 2 máy qua Wifi chung:**
   ```cmd
   python axia_sensor_driver_win.py \Device\NPF_{GUID_CUA_BAN} --ip 192.168.1.13
   ```
   
   **Nếu kết nối 2 máy qua dây LAN trực tiếp (IP tĩnh):**
   ```cmd
   python axia_sensor_driver_win.py \Device\NPF_{GUID_CUA_BAN} --ip 10.0.0.1
   ```

4. Nếu thành công, bạn sẽ thấy:
   ```
   [AxiaDriver-Win] UDP target: 192.168.1.13:50000
   [AxiaDriver-Win] Interface : \Device\NPF_{...}
   [AxiaDriver-Win] Đang chạy (Bấm Ctrl+C để thoát)...
   [AxiaDriver-Win] counts/F=1000000, counts/T=1000000
   [AxiaDriver-Win] EtherCAT OP. Bắt đầu bắn UDP tới ... @ 100Hz...
   ```

5. Khi muốn dừng, bấm **Ctrl + C**.

---

## Xử Lý Lỗi Thường Gặp

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| `ModuleNotFoundError: No module named 'pysoem'` | Chưa cài pysoem | Chạy `pip install pysoem` |
| `OSError: [WinError 10013]` hoặc `Access denied` | Chưa chạy cmd bằng quyền Admin | Mở cmd → chuột phải → **Run as administrator** |
| `RuntimeError: Không tìm thấy cảm biến EtherCAT` | Sai tên Interface hoặc dây chưa cắm | Kiểm tra lại dây LAN và tên GUID ở Bước 4 |
| `No module named 'pysoem'` khi chạy bằng Admin | Python của user và Admin khác nhau | Chạy `py -m pip install pysoem` thay vì `pip install` |

---

## Kiểm Tra Kết Nối Thành Công

Sau khi driver đang chạy trên PC 2 (Windows), trên **màn hình UI của PC 1 (Ubuntu)**, đồ thị **Human Force** sẽ bắt đầu nhúc nhích và hiển thị dữ liệu lực theo thời gian thực.

Nếu đồ thị vẫn đứng yên sau 10 giây, kiểm tra:
1. Trên PC 1, chạy lệnh: `ping <IP-cua-PC2>` để kiểm tra 2 máy có ping được nhau không.
2. Đảm bảo Firewall của Windows trên PC 2 không chặn port UDP 50000.
   - Vào **Windows Defender Firewall** → **Advanced Settings** → **Inbound Rules** → **New Rule** → Port → UDP → 50000 → Allow.

---

*Tài liệu được tạo cho hệ thống CoCarry — Robot Yaskawa HC10DTP + ATI Axia80-M20.*
