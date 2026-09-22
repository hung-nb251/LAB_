# Protocol A/B cho radial deadband của Axia — 21/09

## 1. Vì sao cần protocol này

Ngày 21/09 deadband được hạ `4 N -> 1.5 N`, người vận hành báo robot rung hơn và
dao động quanh một điểm khi nhả tay. Các lượt log có sẵn cho chỉ dấu ủng hộ điều
đó, nhưng **không phải A/B sạch**:

- Sáu lượt ở `4 N` đều có pha LEADER/MJM; ba lượt ở `1.5 N` đều thuần FOLLOWER
  (`role=OFF`). Hai nhóm khác nhau ở nhiều thứ hơn là deadband.
- Mẫu chuyển động do người tạo ra khác nhau giữa các lượt. Median lực tiếp xúc
  có chồng lấn: lượt `114608` ở `4 N` đạt 7,39 N, cao hơn cả ba lượt `1.5 N`.
- Số lượt quá ít (3 so với 6).

Protocol này cố định mọi thứ trừ đúng một biến.

## 2. Biến duy nhất được đổi

Hằng số `DEADBAND_N` trong `scripts/axia_sensor_ui.py`. Ba mức cần so:

| Nhãn | `DEADBAND_N` | Ghi chú |
|------|--------------|---------|
| `4.0N` | `4.0` | Baseline lịch sử, mọi audit trước 21/09 dùng mức này |
| `2.5N` | `2.5` | Giá trị đang đặt sau phiên 21/09 |
| `1.5N` | `1.5` | Mức đã gây dao động khi nhả tay |

Deadband là **kiểu trừ**: `F_out = F * (|F| - dz) / |F|`. Hạ ngưỡng vừa giảm mức
lực bắt đầu có tác dụng, vừa cộng thêm `(4 - dz)` N độ lợi vào mọi mức lực vượt
ngưỡng. Đây là lý do nó không chỉ là một nút chỉnh độ nhạy.

Sau mỗi lần sửa hằng số phải **relaunch** để UI nạp giá trị mới; không cần
`colcon build` vì launch gọi thẳng `scripts/axia_sensor_ui.py`.

## 3. Những thứ phải giữ nguyên

Không đổi bất cứ mục nào dưới đây giữa các lượt:

- `prediction_model:=gru`, cùng model directory.
- Thuần FOLLOWER trong toàn bộ lượt. **Không bấm LEADER**, không Capture Target.
  Pha LEADER/MJM làm hỏng so sánh.
- `max_command_lead_m=0.04`, `max_virtual_velocity_mps=0.25`,
  `max_virtual_acceleration_mps2=1.00`, `prediction_reference_tau_sec=0.4`,
  `prediction_reference_lead_sec=0.15`.
- `M=1`, `K=5`, `critical_damping: true`; `intent_threshold_n=0.0`;
  `additional_z_deadzone_n=2.0`.
- Profile vận tốc joint và speed override trên pendant.
- `logging_profile:=compact` (mặc định). Đủ cho mọi chỉ số ở mục 6.
- Cùng một người vận hành, cùng tư thế đứng, cùng vị trí cầm trên thanh.

## 4. Mẫu chuyển động chuẩn cho mỗi lượt

Đây là phần quan trọng nhất — biến thiên do người là nguồn nhiễu lớn nhất trong
dữ liệu 21/09. Mỗi lượt kéo dài khoảng 90 giây và lặp đúng chu kỳ sau **năm
lần**:

1. Đẩy theo `+X` khoảng 200 mm, tốc độ thoải mái, trong ~3 s.
2. **Nhả tay hoàn toàn, buông rời khỏi thanh, giữ yên 3 s.** Đây là sự kiện
   được đo; buông hẳn tay chứ không chỉ giảm lực.
3. Đẩy về `-X` khoảng 200 mm trong ~3 s.
4. **Nhả tay hoàn toàn 3 s.**
5. Đẩy theo `+Z` lên khoảng 100 mm, rồi **nhả tay 3 s**.

Năm chu kỳ cho khoảng 15 sự kiện nhả tay mỗi lượt. Chỉ số `rev/nha` cần số sự
kiện này mới có ý nghĩa; dữ liệu 21/09 chỉ có 4–8 sự kiện mỗi lượt.

Không cố giữ lực ở một mức cố định — làm vậy là không tự nhiên và cũng không đo
được. Điều cần giữ là **quãng đường và nhịp**, không phải độ lớn lực.

## 5. Thứ tự chạy và số lượt

Ba lượt cho mỗi mức, chạy **xen kẽ** để mỏi tay và trôi nhiệt không dồn vào một
nhóm:

```
4.0N  ->  2.5N  ->  1.5N  ->  1.5N  ->  2.5N  ->  4.0N  ->  4.0N  ->  2.5N  ->  1.5N
```

Giữa hai lượt liền nhau: Stop Run, nghỉ 30 s. Khi đổi mức deadband thì Stop Run,
Disable Robot, `Ctrl+C` launch, sửa hằng số, relaunch, rồi **Calibrate F/T ở
trạng thái không tải** trước khi Enable lại.

Ghi lại timestamp của từng lượt kèm nhãn mức deadband ngay khi chạy. Không suy
ngược nhãn từ giờ trong tên file về sau.

## 6. Chạy phân tích

```bash
cd ~/cocarry_ws
python3 scripts/compare_axia_deadband_ab.py \
    --label 4.0N <stem1> <stem2> <stem3> \
    --label 2.5N <stem4> <stem5> <stem6> \
    --label 1.5N <stem7> <stem8> <stem9>
```

`<stem>` là phần `YYYYMMDD_HHMMSS` trong tên file CSV. Script in **hai dòng cho
mỗi lượt**: một dòng trên toàn bộ mẫu `RUNNING` và một dòng trên tập con
`FOLLOWER_PREDICTION`, để thấy rõ mask đã bỏ đi những gì.

Các chỉ số:

| Cột | Ý nghĩa |
|-----|---------|
| `F>0` | Median độ lớn lực khi vượt deadband. Xác nhận mức deadband thực sự có hiệu lực. |
| `zero%` | Tỷ lệ mẫu lực bằng đúng 0, tức deadband chặn hoàn toàn. |
| `sat%` | Tỷ lệ thời gian reference bị ghim ở giới hạn command-lead 40 mm. |
| `refAcc` | Độ dao động của reference gửi xuống robot. **Chỉ số rung chính.** |
| `eeAcc` | Gia tốc EE thực. Tay máy lọc bớt nên thường ít nhạy hơn `refAcc`. |
| `rev/s` | Số lần vector vận tốc EE quay đầu hơn 120° mỗi giây. |
| `rev/nha` | Số lần quay đầu **sau mỗi sự kiện nhả tay**. Chỉ số bám sát nhất với hiện tượng người dùng mô tả. |
| `trôi_mm` | Quãng đường EE còn đi thêm sau khi đã nhả tay. |

## 7. Tiêu chí đánh giá

Ưu tiên `rev/nha`, rồi `refAcc`, rồi `trôi_mm`.

- Nếu `rev/nha` của một mức **bằng 0 ở cả ba lượt** thì mức đó không gây dao
  động sau nhả tay trong điều kiện đã thử.
- Nếu ba lượt của một mức tách hoàn toàn khỏi ba lượt của mức khác trên
  `refAcc` (không chồng lấn dải), coi là chỉ dấu đáng tin.
- Nếu các dải chồng lấn, **không kết luận**. Ba lượt mỗi mức không đủ cho kiểm
  định thống kê; đây là sàng lọc, không phải chứng minh.

Chỉ hạ deadband xuống mức thấp hơn nếu mức đó vừa đạt `rev/nha = 0` vừa không
làm `refAcc` xấu đi.

## 8. Những gì protocol này KHÔNG trả lời được

- **Rung cơ khí trên 7,5 Hz.** Axia chạy 100 Hz nhưng CSV `compact` ghi 15 Hz,
  nên mọi thành phần trên Nyquist 7,5 Hz là vô hình. Nếu nghi thanh cầm cộng
  hưởng và rò qua deadband thì phải chạy `logging_profile:=calibration` để có
  sidecar raw Axia, và đó là một phép đo riêng.
- **Lực quán tính của payload.** Bù trọng lực trong `axia_sensor_ui.py` chỉ trừ
  `m*g` tĩnh theo pose, không trừ `m*a`. Ở dữ liệu 21/09 thành phần này đo được
  khoảng 0,10 N (p95 0,56 N) tại băng thông 15 Hz — dưới mọi mức deadband đang
  xét, nên chưa phải nghi can. Con số này sẽ khác nếu gắn thêm tải.
- **Vòng ripple prediction–admittance.** Ở chế độ GRU, `nominal` được tính từ
  chính chuỗi robot EE, nên chuyển động của robot hồi tiếp vào nominal
  (`CODEX.md`, lượt `20260906_105435`). Deadband thay đổi độ lợi của vòng này.
  Muốn tách riêng ảnh hưởng đó phải chạy thêm một khối ở `Ground Truth`, nơi
  `nominal` cố định. Đó là phần mở rộng, không nằm trong protocol này.

## 9. An toàn

Người vận hành chạy robot; không tự Enable/Start. Lần chạy đầu sau mỗi lần đổi
deadband phải để speed override thấp và sẵn sàng E-stop, vì hạ deadband làm tăng
độ lợi lực→chuyển động. Không nới workspace, joint limit, force limit hay
tracking threshold trong lúc chạy protocol này — nếu phải nới thì phép so sánh
đã hỏng và cần chạy lại từ đầu.
