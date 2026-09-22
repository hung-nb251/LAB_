# Bằng chứng M-register là external force, không phải robot intent — 21/09/2026

## Kết luận

Hai trial `104650` và `104823` tái hiện đúng phép thử đối chứng: trong pha
LEADER, người vận hành cố ý đẩy ngược hướng robot đang đi. Kết quả cho thấy
ước lượng lực từ M310 vẫn gần như song song với lực Axia, trong khi vận tốc EE
đổi sang ngược hướng lực người.

Kết hợp với định nghĩa của Yaskawa rằng M310–M315 là *estimated external joint
torque* và M320–M325 là *estimated TCP external force*, đây là bằng chứng trực
tiếp rằng trường đang đặt tên `F_robot` thực chất là một ước lượng chậm hơn của
`F_ext`. Nó không mang vector ý định/chủ động của robot và không dùng được cho
`cos(F_robot, F_ext)` để phát hiện conflict.

Nguồn định nghĩa: [MotoROS2 Discussion #509](https://github.com/Yaskawa-Global/motoros2/discussions/509)
và [Yaskawa register list HW1484764](https://github.com/Yaskawa-Global/motoros2/files/13300071/register.list.pdf).

## Kết quả pha LEADER

Chỉ tính các scan M310 duy nhất, không đếm lặp các dòng CSV 15 Hz. Với cosine
lực, yêu cầu cả hai norm >=4 N. Với cosine vận tốc, yêu cầu `|F_ext|>=4 N` và
`|v_EE|>=0,01 m/s`.

| Trial | LEADER [s sau RUNNING] | n | median cos(F_robot,F_ext) | min | số âm | median cos(v_EE,F_ext) | số âm |
|---|---:|---:|---:|---:|---:|---:|---:|
| `104650` | 5,040–10,267 | 20 | +0,976 | +0,931 | 0/20 | −0,831 | 18/20 |
| `104823` | 5,066–9,933 | 19 | +0,971 | +0,938 | 0/19 | −0,820 | 15/19 |
| Gộp mô tả | — | 39 | — | — | **0/39** | — | **33/39** |

Hai đến bốn scan đầu pha LEADER còn phản ánh quá trình chuyển pha/bắt đầu tác
động. Chia mỗi khoảng LEADER thành hai nửa theo thời gian, nửa sau cho kết quả:

| Trial | n | cos(F_robot,F_ext) âm | median | cos(v_EE,F_ext) âm | median |
|---|---:|---:|---:|---:|---:|
| `104650` | 10 | 0/10 | +0,982 | **10/10** | −0,864 |
| `104823` | 10 | 0/10 | +0,971 | **10/10** | −0,842 |
| Gộp mô tả | 20 | **0/20** | — | **20/20** | — |

Đạo hàm của reference cũng cho cùng kết luận: ở nửa sau LEADER,
`cos(v_reference,F_ext)` âm 8/8 mẫu đủ điều kiện trong từng trial. Vì vậy dấu âm
không phải chỉ là nhiễu khi lấy đạo hàm actual EE.

## Chênh lệch tốc độ đo

| Trial | Axia | M310 vector | Tỷ lệ | Span đọc 6 register, median/P95 |
|---|---:|---:|---:|---:|
| `104650` | 100,017 Hz | 4,821 Hz | 20,75× | 153,9 / 174,8 ms |
| `104823` | 100,024 Hz | 4,824 Hz | 20,74× | 154,8 / 174,9 ms |

Axia có sáu kênh trong cùng bản tin. M310–M315 được đọc tuần tự nên một vector
còn trải trên khoảng 154 ms. Vì vậy M-register vừa đo cùng external interaction
wrench, vừa chậm hơn khoảng 20,7 lần và không đồng thời theo sáu trục.

## Phương pháp

1. Lấy từng scan M310 duy nhất từ sidecar và dùng timestamp giữa scan; chỉ giữ
   scan hoàn tất trong RUNNING trước guard dừng 1 s.
2. Replay đúng model runtime từ snapshot để tái tạo `F_robot`; sai số so với
   giá trị node đã publish dưới `2,2e-14 N`.
3. Tái dựng `F_ext` từ Axia raw với bias, payload và rotation đã ghi, trước
   deadband. Nội suy chỉ khi gap <=50 ms.
4. Lấy role từ dòng CSV gần nhất phía trước timestamp scan. Tính `v_EE` bằng
   local-linear derivative của actual EE trên cửa sổ cố định ±0,20 s.
5. Không fit model, không tối ưu lag, không chọn lại dấu hoặc threshold theo
   hai trial này. Ngưỡng lực 4 N trùng deadband vận hành đã dùng trong các audit
   trước; ngưỡng tốc độ chỉ loại cosine không xác định khi robot gần đứng yên.

Các mẫu liên tiếp trong một trial có tương quan thời gian, nên các tỷ lệ gộp là
thống kê mô tả, không được diễn giải như 39 phép thử Bernoulli độc lập. Sức nặng
của kết luận đến từ: định nghĩa register của nhà sản xuất, dấu cosine đối nghịch
trong cùng thời điểm, và sự lặp lại trên hai trial riêng.

## Artifact và tái lập

- Script: `analyze_force_source_leader.py`.
- Artifact chính: `cocarry_logs/20260921_force_source_leader_evidence_v2/`.
- `leader_force_direction_evidence.png`: vùng xám là LEADER, đường chấm là
  trung điểm; chấm tím là cosine hai lực, dấu xanh là cosine vận tốc–lực.
- `paired_cosines.csv`: toàn bộ mẫu ghép và mask.
- `report.json`: hash nguồn, tốc độ, ngưỡng và thống kê đầy đủ.

```bash
cd /home/hungnb/cocarry_ws
OPENBLAS_NUM_THREADS=1 python3 scripts/analyze_force_source_leader.py \
  cocarry_logs/cocarry_admittance_3d_20260921_104650.csv \
  cocarry_logs/cocarry_admittance_3d_20260921_104823.csv \
  --output <thu_muc_moi>
```

Log gốc và model runtime không bị thay đổi. Phân tích này không cấp quyền dùng
M-register cho control, role selection hoặc safety.
