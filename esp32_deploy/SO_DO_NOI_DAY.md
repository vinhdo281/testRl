# Sơ Đồ Nối Dây — Xe Tự Cân Bằng 2 Bánh (ESP32 + L298N + MPU6050 + GA25-370 + LM2596)

Tài liệu này khớp chính xác với cấu hình chân trong [xe_can_bang_rl_esp32.ino](file:///c:/Users/Administrator/.gemini/antigravity/scratch/xe_can_bang_rl/esp32_deploy/xe_can_bang_rl_esp32/xe_can_bang_rl_esp32.ino) (Arduino IDE), [main_rl.cpp](file:///c:/Users/Administrator/.gemini/antigravity/scratch/xe_can_bang_rl/esp32_deploy/main_rl.cpp) (PlatformIO) và file gốc `main.cpp`.

> **Chế độ mặc định hiện tại**: `#define ENABLE_MOVE_COMMANDS 0` — xe **chỉ tự cân bằng tại chỗ** (chưa nhận lệnh `W/S/A/D`, giữ `cmdV = 0` và `cmdYawRate = 0`). Sau này muốn bật điều khiển di chuyển, chỉ cần đổi sang `#define ENABLE_MOVE_COMMANDS 1` rồi nạp lại.

## 1. Sơ đồ khối tổng quát

```text
                       +----------------------+
                       |   PIN 12V (3S LiPo)  |
                       +----------+-----------+
                                  |
             +--------------------+--------------------+
             | (+) & (-)                               | (+) & (-)
             v                                         v
+------------+-------------+              +------------+-------------+
|     MẠCH HẠ ÁP LM2596    |              |       DRIVER L298N       |
|  (Chỉnh sẵn OUT = 5.0V)  |              |  (Rút 2 jumper ENA, ENB) |
+------------+-------------+              +--+---------+----------+--+
             | 5V & GND                      | OUT1/2  | OUT3/4   | ENA,IN1,IN2,ENB,IN3,IN4
             v                               v         v          |
+------------+-------------+          +--------+   +--------+     |
|          ESP32           |<---------+ Motor  |   | Motor  |     |
|  (Cấp 5V vào chân VIN)   |          | GA25 A |   | GA25 B |     |
|                          |          | (Trái) |   | (Phải) |     |
|  GPIO 16,17 <-- Enc A    |          +---+----+   +---+----+     |
|  GPIO 18,19 <-- Enc B    |              |            |          |
|  GPIO 21,22 <-- MPU6050  |              +-----+------+          |
|  GPIO 14,27,26 --------->+--------------------+-----------------+
|  GPIO 25,33,32 --------->+--------------------------------------+
+------------+-------------+
             | 3.3V, GND, SDA(21), SCL(22)
             v
+------------+-------------+
|      IMU MPU6050         |
|   (GY-521, addr 0x68)    |
+--------------------------+
```

---

## 2. Bảng nối dây chi tiết

### A. Nguồn & Hạ áp (Pin 12V -> LM2596 -> L298N & ESP32)
> ⚠️ **Quan trọng**: Cấp nguồn pin vào **LM2596 trước**, vặn biến trở đo đồng hồ ra đúng **5.0V** ở ngõ `OUT+`/`OUT-` rồi mới nối vào ESP32.

| Từ thiết bị | Chân | Nối đến | Chân | Lưu ý |
|---|---|---|---|---|
| **Pin (11.1V–12.6V)** | `(+)` (qua công tắc) | **L298N** | `+12V` (VMS) | Nguồn động lực cho motor |
| **Pin (11.1V–12.6V)** | `(+)` (qua công tắc) | **LM2596** | `IN+` | Đầu vào mạch hạ áp |
| **Pin (11.1V–12.6V)** | `(-)` | **L298N & LM2596** | `GND` / `IN-` | Mass chung toàn xe |
| **LM2596** | `OUT+` (5.0V) | **ESP32** | `VIN` (hoặc `5V`) | **Không** nối chung với chân `+5V` của L298N |
| **LM2596** | `OUT-` | **ESP32** | `GND` | Mass chung toàn xe |

### B. ESP32 ◄──► Driver L298N
> ⚠️ **Rút 2 jumper đen** đang cắm sẵn ở chân `ENA` và `ENB` trên module L298N để nhận xung PWM từ ESP32.

| Kênh | Chân ESP32 | Chân L298N | Chức năng |
|---|---|---|---|
| **Motor A (Trái)** | `GPIO 14` | `ENA` | Băm xung PWM tốc độ bánh trái |
| **Motor A (Trái)** | `GPIO 27` | `IN1` | Điều khiển chiều quay trái |
| **Motor A (Trái)** | `GPIO 26` | `IN2` | Điều khiển chiều quay trái |
| **Motor B (Phải)** | `GPIO 25` | `ENB` | Băm xung PWM tốc độ bánh phải |
| **Motor B (Phải)** | `GPIO 33` | `IN3` | Điều khiển chiều quay phải |
| **Motor B (Phải)** | `GPIO 32` | `IN4` | Điều khiển chiều quay phải |
| **Mass chung** | `GND` | `GND` | Bắt buộc nối chung GND |

### C. Động cơ GA25-370 (Jack 6 dây) ◄──► L298N & ESP32
> ⚠️ **Cấp nguồn `3.3V` cho Encoder** (không cấp `5V` vào dây VCC encoder vì ESP32 chịu tối đa `3.3V` trên GPIO `16, 17, 18, 19`). Nên kiểm tra ký hiệu in trên PCB tròn ở đuôi motor trước khi cắm vì một số lô hàng có thể đổi màu dây.

| Dây GA25-370 (thông dụng) | Ký hiệu trên đuôi motor | Motor A (Trái) nối tới | Motor B (Phải) nối tới |
|---|---|---|---|
| **Đỏ** | `M1` / Motor `+` | L298N `OUT1` | L298N `OUT3` |
| **Trắng** | `M2` / Motor `-` | L298N `OUT2` | L298N `OUT4` |
| **Xanh dương** | `VCC` / `+3.3-5V` | ESP32 `3.3V` | ESP32 `3.3V` |
| **Đen** | `GND` | ESP32 `GND` | ESP32 `GND` |
| **Vàng** | `C1` / `A` | ESP32 `GPIO 16` (`ENCA_A`) | ESP32 `GPIO 18` (`ENCB_A`) |
| **Xanh lá** | `C2` / `B` | ESP32 `GPIO 17` (`ENCA_B`) | ESP32 `GPIO 19` (`ENCB_B`) |

### D. Cảm biến MPU6050 (GY-521) ◄──► ESP32
| Chân MPU6050 | Chân ESP32 | Ghi chú |
|---|---|---|
| `VCC` | `3.3V` | Nếu bị sụt áp/lỗi I2C có thể dùng `VIN (5V)` vì GY-521 có ổn áp 3.3V |
| `GND` | `GND` | Mass chung |
| `SDA` | `GPIO 21` | Đường dữ liệu I2C |
| `SCL` | `GPIO 22` | Đường xung nhịp I2C |
| `AD0` | Để trống (hoặc `GND`) | Địa chỉ mặc định `0x68` (`main_rl.cpp` tự thử cả `0x69`) |

---

## 3. Quy trình kiểm tra trước khi cho xe tự cân bằng
1. **Chưa gắn bánh chạm đất** (hoặc đặt xe nằm nghiêng > 45° để vào chế độ `[SAFE]` ngắt motor).
2. **Nạp code**:
   - **Arduino IDE**: Mở file đơn `esp32_deploy/xe_can_bang_rl_esp32/xe_can_bang_rl_esp32.ino` (không cần copy thêm file `.h`), chọn Board `ESP32 Dev Module`, chọn đúng cổng COM và bấm **Upload**.
   - **PlatformIO**: Mở thư mục `esp32_deploy/` và chạy Upload.
   Sau khi nạp xong, mở Serial Monitor `115200` baud và gõ **`T`** để bật telemetry.
3. **Kiểm tra MPU6050**: Dựng xe nghiêng về phía trước — giá trị `pitch(deg)` và `(acc=...)` phải **cùng tăng dương (+)**. Nếu ngược dấu, đổi `#define IMU_PITCH_SIGN -1.0f`. Nếu nghiêng trước mà số không đổi (trục bị xoay 90°), đổi `#define IMU_PITCH_IS_X_AXIS 1`.
4. **Kiểm tra Encoder GA25-370**: Dùng tay quay bánh trái và bánh phải đúng **1 vòng tiến** — `encL/R` phải tăng dương khớp `TICKS_PER_REV` (nếu âm thì đổi `ENC_L_SIGN` / `ENC_R_SIGN` sang `-1.0f`; nếu số xung/vòng khác `937`, chỉnh `MOTOR_GEAR_RATIO` theo đúng bản RPM động cơ bạn mua).
5. **Tự kiểm tra motor & calib 5 giây đầu khi bật nguồn**: Ngay khi cấp nguồn/reset, nhấc nhẹ xe hoặc giữ xe cho bánh quay tự do: 2 motor sẽ tự quay **ngược (~0.35s) → xuôi (~0.35s)** để kiểm tra chiều quay & xung encoder, rồi **tắt hẳn động cơ**. Lúc này đặt xe đứng yên ở tư thế cân bằng thật trong **5 giây** (đèn LED ESP32 nhấp nháy nhanh, Serial đếm ngược 5→1s) để calib gyro và tự động lấy điểm `0°` bù lệch trọng tâm (nếu xe dựng trong `±25°`). Khi LED sáng đứng yên là bắt đầu cân bằng; có thể gõ thêm **`Z`** bất kỳ lúc nào để zero lại. *(Hiện ở chế độ `#define ENABLE_MOVE_COMMANDS 0` nên chỉ giữ cân bằng tại chỗ; khi nào cân bằng ổn muốn test `W/S/A/D/X`, đổi thành `1`.)*
