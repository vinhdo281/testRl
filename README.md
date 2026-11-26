# Xe Cân Bằng 2 Bánh — RL Balancing & Locomotion Project (MuJoCo + PPO)

Dự án Reinforcement Learning (PPO) huấn luyện xe tự cân bằng 2 bánh trong **MuJoCo**, hỗ trợ vừa giữ thăng bằng tại chỗ vừa nhận lệnh di chuyển (`Tiến / Lùi / Rẽ trái / Rẽ phải / Dừng`), xây dựng theo thông số hình học & phần cứng từ bộ file CAD SolidWorks (`xe can băng`) và firmware `main.cpp` (ESP32 + L298N + MPU6050 + Encoder).

## 1. Thông số mô hình trích xuất từ CAD & `main.cpp`

| Thành phần | Nguồn | Kích thước / Thông số |
|---|---|---|
| Bánh xe (`bánh 65.SLDPRT`) | CAD Parasolid | Đường kính 65 mm (`R = 0.0325 m`), track width `0.156 m` |
| Động cơ DC + Encoder (`môt.SLDPRT`) | CAD Parasolid + `main.cpp` | `Ø24 mm × 68 mm`, 2 kênh ngắt encoder (`16/17`, `18/19`) |
| Gá động cơ (`gá môt.SLDPRT`) | CAD Parasolid | `30 × 20 × 15 mm` |
| Đế pin (`đế pinccc.SLDPRT`) | CAD Parasolid | `80 × 70 × 3 mm` |
| Pin (`pin.SLDPRT`) | CAD Parasolid | `78 × 60 × 21 mm` |
| Driver L298N (`l298n.SLDPRT`) | CAD Parasolid + `main.cpp` | `43 × 43 × 28 mm`, PWM `ENA=14, ENB=25` |
| Mạch hạ áp LM2596 (`mạch hạ ap.SLDPRT`) | CAD Parasolid | `44 × 22 × 14 mm` |
| ESP32 (`esp32.SLDPRT`) | CAD Parasolid | `53 × 29 × 15 mm` |
| IMU MPU6050 (`mpu.SLDPRT`) | CAD Parasolid | `21 × 16 × 5 mm` |

> **Lưu ý về mesh CAD**: Vì máy hiện chưa cài SolidWorks (các file `.SLDPRT` / `.SLDASM` là định dạng nhị phân độc quyền của SolidWorks), mô hình MuJoCo `assets/xe_can_bang.xml` dùng các khối hình học nguyên thủy theo đúng kích thước đo từ stream Parasolid bên trong file CAD. Nếu bạn export các chi tiết ra `.STL` từ SolidWorks sau này, có thể gắn trực tiếp `<mesh file="..."/>` vào `assets/xe_can_bang.xml`.

## 2. Cấu trúc Project

```text
xe_can_bang_rl/
├── assets/
│   └── xe_can_bang.xml        # Mô hình vật lý MuJoCo (500 Hz physics, 50 Hz control)
├── envs/
│   ├── __init__.py
│   └── balance_car_env.py     # Gymnasium Env 12D + Domain Randomization + Lệnh (v_cmd, yaw_cmd)
├── train.py                   # Huấn luyện PPO (Stable-Baselines3)
├── evaluate.py                # Điều khiển trực tiếp trong MuJoCo 3D Viewer, chạy demo & vẽ đồ thị
├── export_esp32.py            # Xuất trọng số MLP + firmware ESP32 (hỗ trợ lệnh Serial W/S/A/D/X)
├── requirements.txt
└── README.md
```

## 3. Hướng dẫn chạy (dùng môi trường conda `mujoco`)

### Huấn luyện PPO (cân bằng + bám lệnh di chuyển)
```powershell
C:\Users\Administrator\miniconda3\envs\mujoco\python.exe train.py --total-timesteps 400000 --n-envs 8
```

### Điều khiển trực tiếp trong MuJoCo 3D Viewer
```powershell
C:\Users\Administrator\miniconda3\envs\mujoco\python.exe evaluate.py
```
Phím điều khiển:
- **Trong cửa sổ 3D Viewer**: Mũi tên `↑ / ↓ / ← / →` hoặc `I / K / J / L` để di chuyển, `X` (hoặc `0`) để dừng, `P` để đẩy thử.
- **Trong cửa sổ Terminal**: `W` (Tiến), `S` (Lùi), `A` (Rẽ trái), `D` (Rẽ phải), `X` (Dừng), `P` (Đẩy thử) — tương thích với `main.cpp`.

### Xem trình diễn tự động các lệnh di chuyển (3D Viewer)
```powershell
C:\Users\Administrator\miniconda3\envs\mujoco\python.exe evaluate.py --demo-move --push-test
```

### Đánh giá định lượng & xuất đồ thị (`eval_plot.png`)
```powershell
C:\Users\Administrator\miniconda3\envs\mujoco\python.exe evaluate.py --headless --push-test --episodes 5
```

### Xuất policy & firmware ESP32 (Arduino IDE 1 file `.ino` & PlatformIO)
```powershell
C:\Users\Administrator\miniconda3\envs\mujoco\python.exe export_esp32.py
```
- **Arduino IDE (khuyên dùng nếu muốn 1 file duy nhất)**: Mở [xe_can_bang_rl_esp32.ino](file:///c:/Users/Administrator/.gemini/antigravity/scratch/xe_can_bang_rl/esp32_deploy/xe_can_bang_rl_esp32/xe_can_bang_rl_esp32.ino) (đã gộp sẵn toàn bộ trọng số RL và code điều khiển GA25-370 + L298N + MPU6050 trong 1 file).
- **PlatformIO**: Thư mục `esp32_deploy/` có sẵn `platformio.ini`, `rl_policy.h` và `main_rl.cpp`.
- **Cấu hình mặc định**: Động cơ **GA25-370 130 RPM** (`MOTOR_GEAR_RATIO = 46.8f`, `2059.2 ticks/vòng`) và `#define ENABLE_MOVE_COMMANDS 1` (**vừa cân bằng vừa di chuyển** `W/S/A/D/X/+/-`, kèm `T` debug, `Z` zero, tự kiểm tra motor ngược/xuôi và calib 5s lúc khởi động). Đổi `ENABLE_MOVE_COMMANDS` về `0` nếu chỉ muốn đứng cân bằng tại chỗ.
