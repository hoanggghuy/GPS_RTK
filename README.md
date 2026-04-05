# GPS_RTK

Node ROS 2 để:
- Nhận RTCM từ RF
- Phát RTCM đồng thời cho 2 rover GNSS (front/rear)
- Đọc NMEA GGA từ 2 rover
- Đồng bộ theo UTC và publish `NavSatFix` cho từng rover

---

## 1) Cấu trúc chính

- `main.py`: node ROS2 chính (`DualGpsHeadingNode`)
- `test/test_rf.py`: đọc và kiểm tra frame RTCM từ RF
- `test/test_rtcm.py`: parse type RTCM từ stream serial
- `test/test_uart.py`: đọc GGA trực tiếp từ UART
- `draw_path.py`: vẽ quỹ đạo từ CSV (`lat/lon` hoặc `latitude/longitude`)
- `gps_front_path.csv`, `gps_rear_path.csv`: dữ liệu quỹ đạo mẫu
- `gps_log.csv`: log NMEA mẫu

---

## 2) Yêu cầu môi trường

- Ubuntu + ROS 2 Humble
- Python 3.10+
- Thiết bị serial:
  - RF input: `/dev/ttyUSB2`
  - Front rover: `/dev/ttyACM1`
  - Rear rover: `/dev/ttyUSB0`

Cài thư viện Python:

```bash
pip install pyserial pynmea2 matplotlib numpy pandas
```

> `rclpy` và `sensor_msgs` đi kèm ROS 2 (cài theo ROS distro).

---

## 3) Cấu hình cổng/baud

Sửa trực tiếp trong `main.py`:

- `COM_RTCM_IN`
- `COM_FRONT_ROVER`
- `COM_REAR_ROVER`
- `BAUD_RTCM_IN`
- `BAUD_ROVER`

Các tham số quan trọng:
- `SYNC_TIMEOUT_SEC`: timeout đồng bộ 2 rover theo UTC
- `PUBLISH_ONLY_WHEN_BOTH_RTK_FIX`: chỉ publish khi cả 2 đều fix quality = 4
- `PRINT_ALL_NMEA`: bật log toàn bộ NMEA

---

## 4) Chạy node chính

```bash
python3 main.py
```

Node sẽ publish:
- `/gps/front/fix`
- `/gps/rear/fix`

Message type: `sensor_msgs/msg/NavSatFix`

---

## 5) Kiểm tra nhanh phần cứng/RTCM

### 5.1 Kiểm tra RF có RTCM không
```bash
python3 test/test_rf.py
```
Kết quả mong đợi: in được các frame `[RTCM] type=...`.

### 5.2 Kiểm tra parse RTCM type
```bash
python3 test/test_rtcm.py
```
Kết quả mong đợi: in `RTCM Type` liên tục.

### 5.3 Kiểm tra rover xuất GGA
```bash
python3 test/test_uart.py
```
Kết quả mong đợi: in `Lat/Lon` và `Fix`.

---

## 6) Vẽ quỹ đạo GPS từ CSV

Ví dụ với rear path:
```bash
python3 draw_path.py --input gps_rear_path.csv --output gps_rear.png
```

Ví dụ hiển thị trực tiếp:
```bash
python3 draw_path.py --input gps_front_path.csv --show
```

---

