# LD14P Wi-Fi Bridge V1 — ESP32-S3 #2 + Ubuntu ROS 2 Jazzy

## Mục đích và phạm vi

- **Không phải micro-ROS**: truyền **byte UART nhị phân thô** của LD14P qua TCP/Wi-Fi; driver `ldlidar_sl_ros2` vốn đã hoạt động trên Ubuntu tiếp tục giải mã thành `/scan` + `/pointcloud2d`.
- **Không đụng tới ESP32-S3 #1, `micro_ros_ws`, PID, IMU hay encoder**.
- Giữ đúng sơ đồ đã sử dụng: LD14P chân 1 PWR/RX -> GND (chế độ tự quay), chân 2 -> GND, chân 3 TX -> ESP32-S3 #2 **GPIO16**, chân 4 -> nguồn **5V phù hợp**. GND chung. Không tự ý đổi chân.
- **UART1 RX mặc định 115200**, giống `~/ld14_usb_bridge/main/main.c` hiện có (đã hoạt động thực tế). UART0 của CH343 chỉ dùng log/nạp, không dùng để phát dữ liệu LiDAR mới. Cổng 230400 trong ROS là cài đặt ở PTY ảo, không quyết định baud UART1 thật.
- TCP server port `7777` mặc định. **Chỉ một máy khách** mỗi lúc. TCP chưa mã hóa ở tầng ứng dụng, chỉ dùng trên WLAN tin cậy (WPA2/WPA3), không port-forward Internet.

## 1. Chuẩn bị Ubuntu

1. Đảm bảo ESP32 #2 và Ubuntu dùng cùng mạng có thể truy cập TCP, Wi-Fi **2,4 GHz** cho ESP32. Không bật AP/client isolation.
2. **Dừng launch LiDAR USB cũ** bằng `Ctrl+C` ở Terminal chạy `ros2 launch ldlidar_sl_ros2 ld14p.launch.py`. Không để cả hai node cùng publish `/scan`.
3. Không cần sửa `ldlidar_ros2_ws`, driver và static TF cũ. `ubuntu/ld14p_wifi.launch.py` dùng lại executable `ldlidar_sl_ros2_node`.
4. Cài socat (nếu thiếu):

   ```bash
   sudo apt update
   sudo apt install -y socat
   ```

## 2. Build + nạp firmware MỚI, riêng ESP32 #2

Sau khi giải nén `LD14P_WIFI_BRIDGE_V1.zip` vào `~/Downloads`:

```bash
cd ~/Downloads/LD14P_WIFI_BRIDGE_V1/esp32_idf
source ~/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py menuconfig
```

Trong `LD14P Wi-Fi Bridge` cấu hình:

- Wi-Fi SSID: mạng 2,4 GHz có kết nối tới Ubuntu;
- Wi-Fi password: mật khẩu của mạng đó;
- TCP port: `7777`;
- LiDAR UART1 RX baud: **`115200`** để khớp firmware USB Bridge cũ. **Không đổi ngay sang 230400**; nếu sau này cần kiểm chứng, dùng các gói UART và kết quả thực tế.

Lưu cấu hình rồi:

```bash
idf.py build
```

Ngắt nguồn driver motor, xác định đúng **ESP32-S3 #2** trong `ls -l /dev/serial/by-id/`. Dừng `ldlidar_sl_ros2` đang giữ `/dev/ttyACM0` trước khi nạp. Nếu đúng ESP32 #2 là `/dev/ttyACM0`:

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

Cần nhìn thấy (dạng ví dụ):

```text
UART1 RX GPIO16, baud 115200, 8N1
WIFI READY. ESP32 IP=192.168.x.y TCP PORT=7777
TCP listening on port 7777
```

**Ghi lại IP thật do DHCP cấp**. Không dùng `192.168.x.y` như IP thực. Gợi ý đặt DHCP reservation trên router để IP ổn định.

## 3. Kiểm tra luồng UART thô qua Wi-Fi

Trong Terminal thứ hai (đảm bảo chưa chạy chương trình socat):

```bash
cd ~/Downloads/LD14P_WIFI_BRIDGE_V1
python3 ubuntu/check_tcp.py <IP_ESP32_2> --port 7777 --seconds 3
```

Đổi `<IP_ESP32_2>` thành IP thật. Kết quả kỳ vọng: nhận nhiều byte và thấy một số header `54 2c` của gói LD14P. Không cần `micro-ROS Agent` trên đường truyền này.

Nếu `0 bytes`: kiểm tra LiDAR có quay, nguồn 5 V, GPIO16, mạng và baudrate. Nếu `0 candidate header`: nghi vấn baudrate hoặc cáp TX, chưa chuyển sang ROS.

## 4. Tạo Serial ảo và khởi động ROS 2 (KHÔNG build lại driver)

Ngắt chương trình `check_tcp.py` trước, vì ESP32 hiện hỗ trợ **một kết nối**.

```bash
cd ~/Downloads/LD14P_WIFI_BRIDGE_V1
./ubuntu/start_ld14p_wifi.sh <IP_ESP32_2> 7777
```

Lệnh làm ba việc: kết nối TCP đến ESP32 #2, tạo link `/tmp/ld14p_wifi` trỏ đến PTY `/dev/pts/*`, rồi chạy driver ROS 2 hiện có cùng TF tĩnh, tự khởi động lại khi kết nối mạng đứt.

**Tham số TF mặc định giống launch cũ**: x=0, y=0, z=0,18 m, yaw=0. Thay bằng số đo thực tế trước khi SLAM. Ví dụ, nếu lắp cao 0,22m:

```bash
LASER_Z=0.22 ./ubuntu/start_ld14p_wifi.sh <IP_ESP32_2> 7777
```

Cổng `serial_baudrate=230400` trong launch ảo **không** yêu cầu UART đầu vào ESP32 là 230400; đó chỉ là thông số termios của PTY mà driver mở.

## 5. Kiểm tra ROS 2 Jazzy

Terminal mới:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list -t
ros2 topic info /scan --verbose
ros2 topic hz /scan
ros2 topic echo /scan --once
```

Kỳ vọng: **một** publisher `/scan` tên `/ldlidar_publisher_ld14`, dữ liệu từ `base_laser`, tần số gần 6 Hz như bản USB. RViz2: Fixed Frame `base_laser`, Add `LaserScan`, topic `/scan`.

Sau khi chạy ổn định khi còn USB monitor, **có thể tháo cáp USB** và cấp nguồn cho board #2 bằng nguồn phù hợp trên robot, để chứng minh Wi-Fi thực sự độc lập USB. Không thao tác rút/nối trên phần cứng đang cấp nguồn không an toàn.

## 6. Xử lý lỗi nhanh

- `Connection refused` / `No route to host`: sai IP/port, ESP32 chưa Wi-Fi, router bật isolation, không cùng LAN hoặc firewall. Kiểm tra IP trong Serial Monitor.
- UART byte count = 0: xem lại nguồn LD14P, baud UART1 RX = 115200, chân TX->GPIO16, GND chung.
- Có byte nhưng mất header/ROS báo `communication abnormal`: kiểm tra baud vật lý và tính nguyên vẹn byte, không đổi `230400` trên PTY để sửa lỗi UART1.
- `/scan` có 2 publisher: dừng launch USB cũ (chỉ giữ Wi-Fi).
- `Cannot open /tmp/ld14p_wifi`: khởi động `start_ld14p_wifi.sh` trước, kiểm tra link `ls -l /tmp/ld14p_wifi` và quyền.
- Sau ngắt Wi-Fi: script tự khởi động lại kết nối TCP và node ROS; có thể mất một vài vòng quét. Không dùng LiDAR không dây làm cơ chế dừng khẩn cấp duy nhất.
- `idf.py` không tìm thấy: chạy `source ~/esp/esp-idf/export.sh` từ đúng Terminal.
- Khi nạp, đừng nhầm ESP32 #2 với ESP32 #1. Không nạp project này vào ESP32 #1.

## 7. Giới hạn kiểm thử

- Đã kiểm tra cấu trúc source, Python/shell syntax và kiểm thử giả lập chuỗi TCP->PTY trên Linux.
- Chưa build/nạp trên **ESP-IDF thực tế của máy bạn**, chưa chạy với ROS 2 Jazzy và cảm biến vật lý. Nếu build báo lỗi, gửi dòng lỗi đầu tiên.
- Một TCP client, chưa mã hóa ứng dụng; ưu tiên WLAN 2,4 GHz ổn định.
- Nếu UART ring buffer quá tải, byte có thể mất; driver LD14P sẽ cần đồng bộ lại khung. So sánh tần số `/scan`, số lượng NaN/CRC và độ trễ trước-sau để xác nhận chất lượng Wi-Fi.

## 8. Quay lại bản USB cũ

Không cần sửa project `~/ld14_usb_bridge` cũ; chỉ nạp lại firmware cũ vào **ESP32 #2** khi cần:

```bash
cd ~/ld14_usb_bridge
source ~/esp/esp-idf/export.sh
idf.py -p /dev/ttyACM0 flash
```

Chỉ dùng cổng thực tế của ESP32 #2; sau đó chạy lại launch USB cũ.
