from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
src = (root/'esp32_idf/main/main.c').read_text()
kconfig = (root/'esp32_idf/main/Kconfig.projbuild').read_text()
launch = (root/'ubuntu/ld14p_wifi.launch.py').read_text()
assert '#define LIDAR_GPIO_RX      16' in src
assert '#define LIDAR_PORT_UART     UART_NUM_1' in src
assert 'send_all(fd, chunk' in src
assert 'uart_read_bytes(LIDAR_PORT_UART' in src
assert 'ESP_ERROR_CHECK(uart_flush_input' in src
assert 'rmw_' not in src and 'rclc_' not in src
assert 'default 115200' in kconfig
assert 'default 7777' in kconfig
assert "'LDLiDAR_LD14P'" in launch
assert "'port_name': serial_port" in launch
assert "'laser_scan_topic_name': 'scan'" in launch
assert 'static_transform_publisher' in launch
print('PASS: GPIO16, UART1, Kconfig, TCP, ROS launch, TF wiring/static checks')
