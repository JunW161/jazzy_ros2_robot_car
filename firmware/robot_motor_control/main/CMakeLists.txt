idf_component_register(
    SRCS "main.cpp" "robot_drive.cpp" "robot_sensors.cpp" "sensor_bus.cpp" "mpu6500_device.cpp" "vl6180x_device.cpp"
    INCLUDE_DIRS "."
    REQUIRES micro_ros_espidf_component esp_driver_gpio esp_driver_ledc esp_driver_i2c esp_timer nvs_flash
)
