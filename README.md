HỆ THỐNG HỖ TRỢ CHĂM SÓC SỨC KHỎE 

Linh kiện sử dụng:
- Module: Loadcell + HX711, BME280, MAX30102, SSD1306
- Vi điều khiển: STM32F103C8T6, ESP32C3 mini
- ESP32: ESP-IDF (framework), Visual Studio Code (IDE)
- STM32: STM32CubeIDE (IDE), STM32 HAL (library)
Mô tả hệ thống:
Stm32:
- RTOS
- Thu thập và tính toán các giá trị từ BME280 và MAX30102 (I2C), sử dụng mutex để bảo vệ I2C tránh xung đột
- Nút nhấn điều khiển nâng hạ (1 mức), sử dụng PWM, chỉ mới thử nghiệm trên led (có thể triển khai sau trên mạch cầu H), sử dụng ngắt ngoài gửi semaphore để bật và               tắt led, timer đếm ngược thời gian tối đa motor(led) được phép hoạt động, vướt quá thời gian sẽ tắt led 
- Các dữ liệu thu thập được và phản hồi trạng thái từ phần điều khiển sẽ được lưu vào queue
- Truyền và nhận dữ liệu: UART non Blocking, sử dụng mutex để bảo vệ UART và sử dụng semaphore làm tín hiệu kết thúc quá trình truyền
ESP32
- RTOS
- Nút nhấn báo động sử dụng ngắt ngoài báo semaphore
- Lưu giá trị khởi tạo ban đầu của Loadcell vào Flash, tính toán tham số trung gian để ra được cân nặng thực tế
- Dùng mutex bảo vệ tài nguyên dùng chung (data, I2C, flash)
- Queue lưu dữ liệu từ uart và HX711
