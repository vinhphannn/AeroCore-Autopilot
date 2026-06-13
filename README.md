# AeroCore Autopilot 🦅

**AeroCore** là một hệ thống điều khiển bay (Flight Controller) được thiết kế từ con số không (from scratch), hướng đến sự tối giản nhưng giữ lại cấu trúc công nghiệp mạnh mẽ nhất.

Dự án này là sự giao thoa hoàn hảo giữa hai thế giới:
1. **Sức mạnh từ cội nguồn (Bare-metal & RTOS):** Khai thác triệt để 100% sức mạnh của phần cứng Cortex-M7 (STM32H7) với DMA, D-Cache, kết hợp hệ điều hành thời gian thực FreeRTOS. Đảm bảo độ trễ (latency) điều khiển gần như bằng không.
2. **Kiến trúc Công nghiệp (Industrial Architecture):** AeroCore vay mượn kiến trúc thiết kế module đỉnh cao của PX4. Thay vì viết mã nguyên khối (monolithic), AeroCore chia tách rõ ràng các thành phần: Estimator, Commander, Controller, và Mixer. Hệ thống giao tiếp nội bộ qua `uORB` và tương thích 100% với trạm điều khiển mặt đất **QGroundControl** thông qua chuẩn **MAVLink**.

Ngắn gọn lại: **AeroCore là phần "hồn" của PX4, được nhúng vào thân xác tốc độ cao và tối giản của STM32H7.**

---

## 🛠 Tính năng nổi bật
- **Hệ thống tin nhắn uORB-lite:** Truyền nhận dữ liệu đa luồng an toàn tuyệt đối.
- **DShot DMA (Cortex-M7 Optimized):** Giao tiếp ESC tốc độ siêu cao bằng DMA, tự động xử lý tính nhất quán D-Cache.
- **Flight Mode Manager:** Quản lý vòng đời bay chuẩn xác.
- **MAVLink Telemetry:** Giao tiếp 2 chiều thời gian thực (Attitude, RC, Heartbeat) với QGroundControl.
- **Tích hợp CI/CD Github Actions:** Tự động build và sinh release khi có push mới.

## 🚀 Hướng dẫn Build (Dành cho nhà phát triển)
Hệ thống sử dụng Toolchain `arm-none-eabi-gcc` và `make`.

```bash
cd firmware/testspih743/Debug
make clean
make -j$(nproc) all
```
File `MyFC_H743.elf` và `.bin` sẽ được sinh ra ở cùng thư mục. Mạch nạp hỗ trợ: ST-Link / STM32CubeProgrammer.
