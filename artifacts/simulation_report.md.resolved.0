# Báo cáo mô phỏng thuật toán Flight Controller (Option C)

Chúng ta đã thiết lập một hệ thống mô phỏng động học Quadcopter 6-DoF hoàn chỉnh bằng Python để kiểm thử chính xác các cấu trúc điều khiển đã viết trong Firmware. Bản mô phỏng này áp dụng đúng các tham số vật lý thực tế:
- **Tần số vòng lặp:** $100\text{ Hz}$ (Chu kỳ $10\text{ ms}$).
- **Ga hover của mạch:** $30\%$ (`_hover_thrust = 0.3`).
- **Trễ động cơ (Motor lag):** $\tau = 80\text{ ms}$.
- **Hệ số cản không khí:** $c_d = 0.1$.
- **Giới hạn góc nghiêng (Tilt limit):** $20^\circ$.

Dưới đây là biểu đồ kết quả mô phỏng quá trình **Cất cánh tự động -> Giữ vị trí (Hover) -> Hạ cánh tự động (Land Crawl) -> Phát hiện chạm đất & Tắt máy (Disarm)**:

![Kết quả mô phỏng cất cánh và hạ cánh tự động](/home/zynh/.gemini/antigravity/brain/fa79e275-6879-431c-afce-2c69c8bd5e6a/artifacts/takeoff_landing_sim.png)

---

### Phân tích biểu đồ và các giai đoạn bay

#### Giai đoạn 1: Chuẩn bị trên mặt đất ($0\text{ s} \to 1.0\text{ s}$)
- Máy bay ở trạng thái `STANDBY` (chưa armed), ga động cơ bằng $0$.
- Vị trí trục Z bằng $0.0\text{ m}$.

#### Giai đoạn 2: Cất cánh tự động (Auto Takeoff) ($1.0\text{ s} \to 3.2\text{ s}$)
- Công tắc Arm được gạt, tay ga đẩy trên $60\%$ kích hoạt trạng thái `AUTO_TAKEOFF` (Navigation State = 7).
- Vận tốc đi lên (`Climb Velocity`) được giới hạn mượt mà ở mức $0.8\text{ m/s}$ (đường nét đứt màu đỏ hướng lên dương).
- Lực đẩy động cơ tăng lên nhanh chóng vượt qua ga hover ($30\%$) để nhấc máy bay lên, sau đó tự động giảm dần về mức hover khi drone bắt đầu tiệm cận độ cao đích.

#### Giai đoạn 3: Bay treo giữ vị trí (POSCTL Hover) ($3.2\text{ s} \to 8.0\text{ s}$)
- Khi độ cao chạm ngưỡng $1.5\text{ m}$, trạng thái bay chuyển sang `POSCTL` (Navigation State = 2).
- Thuật toán PID khóa vị trí độ cao hoạt động cực kỳ ổn định:
  - Độ cao máy bay giữ ổn định chính xác tại $1.5\text{ m}$ (không vọt lố nhờ có khâu D-term chống rung động).
  - Lực đẩy động cơ thực tế (`Actual Motor Thrust`) duy trì ổn định ngay tại mức ga hover $30\%$ (đường nét đứt màu đen).

#### Giai đoạn 4: Hạ cánh tự động (Auto Land với Land Crawl) ($8.0\text{ s} \to 15.6\text{ s}$)
- Lệnh hạ cánh `AUTO_LAND` (Navigation State = 6) được kích hoạt ở giây thứ 8.
- **Tốc độ hạ cánh thông thường:** Máy bay hạ độ cao đều đặn với vận tốc đi xuống $0.5\text{ m/s}$.
- **Giai đoạn tiếp đất nhẹ nhàng (Land Crawl):** Khi độ cao so với đất dưới $1.0\text{ m}$, hệ thống tự động phát hiện bằng ToF cảm biến khoảng cách và hạ tốc độ xuống chỉ còn $0.2\text{ m/s}$. Drone tiếp đất vô cùng êm ái mà không bị nảy lên.

#### Giai đoạn 5: Phát hiện chạm đất & Tự động Disarm ($15.6\text{ s}$)
- Khi drone chạm đất hoàn toàn ở giây thứ $15.6$:
  - Vận tốc dọc $v_z$ triệt tiêu hoàn toàn về $0.0\text{ m/s}$ ($< 0.25\text{ m/s}$).
  - Bộ tích phân của PID lực đẩy giảm ga về mức tối thiểu $12\%$ (`_lim_thr_min`).
- Sau khi thỏa mãn đồng thời hai điều kiện này trong vòng 1 giây liên tục, hệ thống **Touchdown** kích hoạt:
  - Máy bay chuyển sang `STANDBY` (Disarmed).
  - Lực đẩy động cơ ngắt hoàn toàn về $0\%$.
  - Drone hạ cánh thành công, tắt máy an toàn tuyệt đối.
