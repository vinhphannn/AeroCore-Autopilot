# Kiến trúc Quản lý Mission, Waypoint và RTL của PX4 Autopilot

Để đảm bảo mạch Flight Controller của chúng ta tiệm cận thiết kế chuyên nghiệp nhất, dưới đây là phân tích chi tiết về cách PX4 quản lý Sứ mệnh (Mission), Lưu trữ (Dataman), Tham số (Parameters) và các Phương thức cốt lõi.

---

## 1. Lưu trữ Kế hoạch bay (Plan/Mission Storage) ở đâu?

Trong PX4, các điểm Waypoint không được lưu trữ trực tiếp trong bộ nhớ RAM tạm thời của luồng điều khiển vì lý do an toàn (nếu chip bị reset giữa trời, drone phải có khả năng khôi phục lại lộ trình).

### Cơ chế Datamanager (Dataman) của PX4
PX4 thiết lập một dịch vụ ngầm tên là `dataman` (Data Manager). Nhiệm vụ của nó là ghi/đọc dữ liệu lên bộ nhớ bất biến (Non-Volatile Storage):
* **Phần cứng thật:** Lưu vào phân vùng bộ nhớ Flash MTD (`/dev/mtdparam`) hoặc lưu thành tệp nhị phân trên thẻ nhớ SD (`/fs/microsd/missions.dat`).
* **Kiểu dữ liệu lưu trữ:** Định nghĩa bởi cấu trúc `struct mission_item_s` (uORB topic `mission`).

### API đọc ghi của PX4:
Bộ điều hướng `Navigator` giao tiếp với bộ nhớ lưu trữ qua các hàm C-API sau:
```cpp
// Đọc điểm Waypoint thứ index từ bộ lưu trữ dataman
dm_read(DM_KEY_WAYPOINTS, index, &mission_item, sizeof(mission_item_s));

// Ghi điểm Waypoint mới
dm_write(DM_KEY_WAYPOINTS, index, &mission_item, sizeof(mission_item_s));

// Xóa toàn bộ Waypoints
dm_clear(DM_KEY_WAYPOINTS);
```

---

## 2. Kích hoạt Mission/RTL như thế nào? (Activation Flow)

Quá trình kích hoạt chế độ bay tự động trong PX4 diễn ra qua các bước MAVLink và uORB phối hợp:

### A. Từ QGroundControl (QGC) gửi lệnh MAVLink:
QGC gửi gói tin `MAV_CMD_DO_SET_MODE` chứa các tham số:
* **Base Mode:** `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` (Báo dùng chế độ tùy biến của PX4).
* **Main Mode:** `PX4_CUSTOM_MAIN_MODE_AUTO` (Chế độ tự động).
* **Sub Mode:** `PX4_CUSTOM_SUB_MODE_AUTO_MISSION` (Bay theo lộ trình) hoặc `PX4_CUSTOM_SUB_MODE_AUTO_RTL` (Quay về đất).

### B. Trong Commander nhận lệnh và chuyển trạng thái uORB:
1. Module `MavlinkReceiver` nhận tin nhắn, chuyển đổi thành lệnh uORB và gửi lên topic `vehicle_command`.
2. Module `Commander` đọc lệnh từ `vehicle_command`, kiểm tra các điều kiện an toàn (GPS 3D Lock, Pre-flight checks).
3. Nếu đủ điều kiện, Commander thay đổi biến `status.nav_state`:
   * Chuyển sang `NAVIGATION_STATE_AUTO_MISSION` (Mission).
   * Hoặc `NAVIGATION_STATE_AUTO_RTL` (RTL).
4. Commander publish topic `vehicle_status` chứa trạng thái bay mới.
5. Bộ Navigator và Position Controller nhận trạng thái mới từ `vehicle_status` và lập tức chuyển đổi logic xử lý.

---

## 3. Các Tham số (Parameters) cấu hình hệ thống của PX4

PX4 sử dụng hệ thống Parameter (`param_t`) để người dùng cấu hình hành vi bay thông qua QGC:

| Tên Parameter PX4 | Ý nghĩa mặc định | Giá trị mặc định | Ứng dụng trong mã nguồn |
| :--- | :--- | :--- | :--- |
| **`NAV_ACC_RAD`** | Bán kính chấp nhận điểm Waypoint (Acceptance Radius). | `2.0 m` | Nếu khoảng cách tới waypoint $< NAV\_ACC\_RAD$, chuyển sang điểm kế tiếp. |
| **`MIS_DIST_1WP`** | Khoảng cách tối đa cho phép từ Home tới Waypoint đầu tiên. | `900 m` | Tránh trường hợp nạp nhầm lộ trình ở quốc gia khác gây mất máy bay. |
| **`MIS_TAKEOFF_ALT`**| Độ cao cất cánh tự động mặc định của sứ mệnh. | `2.5 m` | Drone sẽ tự cất cánh đứng lên độ cao này trước khi bay ngang tới waypoint 1. |
| **`RTL_ALT`** | Độ cao an toàn khi kích hoạt Return-To-Launch. | `30.0 m` | Drone tự leo lên độ cao này trước khi bay ngang về để tránh cây cối, nhà cửa. |
| **`RTL_LAND_DELAY`** | Thời gian tạm dừng (Loiter Delay) trước khi hạ cánh ở điểm Home.| `0.0 s` | Drone bay về tới Home sẽ đứng treo kiểm tra an toàn $N$ giây rồi mới hạ. |

---

## 4. Tên biến, Cấu trúc dữ liệu và Phương thức cốt lõi của PX4

Dưới đây là ánh xạ (mapping) trực tiếp giữa thiết kế lớp của PX4 và mã nguồn mạch Flight Controller của chúng ta:

### A. Cấu trúc Setpoint Triplet (`position_setpoint_triplet_s`)
PX4 định nghĩa tin nhắn này tại `src/modules/uORB/topics/position_setpoint_triplet.h`:
* `triplet.previous`: Điểm waypoint drone vừa đi qua (dùng để vẽ đường thẳng nối trục quỹ đạo).
* `triplet.current`: Điểm waypoint hiện tại drone cần bay tới.
* `triplet.next`: Điểm tiếp theo để tính toán trước góc rẽ (cua mượt).

### B. Các Phương thức trong lớp `Mission` của Navigator trong PX4
Lớp điều khiển Mission của PX4 (`src/modules/navigator/mission.cpp`) kế thừa từ `NavigatorMode` chứa các phương thức quan trọng:
* **`on_active()`**: Được gọi liên tục khi chế độ Mission được bật. Nó thực hiện cập nhật tiến trình bay.
* **`on_inactive()`**: Được gọi khi thoát chế độ bay tự động (ví dụ phi công gạt cần cướp quyền điều khiển MANUAL).
* **`is_mission_item_reached()`**: Hàm logic kiểm tra điều kiện chạm waypoint:
  ```cpp
  // Logic PX4: Khoảng cách ngang XY nhỏ hơn NAV_ACC_RAD và chiều dọc Z nhỏ hơn dung sai
  bool dist_xy_reached = (dist_xy < _navigator->get_acceptance_radius());
  bool dist_z_reached = (fabsf(dist_z) < _navigator->get_altitude_acceptance_radius());
  ```
* **`setActiveMissionItems()`**: Đọc dữ liệu từ `dataman` và đóng gói nạp vào `position_setpoint_triplet` gửi cho bộ điều khiển vị trí.

---

## 5. Bảng so sánh kiến trúc giữa PX4 và Firmware MyFC của chúng ta

| Thành phần | Kiến trúc của PX4 | Firmware MyFC hiện tại |
| :--- | :--- | :--- |
| **Lưu trữ Waypoints** | File nhị phân lưu ổ đĩa/Flash thông qua `dataman`. | Mảng tĩnh `_waypoints` nằm trong bộ nhớ RAM của lớp `Navigator`. |
| **Độ mượt quỹ đạo** | Sử dụng bộ sinh quỹ đạo S-Curve chống gia tốc giật (`Jerk-limited`). | Sử dụng sai số vị trí nhân với hệ số P (`_gain_pos_p`) để tạo setpoint vận tốc. |
| **Chuyển điểm** | Dựa trên khoảng cách 3D thực tế và góc hướng đầu (Yaw alignment). | Dựa trên khoảng cách Euclidean 3D $< 1.0\text{ m}$. |
| **Kiểm tra an toàn** | Có hàng chục điều kiện kiểm tra (Geofence, GPS, RC connection). | Kiểm tra trạng thái uORB của local position và trạng thái armed. |
