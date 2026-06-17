import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# Định nghĩa các Trạng thái bay (Navigation States) tương thích với Firmware
NAV_STATE_MANUAL = 0
NAV_STATE_ALTCTL = 1
NAV_STATE_POSCTL = 2
NAV_STATE_AUTO_MISSION = 3
NAV_STATE_AUTO_LAND = 6
NAV_STATE_AUTO_TAKEOFF = 7
NAV_STATE_AUTO_RTL = 8

class Waypoint:
    def __init__(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.valid = True

class NavigatorSim:
    def __init__(self):
        self._waypoints = [
            Waypoint(3.0, 0.0, -2.0, 0.0),       # WP1: Tiến Bắc 3m, cao 2m
            Waypoint(3.0, 3.0, -2.0, np.pi/2),   # WP2: Sang Đông 3m, Yaw 90 độ
            Waypoint(0.0, 3.0, -2.0, np.pi),     # WP3: Lùi Nam 3m, Yaw 180 độ
            Waypoint(0.0, 0.0, -2.0, 0.0)        # WP4: Trở về vĩ độ xuất phát X=0, Y=0
        ]
        self._waypoint_count = len(self._waypoints)
        self._current_waypoint_index = 0
        
        # Các tham số cấu hình (Parameters) giống hệt Firmware
        self._acceptance_radius = 1.0    # NAV_ACC_RAD
        self._rtl_altitude = -3.0        # RTL_ALT (được lưu ở dạng NED âm: cao 3m)
        
        self._home_pos = np.array([0.0, 0.0, 0.0])
        self._home_valid = False
        
        # RTL States
        self.RTL_STATE_NONE = 0
        self.RTL_STATE_CLIMB = 1
        self.RTL_STATE_RETURN = 2
        self.RTL_STATE_LAND = 3
        self._rtl_state = self.RTL_STATE_NONE
        
        self.current_sp = Waypoint(0, 0, 0, 0)
        self.current_sp.valid = False

    def update(self, nav_state, armed, pos, yaw):
        if not armed:
            self._current_waypoint_index = 0
            self._rtl_state = self.RTL_STATE_NONE
            self.current_sp.valid = False
            return nav_state

        # Ghi nhận vị trí Home khi Armed
        if armed and not self._home_valid:
            self._home_pos = pos.copy()
            self._home_valid = True

        if nav_state == NAV_STATE_AUTO_MISSION:
            if self._current_waypoint_index >= self._waypoint_count:
                # Hoàn thành sứ mệnh -> Tự động chuyển sang RTL
                nav_state = NAV_STATE_AUTO_RTL
                self._rtl_state = self.RTL_STATE_NONE
                print(f"[Navigator Sim] Mission Completed! Switching to AUTO_RTL...")
            else:
                target_wp = self._waypoints[self._current_waypoint_index]
                self.current_sp.x = target_wp.x
                self.current_sp.y = target_wp.y
                self.current_sp.z = target_wp.z
                self.current_sp.yaw = target_wp.yaw
                self.current_sp.valid = True

                # Tính khoảng cách 3D đến điểm Waypoint mục tiêu
                dist = np.linalg.norm(pos - np.array([target_wp.x, target_wp.y, target_wp.z]))
                if dist < self._acceptance_radius:
                    print(f"[Navigator Sim] Reached Waypoint {self._current_waypoint_index + 1}/{self._waypoint_count}!")
                    self._current_waypoint_index += 1

        elif nav_state == NAV_STATE_AUTO_RTL:
            home_x, home_y, home_z = self._home_pos
            
            if self._rtl_state == self.RTL_STATE_NONE:
                self._rtl_state = self.RTL_STATE_CLIMB
                print(f"[Navigator Sim] RTL Stage 1: Climbing to safe altitude (Z = {self._rtl_altitude}m)")
                
            if self._rtl_state == self.RTL_STATE_CLIMB:
                # Giữ nguyên X-Y, chỉ bay lên độ cao an toàn RTL
                target_z = pos[2] if pos[2] < self._rtl_altitude else self._rtl_altitude
                self.current_sp.x = pos[0]
                self.current_sp.y = pos[1]
                self.current_sp.z = target_z
                self.current_sp.yaw = yaw
                self.current_sp.valid = True

                # Đã đạt độ cao an toàn
                if abs(pos[2] - target_z) < 0.2:
                    self._rtl_state = self.RTL_STATE_RETURN
                    print(f"[Navigator Sim] RTL Stage 2: Altitude reached. Returning to Home...")

            elif self._rtl_state == self.RTL_STATE_RETURN:
                # Bay về Home ở độ cao an toàn RTL
                self.current_sp.x = home_x
                self.current_sp.y = home_y
                self.current_sp.z = self._rtl_altitude
                self.current_sp.yaw = 0.0
                self.current_sp.valid = True

                dist_2d = np.linalg.norm(pos[:2] - np.array([home_x, home_y]))
                if dist_2d < 0.8:
                    self._rtl_state = self.RTL_STATE_LAND
                    print(f"[Navigator Sim] RTL Stage 3: Arrived home. Initiating Auto Land...")

            elif self._rtl_state == self.RTL_STATE_LAND:
                # Bàn giao hoàn toàn cho chế độ AUTO_LAND xử lý
                nav_state = NAV_STATE_AUTO_LAND
                self._rtl_state = self.RTL_STATE_NONE
                self.current_sp.valid = False

        else:
            self.current_sp.valid = False

        return nav_state


class PositionController:
    def __init__(self):
        self._gain_pos_p = np.array([1.2, 1.2, 1.2])
        self._gain_vel_p = np.array([0.15, 0.15, 0.20])
        self._gain_vel_i = np.array([0.02, 0.02, 0.03])
        self._gain_vel_d = np.array([0.02, 0.02, 0.05])
        
        self._lim_vel_horizontal = 3.0
        self._lim_vel_up = 1.5
        self._lim_vel_down = 1.5
        self._lim_thr_min = 0.12
        self._lim_thr_max = 0.9
        self._lim_tilt = 0.35 # 20 degrees
        self._hover_thrust = 0.3 # 30% ga hover
        
        self._pos_sp = np.array([np.nan, np.nan, np.nan])
        self._vel_sp = np.array([np.nan, np.nan, np.nan])
        self._acc_sp = np.array([np.nan, np.nan, np.nan])
        self._thr_sp = np.zeros(3)
        self._vel_int = np.zeros(3)
        
        self._hold_pos_xy = False
        self._hold_pos_z = False
        
    def update(self, dt, nav_state, pos, vel, vel_dot, dist_sensor, nav_sp):
        # Reset setpoints
        if not self._hold_pos_xy:
            self._pos_sp[0] = np.nan
            self._pos_sp[1] = np.nan
        if not self._hold_pos_z:
            self._pos_sp[2] = np.nan
        self._vel_sp = np.array([np.nan, np.nan, np.nan])
        self._acc_sp = np.array([np.nan, np.nan, np.nan])
        
        # Đồng bộ setpoint từ Navigator
        if (nav_state in [NAV_STATE_AUTO_MISSION, NAV_STATE_AUTO_RTL]) and nav_sp.valid:
            self._pos_sp[0] = nav_sp.x
            self._pos_sp[1] = nav_sp.y
            self._pos_sp[2] = nav_sp.z
            self._hold_pos_xy = False
            self._hold_pos_z = False

        # 1. TRỤC DỌC Z
        if nav_state == NAV_STATE_AUTO_LAND:
            land_speed = 0.5
            if dist_sensor < 1.0:
                land_speed = 0.2 # Crawl mode hạ cánh nhẹ nhàng
            self._vel_sp[2] = land_speed
            self._hold_pos_z = False
        elif nav_state == NAV_STATE_AUTO_TAKEOFF:
            self._vel_sp[2] = -0.8 # Bay lên 0.8 m/s
            self._hold_pos_z = False
        elif nav_state in [NAV_STATE_AUTO_MISSION, NAV_STATE_AUTO_RTL]:
            # Do Z đã được set bằng vị trí từ triplet phía trên nên không ghi đè ở đây
            self._hold_pos_z = False
        else:
            if not self._hold_pos_z:
                self._pos_sp[2] = pos[2]
                self._hold_pos_z = True
                
        # 2. TRỤC NGANG X-Y
        control_xy = (nav_state in [NAV_STATE_POSCTL, NAV_STATE_AUTO_LAND, NAV_STATE_AUTO_TAKEOFF, NAV_STATE_AUTO_MISSION, NAV_STATE_AUTO_RTL])
        if control_xy:
            if not self._hold_pos_xy and np.isnan(self._pos_sp[0]):
                self._pos_sp[0] = pos[0]
                self._pos_sp[1] = pos[1]
                self._hold_pos_xy = True
        else:
            self._hold_pos_xy = False
            self._vel_int[0] = 0.0
            self._vel_int[1] = 0.0

        # 3. VÒNG ĐIỀU KHIỂN VỊ TRÍ P
        vel_sp_position = np.zeros(3)
        for i in range(3):
            if not np.isnan(self._pos_sp[i]):
                vel_sp_position[i] = (self._pos_sp[i] - pos[i]) * self._gain_pos_p[i]
                if np.isnan(self._vel_sp[i]):
                    self._vel_sp[i] = vel_sp_position[i]
                else:
                    self._vel_sp[i] += vel_sp_position[i]
                    
        # Giới hạn vận tốc ngang
        vel_sp_xy_norm = np.linalg.norm(self._vel_sp[:2])
        if vel_sp_xy_norm > self._lim_vel_horizontal:
            self._vel_sp[:2] = (self._vel_sp[:2] / vel_sp_xy_norm) * self._lim_vel_horizontal
            
        # Giới hạn vận tốc dọc
        self._vel_sp[2] = np.clip(self._vel_sp[2], -self._lim_vel_up, self._lim_vel_down)
        
        # 4. VÒNG ĐIỀU KHIỂN VẬN TỐC PID
        self._vel_int[2] = np.clip(self._vel_int[2], -9.80665, 9.80665)
        
        vel_error = self._vel_sp - vel
        acc_sp_velocity = vel_error * self._gain_vel_p + self._vel_int - vel_dot * self._gain_vel_d
        
        for i in range(3):
            if not np.isnan(self._acc_sp[i]):
                self._acc_sp[i] += acc_sp_velocity[i]
            else:
                self._acc_sp[i] = acc_sp_velocity[i]
                
        # 5. PHÂN BỔ LỰC ĐẨY & GIỚI HẠN GÓC TILT
        g = 9.80665
        z_specific_force = -g + self._acc_sp[2]
        
        body_z = np.array([-self._acc_sp[0], -self._acc_sp[1], -z_specific_force])
        body_z_norm = np.linalg.norm(body_z)
        if body_z_norm > 1e-6:
            body_z = body_z / body_z_norm
            
        # Giới hạn góc nghiêng tối đa
        tilt = np.arccos(np.clip(body_z[2], 0.0, 1.0))
        if tilt > self._lim_tilt:
            xy_norm = np.linalg.norm(body_z[:2])
            if xy_norm > 1e-6:
                body_z[:2] = (body_z[:2] / xy_norm) * np.sin(self._lim_tilt)
            body_z[2] = np.cos(self._lim_tilt)
            
        thrust_ned_z = self._acc_sp[2] * (self._hover_thrust / g) - self._hover_thrust
        cos_ned_body = body_z[2]
        
        collective_thrust = np.minimum(thrust_ned_z / cos_ned_body, -self._lim_thr_min)
        collective_thrust = np.maximum(collective_thrust, -self._lim_thr_max)
        
        self._thr_sp = body_z * collective_thrust
        
        # Use tracking Anti-Windup for horizontal direction
        acc_sp_xy_produced = self._thr_sp[:2] * (g / self._hover_thrust)
        acc_sp_xy = self._acc_sp[:2]
        
        if np.sum(acc_sp_xy**2) > np.sum(acc_sp_xy_produced**2):
            arw_gain = 2.0 / self._gain_vel_p[0]
            vel_err_xy = vel_error[:2] - (acc_sp_xy - acc_sp_xy_produced) * arw_gain
            vel_error[0] = vel_err_xy[0]
            vel_error[1] = vel_err_xy[1]
            
        # Anti-windup tích phân vận tốc
        if (self._thr_sp[2] >= -self._lim_thr_min and vel_error[2] >= 0.0) or \
           (self._thr_sp[2] <= -self._lim_thr_max and vel_error[2] <= 0.0):
            vel_error[2] = 0.0
            
        self._vel_int += vel_error * self._gain_vel_i * dt
        return self._thr_sp


def run_simulation():
    dt = 0.01 # 100Hz
    t_max = 45.0
    steps = int(t_max / dt)
    
    # Khởi động mô hình bay vật lý
    pos = np.array([0.0, 0.0, 0.0]) # NED frame
    vel = np.array([0.0, 0.0, 0.0])
    acc = np.array([0.0, 0.0, 0.0])
    yaw = 0.0
    
    motor_thrust = np.array([0.0, 0.0, 0.0])
    motor_tau = 0.08 # Phản hồi động cơ trễ 80ms
    
    controller = PositionController()
    navigator = NavigatorSim()
    
    armed = False
    nav_state = NAV_STATE_MANUAL
    takeoff_complete = False
    land_detect_counter = 0
    
    # Mảng lưu kết quả vẽ biểu đồ
    t_arr, pos_x_arr, pos_y_arr, pos_z_arr = [], [], [], []
    vel_x_arr, vel_y_arr, vel_z_arr = [], [], []
    nav_state_arr, arm_state_arr = [], []
    sp_x_arr, sp_y_arr, sp_z_arr = [], [], []

    g = 9.80665
    hover_thrust = 0.3
    
    for step in range(steps):
        t = step * dt
        dist_sensor = -pos[2]
        if dist_sensor < 0.05:
            dist_sensor = 0.0
            
        # Kế hoạch điều phối hành vi bay trong Sim
        if t < 1.0:
            # Chờ trên mặt đất
            armed = False
            nav_state = NAV_STATE_MANUAL
        elif t >= 1.0 and t < 4.0:
            # Arm máy bay và kích hoạt Takeoff tự động
            armed = True
            if not takeoff_complete:
                nav_state = NAV_STATE_AUTO_TAKEOFF
                # Cất cánh lên độ cao an toàn 1.5m
                if pos[2] <= -1.5:
                    takeoff_complete = True
                    nav_state = NAV_STATE_POSCTL # Hoàn thành, giữ vị trí tạm thời
        elif t >= 4.0:
            # Kích hoạt Mission bay hình vuông 3x3m quanh điểm xuất phát
            if nav_state not in [NAV_STATE_AUTO_MISSION, NAV_STATE_AUTO_RTL, NAV_STATE_AUTO_LAND]:
                nav_state = NAV_STATE_AUTO_MISSION
                
        # Cập nhật Navigator
        nav_state = navigator.update(nav_state, armed, pos, yaw)
        
        # Chạy bộ điều khiển PID vị trí & vận tốc đa trục
        if armed:
            thr_sp = controller.update(dt, nav_state, pos, vel, acc, dist_sensor, navigator.current_sp)
        else:
            thr_sp = np.zeros(3)
            controller._vel_int.fill(0.0)
            controller._hold_pos_xy = False
            controller._hold_pos_z = False
            
        # Mô phỏng phản hồi cơ cấu chấp hành
        motor_thrust += (thr_sp - motor_thrust) / motor_tau * dt
        
        # Động lực học chuyển động tịnh tiến
        acc_thrust = (motor_thrust / hover_thrust) * g
        acc_gravity = np.array([0.0, 0.0, g])
        acc_drag = -0.15 * vel # Hệ số cản gió
        
        acc = acc_gravity + acc_thrust + acc_drag
        
        # Mặt đất cứng ngăn đâm xuyên
        if pos[2] >= 0.0 and acc[2] >= 0.0:
            pos[2] = 0.0
            vel[2] = 0.0
            acc[2] = 0.0
            
        # Tích phân Euler cập nhật Trạng thái bay thực tế
        vel += acc * dt
        pos += vel * dt
        
        if pos[2] > 0.0:
            pos[2] = 0.0
            vel[2] = 0.0

        # Cập nhật hướng xoay Yaw mô phỏng bám mục tiêu
        if navigator.current_sp.valid:
            target_yaw = navigator.current_sp.yaw
            yaw += (target_yaw - yaw) * 2.0 * dt # Xoay mượt bám yaw
            
        # Touchdown Detection khi hạ cánh
        if armed and nav_state == NAV_STATE_AUTO_LAND:
            thrust_z = -thr_sp[2]
            is_on_ground = False
            if abs(vel[2]) < 0.20:
                if thrust_z < 0.18: # Lực đẩy giảm hẳn
                    is_on_ground = True
                if dist_sensor > 0.0 and dist_sensor < 0.10:
                    is_on_ground = True
                    
            if is_on_ground:
                land_detect_counter += 1
                if land_detect_counter >= 80: # 0.8 giây chạm đất ổn định -> Tắt động cơ
                    armed = False
                    nav_state = NAV_STATE_MANUAL
                    land_detect_counter = 0
            else:
                land_detect_counter = 0
                
        # Lưu trữ các dữ liệu theo dòng thời gian
        t_arr.append(t)
        pos_x_arr.append(pos[0])
        pos_y_arr.append(pos[1])
        pos_z_arr.append(-pos[2]) # Chuyển thành độ cao dương
        vel_x_arr.append(vel[0])
        vel_y_arr.append(vel[1])
        vel_z_arr.append(-vel[2])
        nav_state_arr.append(nav_state)
        arm_state_arr.append(2 if armed else 1)
        
        if navigator.current_sp.valid:
            sp_x_arr.append(navigator.current_sp.x)
            sp_y_arr.append(navigator.current_sp.y)
            sp_z_arr.append(-navigator.current_sp.z)
        else:
            sp_x_arr.append(np.nan)
            sp_y_arr.append(np.nan)
            sp_z_arr.append(np.nan)
            
    return (t_arr, pos_x_arr, pos_y_arr, pos_z_arr, 
            vel_x_arr, vel_y_arr, vel_z_arr, 
            nav_state_arr, arm_state_arr, 
            sp_x_arr, sp_y_arr, sp_z_arr, navigator)


if __name__ == '__main__':
    (t, x, y, z, vx, vy, vz, nav, arm, sp_x, sp_y, sp_z, nav_obj) = run_simulation()
    
    # ---------------- DỰNG HÌNH ĐỒ THỊ 3D ----------------
    fig = plt.figure(figsize=(14, 10))
    ax3d = fig.add_subplot(2, 2, 1, projection='3d')
    
    # Vẽ đường bay thực tế
    ax3d.plot(y, x, z, 'b-', label='Quỹ đạo bay thực tế', linewidth=2)
    # Vẽ các setpoints mục tiêu
    ax3d.plot(sp_y, sp_x, sp_z, 'r--', label='Đường dẫn Setpoint', alpha=0.7)
    
    # Đánh dấu các Waypoints bằng các điểm tròn đỏ
    wps_x = [wp.x for wp in nav_obj._waypoints]
    wps_y = [wp.y for wp in nav_obj._waypoints]
    wps_z = [-wp.z for wp in nav_obj._waypoints]
    ax3d.scatter(wps_y, wps_x, wps_z, color='red', s=50, marker='o', label='Waypoints Sứ mệnh')
    
    # Đánh dấu điểm Home (0,0,0) màu xanh lá
    ax3d.scatter([0.0], [0.0], [0.0], color='green', s=100, marker='^', label='Home (Điểm cất/hạ cánh)')
    
    ax3d.set_xlabel('Trục Đông Y (m)')
    ax3d.set_ylabel('Trục Bắc X (m)')
    ax3d.set_zlabel('Độ cao Z (m)')
    ax3d.set_title('Mô phỏng Quỹ đạo bay 3D tự động', fontsize=12, fontweight='bold')
    ax3d.legend()
    ax3d.grid(True)
    
    # ---------------- BIỂU ĐỒ XY MẶT BẰNG ----------------
    ax_xy = fig.add_subplot(2, 2, 2)
    ax_xy.plot(y, x, 'b-', label='Flight Path', linewidth=2)
    ax_xy.scatter(wps_y, wps_x, color='red', label='Waypoints')
    ax_xy.scatter(0, 0, color='green', marker='^', s=80, label='Home')
    ax_xy.set_xlabel('East position Y (meters)')
    ax_xy.set_ylabel('North position X (meters)')
    ax_xy.set_title('Quỹ đạo mặt bằng ngang 2D (X-Y)', fontsize=12, fontweight='bold')
    ax_xy.legend()
    ax_xy.grid(True)
    ax_xy.set_aspect('equal')
    
    # ---------------- BIỂU ĐỒ ĐỘ CAO Z THEO THỜI GIAN ----------------
    ax_z = fig.add_subplot(2, 2, 3)
    ax_z.plot(t, z, 'b-', label='Altitude Z (m)', linewidth=2)
    ax_z.plot(t, sp_z, 'r--', label='Setpoint Z (m)')
    ax_z.set_xlabel('Time (seconds)')
    ax_z.set_ylabel('Height (m)')
    ax_z.set_title('Độ cao và Setpoint theo thời gian', fontsize=12, fontweight='bold')
    ax_z.legend()
    ax_z.grid(True)
    
    # ---------------- BIỂU ĐỒ TRẠNG THÁI BAY (STATES) ----------------
    ax_st = fig.add_subplot(2, 2, 4)
    ax_st.plot(t, nav, 'k-', label='Navigation State', linewidth=2)
    ax_st.plot(t, arm, 'g-.', label='Arming State (1:STANDBY, 2:ARMED)')
    ax_st.set_xlabel('Time (seconds)')
    ax_st.set_ylabel('State ID')
    ax_st.set_title('Trạng thái của Flight Controller (States)', fontsize=12, fontweight='bold')
    ax_st.legend()
    ax_st.grid(True)
    
    plt.tight_layout()
    
    # Lưu hình ảnh kết quả mô phỏng
    output_dir = '/home/zynh/.gemini/antigravity/brain/fa79e275-6879-431c-afce-2c69c8bd5e6a/artifacts'
    os.makedirs(output_dir, exist_ok=True)
    plt.savefig(os.path.join(output_dir, 'takeoff_landing_sim.png'), dpi=300)
    print("Simulation completed successfully. Plot saved.")
