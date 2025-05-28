import numpy as np

class DronePhysics:
    def __init__(self):
        self.mass = 1.5  # кг
        self.gravity = 9.81 # м/с^2
        self.drag_coefficient = 0.1 # Пока не используется активно

        self.position = np.array([0.0, 0.0, 5.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.attitude = np.array([0.0, 0.0, 0.0]) # roll, pitch, yaw (радианы)
        self.angular_velocity = np.array([0.0, 0.0, 0.0]) # wx, wy, wz (рад/с) в системе координат дрона

        self.num_motors = 4
        self.max_motor_thrust = 5.0  # Ньютоны на один мотор
        self.arm_length = 0.225 # м
        self.propeller_reaction_torque_coefficient = 0.05 

        self.Ixx = 0.03; self.Iyy = 0.03; self.Izz = 0.06
        self.inertia_matrix = np.diag([self.Ixx, self.Iyy, self.Izz])
        self.inv_inertia_matrix = np.linalg.inv(self.inertia_matrix)

    def _euler_to_rotation_matrix(self, roll, pitch, yaw):
        # Рассчитывает матрицу поворота из углов Эйлера (ZYX конвенция)
        # Поворот сначала по Yaw (Z), затем по Pitch (Y'), затем по Roll (X'')
        cos_r, sin_r = np.cos(roll), np.sin(roll)
        cos_p, sin_p = np.cos(pitch), np.sin(pitch)
        cos_y, sin_y = np.cos(yaw), np.sin(yaw)

        R_x = np.array([[1, 0, 0], [0, cos_r, -sin_r], [0, sin_r, cos_r]])
        R_y = np.array([[cos_p, 0, sin_p], [0, 1, 0], [-sin_p, 0, cos_p]])
        R_z = np.array([[cos_y, -sin_y, 0], [sin_y, cos_y, 0], [0, 0, 1]])
        
        R = R_z @ R_y @ R_x 
        return R

    def calculate_forces_and_moments(self, motor_speeds):
        # motor_speeds: массив/список из 4 значений (0 до 1), тяга каждого мотора.
        # Предполагаемая нумерация моторов для X-квадрокоптера:
        # 0: Передний-Правый (FR)
        # 1: Передний-Левый (FL)
        # 2: Задний-Левый (RL)
        # 3: Задний-Правый (RR)

        t = np.array(motor_speeds) * self.max_motor_thrust
        t0, t1, t2, t3 = t[0], t[1], t[2], t[3]

        total_thrust_local = np.array([0.0, 0.0, np.sum(t)])
        R_body_to_world = self._euler_to_rotation_matrix(self.attitude[0], self.attitude[1], self.attitude[2])
        thrust_global = R_body_to_world @ total_thrust_local
        
        gravity_force_global = np.array([0.0, 0.0, -self.gravity * self.mass])
        net_force_global = thrust_global + gravity_force_global
        
        tau_x = self.arm_length * (t1 + t2 - t0 - t3)
        tau_y = self.arm_length * (t0 + t1 - t2 - t3)
        tau_z = self.propeller_reaction_torque_coefficient * (t0 - t1 + t2 - t3)
        net_moment_body = np.array([tau_x, tau_y, tau_z])

        return net_force_global, net_moment_body

    def update_physics(self, dt, motor_speeds):
        # 1. Расчет суммарных сил и моментов
        net_force_global, net_moment_body = self.calculate_forces_and_moments(motor_speeds)
        
        # 2. Обновление линейного движения (в глобальной системе координат)
        acceleration_global = net_force_global / self.mass
        self.velocity += acceleration_global * dt
        self.position += self.velocity * dt
        
        # Ограничение по земле
        if self.position[2] < 0.0:
            self.position[2] = 0.0
            if self.velocity[2] < 0.0:
                self.velocity[2] = 0.0
            # Можно также обнулить крен и тангаж и угловые скорости при контакте с землей,
            # чтобы предотвратить "проталкивание" сквозь землю из-за моментов.
            # self.attitude[0] = 0.0 # roll
            # self.attitude[1] = 0.0 # pitch
            # self.angular_velocity = np.array([0.0, 0.0, self.angular_velocity[2]]) # Stop roll/pitch rotation

        # 3. Обновление вращательного движения (в системе координат дрона - body frame)
        omega = self.angular_velocity
        inertia_val = self.inertia_matrix # I
        inv_inertia_val = self.inv_inertia_matrix # inv(I)

        angular_acceleration_body = inv_inertia_val @ (net_moment_body - np.cross(omega, inertia_val @ omega))
        self.angular_velocity += angular_acceleration_body * dt
        
        # 4. Обновление ориентации (углов Эйлера)
        roll, pitch, _ = self.attitude # yaw не используется в этой части матрицы напрямую, но обновляется
        sr, cr = np.sin(roll), np.cos(roll)
        
        # Используем np.tan(pitch) и sec(pitch) = 1/cos(pitch)
        # Обработка случая, когда cos(pitch) близок к нулю (тангаж +/- 90 градусов)
        cos_pitch = np.cos(pitch)
        if np.abs(cos_pitch) < 1e-7: # Достаточно малое значение для предотвращения деления на ноль
            # В случае gimbal lock, tan(pitch) и sec(pitch) стремятся к бесконечности.
            # Можно ограничить pitch или использовать альтернативную формулу, но это сложно.
            # Для простоты, если cos_pitch очень мал, можно использовать большое значение для sec_pitch
            # и соответствующее для tan_pitch, или даже пропустить обновление ориентации в этом шаге.
            # Однако, правильное ограничение pitch в конце обновления (np.clip) более надежно.
            # Здесь мы просто вычисляем, и np.clip ниже должен помочь.
            tan_pitch = np.sign(pitch) * 1e7 # Большое значение
            sec_pitch = np.sign(pitch) * 1e7 # Большое значение
        else:
            tan_pitch = np.tan(pitch)
            sec_pitch = 1.0 / cos_pitch
            
        euler_rate_of_change = np.array([
            self.angular_velocity[0] + self.angular_velocity[1] * sr * tan_pitch + self.angular_velocity[2] * cr * tan_pitch,
            self.angular_velocity[1] * cr - self.angular_velocity[2] * sr,
            self.angular_velocity[1] * sr * sec_pitch + self.angular_velocity[2] * cr * sec_pitch
        ])
        
        self.attitude += euler_rate_of_change * dt

        # Нормализация углов Эйлера
        self.attitude[0] = (self.attitude[0] + np.pi) % (2 * np.pi) - np.pi # Roll: [-pi, pi]
        # Pitch: ограничиваем до почти +/- pi/2, чтобы избежать сингулярностей в tan(pitch) и sec(pitch)
        self.attitude[1] = np.clip(self.attitude[1], -np.pi/2 + 1e-5, np.pi/2 - 1e-5) 
        self.attitude[2] = (self.attitude[2] + np.pi) % (2 * np.pi) - np.pi # Yaw: [-pi, pi]

        # print(f"Att: {np.degrees(self.attitude)}, AngVel: {np.degrees(self.angular_velocity)}")


# Пример использования:
if __name__ == '__main__':
    drone_phy = DronePhysics()
    delta_time = 0.01 # Уменьшил dt для большей стабильности при тестах
    
    hover_thrust_factor = (drone_phy.mass * drone_phy.gravity) / (drone_phy.num_motors * drone_phy.max_motor_thrust)
    print(f"Calculated hover thrust factor: {hover_thrust_factor:.4f}")

    # Тест 1: Простое зависание
    motor_speeds_hover = np.array([hover_thrust_factor] * 4)
    
    # Тест 2: Небольшой крен вправо
    # Увеличить тягу FL(1) и RL(2), уменьшить FR(0) и RR(3) для крена вправо (положительный roll_moment)
    # t0: FR, t1: FL, t2: RL, t3: RR
    # tau_x = L * (t1 + t2 - t0 - t3) -> полож. момент -> полож. крен (правое крыло вниз)
    # motor_speeds_roll_right = np.array([
    #     hover_thrust_factor - 0.05, # FR
    #     hover_thrust_factor + 0.05, # FL
    #     hover_thrust_factor + 0.05, # RL
    #     hover_thrust_factor - 0.05  # RR
    # ])

    # Тест 3: Небольшой тангаж вверх (нос вверх)
    # Увеличить тягу FR(0) и FL(1), уменьшить RL(2) и RR(3) для тангажа вверх (положительный pitch_moment)
    # tau_y = L * (t0 + t1 - t2 - t3) -> полож. момент -> полож. тангаж (нос вверх)
    motor_speeds_pitch_up = np.array([
        hover_thrust_factor + 0.1, # FR
        hover_thrust_factor + 0.1, # FL
        hover_thrust_factor - 0.1, # RL
        hover_thrust_factor - 0.1  # RR
    ])
    
    # Выбираем тестовый набор скоростей
    current_motor_speeds = motor_speeds_pitch_up # motor_speeds_hover # motor_speeds_roll_right 

    print(f"Initial - Pos: {drone_phy.position}, Vel: {drone_phy.velocity}, Att: {np.degrees(drone_phy.attitude)}, AngVel: {np.degrees(drone_phy.angular_velocity)}")
    
    for i in range(100): # Симулируем 1 секунду
        drone_phy.update_physics(delta_time, current_motor_speeds)
        if (i + 1) % 10 == 0: # Печатаем каждые 0.1 секунды
             print(f"Step {(i+1):3d} - Pos: [{drone_phy.position[0]:.2f}, {drone_phy.position[1]:.2f}, {drone_phy.position[2]:.2f}], Att: [{np.degrees(drone_phy.attitude[0]):.2f}, {np.degrees(drone_phy.attitude[1]):.2f}, {np.degrees(drone_phy.attitude[2]):.2f}], AngVel: [{np.degrees(drone_phy.angular_velocity[0]):.2f}, {np.degrees(drone_phy.angular_velocity[1]):.2f}, {np.degrees(drone_phy.angular_velocity[2]):.2f}]")

```
