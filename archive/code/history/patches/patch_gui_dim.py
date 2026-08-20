with open('sensor_dashboard.py', 'r') as f:
    text = f.read()

old_cb = """        vel = np.array(msg.velocity)
        if self.last_joint_vel is not None and self.last_joint_time is not None:
            dt = current_time - self.last_joint_time
            if dt > 0:
                accel = (vel - self.last_joint_vel) / dt
                # 均方根加速度作为平滑度指标
                self.current_accel_norm = np.linalg.norm(accel)
        
        self.last_joint_vel = vel"""

new_cb = """        vel = np.array(msg.velocity)
        if self.last_joint_vel is not None and self.last_joint_time is not None:
            if len(vel) == len(self.last_joint_vel):
                dt = current_time - self.last_joint_time
                if dt > 0:
                    accel = (vel - self.last_joint_vel) / dt
                    # 均方根加速度作为平滑度指标
                    self.current_accel_norm = np.linalg.norm(accel)
            
        self.last_joint_vel = vel"""

text = text.replace(old_cb, new_cb)

with open('sensor_dashboard.py', 'w') as f:
    f.write(text)
