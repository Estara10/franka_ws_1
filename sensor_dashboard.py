#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import tkinter as tk
from tkinter import ttk
import threading
import math
import sys
import numpy as np

# ========== 增加 Matplotlib 绘图依赖 ==========
import matplotlib
import functools
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.sans-serif"] = ["WenQuanYi Zen Hei", "Noto Sans CJK SC", "Microsoft YaHei"]
matplotlib.rcParams["axes.unicode_minus"] = False
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import collections
import datetime
import os
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState

class SensorDashboard(Node):
    def __init__(self, mode="compliant_chomp"):
        super().__init__('sensor_dashboard')
        self.mode = mode
        self.output_data_dir = os.getenv("EXPERIMENT_DATA_DIR", "experiment_data")
        os.makedirs(self.output_data_dir, exist_ok=True)
        self.left_wrench = WrenchStamped()
        self.right_wrench = WrenchStamped()
        self.task_done = False
        
        self.current_K_scale = 100.0
        self.current_D_scale = 100.0
        
        self.history_time = collections.deque() # 存放时间轴
        self.history_ly = collections.deque()   # 左臂Y向受力
        self.history_ry = collections.deque()   # 右臂Y向受力
        self.history_diff = collections.deque() # 双臂内应力(两臂Y向挤压力)
        self.history_smoothness = collections.deque() # 运动平滑度(速度导数)
        self.history_k_scale = collections.deque()
        self.history_d_scale = collections.deque()
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # 订阅由 ros2_control 发布的双臂力传感器数据
        self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster_left/wrench', self.left_cb, 10)
        self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster_right/wrench', self.right_cb, 10)
        
        # 订阅任务状态
        self.create_subscription(String, '/task_status', self.status_cb, 10)
        
        # 订阅关节状态计算运动平滑度
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        
        # 订阅自适应参数
        self.create_subscription(Float64MultiArray, '/adaptive_impedance_params', self.adaptive_cb, 10)
        self.last_joint_vel = None
        self.last_joint_time = None
        self.current_accel_norm = 0.0
        
        # TODO: 未来可以在这里添加发布器，向你的柔顺控制器(Impedance Controller)实时发布刚度/阻尼等参数
        # self.stiffness_pub = self.create_publisher(Float64MultiArray, '/compliance_controller/stiffness', 10)

    def status_cb(self, msg):
        print(f"DEBUG: 收到状态发布消息 -> {msg.data}")
        if msg.data == "DONE":
            self.task_done = True

    def left_cb(self, msg):
        self.left_wrench = msg

    def right_cb(self, msg):
        self.right_wrench = msg

    def adaptive_cb(self, msg):
        # 兼容不同自适应参数消息格式：
        # 1) [K, D, ...]
        # 2) [left_fz, right_fz, imbalance, avg_abs_force, K, D, ...]
        if len(msg.data) >= 6:
            self.current_K_scale = msg.data[4]
            self.current_D_scale = msg.data[5]
        elif len(msg.data) >= 2:
            self.current_K_scale = msg.data[0]
            self.current_D_scale = msg.data[1]

    def joint_cb(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not msg.velocity:
            return
            
        vel = np.array(msg.velocity)
        if self.last_joint_vel is not None and self.last_joint_time is not None:
            if len(vel) == len(self.last_joint_vel):
                dt = current_time - self.last_joint_time
                if dt > 0:
                    accel = (vel - self.last_joint_vel) / dt
                    # 均方根加速度作为平滑度指标
                    self.current_accel_norm = np.linalg.norm(accel)
            
        self.last_joint_vel = vel
        self.last_joint_time = current_time


def update_ui(root, node, vars, canvas, ax1, ax2, ax3, ax4, line_l, line_r, line_diff, line_smooth, line_k, line_d):
    try:
        current_t = node.get_clock().now().nanoseconds / 1e9 - node.start_time
        
        # 获取受力值
        lf = node.left_wrench.wrench.force
        rf = node.right_wrench.wrench.force
        
        # 计算相对内应力
        internal_stress = abs(lf.y + rf.y) / 2.0 
        
        node.history_time.append(current_t)
        node.history_ly.append(lf.y)
        node.history_ry.append(rf.y)
        node.history_diff.append(internal_stress)
        node.history_smoothness.append(node.current_accel_norm)
        node.history_k_scale.append(node.current_K_scale)
        node.history_d_scale.append(node.current_D_scale)

        # ====== 绘图更新 ======
        line_l.set_data(node.history_time, node.history_ly)
        line_r.set_data(node.history_time, node.history_ry)
        line_diff.set_data(node.history_time, node.history_diff)
        line_smooth.set_data(node.history_time, node.history_smoothness)
        line_k.set_data(node.history_time, node.history_k_scale)
        line_d.set_data(node.history_time, node.history_d_scale)

        # 动态调整x轴
        x_min = max(0, current_t - 5)
        x_max = max(5, current_t)
        ax1.set_xlim(x_min, x_max)
        ax2.set_xlim(x_min, x_max)
        ax3.set_xlim(x_min, x_max)
        ax4.set_xlim(x_min, x_max)
        
        # 动态调整y轴 (给一点余量)
        y_min1 = min(min(node.history_ly, default=-1), min(node.history_ry, default=-1))
        y_max1 = max(max(node.history_ly, default=1), max(node.history_ry, default=1))
        ax1.set_ylim(y_min1 - 2, y_max1 + 2)

        y_max2 = max(node.history_diff, default=1)
        ax2.set_ylim(0, y_max2 + 5)
        
        y_max3 = max(node.history_smoothness, default=1)
        ax3.set_ylim(0, y_max3 + 2)

        k_max = max(node.history_k_scale, default=100.0)
        d_max = max(node.history_d_scale, default=100.0)
        max_kd = max(k_max, d_max)

        ax4.set_ylim(0, max_kd + 20)
        
        canvas.draw_idle()

        # 更新左臂数据文字
        lt = node.left_wrench.wrench.torque
        vars['l_f'].set(f"Fx: {lf.x:7.2f}  Fy: {lf.y:7.2f}  Fz: {lf.z:7.2f} (N)")
        vars['l_t'].set(f"Tx: {lt.x:7.2f}  Ty: {lt.y:7.2f}  Tz: {lt.z:7.2f} (Nm)")
        vars['l_mag'].set(f"受力幅值: {math.sqrt(lf.x**2 + lf.y**2 + lf.z**2):6.2f} N")

        # 更新右臂数据
        rt = node.right_wrench.wrench.torque
        vars['r_f'].set(f"Fx: {rf.x:7.2f}  Fy: {rf.y:7.2f}  Fz: {rf.z:7.2f} (N)")
        vars['r_t'].set(f"Tx: {rt.x:7.2f}  Ty: {rt.y:7.2f}  Tz: {rt.z:7.2f} (Nm)")
        vars['r_mag'].set(f"受力幅值: {math.sqrt(rf.x**2 + rf.y**2 + rf.z**2):6.2f} N")

        # 更新统计数据文字
        if len(node.history_diff) > 0 and len(node.history_smoothness) > 0:
            avg_stress = sum(node.history_diff) / len(node.history_diff)
            max_stress = max(node.history_diff)
            avg_accel = sum(node.history_smoothness) / len(node.history_smoothness)
            max_accel = max(node.history_smoothness)
            
            stats_text = (
                f"【实时性能指标】\n"
                f"➢ 柔顺优越性 (内应力 N)：平均 = {avg_stress:.2f} | 最大值 = {max_stress:.2f}  (越小越好)\n"
                f"➢ CHOMP平滑度 (关节加速度范数)：平均 = {avg_accel:.2f} | 峰值 = {max_accel:.2f}  (越小越平滑)"
            )
            vars['stats'].set(stats_text)

        if node.task_done:
            # 任务完成，保存CSV数据并退出
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            os.makedirs(node.output_data_dir, exist_ok=True)
            filename = os.path.join(node.output_data_dir, f"experiment_{node.mode}_{timestamp}.csv")
            
            import csv
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Time", "Left_Y_Force", "Right_Y_Force", "Internal_Stress", "Joint_Accel_Norm", "K_Scale", "D_Scale"])
                for t, ly, ry, diff, sm, ks, ds in zip(node.history_time, node.history_ly, node.history_ry, node.history_diff, node.history_smoothness, node.history_k_scale, node.history_d_scale):
                    writer.writerow([t, ly, ry, diff, sm, ks, ds])
            
            print(f"\\n✅ 实时数据已保存至CSV: {filename}")
            import sys
            sys.stdout.flush()
            
            # 给节点一点时间再退出
            root.after(1000, root.quit)
            node.task_done = False # 避免重复触发保存

    except BaseException as e:
        print(f"UI update Error: {e}")


def main():
    rclpy.init()
    # 解析命令行参数
    mode = "compliant_chomp"
    if len(sys.argv) > 1:
        mode = sys.argv[1]
        
    node = SensorDashboard(mode=mode)
    
    # 后台线程运行ROS 2 spin，不阻塞UI线程
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        root = tk.Tk()
    except tk.TclError as e:
        print("错误: 无法启动图形界面，请确保系统已安装 python3-tk 并具有显示环境。")
        sys.exit(1)

    root.title(f"双臂控制 & 实时内应力观测 - 当前模式: [{mode.upper()}]")
    root.geometry("900x1000")
    root.configure(padx=10, pady=10)
    
    style = ttk.Style()
    try:
        style.theme_use('clam')
    except:
        pass
    
    # ================= 1. 传感器监控区 =================
    sensor_frame = ttk.LabelFrame(root, text=" 实时末端受力监控 ", padding=(15, 15))
    sensor_frame.pack(fill="x", pady=(0, 15))
    
    vars = {
        'l_f': tk.StringVar(value="Fx:    0.00  Fy:    0.00  Fz:    0.00 (N)"),
        'l_t': tk.StringVar(value="Tx:    0.00  Ty:    0.00  Tz:    0.00 (Nm)"),
        'l_mag': tk.StringVar(value="受力幅值:   0.00 N"),
        'r_f': tk.StringVar(value="Fx:    0.00  Fy:    0.00  Fz:    0.00 (N)"),
        'r_t': tk.StringVar(value="Tx:    0.00  Ty:    0.00  Tz:    0.00 (Nm)"),
        'r_mag': tk.StringVar(value="受力幅值:   0.00 N"),
        'stats': tk.StringVar(value="[实时指标] 正在采集数据..."),
    }
    
    # 左臂
    ttk.Label(sensor_frame, text="🟢 左臂 (mj_left_hand)", font=("WenQuanYi Zen Hei", 11, "bold"), foreground="blue").grid(row=0, column=0, sticky="w", pady=(0, 5))
    ttk.Label(sensor_frame, textvariable=vars['l_f'], font=("Courier", 11)).grid(row=1, column=0, sticky="w", padx=10)
    ttk.Label(sensor_frame, textvariable=vars['l_t'], font=("Courier", 11)).grid(row=2, column=0, sticky="w", padx=10)
    ttk.Label(sensor_frame, textvariable=vars['l_mag'], font=("Courier", 11, "bold"), foreground="darkred").grid(row=3, column=0, sticky="w", padx=10, pady=(0, 15))

    # 右臂
    ttk.Label(sensor_frame, text="🔵 右臂 (mj_right_hand)", font=("WenQuanYi Zen Hei", 11, "bold"), foreground="blue").grid(row=4, column=0, sticky="w", pady=(0, 5))
    ttk.Label(sensor_frame, textvariable=vars['r_f'], font=("Courier", 11)).grid(row=5, column=0, sticky="w", padx=10)
    ttk.Label(sensor_frame, textvariable=vars['r_t'], font=("Courier", 11)).grid(row=6, column=0, sticky="w", padx=10)
    ttk.Label(sensor_frame, textvariable=vars['r_mag'], font=("Courier", 11, "bold"), foreground="darkred").grid(row=7, column=0, sticky="w", padx=10)
    
    # ================= 2. 实时绘图区 (评估优越性) =================
    plot_frame = ttk.LabelFrame(root, text=" 实时实验指标曲线 (柔顺 & CHOMP 评估) ", padding=(10, 10))
    plot_frame.pack(fill="both", expand=True)
    
    # 增加统计结果显示标签
    ttk.Label(plot_frame, textvariable=vars['stats'], font=("WenQuanYi Zen Hei", 12, "bold"), foreground="indigo").pack(anchor="w", pady=(0, 10))

    fig = Figure(figsize=(7, 8), dpi=100)
    fig.subplots_adjust(hspace=0.5, top=0.95, bottom=0.1)
    
    # 子图1：左右臂各自受力
    ax1 = fig.add_subplot(411)
    ax1.set_title("双臂Y轴接触力 (N)")
    ax1.grid(True)
    line_l, = ax1.plot([], [], 'g-', label='Left Y-Force')
    line_r, = ax1.plot([], [], 'b-', label='Right Y-Force')
    ax1.legend(loc='upper right')

    # 子图2：双臂挤压内力
    ax2 = fig.add_subplot(412)
    ax2.set_title("内应力 (N) [柔顺性指标 - 越低越好]")
    ax2.grid(True)
    line_diff, = ax2.plot([], [], 'r-', linewidth=2, label='Clamp Internal Stress')
    ax2.axhline(y=5.0, color='grey', linestyle='--', label='Stress Limit')
    ax2.legend(loc='upper right')

    # 子图3：运动平滑度 - 关节加速度范数
    ax3 = fig.add_subplot(413)
    ax3.set_title("关节加速度范数 [CHOMP平滑度指标 - 越低越平滑越好]")
    ax3.grid(True)
    line_smooth, = ax3.plot([], [], 'm-', linewidth=2, label='Joint Accel Norm')
    ax3.legend(loc='upper right')

    # 子图4：自适应阻抗缩放
    ax4 = fig.add_subplot(414)
    ax4.set_title("实时自适应阻抗缩放 (%)")
    ax4.grid(True)
    line_k, = ax4.plot([], [], 'k-', linewidth=1.5, label='K Scale')
    line_d, = ax4.plot([], [], 'orange', linewidth=1.5, label='D Scale')
    ax4.legend(loc='upper right')

    canvas = FigureCanvasTkAgg(fig, master=plot_frame)
    canvas.get_tk_widget().pack(fill="both", expand=True)

    def update_loop():
        update_ui(root, node, vars, canvas, ax1, ax2, ax3, ax4, line_l, line_r, line_diff, line_smooth, line_k, line_d)
        root.after(50, update_loop)

    root.after(100, update_loop)
    root.protocol("WM_DELETE_WINDOW", lambda: (rclpy.shutdown(), root.destroy()))
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
