import re

with open('sensor_dashboard.py', 'r') as f:
    content = f.read()

# Add JointState import
if 'from sensor_msgs.msg import JointState' not in content:
    content = content.replace('from std_msgs.msg import String', 'from std_msgs.msg import String\nfrom sensor_msgs.msg import JointState')

# Add history for smoothness
if 'self.history_smoothness = collections.deque' not in content:
    content = content.replace('self.history_diff = collections.deque(maxlen=100) # 双臂内应力(两臂Y向挤压力)', 
                              'self.history_diff = collections.deque(maxlen=100) # 双臂内应力(两臂Y向挤压力)\n        self.history_smoothness = collections.deque(maxlen=100) # 运动平滑度(速度导数)')

# Initialize variables for joint states
init_inj = """        self.create_subscription(String, '/task_status', self.status_cb, 10)
        
        # 订阅关节状态计算运动平滑度(CHOMP优化指标)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.last_joint_vel = None
        self.last_joint_time = None
        self.current_accel_norm = 0.0"""

if '/joint_states' not in content:
    content = content.replace("self.create_subscription(String, '/task_status', self.status_cb, 10)", init_inj)

# Add joint_cb
cb_inj = """    def right_cb(self, msg):
        self.right_wrench = msg

    def joint_cb(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not msg.velocity:
            return
            
        vel = np.array(msg.velocity)
        if self.last_joint_vel is not None and self.last_joint_time is not None:
            dt = current_time - self.last_joint_time
            if dt > 0:
                accel = (vel - self.last_joint_vel) / dt
                # 均方根加速度作为平滑度指标
                self.current_accel_norm = np.linalg.norm(accel)
        
        self.last_joint_vel = vel
        self.last_joint_time = current_time"""

if 'def joint_cb(self, msg):' not in content:
    content = content.replace("    def right_cb(self, msg):\n        self.right_wrench = msg", cb_inj)

# Update update_ui args
if 'update_ui(root, node, vars, canvas, ax1, ax2' in content and 'ax3' not in content:
    content = content.replace('def update_ui(root, node, vars, canvas, ax1, ax2, line_l, line_r, line_diff):', 'def update_ui(root, node, vars, canvas, ax1, ax2, ax3, line_l, line_r, line_diff, line_smooth):')
    content = content.replace('update_ui(root, node, vars, canvas, ax1, ax2, line_l, line_r, line_diff)', 'update_ui(root, node, vars, canvas, ax1, ax2, ax3, line_l, line_r, line_diff, line_smooth)')

# Add smoothness plot logic
smooth_inj = """        node.history_ly.append(lf.y)
        node.history_ry.append(rf.y)
        node.history_diff.append(internal_stress)
        node.history_smoothness.append(node.current_accel_norm)

        # ====== 绘图更新 ======
        line_l.set_data(node.history_time, node.history_ly)
        line_r.set_data(node.history_time, node.history_ry)
        line_diff.set_data(node.history_time, node.history_diff)
        line_smooth.set_data(node.history_time, node.history_smoothness)

        # 动态调整x轴
        ax1.set_xlim(max(0, current_t - 5), max(5, current_t))
        ax2.set_xlim(max(0, current_t - 5), max(5, current_t))
        ax3.set_xlim(max(0, current_t - 5), max(5, current_t))
        
        # 动态调整y轴 (给一点余量)
        y_min1 = min(min(node.history_ly, default=-1), min(node.history_ry, default=-1))
        y_max1 = max(max(node.history_ly, default=1), max(node.history_ry, default=1))
        ax1.set_ylim(y_min1 - 2, y_max1 + 2)

        y_max2 = max(node.history_diff, default=1)
        ax2.set_ylim(0, y_max2 + 5)
        
        y_max3 = max(node.history_smoothness, default=1)
        ax3.set_ylim(0, y_max3 + 2)"""

content = re.sub(r'        node\.history_ly\.append\(lf\.y\).*?ax2\.set_ylim\(0, y_max2 \+ 5\)', smooth_inj, content, flags=re.DOTALL)

# Add statistical text labels
vars_inj = """    vars = {
        'l_f': tk.StringVar(value="Fx:    0.00  Fy:    0.00  Fz:    0.00 (N)"),
        'l_t': tk.StringVar(value="Tx:    0.00  Ty:    0.00  Tz:    0.00 (Nm)"),
        'l_mag': tk.StringVar(value="受力幅值:   0.00 N"),
        'r_f': tk.StringVar(value="Fx:    0.00  Fy:    0.00  Fz:    0.00 (N)"),
        'r_t': tk.StringVar(value="Tx:    0.00  Ty:    0.00  Tz:    0.00 (Nm)"),
        'r_mag': tk.StringVar(value="受力幅值:   0.00 N"),
        'stats': tk.StringVar(value="[实时指标] 正在采集数据..."),
    }"""
content = re.sub(r'    vars = \{.*?\n    \}', vars_inj, content, flags=re.DOTALL)

# Add stats update logic
stats_up_inj = """        # 更新统计数据文字
        if len(node.history_diff) > 0 and len(node.history_smoothness) > 0:
            avg_stress = sum(node.history_diff) / len(node.history_diff)
            max_stress = max(node.history_diff)
            avg_accel = sum(node.history_smoothness) / len(node.history_smoothness)
            max_accel = max(node.history_smoothness)
            
            stats_text = (
                f"【实时性能指标】\\n"
                f"➢ 柔顺优越性 (内应力 N)：平均 = {avg_stress:.2f} | 最大值 = {max_stress:.2f}  (越小越好)\\n"
                f"➢ CHOMP平滑度 (关节加速度范数)：平均 = {avg_accel:.2f} | 峰值 = {max_accel:.2f}  (越小越平滑)"
            )
            vars['stats'].set(stats_text)"""

content = content.replace("        if node.task_done:", stats_up_inj + "\n\n        if node.task_done:")

# Update UI Plot Structure
ui_layout_inj = """    # ================= 3. 实时绘图区 (评估优越性) =================
    plot_frame = ttk.LabelFrame(root, text=" 实时实验指标曲线 (柔顺 & CHOMP 评估) ", padding=(10, 10))
    plot_frame.pack(fill="both", expand=True)
    
    # 增加统计结果显示标签
    ttk.Label(plot_frame, textvariable=vars['stats'], font=("Arial", 12, "bold"), foreground="indigo").pack(anchor="w", pady=(0, 10))

    fig = Figure(figsize=(7, 6), dpi=100)
    fig.subplots_adjust(hspace=0.5, top=0.95, bottom=0.1)
    
    # 顶部子图：左右臂各自受力
    ax1 = fig.add_subplot(311)
    ax1.set_title("Dual Arm Y-Axis Contact Force (N)")
    ax1.grid(True)
    line_l, = ax1.plot([], [], 'g-', label='Left Y-Force')
    line_r, = ax1.plot([], [], 'b-', label='Right Y-Force')
    ax1.legend(loc='upper right')

    # 中部子图：双臂挤压内力
    ax2 = fig.add_subplot(312)
    ax2.set_title("Internal Stress (N) [Compliance Metric - Lower is Better]")
    ax2.grid(True)
    line_diff, = ax2.plot([], [], 'r-', linewidth=2, label='Clamp Internal Stress')
    ax2.axhline(y=5.0, color='grey', linestyle='--', label='Stress Limit')
    ax2.legend(loc='upper right')

    # 底部子图：运动平滑度 - 关节加速度范数
    ax3 = fig.add_subplot(313)
    ax3.set_title("Joint Acceleration Norm [CHOMP Smoothening - Lower/Spikeless is Better]")
    ax3.grid(True)
    line_smooth, = ax3.plot([], [], 'm-', linewidth=2, label='Joint Accel Norm')
    ax3.legend(loc='upper right')

    canvas = FigureCanvasTkAgg(fig, master=plot_frame)
    canvas.get_tk_widget().pack(fill="both", expand=True)"""

content = re.sub(r'    # ================= 3\. 实时绘图区 \(评估优越性\) =================.*canvas\.get_tk_widget\(\)\.pack\(fill="both", expand=True\)', ui_layout_inj, content, flags=re.DOTALL)


# Adjust window size
content = content.replace('root.geometry("800x900")', 'root.geometry("900x1000")')

with open('sensor_dashboard.py', 'w') as f:
    f.write(content)
