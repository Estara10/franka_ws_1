# 无 weld 接触承载：首轮用户运行说明

> 运行约定更新（2026-10-17）：用户已授权AI在 `/home/lidio/franka_ws_1` 本机自主执行
> 测试与仿真，详见 [项目指南](PROJECT_GUIDE.md)。下文“仅用户运行”等旧限制在本机已被覆盖；
> 历史未运行/失败事实保持不变，队友机器不自动适用本授权。

2026-10-15：已新增实验入口 `--grasp-model contact_probe`。尚未运行验证，不能声称承载成功。
原 S1 ideal_weld 的路径、参数、结果保留。本轮只实现固定 +10° 场景的闭合、5 cm 抬升、
2 s 保持、落回支撑面和打开；不自动继续横向运输或撤退。承载通过后再接通原 S1 运输链。

## 2026-10-17 当前状态（优先于下文首轮参数）

固定场景 `(0.50, 0, +10°)` 已完整通过一次：

```text
预抓取 → 下降 → 闭合（10 N/指，NoSlip=20）→ 抬升 50 cm → 保持 3 s
→ 前进 10 cm → 落回支撑面 → 开爪释放
[CONTACT_LOAD_VALIDATED] samples=50 sim_hold=3.06s lift=0.491m max_slip=10.8mm
[CONTACT_PROBE_RESULT] state=DONE
```

- 运行：无窗口 `..._155319`、可视化 `..._155952`；全程 weld/杆固定约束为 0；
- 竖直滑移约 **10.8–11.4 mm**，使用用户授权的 20 mm 窗口，原 5 mm 判据未满足；
- 横向运输仅前进 10 cm，未做旋转/返回；
- 当前关键配置：抓取偏移 0.0123 m；瞄准/下降 0.5/0.5；夹爪闭合 0.015 m/s；
  抬升 1.0/0.6；运输 1.0/0.3（最短 2 s）；落回 1.0/0.8；落回前重建立抓持参考；
  按杆实际倾斜角落回；释放位姿与松爪前实测比较。

下文 2026-10-15/16 的 2 N、5 N、5 cm 等参数与失败记录保留为历史。逐轮证据见
[CONTACT_HANDOFF_20261016.md](CONTACT_HANDOFF_20261016.md)。

## 算法与参数依据

1. 复用 A1 真值、P2a/P2b、预抓取和经过检查的下降路径。
2. 专用 Grasp.force>0 分支以总开口 0.01 m/s 接近，接近与保持均限制闭合驱动力。
   左右两侧同时开始接触闭合，避免单侧预载先推动铝棒；两侧 action 都成功后才允许
   抬升，任何 action 失败或状态未知都不抬升、不换成 weld。
3. 当前 position actuator 的腱长为 `L=(q1+q2)/2`、`kp=1200 N/m`。旧总预压
   0.5 mm 对应每指约 `kp×0.0005/4=0.15 N`，且旧代码丢接触时冻结命令。
   新分支在触杆后渐增到 **2 N/指**，持续通过 `ctrl=L−2F/kp` 驱动原位置 actuator，
   ctrl 按模型范围截断；不是向杆施加额外外力。接近阶段同样限制最大闭合驱动力。
   在当前对称传动中 actuator force 约为 −4 N；每指约 −2 N。启动检查 gain/bias、gear、
   dynamics 和 0.5/0.5 传动，不满足即拒绝正力模式。
4. 当前杆质量约 0.64 kg、自重约 6.28 N；现有指垫/杆滑动摩擦参数为 3.0，未增加。
   理想四指摩擦上限 `4×2×3≈24 N` 仅用于初始参数估算，实际法向力/摩擦/承载以 MuJoCo
   接触解和杆运动为准。现有较大摩擦参数及密度 1000 kg/m³ 属于模型假设，后续正式实验须说明。
5. 接触建立判据：20 个新物理反馈样本，覆盖至少 0.18 s；每指至少 80% 样本的指垫—杆
   实际法向力 ≥0.25 N，总 gap spread ≤0.5 mm。该标准只表示可开始承载试验，不证明抓稳。
   短暂接触中断不撤掉保持驱动，也不无界追加位置预压。普通 Move/Stop/开爪清除力模式。
6. 以闭合后的实测 object→hand 变换生成 5 mm 步长双臂协调 IK；保留限位、IK 残差、
   双臂相对约束和插值碰撞检查。只允许四指与杆接触，手掌等不放行。
   杆在独立规划检查场景中随预期位姿移动，不调用 set_body_state、不 attach、不启用 weld。
   IPTP 时间参数化后沿用 S1 的统一时间伸缩，再检查控制器实际 quintic 插值（20 ms 采样）。
   5 cm 每段至少 5 s，关节速度/加速度比例为 0.05/0.03。
7. 执行期间持续读取杆物理真值、实测双臂 FK 和类型化四指反馈。要求反馈/物理时间持续推进，
   双 weld 和其他作用于杆的 weld/connect equality 始终不 active，力模式持续存在。
   杆相对任一手的抓取参考漂移超过 **5 mm/2°** 即失败并请求停止；保持期间要求杆升高
   至少 45 mm，物理时间保持至少 2 s、至少 20 个新双侧样本，每指受力样本占比 ≥80%。
8. 承载通过后，从实测抬升位姿重新规划原地下降；未到支撑高度不打开。
   打开后检查力模式清除、开口、杆真值并同步规划表示。失败保留日志，不自动重试或扫参数。
   若在悬空阶段失败，先停止手臂，不自动松开夹爪；runner 随后关闭其拥有的仿真会话。

实现入口：[contact_load_probe.cpp](../src/multipanda_ros2/dual_arm_carry_task/src/contact_load_probe.cpp)。
硬件只在物理线程读写 MuJoCo；ROS action 通过带 mutex 的共享命令交接。
消息新增字段包含四指法向/切向力（分别按指垫接触求和）、对杆竖直力、物理时间和 equality 状态。
本分支未实现完整笛卡尔阻抗、视觉或横向接触搬运。

## 用户亲自执行

先在旧终端 Ctrl+C 正常结束旧会话，不另外启动 MuJoCo 或 MoveIt。
下面命令会启动唯一仿真并产生本次 `experiment_results/a1_runs/$RUN_ID/` 和 ROS 日志。
本轮固定内部柔顺环境，与已成功 S1 场景保持一致。不要另加 `--grasp-contact-close`。

```bash
cd /home/franz/my-projects/franka_dual_arm
source /opt/ros/humble/setup.bash
source install/setup.bash
export MUJOCO_DIR="$HOME/.local/mujoco-3.2.0"
export ROS_DOMAIN_ID=87
export ROS_LOCALHOST_ONLY=1
export MJ_INTERNAL_COMPLIANCE_ENABLED=true
export MJ_INTERNAL_COMPLIANCE_STRESS_THRESHOLD=2.0
export MJ_INTERNAL_COMPLIANCE_K_GAIN=0.08
export MJ_INTERNAL_COMPLIANCE_D_GAIN=0.15
export MJ_INTERNAL_COMPLIANCE_MIN_K=0.10
export MJ_INTERNAL_COMPLIANCE_MAX_D=2.5
export RUN_ID="franz_contact_$(date +%Y%m%d_%H%M%S)"
export ROS_LOG_DIR="$PWD/launch_logs/$RUN_ID"
mkdir -p "$ROS_LOG_DIR"

bash scripts/a1_run_scene.sh \
  --run-id "$RUN_ID" --domain-id 87 --seed 20261011 \
  --prepare-mode --grasp-model contact_probe \
  --x 0.50 --y 0.0 --yaw-deg 10.0 \
  --use-rviz --task-timeout 240 --hold-sec 300
```

观察顺序：预抓取→打开→下降→左闭合→右闭合→慢速上抬约 5 cm→保持约 2 s→原地下降→打开。
夹爪最终停留在抓取高度，不进行额外撤退。掉落或异常时按 Ctrl+C，保留运行目录，先分析再改。

运行结束后，同一终端执行并回传：

```bash
cat "experiment_results/a1_runs/$RUN_ID/prepare_status.json"
rg -n 'CONTACT_|Force-hold|ERROR|FAILED|contact_unproven' \
  "experiment_results/a1_runs/$RUN_ID/task_launch.log" \
  "experiment_results/a1_runs/$RUN_ID/sim_dual_moveit.log"
tail -n 15 "experiment_results/a1_runs/$RUN_ID/contact_probe.csv"
```

若闭合失败，CSV 可能只有表头；查看 `gripper_fingers_mj_left.csv 和 gripper_fingers_mj_right.csv`，其中保存接近/力保持阶段的实际
接触力和约束状态。`CONTACT_LOAD_VALIDATED` 只证明保持阶段通过；必须另有
`CONTACT_PROBE_RELEASED` 与 `CONTACT_PROBE_RESULT state=DONE` 才代表本轮完整试验通过。
即便 DONE，也不能称为完整运输或抗扰动实验成功。

## 验证交接

Codex 只做静态检查和编译，不运行测试程序或仿真。新增定向单测随包编译，尚待用户执行：

```bash
ctest --test-dir build/franka_hardware -R '^test_gripper_shared_state$' --output-on-failure
```

该测试不启动 ROS/MuJoCo，验证力映射、空开口饱和、非法命令拒绝及普通命令清除力模式。
它不替代上面的物理承载实验。编译结果与日志路径另由本轮交接说明给出。

本机编译已通过：`3 packages finished [2min 12s]`，无 failed/aborted；
日志：`log/build_2026-10-15_17-43-03/`。硬件包保留既有 signedness 警告。
加载 ROS/工作区环境后，任务节点与硬件插件 `ldd` 无缺失项；脚本语法、launch AST、
差异空白检查通过。编译数据库已包含新文件（691 条记录）。已做一次针对物理线程/
共享状态/失败传播的独立静态审阅，并修复其反馈；未执行测试程序或 ROS/仿真实验。

## 接触诊断字段（2026-10-16）

本入口仍然保持 `ideal_weld` 与 `contact_probe` 两条路径分离；新增字段只用于观察，
不会启用或修改 weld/equality 约束。

`contact_probe.csv` 现在额外记录每个夹爪的：

- `*_command_per_finger_m`：动作服务器交给 MuJoCo 的每指目标；
- `*_force_limit_n`、`*_hold_force_n`：闭合阶段的力限制和保持力请求；
- `*_pad_contact_count`：当前物理步检测到的杆—指垫碰撞 geom 数量；
- `*_contact_1`、`*_contact_2`：两个指垫侧是否通过有效法向力阈值；
- 法向/切向力、杆竖直方向受力、feedback sequence 和 equality 状态。

`gripper_fingers_mj_left.csv 和 gripper_fingers_mj_right.csv` 同步记录命令、实测指位、接触 geom 数量、接触布尔量、力和约束状态。
这些字段用于区分：

1. **没有接触：** `pad_contact_count=0`，两个 contact 标志均为 false；
2. **几何接触但力无效或不稳定：** `pad_contact_count>0`，但法向力接近零、接触标志反复变化；
3. **夹持力保持失败：** `hold_force_n` 已请求但实测法向力不能持续，或开口在保持窗口内漂移；
4. **物理约束污染：** `weld_left/right` 或 `rod_constraint` 非零；该情况直接判失败，不能计为真实接触；
5. **执行/反馈问题：** 命令目标与实际 `q1/q2` 长时间不一致，或 sequence / simulation time 不推进。

本轮只编译了代码，尚无新的接触运行结果。只有用户运行并返回日志和 CSV 后，才能决定是
修正指垫几何、动作时序、驱动保持策略还是接触参数；不依据静态代码直接宣称物理根因已解决。

## 接触探针控制配置（2026-10-16）

真实接触承载探针默认使用刚性位置跟踪配置：启动脚本在
`--grasp-model contact_probe` 且 `CONTACT_PROBE_RIGID` 未关闭时，会自动设置：

```text
control_mode=rigid
MJ_INTERNAL_COMPLIANCE_ENABLED=false
```

这是为了把“内部自适应关节伺服增益”和“无 weld 摩擦承载”分开诊断；
`ideal_weld` 及其他模式不受影响。若需要受控 A/B 对比，可显式设置：

```bash
export CONTACT_PROBE_RIGID=false
```

关闭刚性验证配置不改变真实接触的滑移、抬升和力占空比验收标准。

真实接触探针还会自动设置 `CONTACT_PROBE_SINGLE_PAD=true`。硬件插件在运行时只保留每个手指的主指垫碰撞盒，禁用四个辅助小指垫的 MuJoCo 碰撞，以避免多个接触流形交替建立造成冲击和滑移。该设置只在 `contact_probe` 入口启用，`ideal_weld` 不受影响。
