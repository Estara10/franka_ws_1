# lidio 本机刚性无 weld hover 复现（2026-10-20）

## 配置与验证

- 提交：`1161a7a829883e70b3b1c18a8467895ebfe99f4b`；运行前后源码工作树无修改。本机 MuJoCo：`/home/lidio/Libraries/mujoco`；ROS 域 `113`，仅本机通信。
- 固定场景 `(0.50, 0.00, +10°)`；`contact_hover_only=true`，命令高度 `0.05 m`、保持 `3 s`；rigid=true、adaptive force hold=false、内部 MuJoCo compliance=false。冻结质量、摩擦、接触几何和夹持力（10 N/指）；single-primary-pad trace 开启。保护保持 20 mm 用户授权窗口、2°姿态限值，5 mm 仍为研究目标。
- 唯一物理运行：`CONTACT_PROBE_RUN_ID=contact_hover_rigid_lidio_20261020_0932 bash scripts/run_contact_probe_local.sh --headless`，并显式设置上列参数、`MUJOCO_DIR`、`CONTACT_PROBE_DOMAIN_ID=113` 和 `/tmp/contact_hover_ros_home_lidio_20261020`。命令由 `scripts/run_contact_probe_local.sh` 调用 `scripts/a1_run_scene.sh`；MoveIt launch 已包含 MuJoCo。
- 构建：`colcon build --base-paths src/multipanda_ros2 --parallel-workers 2 --packages-up-to dual_arm_carry_task franka_hardware franka_moveit_config multi_mode_controller_impl --cmake-args -DMUJOCO_DIR=/home/lidio/Libraries/mujoco`，11 包通过，覆盖 `multi_mode_control_msgs`、MMC 核心/实现、硬件、MoveIt 配置、任务包及依赖。先前本机 helper 因缺少可选 DQ Robotics 目录退出；清理 Python 注入变量又导致 `ament_package` 不可见，恢复 ROS `PYTHONPATH` 后，两个旧消息生成目录与 symlink 目标冲突，已移存到 `/tmp/contact_hover_build_conflicts_20261020/`，未删除。最终构建日志：`/tmp/contact_hover_lidio_build_20261020_final.log`。
- 定向测试：`test_grasp_contact_acceptance` 3/3、`test_torque_limits` 5/5 通过；`test_hover_acceptance_standalone.cpp` 编译并运行通过。全局 `colcon test-result --verbose` 混入了其它包的历史 lint 结果，本轮未处理。
- 安装入口核对：launch 从 `/home/lidio/franka_ws_1/install/franka_moveit_config` 读取 `dual_panda_generated.srdf` 和 `sim_dual_panda_ros_controllers.yaml`，两者与源码逐字一致。运行快照为 `dual_panda_arm_controller`（JTC）active；`sim_multi_mode_controller` 未加载/非 active。任务日志的参数显示 hover=true、自适应=false、rigid 模式。

## 本轮结果与 10-19 基线

| 项目 | 10-19 `contact_hover_gate_20261019_222642` | lidio 本轮 `contact_hover_rigid_lidio_20261020_0932` |
|---|---:|---:|
| P2a / P2b、预抓取、下降 | 通过 | 通过 |
| 接触闭合 / bilateral gate | 通过 | 闭合 action 成功；gate 在 3 s 超时 |
| 抬升 / HOLD / 下降 / 释放 | 完整通过 | 未开始 / 未发生 / 未发生 / 未发生 |
| 抬升峰值、保持时间、峰后下降 | 46.243 mm、3.036 s、3.050 mm | 无有效悬空数据 |
| 原抓取参考最大位移 / 姿态残差 | 9.282 mm / 1.445° | 未测：`contact_probe.csv`、`hover_diagnostics.csv` 只有表头 |
| weld 左/右、rod constraint | 全程关闭 | trace 与失败快照全为 `0/0/0` |
| 任务终态 | `DONE` | `ERROR: E_CONTACT_BILATERAL_CONTACT_NOT_ESTABLISHED` |

本轮链路为 truth → P2a/P2b → 预抓取 → 下降至抓取位 → 两侧闭合 action 成功 → bilateral gate 失败 → 未抬升；lower、release 未执行。门控时读数为 `contacts=(1,1;1,0)`、法向 `(8.796,10.828;15.699,0.000) N`。随后失败诊断的左右反馈相差 6 ms（sim time 29.534 / 29.528，序号 14769 / 14766），并显示四侧暂时都有接触；这些异步读数不是同一物理步的受力平衡。夹爪 measured-position stop 获确认，任务如实发布 `ERROR`。

接触 trace 有 3909 行；左右各约 310 个采样帧，单臂接触点数在 0、4、8 间变化。四侧采样占比依次为左手左/右侧 77.5%/82.3%、右手左/右侧 72.3%/76.8%；对齐采样中四侧同时接触为 142/311（45.7%）。这些是采样占比，不是物理时间占空比。raw 接触 `wrench_n` 共 3836 点，中位数 2.344 N、p95 5.299 N、峰值 19897.599 N；22 行超过 150 N，峰值出现在 sim time 26.02、feedback seq 13012。尖峰和所有原始行均保留，未裁剪；异常原因未确认。左右 trace 最近时间差中位数/p95 为 0、最大 20 ms；各臂 311/310 个不同序号，没有同一序号跨时间重复。

因此本轮结论是：P2 到抓取闭合链通过；刚性 hover 功能失败于既有 bilateral gate，未形成 JTC 悬空承载基线，也没有达到可与 MMC 比较的 HOLD 数据。不得据此重调摩擦/夹持力或认定几何机理根因。10-19 的 `DONE` 与约 9.28 mm 残差仍是有效历史基线；它说明 5 mm 目标未达，不替代 lidio 本轮失败事实。

## MMC 接触保持接线核查（只读）

- 切换入口在 `dual_impedance_no_load_bridge.cpp`：请求以 `STRICT` 同时停用 JTC、启用 `sim_multi_mode_controller`，满足一停一启；MoveIt launch 默认只激活 JTC。MMC `PandaControllerBase::start_()` 在激活时读取当前实测 pose/q 并把它设为初始目标，避免从零参考启动。
- 控制器内部 Cartesian pose 来自 Franka `O_T_EE`，任务的 q 来自同一 `GetRobotStates` 响应；Cartesian 控制使用 base/零 Jacobian。两者应按各臂 link0 局部坐标传递。未来若从 task 的 world pose 建目标，须先变换到各自 link0；当前双臂 goal message 没有 frame 字段。
- watchdog 由 inner controller 锁存，日志报错并让 `MultiModeController::update()` 首次返回 `ERROR`，之后继续保留最后有效 torque。现有 no-load bridge 没有把 watchdog/controller-manager 状态接入任务结果判定或故障恢复逻辑；下一步需由编排层监测 controller state/fault，并给出受控 hold/退出路径。
- 夹爪 tendon/action 与臂 JTC/MMC 控制器资源分开；硬件 command-mode 切换只重设七个臂关节 actuator 的 torque/position/velocity 模式，代码上不会主动清除夹爪保持。但本轮没有实际切换 MMC 承载，需在下一任务做受控验证。
- 实际接入点应在 `contact_load_probe.cpp` 的 bilateral gate 通过后、抬升轨迹下发前：从当前双手 `O_T_EE`、q 初始化目标，严格切到 MMC 后再发送同坐标参考。当前明确阻断项是 rigid gate 未通过、缺少有效悬空基线，以及 watchdog 故障编排和失败退出尚未接入；本轮不切 MMC。

原始运行目录：`experiment_results/a1_runs/contact_hover_rigid_lidio_20261020_0932/`；保留 `task_launch.log`、`sim_dual_moveit.log`、`contact_probe.csv`、`contact_probe_contacts.csv`、`hover_diagnostics.csv`、分析 JSON/TXT、控制器状态和 manifest。wrapper 确认 simulator reaped；本轮 task/runner/simulator PID 均退出。启动警告（相机标定文件缺失、MoveIt 中 world/camera TF 树警告、JTC deprecated 参数）与实际 gate 失败分开记录，均未阻止 P2 和抓取动作。

## 2026-10-20 gate 离线诊断与 watchdog 故障传播

### gate 窗口（离线，无新物理运行）

本次 task 没记录 gate 入口的 sim timestamp。以失败快照 `sim_time=29.534` 和 gate 的 3.0 s steady-clock 截止窗口回推，按近 1×仿真推进率分析约 `26.534–29.534 s`；CSV 该窗口每臂各 150 个 20 ms trace 帧。此窗口定位有约 20 ms 的入口误差可能，不把它描述成逐物理步轨迹。

夹爪 action 与额外 gate 是两层判据。每个 gripper action 在满请求力后用 20 个不同反馈序号构成稳定窗口：窗口至少 0.18 s、每指侧至少 16/20 帧接触、间隙变化不超过 1 mm。本次左右 action 分别记录接触占比 `(17,17)` 与 `(16,16)`，实测指侧法向力约 `(10.339,9.310) N` 与 `(10.855,8.769) N`，两 action 成功。额外 gate 又要求四指侧几何接触、每侧法向力 ≥0.25 N，并连续跨过 0.25 s 仿真时间；该 gate 用 3 s 墙钟超时。

按 trace 的 `feedback_seq` 聚合接触点；`contact_index=-1` 是显式零接触帧。法向力按 `force_included=1` 的各接触 `abs(wrench_n)` 对每指侧求和，与硬件反馈的按侧累加口径一致。

| 指侧 | 有几何接触帧 | 接触且法向力 ≥0.25 N | 法向力 median / p95 / max (N) |
|---|---:|---:|---:|
| 左臂左指 | 84.0% | 83.3% | 10.182 / 16.639 / 176.193 |
| 左臂右指 | 88.7% | 88.7% | 9.699 / 40.366 / 270.562 |
| 右臂左指 | 80.0% | 79.3% | 9.794 / 16.408 / 307.565 |
| 右臂右指 | 86.0% | 85.3% | 9.833 / 151.389 / 205.686 |

四侧同一序号、同一采样帧达标为 `81/150`。最长四侧同时达标连续观测段为 6 帧、约 0.10 s（`28.48–28.58 s`）。最长单侧零接触段为右臂左指 4 帧、约 0.06 s（`26.84–26.90 s`）；其余指侧最长 0.02–0.04 s。左右 trace 时间差 median/p95/max 均为 0 ms；采样间隔固定 20 ms；两侧各 150 个不同序号，无重复、无缺失的 trace 帧。序号步长为 10 是 trace 每 10 个更新序号采一次，不表示丢了 9 条 trace。采样间隔内的物理接触变化仍不可见。

异常峰值是保留的原始按侧累加峰：单接触的分量过滤上限为 150 N，但多个有效接触可相加超过该值。峰值不能当作稳定承载力。task 在失败边界记录的四侧法向力约 `(10.260,9.393;10.597,9.287) N`，随后零延迟诊断变成四侧接触，进一步表明接触状态短时切换。

gate `feedback()` 对每臂分别要求反馈接收年龄 ≤0.25 s、物理时间和序号前进，并对重复无进展超过 0.25 s 失败；它没有要求左右 timestamp 对齐，也没有要求每次 gate 轮询都拿到新序号。故 `good_samples` 日志计数可能把仍新鲜的缓存快照重复累计，但放行条件实际检查 `min(left_sim,right_sim)` 跨度 ≥0.25 s，单纯重复计数不会独自使 gate 通过。当前 trace 窗口没有观测到左右异步，但 gate 内部没有跨臂 skew 上限，因此无法排除低于 20 ms 采样间隔的异步拼接。可见间歇与不足 0.25 s 的最长四侧连续段支持真实接触间断是主要失败来源；不能仅凭有限采样把它定为唯一原因。

力请求没有证据显示错配：任务配置请求保持 10 N/指、运行上限 ±20 N；trace 末段为 actuator `-17.69` / `-17.76 N`、限幅 20 N、保持请求 10 N。action 初始化中的临时 ramp 不能代表整个保持阶段。

**下一轮唯一建议：**保留夹爪 action 的稳定窗口，把额外 0.25 s dwell 改成“两个 action 均成功后，要求一次新鲜、左右时间差 ≤20 ms 的四侧接触且每侧法向力 ≥0.25 N 快照”，不要求连续保持 0.25 s。本轮不实现、不以离线回放宣称承载成功。代价是减少因短暂间歇导致的误拒绝，但不再证明 gate 时间窗内的持续接触；抬升结果仍需真实受控悬空验证。

### MMC watchdog 接口与边界

核对当前 watchdog：首次 reference 等待 2 s，收到首个目标后超时 0.5 s；故障锁存并仅报一次。内层 controller 继续以最后有效 Cartesian 目标运行控制律、每周期重新计算 torque，这不等于保持最后力矩或已证明安全保持。`MultiModeController::update()` 第一次发现内层错误返回 `ERROR`，后续为了不刷屏返回 `OK`。本机 ROS Humble `ros2_control_node` 控制循环调用 manager 的 read/update/write、但忽略 update 返回值；因此这一实现里不会因这一次 `ERROR` 自动停 MMC。外层生命周期状态仍需由 `/controller_manager/list_controllers` 读取。

扩展已有 `~/get_controllers` 状态响应，带 `faulted`、来源、单调时钟时间/序号和状态时间；bridge 同时查询该接口和 controller manager。状态查询按 50 ms 周期、每个同步响应最多等待 50 ms，缺服务、超时、状态过期、MMC 非 active 或锁存故障一律失败闭锁。故障后当前 bridge 立即停止后续参考发布并以非零状态退出，不再自动切换或重发目标；正常完成的 JTC 回切仍是 STRICT，已有流程在切换后从实测 `/joint_states` 构造 seed。故障分支没有自动切 JTC，也未声称安全恢复：MMC 留在 manager 当前状态，最后有效目标的控制律可能继续工作，受控 JTC 恢复和接触载荷下的保持/退出仍待独立设计与验证。

针对性构建：`colcon build --base-paths src/multipanda_ros2 --packages-up-to dual_arm_carry_task multi_mode_controller_impl --parallel-workers 2 --cmake-args -DMUJOCO_DIR=/home/lidio/Libraries/mujoco`，8 包通过；编译日志 `/tmp/watchdog_gate_build_20261020.log`。已重新 source `install/setup.bash`，确认安装的 `GetControllers.srv` 带新字段。`test_controller_fault_latch` 与既有 `test_torque_limits` 定向 CTest 均通过。测试覆盖正常健康、watchdog 锁存、失联/过期、manager 已停用和故障后不能恢复为健康。另一次 `colcon test-result --verbose` 汇总了整个 build 目录历史 lint 结果（非本次运行测试），未据此改全包 lint。未启动 MuJoCo；故障恢复未做物理验证；无提交或推送。
