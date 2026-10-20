# 本机 bilateral gate 与 MMC HOLD 接入状态（2026-10-20）

## 判据与实现

提交仍为 `1161a7a`，工作区保留此前所有修改。gate 现在在双侧夹爪 action 成功后记录各自完成序号，并在原 3 s 有界等待内接受一次合格快照：两侧序号各自大于完成栅栏、反馈新鲜且有效、左右仿真时间差不超过 20 ms、四侧均接触且法向力均不低于 0.25 N。日志记录判据类型、序号、时间差、四侧接触/力和结论。该判据减少间歇接触导致的额外拒绝，不证明连续接触。

| 本机运行 | bilateral 判据 | 物理结果 |
|---|---|---|
| `contact_hover_rigid_lidio_20261020_0932`（旧） | 连续维持 0.25 s；夹爪 action 成功，但 gate 未通过 | 未抬升；保持为旧失败记录 |
| `contact_hover_gate_snapshot_jtc_lidio_20261020_01_retry1`（本轮 A） | 单次新鲜快照；左右 seq `12662/12656`，完成栅栏 `12361/12356`，时间差 `12 ms`；四侧接触，法向力 `10.423/9.420/10.224/9.615 N` | gate、JTC 抬升/保持/下降/释放全链通过 |

## 构建、测试和运行

构建命令：

```bash
rtk colcon build --base-paths src/multipanda_ros2 --packages-up-to dual_arm_carry_task multi_mode_controller_impl --parallel-workers 2 --cmake-args -DMUJOCO_DIR=/home/lidio/Libraries/mujoco
```

8 个依赖包构建通过，包括 `multi_mode_control_msgs`、`multi_mode_controller`、`multi_mode_controller_impl`、`franka_hardware`、`franka_moveit_config` 和 `dual_arm_carry_task`。构建后在运行 shell 重新 source 了 `/opt/ros/humble/setup.bash` 与 `install/setup.bash`。

定向测试通过：`test_bilateral_snapshot_gate` 7 项、`test_controller_fault_latch` 4 项、`test_torque_limits` 5 项；wrapper 脚本 `bash -n` 与 `git diff --check` 通过。gate 测试覆盖合格快照、action 失败、旧/重复序号、时间差超限、缺接触、力不足/非有限与过期反馈。

唯一有效物理运行使用本机 `/home/lidio/Libraries/mujoco`、ROS domain 113、独立 `ROS_HOME=/tmp/contact_hover_gate_ros_home_lidio_20261020_01_retry1`，固定场景 `(0.50, 0.00, +10°)`、`CONTACT_PROBE_RIGID=true`、`CONTACT_PROBE_ADAPTIVE_FORCE_HOLD=false`、hover 50 mm/3 s。首次 run 在任何任务动作前因沙箱禁止 DDS 网卡/UDP 而无法发现服务，错误目录 `_01` 保留；确认原因后仅重启一次，独立目录 `_01_retry1` 成功。第二次实例由 wrapper 有界回收。

有效运行命令：

```bash
env -u CONTACT_PLAN_COMPARE -u CONTACT_TRANSPORT_PLAN_ONLY -u CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY \
  CONTACT_PROBE_RUN_ID=contact_hover_gate_snapshot_jtc_lidio_20261020_01_retry1 \
  CONTACT_PROBE_DOMAIN_ID=113 MUJOCO_DIR=/home/lidio/Libraries/mujoco \
  CONTACT_PROBE_RIGID=true CONTACT_PROBE_ADAPTIVE_FORCE_HOLD=false \
  CONTACT_PROBE_HOVER_ONLY=1 CONTACT_PROBE_HOVER_HEIGHT_M=0.05 \
  CONTACT_PROBE_HOVER_HOLD_SECONDS=3.0 CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY=0 \
  ROS_HOME=/tmp/contact_hover_gate_ros_home_lidio_20261020_01_retry1 \
  bash scripts/run_contact_probe_local.sh --headless
```

数据目录：`experiment_results/a1_runs/contact_hover_gate_snapshot_jtc_lidio_20261020_01_retry1/`。保留了任务和仿真日志、双侧接触/夹爪 CSV、hover diagnostics、分析 JSON/TXT、控制器状态与 runner 状态。运行入口所用 SRDF 和控制器 YAML 来自 `install/franka_moveit_config`；控制器清单显示 JTC active，MMC 未加载。运行参数日志确认刚性模型、adaptive force hold=false、请求 10 N/指。

## 物理结果

- 功能链：truth、P2a/P2b、预抓取、下降、闭合 action、bilateral gate、JTC 抬升、保持、下降、开爪释放全部通过，最终任务状态 `DONE`。释放日志为 `open=true force_hold=OFF weld=OFF attach=none`。
- 约束：日志全程 `weld=OFF rod_constraint=OFF`；hover analysis 的 `constraints_active_samples=0`。
- 抬升/保持：杆峰值升高 `46.289 mm`，保持 `3.040 s`，保持结束升高 `43.309 mm`，峰后下降 `2.979 mm`。离地确认和既有 20 mm/2°保护均未放宽。
- 闭合后原参考残差：左右最大平移残差分别 `7.124 mm / 8.197 mm`，最大姿态残差 `1.383°`。HOLD 入口平移残差为 `2.531 mm / 2.830 mm`；HOLD 内残差模长最大增加约 `4.594 mm / 5.367 mm`。这些是原参考下的残差变化，不等价于局部 pad 滑移。功能通过，5 mm 性能目标未达。
- HOLD 采样：50 对观测、仿真跨度约 `3.02 s`；左右反馈时间差中位数 `9 ms`、最大 `18 ms`。两指接触采样占比左 `88%/84%`、右 `84%/92%`。各法向力中位数约 `9.76–9.94 N`；原始统计保留了超过 100 N 的 5 行读数及最高约 `282.8 N` 的单侧峰值，未删尖峰或改滤波。

## MMC HOLD 接入与 B 结果（2026-10-20）

工作树基于 `1161a7a`，保留此前全部修改。新增默认关闭的 `contact_hover_control_mode=jtc|mmc_hold`，经本机 wrapper、scene runner、launch 和节点参数贯通；MMC 模式只影响 HOLD。wrapper 在仿真内将 `sim_multi_mode_controller` 加载为 inactive，核验 JTC active 后才启动任务。接触、JTC 抬升和离地保护不变。HOLD 入口先记录杆真值及双手原参考残差，再启动独立 bridge 子进程；bridge 从实测 `O_T_EE`/q（各自 link0）初始化，STRICT 停 JTC/启 MMC，以仿真时钟发布静止 pose、零 twist 和实测冗余构型满 3 s，再 STRICT 回切 JTC，并用新鲜 `/joint_states` 实测状态 seed。抓手不参与控制器切换，闭合后的原残差基准未重置。

保护循环在任务线程逐样本运行，bridge 等待切换期间不会占用任务线程。生产与测试共用的 `runProtectedChild` 会在保护失败时停止并回收 bridge；非零 bridge 退出和有界超时均锁存任务失败，失败路径不进入下降/释放，wrapper 随后停止本轮 task/runner 和唯一仿真实例。bridge 限时最多 20 s，父级 HOLD 有界；`readMujocoRodTruth()` 内部服务发现和响应等待各最多 2 s，代码级最坏总阻塞约 4 s，因此不能将保护描述成硬实时。hover CSV 记录实际观测间隔、单次 observe 用时；B 观测到的最大间隔约 `62 ms`，最慢一次 observe 用时 `421 ms`。运行时若发生服务异常，应按日志报告实际间隔及其风险。

构建先以 `colcon build --base-paths src/multipanda_ros2 --packages-up-to dual_arm_carry_task franka_hardware franka_moveit_config multi_mode_controller_impl --parallel-workers 2 --cmake-args -DMUJOCO_DIR=/home/lidio/Libraries/mujoco` 构建 11 个依赖包通过；最终 task 重建也通过，安装 bridge 存在。定向 CTest：`test_bilateral_snapshot_gate`、`test_controller_fault_latch`、`test_bridge_process`、`test_torque_limits` 全通过；`bash -n` 和 launch `py_compile` 通过。全局 `colcon test-result` 汇总包含大量历史 lint/copyright 失败，本轮没有处理。

实施后完成一次聚焦独立只读审阅及针对审阅意见的窄范围跟进。审阅没有发现控制器互斥、link0 坐标、夹爪保持、保护并行或故障收尾的 B 阻断；确认服务最多 4 s 和 CSV 监测间隔须如实报告。测试直接覆盖生产共享的“保护回调运行—失败 stop/reap—非零退出失败—正常退出仍等完成条件”循环。

唯一 B 使用本机 `/home/lidio/Libraries/mujoco`、ROS domain 113、`ROS_HOME=/tmp/contact_hover_ros_home_lidio_20261020`，固定 `(0.50,0,+10°)`，rigid=true、自适应保持=false、10 N/指、命令抬升 50 mm、原 20 mm/2°保护、HOLD 3 s。命令：

```bash
env -u CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY -u CONTACT_PLAN_COMPARE -u CONTACT_TRANSPORT_PLAN_ONLY \
  CONTACT_PROBE_RUN_ID=contact_hover_mmc_hold_lidio_20261020_01 \
  CONTACT_PROBE_DOMAIN_ID=113 CONTACT_PROBE_HOVER_ONLY=1 \
  CONTACT_PROBE_HOVER_HEIGHT_M=0.05 CONTACT_PROBE_HOVER_HOLD_SECONDS=3.0 \
  CONTACT_PROBE_HOVER_CONTROL_MODE=mmc_hold CONTACT_PROBE_RIGID=true \
  CONTACT_PROBE_ADAPTIVE_FORCE_HOLD=false MUJOCO_DIR=/home/lidio/Libraries/mujoco \
  ROS_HOME=/tmp/contact_hover_ros_home_lidio_20261020 \
  bash scripts/run_contact_probe_local.sh --headless
```

运行目录：`experiment_results/a1_runs/contact_hover_mmc_hold_lidio_20261020_01/`。gate 按新定义通过：左右 seq `14866/14860`，action 完成栅栏 `14576/14580`，时间差 `12 ms`，四侧接触，法向力 `10.140/9.552/10.279/9.901 N`。P2a/P2b、预抓取/下降、闭合、gate 和 JTC 抬升通过；JTC 抬升峰值 `46.067 mm`。

| 指标 | 已通过的 JTC A | MMC B（失败，未切换） |
|---|---:|---:|
| HOLD 入口杆升高 | A 记录 `46.289 mm` 峰值；入口原参考残差 `2.531/2.830 mm` | `46.098 mm`；左右原参考残差 `2.540/2.255 mm`，姿态 `0.717°/0.487°` |
| HOLD 内结果 | 保持 `3.040 s`；峰值后下降 `2.979 mm`；最大累计位移残差 `8.197 mm`、姿态 `1.383°`；HOLD 内新增位移残差约 `4.594/5.367 mm` | bridge 在读取初始 arm state 时退出；仅有约 `0.074 s` 的失败前 JTC 观测，杆从 `46.113` 降至 `45.569 mm`；左/右最大累计残差 `3.541/2.322 mm`、姿态 `0.797°/0.541°`，不能算 MMC HOLD 指标 |
| 切换/结束 | JTC 保持、正常回切路径不适用；下降释放通过 | 没有发出切换请求，MMC 保持 inactive、JTC active；无 MMC trace，无正常回切、下降或释放；任务 `ERROR` |
| 约束与数据 | weld/rod constraint 全程关闭 | 有效样本 weld/rod constraint 均 `0/0/0`；夹持请求仍为 10 N/指，失败清理确认夹爪 measured-position stop；释放未执行 |

B 停在 **JTC 抬升完成后、MMC 切换前**。bridge 日志为 `read initial arm state` 失败，退出码 1；当前实现的 `readArms()` 对服务发现只等待 0 ms，且失败未区分服务未发现、请求超时或状态无效。现有 `services_seen.txt` 在 inactive spawner 之前采集，不能单独证明 MMC 配置后的真实服务路径/可用性。下一步应先补充分型日志、在 bridge 内作短且有界的服务发现等待，并在 spawner 后记录实际 `get_robot_states` 服务路径和 JTC/MMC 状态；本轮不重启 B。不是 watchdog 故障，也没有进入 MMC 承载，不能声称 MMC HOLD 通过或有控制效果。

接触和 hover 原始 CSV、任务/MuJoCo 日志、controller/spawner 清单、truth/runner 状态及分析 JSON/TXT 均留在该独立目录。wrapper 已有界回收 task、runner 和本轮 MuJoCo 实例；本轮 bridge 已退出并回收，没有本轮残留发布者。结论：A 功能基线仍有效；gate 按批准判据工作；本次 B 因初始状态服务读取失败而失败，MMC 无有效承载证据。未调参、未重跑、未提交、未推送。

## 状态读取/保护修复与修复后 B（2026-10-20）

工作树仍基于 `1161a7a`，旧 B `contact_hover_mmc_hold_lidio_20261020_01` 完整保留并继续记为失败。本轮未改 gate、接触/夹持、模型、增益或保护门槛。状态 bridge 改用任务节点 `/dual_arm_carry_task/current_robot_states`：服务从同一条新鲜 `/joint_states` 样本构造双臂 q，并以 MoveIt FK 生成各自 `link0^-1 * link8` 位姿；左右顺序固定。服务要求 steady 到达年龄不超过 250 ms、仿真时间戳相对当前仿真时钟不旧于 0.5 s，bridge 另核对响应 wall 时间戳和与其最新 joint sample 的仿真时间差。抬升前执行真实 service request 预检，有 3 s 请求预算、5 s 子进程总预算；预检状态不缓存给抬升后的初始化。

杆真值保护路径改为单个在途异步请求：无新数据时不复用旧值，2 s 数据年龄/请求超时保持原限值并锁存失败；迟到响应不恢复 guard。hover 数据记录请求耗时、真值年龄、最大监测循环间隔和单次 observe 用时。定向重建 `dual_arm_carry_task` 通过；测试通过 `test_controller_fault_latch`、`test_bridge_process`、`test_mmc_state_service`（含延迟发现/响应、超时、非法与过期状态、近期到达但旧仿真戳、执行器响应处理和停止回收）及 `test_torque_limits`。修复后进行了窄范围独立只读跟进审阅：无阻断项；审阅指出的旧仿真戳缺口已补并有回归测试。审阅说明该回归用例直接测试生产校验函数，没有端到端构造 ROS JointState 穿过实际服务回调，作为非阻断边界记录。

修复后唯一 B 使用新目录 `experiment_results/a1_runs/contact_hover_mmc_hold_lidio_20261020_02/`，命令如下；没有重跑 A 或旧 B：

```bash
env -u CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY -u CONTACT_HOLD_DIAGNOSTIC_ONLY \
  -u CONTACT_PLAN_COMPARE -u CONTACT_TRANSPORT_PLAN_ONLY \
  CONTACT_PROBE_RUN_ID=contact_hover_mmc_hold_lidio_20261020_02 \
  CONTACT_PROBE_DOMAIN_ID=113 CONTACT_PROBE_HOVER_ONLY=1 \
  CONTACT_PROBE_HOVER_HEIGHT_M=0.05 CONTACT_PROBE_HOVER_HOLD_SECONDS=3.0 \
  CONTACT_PROBE_HOVER_CONTROL_MODE=mmc_hold CONTACT_PROBE_RIGID=true \
  CONTACT_PROBE_ADAPTIVE_FORCE_HOLD=false MUJOCO_DIR=/home/lidio/Libraries/mujoco \
  ROS_HOME=/tmp/contact_hover_ros_home_lidio_20261020_02 \
  bash scripts/run_contact_probe_local.sh --headless
```

抬升前实测状态预检成功，service 为 `/dual_arm_carry_task/current_robot_states`，左右样本仿真时间均 `28.932 s`，与 bridge joint sample 一致，请求加发现耗时 `1.046 s`。bridge 日志中的 `source=SwitchEngine.robot_data` 是遗留硬编码文本，不能代表实际服务来源；实际响应由上面的 task-side joint feedback/FK 服务生成。杆峰值升高 `46.048 mm`（峰值中心 Z `0.066852 m`，基线约 `0.020804 m`）；离地保护通过，进入 HOLD 入口时升高 `45.408 mm`。任务侧持续保护观测期间最大循环间隔 `52 ms`，最大单次 observe `2 ms`，最后记录真值请求 `0.059 ms`、年龄 `0.175 ms`。监测阶段累计 `9.278 s`，它包含 bridge 等待时间，**不是 MMC 实际保持时间**。

bridge 在等待目标 topic `mj_left_and_mj_right/dual_cartesian_impedance_controller/desired_pose` 的订阅端时，约 9.2 s 后以 `impedance goal subscriber never appeared` 退出。它尚未发出 JTC/MMC 切换请求，也没有到达抬升后的新状态读取；控制器清单保持 JTC active、MMC inactive。因此没有进入 MMC、实际 MMC HOLD 时间为 `0 s`，没有正常回切、下降或释放。MMC_HOLD 监测标签覆盖的这段 JTC 等待开始于杆升高 `45.408 mm`、左右原抓取参考位移残差 `2.930/3.567 mm`、姿态残差 `0.934°/0.612°`；截至失败，最大累计位移残差 `12.962/14.683 mm`，最大姿态残差 `2.002°`。任务保护因此以姿态残差略超 `2°` 的既有门槛报 `E_CONTACT_SLIP_OR_DROP`，确认夹爪位置停止，任务返回失败。失败路径没有走正常释放。原始接触数据仍显示约 113 N 的单侧法向力峰值，未滤除；weld 与 rod constraint 记录均为关闭。

这次证明了抬升前服务发现、真实请求、executor 响应、双臂顺序/坐标/时间校验以及异步保护循环可用；**它是 MMC 接线失败，不能称为悬空承载或 HOLD 验证**。下一轮若要继续，需先修复 inactive MMC 阶段目标订阅者不可见/bridge 等待与切换顺序的具体接线问题，再单独安排验证；本轮不追加试跑。runner 已回收 task、runner 与单个 MuJoCo 实例；bridge 已由 task stop/reap 回收，复查无本轮遗留进程。旧 B 仍失败，本次 B 独立记录为失败；未提交、未推送。

## MMC 激活顺序修复与新 B 尝试（2026-10-20）

工作树仍基于 `1161a7a`，此前未提交修改与旧运行目录均保留。修复了 bridge 的生命周期循环：抬升前只核验 MMC 已加载为 inactive、JTC active、状态与切换服务可请求，并核对目标话题和消息类型；不再等待 inactive 控制器的运行期订阅。抬升后在保护运行期间读取新鲜双臂状态，再以单一 1.5 s 截止预算完成 STRICT 切换、实际 active 互斥核验、激活后订阅发现、首条实测 pose/q 与零 twist 参考发布及接收序号确认。1.5 s 小于现有 2 s watchdog 首目标宽限；仅订阅计数不视作控制器已接受参考。状态来源日志更正为 task joint feedback + MoveIt FK。

构建 `colcon build --base-paths src/multipanda_ros2 --packages-up-to dual_arm_carry_task multi_mode_controller_impl --parallel-workers 2 --cmake-args -DMUJOCO_DIR=/home/lidio/Libraries/mujoco` 通过，覆盖 8 个依赖包；重新 source 安装空间后，`test_mmc_activation_handshake`、`test_mmc_state_service`、`test_controller_fault_latch`、`test_bridge_process` 和 `multi_mode_controller/test_torque_limits` 均通过。假接口测试从 MMC inactive 且无订阅开始，仅在模拟激活后创建订阅，确认顺序、首参考确认、切换/互斥失败、订阅超时、保护中止和无接收序号时拒绝继续。一次窄范围独立只读审阅未发现阻断项；它确认生产 handshake 使用相同顺序。审阅边界：假接口测试不测真实 DDS 延迟，需由物理 B 记录实测时序。

按授权启动的唯一新目录为 `experiment_results/a1_runs/contact_hover_mmc_hold_lidio_20261020_03/`，本机 MuJoCo `/home/lidio/Libraries/mujoco`、ROS domain 113、rigid=true、自适应保持=false，固定场景 `(0.50, 0.00, +10°)`，请求抬升 50 mm、HOLD 3 s；wrapper 和仿真均只启动一个实例。truth 与 P2a 静态检查通过（几何/IK/碰撞均通过），但 P2b 联合预抓取 PTP planning action 被 MoveIt abort，返回 error code `-2`，`p2_acceptance=false`。task/wrapper 明确报告 `E_P2B_PATH_FAILED` 并在任何预抓取执行前退出；抓取下降与闭合均未执行。该失败不是 MMC 激活失败：本次没有到达抬升前接口预检，故真实服务就绪流程和真实控制器激活/首参考接收仍未由仿真运行验证；它也不是一次有效 MMC B，MMC 是否进入 active、HOLD 时长、回切和释放均不适用。未更换起点、未放宽规划/碰撞条件，也未重试。

该目录保存 `prepare_status.json`、manifest、`task_launch.log`、`sim_dual_moveit.log`、`runner.log`、控制器清单、hover 分析和空接触/夹爪 trace；`hover_analysis.txt` 明确标记无接触样本且建议检查 P2b。wrapper 已确认回收 task、runner 与唯一仿真实例；复查没有本轮 bridge、task 或 MuJoCo 残留进程。结论：激活顺序修复通过构建、假接口测试和窄独立审阅；本次新 B 被 P2b 规划拒绝挡在物理动作前，MMC 集成尚未得到真实运行验证。旧 A/B 结果不变。本轮未提交、未推送。

## P2b live 碰撞与起点状态诊断（2026-10-20）

旧失败 `contact_hover_mmc_hold_lidio_20261020_03` 的 MoveIt live pipeline 确实先生成 29 点 PTP 轨迹，再因第 0–3 点最终碰撞验证失败而拒绝；日志逐次报告 `mj_right_rightfinger ↔ mj_right_link5` 和 `mj_right_link5 ↔ mj_right_link7`。MoveIt 明确写明 contact information 未存储，旧 run 也没有完整 q 路径，因此无法对旧第 0–3 点做同状态重放，旧接触位置/深度未知。

用单一新 MuJoCo/MoveIt 实例做了 `prepare_only + p2b_check` 诊断，没有启用 `pregrasp_execute`，没有发送手臂、夹爪或 weld 目标。命令：

```bash
source /opt/ros/humble/setup.bash
MUJOCO_DIR=/home/lidio/Libraries/mujoco ROS_HOME=/tmp/p2b_diag_ros_home_lidio_20261020_01 \
  bash scripts/a1_run_scene.sh --run-id p2b_collision_diag_lidio_20261020_01 \
  --domain-id 114 --prepare-mode --p2b-check --x 0.50 --y 0.0 --yaw-deg 10.0 \
  --headless --ready-timeout 120 --task-timeout 300 --node-timeout 360
```

实际入口配置来自安装空间：任务参数 `install/dual_arm_carry_task/share/dual_arm_carry_task/config/a1_scene_sampling.yaml`，MuJoCo 模型 `install/franka_description/share/franka_description/mujoco/franka/dual_scene.xml`，手 mesh `.../assets/hand.stl`；runner 从 `install/franka_moveit_config` 解析生成 SRDF。P2b 当前起点来自 `/joint_states` 的 14 个 arm joints；新鲜样本年龄 `0.106 ms`，按名称与 MoveIt live CSM 的 arm q 一致到日志精度，和 P2a 记录状态最大差约 `1.13e-7 rad`。四指不在该 `/joint_states` 样本内；typed hardware 反馈新鲜（左 `seq=7025, age=19.2 ms`，右 `seq=7030, age=12.5 ms`），四值均约 `0.0004945 m`。P2a/P2b 记录起点原来把四指留作默认 `0`，因此与实测值不一致，虽然差值仅约 `0.495 mm`。

诊断把同一当前状态放入两模型：live 原机器人碰撞几何，以及同 world/ACM 的 criterion `mujoco_hand_aabb` 模型。起点两者均无碰撞、无接触项，故该点无位置/穿透值。新规划返回 24 点，point 0 相对 fresh/live arm q 最大差 `8.51e-8 rad`，criterion 全路径检查 81 个状态、碰撞 0；MoveIt live 的最终规划验证也通过。这是本次新生成路径，不是旧 29 点失败路径；旧状态差异仍无法确定。新路径与旧路径点数、轨迹不相同，不能凭这次成功宣布旧碰撞已消失或靠重复规划筛选成功。

据确认的四指来源缺口，P2b 现已将 fresh `/joint_states` arm q 与左右 typed hardware finger q 合并为具名 18 变量起点，显式送入 MoveGroup，并用于 criterion 的路径状态重建；缺项、非有限或超过既有 0.5 s 样本年龄语义的反馈拒绝规划。定向构建通过；`test_a1_sample_validator` 63 项通过，新增用例覆盖 18 维具名映射、缺 arm joint 和过期 finger sample。窄范围代码审阅确认此修复只改变状态来源/初始化，不改变模型、碰撞几何、ACM 或规划阈值。碰撞规则未修改。

| 核对项 | 结果 |
|---|---|
| 旧失败路径第 0 点是否等于实际起点 | 旧路径未保存，不能比较；新诊断 point 0 对 fresh arm q 最大差 `8.51e-8 rad` |
| 四指是否新鲜且一致 | typed hardware 四值均新鲜且互相一致；`/joint_states` 缺四指，旧 P2b/criterion 默认 `0`，本轮已修正其映射 |
| `rightfinger↔link5` | 旧 live 路径报告碰撞；旧接触位置/穿透未知。新诊断同起点 live 与 criterion 均无碰撞；新路径最终 live/criterion 检查均无碰撞 |
| `link5↔link7` | 旧 live 路径报告碰撞；这与四指映射无关，旧路径不可重放。新诊断同起点与新路径均未报告该碰撞 |
| 碰撞规则是否改动 | 否；没有新增 SRDF/ACM 豁免 |
| 是否达到 MMC B 条件 | 否。四指映射缺口已修复，但旧路径另有未解释的 live `link5↔link7` 碰撞；新诊断路径不同且发生在修复前，不能替旧失败消因或作为 MMC B 前置路径证据 |

完整诊断状态和 q 轨迹保存在 `experiment_results/a1_runs/p2b_collision_diag_lidio_20261020_01/p2b_state_diagnostic.csv` 与 `p2b_joint_path.csv`，规划、碰撞、场景来源在同目录日志和 `manifest.json`。原失败目录未覆盖。诊断实例正常回收，本机未发现本轮残留 MuJoCo/MoveIt/task 进程。本轮未运行 B、未改接触或 MMC 参数、未提交、未推送。

## 新 B 运行记录（2026-10-20）

按正式 wrapper 启动唯一新目录 `experiment_results/a1_runs/contact_hover_mmc_hold_lidio_20261020_04/`，固定 `(0.50, 0, +10°)`、刚性模型、50 mm/3 s 请求、MMC HOLD；domain 113，本机 MuJoCo `/home/lidio/Libraries/mujoco`，自适应夹持关闭。启动前定位实际包前缀 `install/dual_arm_carry_task`；安装程序 symlink 指向本工作树 `build/`，包含上一轮具名四指映射和激活握手实现。定向构建 `colcon build --base-paths src/multipanda_ros2 --packages-up-to dual_arm_carry_task --parallel-workers 2 --cmake-args -DMUJOCO_DIR=/home/lidio/Libraries/mujoco` 完成，覆盖 `multi_mode_control_msgs` 与 `dual_arm_carry_task`；wrapper 会重新 source 安装空间。

```bash
rtk proxy bash -lc 'env -u CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY -u CONTACT_HOLD_DIAGNOSTIC_ONLY -u CONTACT_PLAN_COMPARE -u CONTACT_TRANSPORT_PLAN_ONLY -u CONTACT_PROBE_HOVER_CONTROL_MODE -u CONTACT_PROBE_HOVER_HEIGHT_M -u CONTACT_PROBE_HOVER_HOLD_SECONDS -u CONTACT_PROBE_HOVER_ONLY -u CONTACT_PROBE_RIGID -u CONTACT_PROBE_ADAPTIVE_FORCE_HOLD CONTACT_PROBE_RUN_ID=contact_hover_mmc_hold_lidio_20261020_04 CONTACT_PROBE_DOMAIN_ID=113 CONTACT_PROBE_HOVER_ONLY=1 CONTACT_PROBE_HOVER_HEIGHT_M=0.05 CONTACT_PROBE_HOVER_HOLD_SECONDS=3.0 CONTACT_PROBE_HOVER_CONTROL_MODE=mmc_hold CONTACT_PROBE_HOLD_DIAGNOSTIC_ONLY=0 CONTACT_PROBE_RIGID=true CONTACT_PROBE_ADAPTIVE_FORCE_HOLD=false MUJOCO_DIR=/home/lidio/Libraries/mujoco ROS_HOME=/tmp/contact_hover_ros_home_lidio_20261020_04 bash scripts/run_contact_probe_local.sh --headless'
```

本次 P2a 静态几何、IK、碰撞检查通过；P2b 在路径规划前拒绝，错误为 `E_P2B_START_FEEDBACK_INVALID`：`mapMeasuredFingerPositions` 认为 typed hardware 输入 `mj_left_finger_joint1` 非有限或超出 `[0, 0.0401] m`。启动状态 CSV 只有表头，左右原始夹爪 trace 也只有表头；失败分支没有记录四个 typed 数值及序号/年龄，因此只能确认校验拒绝，无法区分具体数值、消息/字段映射或异常来源。没有生成 P2b 关节路径；`p2b_state_diagnostic.csv` 只有表头。日志中的“记录值与当前 live 状态最大关节差 0”是规划前状态比较，不能替代 18 变量新鲜状态映射成功的证据。

| 本次检查 | 结果 |
|---|---|
| P2b 新鲜实际起点与规划第 0 点 | 未调用规划器、无路径第 0 点；实际 arm+finger 完整起点映射未通过，不能判为一致 |
| 四指 typed 来源 | 反馈未能通过有效性检查；数值/序号/年龄未落盘，具体异常未明 |
| 路径及碰撞验证 | 未生成路径；本次不是碰撞拒绝，与旧 link5↔link7 历史失败无关 |
| MMC 激活/首参考/HOLD | 未执行；MMC inactive、JTC active（见 `controllers.txt`）；保持时间 0 s |
| 抬升、杆高度、姿态、约束、回切和释放 | 均未进入；无接触/抬升数据，不适用 |
| 收尾 | wrapper 返回 5；task/runner 已退出，唯一 sim 已 reaped；复查无本轮 task、bridge、MoveIt/MuJoCo 残留 |

本次是起点反馈校验失败，不声称 P2b 碰撞通过或旧碰撞问题已修复。没有换解、重规划或启动第二次 B；保留完整 task/sim/wrapper 日志、`prepare_status.json`、控制器清单、状态诊断空表和分析文件。下一次运行前需要让 P2b 失败分支记录 typed 四指原始数值、序号、时间戳/到达年龄及具体校验项，以便明确修复原因；本轮到此停止。

## P2b 四指入口诊断补充（2026-10-20）

硬件发布链核对：MuJoCo physics thread 从对应 gripper joint qpos 写入 `GripperSharedState.finger_q1/q2`；`readFeedback()` 只在 `feedback_seq>0` 且 q1/q2 与 command 有限时令 `GripperFeedback.valid=true`；`publishGripperState()` 对无效快照不发布，并将 q1/q2 原值分别写入 `GripperContactFeedback.finger_q1/finger_q2`。消息没有 `valid` 字段；任务订阅 `receiveGripperContact()` 按同名字段直接复制，P2b 数组顺序为左 q1、左 q2、右 q1、右 q2，随后按 joint name 映射。运行时 `ros2 interface show` 与源码 `.msg` 一致。URDF 四指边界均为 `[0,0.04] m`；当前验收范围仍为 `[0,0.0401] m`，即保留既有 `0.1 mm` 上侧余量，没有改阈值。

校验前现在总是落盘 `p2b_finger_input_diagnostic.csv` 和 `p2b_finger_raw_values.csv`：含原始值（17 位有效数字；缺样本写 `NA`）、反馈存在/序号/simulation time/header stamp/steady 接收时刻与年龄、左右时间差、source arm 与字段顺序、有限/有效状态、typed 消息没有 valid 字段这一事实，以及 live/criterion 模型边界和 `missing/stale/mapping_mismatch/nonfinite/below_lower_bound/above_upper_bound/valid` 分类。失败日志同时带首个拒绝关节、未经修改的原值和分类。

上次 B `contact_hover_mmc_hold_lidio_20261020_04` 的具体 q 数值无法恢复：其原始 trace 只有表头，诊断修改前的失败分支没有写原始值。错误说明校验拒绝左 q1 为非有限或越界，但不能再精确到数值/方向。新单测覆盖四个具名关节映射、无消息默认态、stale、NaN/Inf、负值、上界超限、mapping mismatch，以及失败 raw CSV 的非裁剪序列化；`test_a1_sample_validator` 65/65 通过。源码构建通过。`colcon test-result` 的包级总览还包含既有 copyright/cpplint/flake8/pep257/uncrustify 失败；本轮用直接 GTest 二进制验证了目标测试，没有处理这些无关 lint。

一次采样器启动曾误用 Conda Python 3.13，无法加载 ROS Humble Python 3.10 扩展；run `p2b_finger_input_diagnostic_lidio_20261020_01` 保留该基础设施问题。该次 task 仍记录到四个合法 typed q；同状态 live/criterion 检查报告自碰撞，随后 P2b planner 返回 `-2`，与四指有效性拒绝不同；由于 sampler 未启动，不把该目录算作连续采样。确认用 `/usr/bin/python3` 可导入 `rclpy` 后，独立目录 `p2b_finger_input_diagnostic_lidio_20261020_02` 完成 5 s 只读采样（300 条，左右各 150 条）及 plan-only P2b，未执行任何 arm/gripper/weld 目标。采样 q 全部有限、`0≤q≤0.0401 m`；每侧开头 15 条重复 seq=1、simulation_time=0 的零位初值，左侧约在仿真 `0.022 s`、右侧约 `0.008 s` 开始更新到约 `0.0004945 m`，属于初始化暂态且本身仍在模型限位内。其后没有出现越界。P2b 快照四值分别为 `0.0004945204169 / 0.0004945416205 / 0.0004945203954 / 0.0004945416420 m`，左右 seq `6183/6190`、仿真时间 `12.362/12.376 s`、接收年龄 `18.8/5.0 ms`；数组顺序与 source arm、字段名及 joint name 一致。该时刻全局 `/joint_states` 未含四指，不以默认值替代 typed 测量。新鲜 18 变量起点映射通过，24 点路径的 P2b 最终检查通过；这是 plan-only，不是运动或 MMC 验证。

| 问题 | 本轮结论 |
|---|---|
| 旧失败实际值 | 不可恢复；旧错误只保留了 joint 名和泛化分类 |
| 本轮是否重现越界/非有限 | 5 s 观测未重现；观察到的仅是短暂零位初值，仍合法 |
| 物理反馈→消息→数组一致性 | q1/q2 原值逐字段贯通；左右及四指 joint 映射通过，采样值与 P2b 快照对应 |
| 修复后入口是否通过 | 是；新鲜 typed 四指 + 14 arm joints 组成 18 变量起点，P2b plan-only 通过 |
| 是否调整容差 | 否。保留 `[0,0.0401] m`；当前证据不支持扩大模型/规划容差。若未来 raw log 显示有限值仅略超 URDF `0.04 m`，先记录方向、幅度与持续时间，再由规划器决定 |

结论取舍：本轮关闭诊断缺失并验证当前入口链路可用；旧 run 的单次拒绝值仍未知，因此不宣称该历史样本根因已查明，也不因未复现而改容差。下一轮可直接推进 MMC；若入口再次拒绝，新诊断文件会保存可判定的原始值和来源。本轮仿真均已回收；未运行完整 B、未改 gate/夹持/碰撞/控制或保护阈值，未提交、未推送。

## 新 B 运行记录（2026-10-20，P2b 输入拒绝）

使用正式 `scripts/run_contact_probe_local.sh --headless` 启动独立 run `contact_hover_mmc_hold_lidio_20261020_05`；domain 113、`/home/lidio/Libraries/mujoco`、场景 `(0.50, 0, +10°)`、rigid=true、自适应保持=false、请求 50 mm/3 s MMC HOLD。安装任务节点 symlink 指向当前 `build/dual_arm_carry_task/dual_arm_carry_task_node`，包含本轮四指原始诊断。wrapper 单实例启动 MuJoCo，并加载 `sim_multi_mode_controller` 为 inactive；`dual_panda_arm_controller` 是唯一 active 控制器。

P2a 静态检查通过。P2b 读取到新鲜双侧 typed feedback：左右 seq 均为 6905、simulation time 均为 13.806 s，年龄分别 12.641/13.152 ms，左右接收时差 0.511 ms；字段和数组顺序为 `left.q1,left.q2,right.q1,right.q2`，映射名称一致。原始值：左 q1 `-1.5315476842799908e-08 m`、左 q2 `+1.5315476842799905e-08 m`、右 q1 `-1.5315530607144407e-08 m`、右 q2 `+1.5315530607144407e-08 m`。两侧 q1 均被既有下界分类拒绝为 `below_lower_bound`（下界 0 m）；未裁剪、未扩大容差。14 个手臂关节的规划起点对比最大差为日志精度下 `0 rad`，但四指校验失败，因此完整 18 变量起点未被接受，也没有生成可执行 P2b 路径。直接阻断项是两个约 `−1.53e-8 m` 的 q1 负值；本次数据本身不足以判定它是物理反馈微小越界还是数值/映射问题。

本次在 P2b 拒绝后、任何预抓取运动前停止；没有接触闭合、gate、JTC 抬升或 MMC 切换，因此首参考/HOLD/回切/下降/释放均未发生，weld 与杆约束未进入承载流程。没有重规划或重试。原始数据见本目录 `p2b_finger_input_diagnostic.csv`、`p2b_finger_raw_values.csv`、`p2b_state_diagnostic.csv`、`task_launch.log`、`sim_dual_moveit.log`、`controllers.txt` 与 `script_status.json`。wrapper 已回收 task、runner 和唯一 MuJoCo 实例；复查无本轮任务/bridge/simulator 残留。结论：这是四指起点下界拒绝，非 MMC 接入结果，不计为 MMC B 通过；本轮停止，不改容差、不提交、不推送。

## 四指数值容差与修复后 B（2026-10-20）

依据规划器批准的口径，四指硬件反馈接受范围改为 `[-1e-6, 0.0401] m`：下侧 1 µm 仅覆盖数值范围，B_05 中约 `−1.53e-8 m` 的轻微负值远小于该容差；上界不变。范围仅作用于四指共享映射和 P2b 反馈分类，不改变 URDF/MuJoCo 限位、硬件状态、碰撞状态、机械臂关节边界或非有限/缺失/过期拒绝语义。负值原样映射，日志/CSV 区分 `valid`（名义范围 `[0,0.0401]`）和 `accepted_with_numeric_tolerance`；P2b 碰撞查询仍使用原始四指值。

定向构建 `colcon build --base-paths src/multipanda_ros2 --packages-up-to dual_arm_carry_task --parallel-workers 2 --cmake-args -DMUJOCO_DIR=/home/lidio/Libraries/mujoco` 成功，覆盖 4 个依赖包；重新 source 后 `test_a1_sample_validator` 67/67 通过。测试覆盖 B_05 四个原始值映射后不变、`−1e-6 m` 接受、低于该边界拒绝、`0.0401 m` 上界保持、超上界拒绝，以及 NaN/Inf、缺失、过期反馈仍拒绝。

修复后唯一 B 使用正式 wrapper，run `contact_hover_mmc_hold_lidio_20261020_06`，场景 `(0.50,0,+10°)`、domain 113、本机 MuJoCo、rigid=true、自适应保持关闭、请求 50 mm/3 s。P2b 原始 q1 为左 `−1.6156511378463364e-08 m`、右 `−1.6156561293347057e-08 m`，均分类为 `accepted_with_numeric_tolerance`；q2 均约 `+1.61565e-08 m`，名义范围内。反馈新鲜、映射次序一致，手臂+四指 `18/18` 变量进入规划起点；新生成 25 点 P2b 路径通过 83 个状态的既有碰撞验证。随后预抓取、下降到抓取位姿、真实接触闭合及 gate 均到达；gate 单次快照结果为 pass，四侧接触成立、法向力约 `9.635/9.965/9.951/9.821 N`，约束诊断为 weld `(0,0)`、rod constraint `0`。

任务在计划抬升执行前由 bridge 返回 `E_MMC_STATE_PREFLIGHT_REJECTED`。启动清单记录 JTC active、MMC inactive；bridge 仅记录 `preflight expected JTC active and MMC inactive`，没有输出 `[MMC_ACTIVE_VERIFY]` 观测行，因此它未能确认运行期初始互斥状态，日志不足以区分控制器状态服务未就绪/响应失败与实际状态变化。没有发起切换、发送首参考或进入 MMC HOLD；没有杆升高、回切、下降/释放结果，故不是 MMC 承载通过。失败清理确认夹爪停止服务响应，wrapper 已回收 task、runner 和唯一 MuJoCo 实例；没有重试。原始证据在 `experiment_results/a1_runs/contact_hover_mmc_hold_lidio_20261020_06/`（含 `p2b_finger_input_diagnostic.csv`、`p2b_finger_raw_values.csv`、`p2b_joint_path.csv`、接触 CSV、任务/仿真日志、控制器清单、`script_status.json`）及同名 `launch_logs` 目录。结论：四指容差修复和 P2b/接触 gate 本次通过；本轮 MMC B 因动作后 MMC 状态预检失败而中止，不做第二次运行、不调整其他参数、不提交、不推送。

## 控制器状态服务截止时间修复与 B_07（2026-10-20）

修复 `dual_impedance_no_load_bridge` 的控制器服务预检：`controller_manager/list_controllers`、MMC `get_controllers` 和 STRICT switch 均在调用方传入的同一 steady 截止时间内等待发现与响应，预算耗尽后不再发请求。运行期 watchdog 的零等待健康检查保持不变。控制器清单按名称分类缺失/状态不符，并记录 JTC/MMC 名称和返回的原始状态字符串；查询失败不会再写成状态冲突。任务初始化时创建由 task joint feedback + MoveIt FK 提供的状态服务；MMC 模式在 P2 规划和运动前启动新 bridge 的 `preflight_only`，闭合后、抬升前原有检查仍保留。

定向构建 `colcon build --base-paths src/multipanda_ros2 --packages-up-to dual_arm_carry_task --parallel-workers 2 --cmake-args -DMUJOCO_DIR=/home/lidio/Libraries/mujoco` 成功，重新 source 后安装节点和 bridge 均解析到本工作树 `build/`。`test_mmc_state_service` 直接运行 8/8 通过，覆盖延迟服务、缺失服务有界超时、响应 executor、控制器状态不符/缺失分类及状态验证。首次 `colcon test` 在 sandbox 中因 ROS 日志目录与 DDS socket 限制以 `-11` 退出；目标断言已通过，将 `ROS_HOME` 指向 `/tmp` 后直接测试二进制完整通过。包级既有 copyright/cpplint/flake8/pep257/uncrustify 检查仍有历史失败，本轮未处理。

唯一新运行 `contact_hover_mmc_hold_lidio_20261020_07` 使用正式 wrapper，domain 113、本机 MuJoCo `/home/lidio/Libraries/mujoco`，场景 `(0.50,0,+10°)`，rigid=true、自适应保持=false，50 mm 抬升、3 s MMC HOLD 请求。任务准备阶段启动预检通过，墙钟耗时 1.185 s：`/controller_manager/list_controllers` 观察到 `dual_panda_arm_controller=active`、`sim_multi_mode_controller=inactive`；`/sim_multi_mode_controller/get_controllers` 返回 `faulted=0`；双臂状态来自新鲜 task joint feedback + MoveIt FK，坐标为 `mj_left_link0/mj_right_link0`；STRICT no-op switch 响应成功。P2a 与 P2b 通过（18/18 起点变量、25 点新路径最终验证通过），预抓取和下降到抓取位姿完成。接触 action 与单次 bilateral snapshot gate 通过，左右仿真时间差 0.010 s，四侧接触成立，法向力为 `(10.504,9.428;11.393,8.372) N`。

抬升前 bridge 预检再次通过。JTC 抬升使杆升高 45.959 mm，weld 与 rod constraint 均关闭。MMC 严格切换成功；实际状态为 `sim_multi_mode_controller=active`、`dual_panda_arm_controller=inactive`。激活后 58 ms 目标订阅出现，首个实测双手参考在切换后 69 ms 内发布并由参考序号 `1` 确认。HOLD trace 记录 1.218 s 仿真时间后中止，未完成 3 s。根因分类为状态服务收到一条 `/joint_states` 样本，其仿真时间戳跳到墙钟值：`sample_sim_time=1792478357.988610029`，当时仿真时间 `36.684 s`，墙钟年龄仅 1.762 ms；状态源按既有时效语义拒绝，未返回双臂状态，bridge 随后报空状态 `invalid_arm_count_or_order`，不是 controller-manager 状态冲突。桥接失败时任务保护最大循环间隔 52 ms，真值请求耗时约 0.04 ms；保持阶段最后观测杆比 HOLD 入口低 7.938 mm，杆姿态变化约 0.703°。失败诊断保留左侧法向/切向读数 277.896/382.342 N，不裁剪、不据此归因。夹爪测量位置 stop 得到双侧确认；未正常切回 JTC、未下降/释放。wrapper 回收 task、runner 和唯一 MuJoCo 实例，复查无残留进程。

结论：即时服务发现拒绝已修复，新进程真实预检及 MMC 严格激活/首参考握手通过；本次进入真实接触下 MMC HOLD，但运行期 joint-state 时间戳异常使 HOLD 提前失败，不构成 3 s 承载链路通过。动作后失败不重跑。原始数据保存在 `experiment_results/a1_runs/contact_hover_mmc_hold_lidio_20261020_07/` 和同名 `launch_logs/`。未改夹爪、接触、碰撞、控制增益或保护阈值，未提交、未推送。

## `/joint_states` 仿真时钟修复与新 B（2026-10-20）

启动链核对发现 `joint_state_publisher` 的 `source_list` 曾含 `/joint_states`，同时也向 `/joint_states` 发布；它会订阅自身，且原 launch 未给它设置 `use_sim_time`。本次最小拆分为：`joint_state_broadcaster` 开启 `use_local_topics`，以 `/joint_state_broadcaster/joint_states` 作为原始双臂输入；聚合器只合并该话题与左右夹爪私有话题，独占发布全局 `/joint_states`，显式使用仿真时钟。任务 `/joint_states` 回调现在在缓存替换前校验仿真戳；零时间戳、墙钟时间和未来仿真时间样本均拒绝，记录戳/仿真时钟和拒绝原因，不改有效缓存序号或原始 steady 接收时刻。过期缓存仍按原 250 ms 到达年龄和既有仿真时差规则失效。

定向构建 `colcon build --base-paths src/multipanda_ros2 --packages-up-to dual_arm_carry_task franka_moveit_config --parallel-workers 2 --cmake-args -DMUJOCO_DIR=/home/lidio/Libraries/mujoco` 通过；新增时钟缓存用例所在的 `test_mmc_state_service` 通过。之后发现 MoveGroup 和 `robot_state_publisher` 在双臂仿真 launch 中也没有显式 `use_sim_time`，遂将两者设为 true 并单独重建 `franka_moveit_config`；安装版 launch 语法/配置检查通过。`bash -n`、观测脚本及 launch Python 语法、`git diff --check` 均通过。`colcon test-result` 总览仍显示仓库既有 lint/copyright 失败，本轮未处理。

首次 `_08` 启动目录 `contact_hover_mmc_hold_lidio_20261020_08/` 在任务动作前因默认沙箱禁止 DDS UDP 而中止，保留启动日志；经明确改用可访问本机 DDS 的执行方式，在新目录 `contact_hover_mmc_hold_lidio_20261020_08_retry1/` 只启动了一个实例。无动作观察 2.5 s：publisher 分别为 `/joint_state_broadcaster/joint_states`→`joint_state_broadcaster`、两侧 gripper 私有 topic→各自 gripper node、`/joint_states`→`joint_state_publisher`；聚合器不订阅自己的输出。聚合器、broadcaster 所在 controller_manager、两侧 gripper 和聚合器的 `use_sim_time` 均为 true；`/clock` 来自 `mujoco_server`。采到全局 75 条消息，含 14 个手臂关节和 4 个手指关节；所有观察消息时间戳相对接收时 `/clock` 的最大绝对差约 20 ms。此时 JTC active、MMC inactive。证据为本目录 `joint_state_publishers.txt` 与 `joint_state_clock_observation.json`。

新 B 的 P2a、P2b 通过：四指原始值和 18 变量新鲜起点有效，25 点路径通过原有逐点/插值验证。任务执行到 P3A-1 预抓取轨迹，随后 MoveIt 在 1.052 s 后返回 controller error `-4`，未完成原计划 2.337 s。仿真日志给出的直接原因是 trajectory execution manager 在 1 s 内“couldn't receive full current joint state”。因此未到下降/闭合/gate/JTC 抬升，MMC 未激活，HOLD 为 0 s；本轮无接触承载证据，也没有回切、下降或释放。当前日志能确认 MoveGroup/RSP 原 launch 缺少仿真时钟设置；它与完整当前状态超时相符，但该次未记录 MoveGroup 参数值，故不把两者的因果关系写成已实测证明。

wrapper 已回收该目录的 task、runner 和唯一 MuJoCo 实例，复查无本轮进程残留。结论：原始发布源/聚合器时钟问题已按观测修复；B 在预抓取执行阶段因 MoveIt 完整状态超时失败，未完成 MMC 接入。`_08` 与 `_08_retry1` 均保留，不再重跑。无动作观测与 task/sim 日志及控制器、准备状态文件均留在各自独立目录；本轮未改增益、夹持或验收阈值，未提交、未推送。

## 运行时关节状态核验与新 B（2026-10-20，状态源不一致）

本轮定向构建 `dual_arm_carry_task` 后，`test_bridge_process`、`test_mmc_state_service`（11 项）和 `test_mmc_activation_handshake` 均通过。新加的任务启动核验实际调用 MoveGroupInterface `getCurrentState(1.0)`，并按名称与新鲜任务反馈比较；bridge 与任务均记录运行时 `use_sim_time`。唯一运行目录为 `experiment_results/a1_runs/contact_hover_mmc_hold_lidio_20261020_09/`，ROS 日志在同名 `launch_logs/` 子目录；使用正式 wrapper、domain 113、本机 MuJoCo `/home/lidio/Libraries/mujoco`、rigid=true、自适应保持=false、50 mm/3 s 请求。

无动作窗口记录 `/joint_states` 的唯一发布者为 `joint_state_publisher`，只订阅 broadcaster 局部话题和两侧 gripper 私有话题，没有订阅自身输出；MoveGroup、robot_state_publisher、JSP、controller_manager、两侧 gripper 的运行时 `use_sim_time` 均为 true。观察 75 条全局消息，结构上包含 14 个手臂及 4 个具名手指关节，最大戳与 `/clock` 偏差 4 ms。任务真实 CurrentStateMonitor 核验通过：14/14 手臂变量齐全，样本年龄 0.179 ms，和任务具名反馈最大差 0 rad；耗时 29.976 ms。新 bridge 预检通过，JTC active、MMC inactive，状态请求有效。P2a/P2b 通过，18 变量起点有效，新路径 25 点；预抓取执行成功（2.404 s）。

状态内容检查暴露了启动采样未覆盖的夹爪名称问题：私有话题 `/mj_left_gripper_sim_node/joint_states` 实际发布名为 `_finger_joint1/2`，右侧同样缺 `mj_left/mj_right` 前缀；无动作时它们约为 ±1.56e-8 m，聚合 `/joint_states` 的四个预期具名 finger 项则一直为 0。打开夹爪动作本身成功（结果位置约 0.080 m），但下降前新鲜快照保护发现左 q1 私有硬件值 0.040 m、全局值 0.000 m，报 `E_2B_FINGER_SNAPSHOT_INVALID` 并拒绝下降。未下降、闭合、gate、抬升或切 MMC；无承载/HOLD、回切/释放数据。该错误并非 MoveIt 当前状态监视超时。

根因在 `GripperSimActionServer` 构造函数：`robot_name` 已 move 到 `robot_name_` 后，joint 名仍使用已移动的局部字符串。本轮将其改为使用成员 `robot_name_`，定向构建 `franka_hardware` 并运行现有 `test_gripper_shared_state`（通过）。此构建发生在失败仿真之后；没有重启仿真验证新消息名，也没有重试 B，因为本轮已执行预抓取和夹爪动作。`prepare_status.json`、完整路径、四指输入诊断、topic 参数/端点、时钟观察、task/sim/wrapper 日志均保存在上述目录。wrapper 已回收 task、runner 和 PID 161253 的唯一 MuJoCo 实例；本轮未完成 MMC B，不能以时钟检查通过代替承载通过。未调整控制/接触参数、阈值或超时，未提交、未推送。

## 夹爪关节名称修复与新 B（2026-10-20）

检查构造函数确认，`robot_name` move 到 `robot_name_` 后没有其他局部变量使用。关节名生成现在复用 `fingerJointNamesForRobot(robot_name_)`；现有 hardware GTest 增加左右两臂四个完整名称回归断言。`franka_hardware` 定向构建成功，`test_gripper_shared_state` 通过；连续观察脚本 Python 编译、wrapper Bash 语法和 `git diff --check` 通过。安装插件来自本工作树，更新时间晚于本次构建。

新唯一运行 `contact_hover_mmc_hold_lidio_20261020_10` 使用正式 wrapper、domain 113、本机 MuJoCo、rigid=true、自适应保持=false、50 mm/3 s 请求。启动的 2.5 s 无动作观察确认私有左右话题使用 `mj_left/right_finger_joint1/2`，没有 `_finger_joint1/2`；聚合器无自订阅，`/joint_states` 唯一发布者仍是 JSP，18 个预期名称齐全。新连续 trace 在现有开爪动作前启动并由 wrapper 停止落盘（`finger_feedback_trace.csv`，14,642 行）；typed、两侧私有和全局三路都记录关节名、值、仿真戳，typed 另有 `update_seq`。所有四指从约 0 变化到约 0.04 m。到位快照：typed seq `19272/19272`、年龄约 5 ms；左两指 `0.040000 m` 对全局 `0.040000 m`，右两指 `0.040041/0.040040 m` 对全局 `0.039982 m`，最大差约 `0.059 mm`，低于现有 `0.1 mm` 容差，任务 `[finger_snapshot] global_joint_states_match=1`。私有 topic 与同戳 typed 样本吻合；异步 trace 配对的戳差最大约 14 ms，开爪稳定段最高瞬时差约 `0.146 mm`，保留原始样本，不把异步值当同一步。trace 还保留约 `0.040423 m` 的开爪峰值，没有裁剪或改写。

B 到达 P2a/P2b、预抓取、开爪、下降到抓取位姿、真实接触闭合和 gate；18 变量起点、新生成 25 点路径及既有检查通过。夹爪 action 成功，单快照 gate 通过，左右仿真时间差 `0.002 s`，四侧接触成立，法向力为 `(10.637,8.992;9.880,9.812) N`，weld `(0,0)`、rod constraint `0`。随后 lift 轨迹已规划，但在发送 lift 前、MMC 切换前的保护检查失败：`E_CONTACT_TRUTH_STALE service=/get_body_state guard=sample_stale age_wall_sec=2.843116`，请求耗时字段为 `0.000000 s`。bridge 的只读 preflight 仍报告 JTC active/MMC inactive、状态有效并完成 STRICT no-op；没有执行真实 JTC→MMC 切换，也没有执行 lift，所以 MMC HOLD 为 0 s，无杆升高、MMC 保持、回切或正常释放结果。任务确认双夹爪测量位置 stop，失败状态保持；wrapper 回收 task、observer、runner 和唯一仿真。该运行已发生手臂运动及接触闭合，本轮不重试、不调整现有时效阈值。数据与日志保存在 `experiment_results/a1_runs/contact_hover_mmc_hold_lidio_20261020_10/` 和同名 `launch_logs/`。结论：夹爪三路反馈名称及动态对应通过；真实接触 B 未进入 JTC lift/MMC，直接阻断项是杆真值样本过期。本轮未调柔顺或夹持参数，未提交、未推送。

## 杆真值刷新与本轮 MMC B 收口（2026-10-20）

B_10 的 `sample_stale age_wall_sec=2.843116` 使用 `steady_clock` 的有效服务响应接收时刻计算，与 `AsyncTruthGuard` 现有 2.0 s 新鲜度预算同一时钟/定义；`request_elapsed_wall_sec=0` 表示报错时没有在途请求。调用链确认 guard 在接触 settle 与 lift 规划/同步只读 bridge preflight 之后才创建；这些阶段没有发起杆真值异步请求，故无请求/响应可推进，也不是已观察到的服务响应失败。错误源于同步阶段没有新样本、旧样本自然过期。没有用 PlanningScene 位姿代替物理真值，也未放宽预算。

最小修复是在抬升下发前、bridge preflight 后，以同一 `GetBodyState` client/future 在原 `mujoco_truth_timeout_sec=2.0 s` 预算内完成有界刷新，再执行原始 pose、手部参考、滑移/姿态与高度保护；刷新只在 lift 前等待，运动期间仍使用现有非阻塞保护。夹爪接触闭合前另连续请求两次 `/get_body_state`，记录 response、steady 耗时及仿真时钟推进。`test_mmc_state_service` 新增 aged-cache fresh-response 用例；定向构建 `dual_arm_carry_task` 依赖链成功，目标 CTest 通过。包级 `colcon test` 还触发既有 copyright/cpplint 等全包 lint 失败，本轮未处理；直接 CTest 目标为 1/1 通过。时钟观察器另加最多 8 s 的 publisher graph discovery 等待，随后仍保留 2.5 s 数据窗口；Python 编译及 `git diff --check` 通过。

唯一完整任务动作尝试为 `contact_hover_mmc_hold_lidio_20261020_11_retry1/`，使用正式 headless wrapper、domain 113、本机 MuJoCo `/home/lidio/Libraries/mujoco`、刚性模式、自适应保持关闭、50 mm lift/3 s HOLD。先前 `_11/` 在任务动作前 observer 的 2.5 s 窗口未发现任何 ROS publisher（JSON 为零行/零 publisher），而同期 CLI 已列出四个 joint-state 话题发布端且相关节点 `use_sim_time=true`；这与 DDS graph discovery 延迟相符，task 未启动、无动作。修复 observer 后 `_11_retry1` 的无动作状态链检查通过；采样有 18 个全局具名关节、所有来源时间戳与 `/clock` 最大偏差见 `joint_state_clock_observation.json`，连续三路四指 trace ready。任务接触动作前真值就绪记录为 `responses=2`、墙钟 `0.062659 s`、仿真时间推进 `0.062 s`，两次测得杆 `z=0.020654 m`。P2a/P2b 通过，预抓取/开爪/下降/闭合完成，bilateral gate 单快照通过（左右时间差 `0.012 s`，四侧均接触、法向力约 `(10.697,9.247;9.578,10.365) N`），weld `(0,0)`、rod constraint `0`。

lift 前真值刷新成功：`/get_body_state` 请求耗时 `0.000102 s`，样本年龄 `0.000328 s`；随后现有 JTC 抬升 trajectory action 返回 success。抬升监测最大循环间隔 `51.54 ms`、最大真值年龄 `2.03 ms`，左右样本时间差最高 `14 ms`（异步观测）。杆由 `z=0.020776 m` 升至 `0.064550 m`，实际升高 `43.774 mm`，低于现有 `45 mm` 离地确认要求 `1.226 mm`，故任务触发 `E_CONTACT_ROD_NOT_LIFTED` 并在 MMC 切换前停止。该次 lift 观测仿真跨度 `0.860 s`；左右手分别升高 `44.517/44.225 mm`。原抓取参考位移残差从 lift 入口 `0.557/2.371 mm` 增至 `2.281/3.680 mm`，峰值 `4.527/4.863 mm`；姿态残差峰值约 `0.864°/0.551°`。保持没有开始（MMC HOLD `0 s`），没有实际 MMC activation/首参考、回切、下降或正常释放。原始法向/切向尖峰保留，观测最大超过 `100 N` 的读数共 2 行，不能据此判定具体接触原因。失败时测量位置夹爪 stop 双侧确认，wrapper 回收 task、observer、runner 和唯一 MuJoCo 实例；复查无本轮残留。

结论：B_10 真值过期阻断已由 lift 前刷新绕过并留下成功新鲜样本；本轮真实接触链到达 JTC lift，但既有 45 mm 离地保护未通过，未进入 MMC HOLD，故 MMC 3 s 保持/回切/下降释放仍未验证。该物理保护失败后按边界停止，不再尝试，不据此调整夹持或控制参数。`_11/`、`_11_retry1/` 及各自日志均保留；未改物理模型、接触、控制增益、互斥规则或保护阈值，未提交、未推送。

## Hover 入口研究口径更新（2026-10-20）

项目定位按大学生创新创业仿真研究执行，优先获得可解释的功能与实验结果；相关接口缺陷可在同一轮连续修复，不把旧逐缺陷提示词当成额外审批链。此口径只调整 hover/MMC 研究入口，不改变其他任务验收。

JTC 原命令仍为抬升 50 mm；实际杆中心升高 45 mm 是跟踪指标，不再单独否决进入 HOLD。hover 入口使用新鲜 MuJoCo 杆位姿和现有杆尺寸/支撑面高度计算杆最低点净空，净空为正确认离台。保留真实接触及 bilateral gate、weld/rod constraint 关闭、抓持残差/姿态和控制器互斥检查；HOLD 期间原有净空大于 3 mm、相对保持起点下降不超过 20 mm、滑移/姿态、watchdog 与故障退出保护继续生效。手位姿、轨迹目标和 PlanningScene 均不替代杆真值。完整单次通过只证明链路可用；之后在一致入口、模型与夹持参数下比较 JTC/MMC，报告承载稳定与柔顺让步，不要求 MMC 各项优于 JTC。

B_11 原始结论仍为按当时 45 mm 门槛 `E_CONTACT_ROD_NOT_LIFTED` 失败，未进入 MMC；不追改历史标签。离线使用 `contact_hover_mmc_hold_lidio_20261020_11_retry1/hover_diagnostics.csv` 的 19 个 `HOVER_LIFT` 样本按现有物理净空公式复算：杆最低点净空最小 `0.575 mm`、最大 `44.499 mm`；最后中心升高 `43.767 mm`。因此末段数据符合新 hover 入口的正净空条件，且不需要新增高度阈值；这是对新口径的离线适用性说明，不是旧 B_11 的成功运行或 MMC 证据。

## 正净空入口后的 MMC B（2026-10-20）

本轮把获批入口口径落实到 `hover_acceptance.hpp` 与 hover 任务链：45 mm 仅由 `rise_target_reached` 记录；JTC lift action 成功后，以 `observe()` 刷新的 MuJoCo 杆 pose、杆尺寸和既有支撑面高度计算最低点净空，严格大于零才允许进入 HOLD。HOLD 原有 `clearance>3 mm`、起点下降不超过 20 mm、原抓取参考滑移/姿态、真实接触、watchdog 和控制互斥保护均未放宽。`test_hover_acceptance_standalone` 通过，覆盖 45 mm 只跟踪、B_11 数值可进入正净空、零/负净空拒绝及 HOLD 保护；`dual_arm_carry_task` 依赖链定向构建成功。`test_mmc_state_service` 本轮 CTest 在 `DelayedServiceAppearsAndExecutorProcessesRequest` 处以 `terminate called without an active exception` 中止，没有完成断言；此项保留为测试运行失败，不能记作通过。

新运行 `contact_hover_mmc_hold_lidio_20261020_12` 使用正式 headless wrapper，domain 113、本机 MuJoCo `/home/lidio/Libraries/mujoco`、场景 `(0.50,0,+10°)`、rigid=true、自适应保持=false、10 N/指、50 mm 指令与 3 s MMC HOLD。P2a/P2b、预抓取、开爪/下降、左右接触 action 和单快照 bilateral gate 通过；新生成 25 点 P2b 路径通过既有检查。Gate 四侧均接触，法向力约 `(10.287,9.403;9.852,9.811) N`，weld `(0,0)`、rod constraint `0`。JTC lift action 成功；新鲜物理真值测得杆中心升高 `44.917 mm`、最低点净空 `45.487 mm`，所以按本轮入口进入 HOLD，45 mm 跟踪项记录为未达到。

MMC 切换为 STRICT 成功并核实 `sim_multi_mode_controller=active`、JTC inactive；激活后约 `1.048 s` 订阅出现，首条实测双手 pose/q、零 twist 参考在 `1.059 s` 被 accepted seq `1` 确认，符合 1.5 s 握手预算与 2.0 s watchdog 首目标宽限。随后 bridge 报 `MMC status: controller fault status unavailable` 并停止发布；MuJoCo 在最后目标年龄 `0.500 s` 时触发参考 watchdog 并锁存故障。控制器进入约 `2.2 s` 的实际承载时间，未完成请求的 3 s HOLD；未正常回切、下降或释放。Task 对 bridge 子进程执行失败传播并确认回收，明确不继续正常运动流程。

故障前诊断样本记录：MMC HOLD 入口杆中心升高 `44.917 mm`、净空 `45.487 mm`；末样本分别为 `37.374 mm`、`37.218 mm`，即入口后杆下移约 `7.543 mm`（净空减少 `8.269 mm`）。杆姿态相对 HOLD 入口变化约 `0.788°`。同区间左右手 Z 分别下降约 `5.906/5.213 mm`，原抓取参考位移残差由 `2.722/3.382 mm` 增至末样本 `7.571/8.942 mm`；以每个 arm 各自的杆接触点相对手坐标观察，其 XYZ 变化约左 `(-3.570,+3.258,+2.277) mm`、右 `(+4.011,-3.753,+1.607) mm`。这些是异步物理/反馈样本的描述量，不视为同一步受力平衡。任务失败诊断保留左侧总法向/切向读数 `272.315/389.760 N`、右侧 `19.822/6.065 N`；全程原始 contact CSV 未删尖峰。不得仅由这些读数断言单一滑移机理。

此次没有发生释放；因此不能报告释放状态，也不能称真实接触 MMC 3 s 链路通过。直接阻断是运行中的 MMC 状态查询不可用导致 bridge 停止目标发布，随后 watchdog 锁存；同一物理窗口有杆下移和显著力读数，故本轮不重试、不调参。`hover_analysis.json/txt`、`hover_diagnostics.csv`、`mmc_hold_trace.csv`、接触/逐接触 CSV、P2b 输入及完整路径、控制器清单、task/sim/wrapper 日志保存在 `experiment_results/a1_runs/contact_hover_mmc_hold_lidio_20261020_12/` 和同名 `launch_logs/`。Wrapper 已回收 task、bridge、runner 与唯一 MuJoCo 实例，复查无本轮进程残留。结论：新的正净空入口和 MMC 握手工作；承载 HOLD 因 watchdog 故障未完成，未证明完整保持/正常释放。未改变夹持力、摩擦、模型或阻抗增益，未提交、未推送。

## 固定增益下沉分析与水平扰动设计（2026-10-20）

> 本节下方包含方向性扰动实现前的历史分析与计划。后续实验已完成，最终 3+3 扰动对照、参数和结论以 [CONTACT_HOVER_AB_COMPARISON_20261020.md 的第 6 节](CONTACT_HOVER_AB_COMPARISON_20261020.md) 为准。B_12 仅用于故障中断分析，不是固定增益基线；下沉基线采用有效无扰动 B1–B3。

代码中的实际默认参数来自 `ComlessDualCartesianImpedanceController::defaultParameters()`：两臂平移 `K=diag(400,400,400) N/m`、转动 `K=diag(20,20,20)`（转动量纲为 Nm/rad），`damping_ratio=(0.8,...,0.8)`，nullspace stiffness `10`。参考是各自 `mj_left_link0` / `mj_right_link0` 坐标中的实测末端 pose、零 twist 与实测冗余关节构型；`computeTauImpl()` 在相同 link0/Jacobian 坐标中计算位置/姿态误差。阻尼不是固定常数矩阵：按 `D=M̂^(1/2) ζ K^(1/2)+K^(1/2) ζ M̂^(1/2)`，使用当前配置下操作空间惯量在线计算。因此本记录报告实际 K 与阻尼比，不伪报一个不随构型变化的 D 数值。控制器只加 task impedance、nullspace、Coriolis 和避碰项；MuJoCo 双臂 link 的 `gravcomp=1` 是机器人本体重力补偿，杆没有对应载荷前馈，杆仍受模型重力。

模型杆为 `0.04×0.40×0.04 m`、density `1000 kg/m³`，质量 `0.64 kg`、重量约 `6.28 N`。若杆重量由两侧末端竖直弹簧平均分担，且接触无滑移、系统已达静态平衡，则等效总刚度 `400+400=800 N/m`，预测共模下沉约 `6.28/800=7.85 mm`。这是简化静态假设，不含机器人姿态/操作空间耦合、夹持接触滑移和 transient damping，接近实测不构成根因证明。

从 B_12 首条被 MMC 接收参考后的首个可用真值样本（仿真时间 `50.722 s`）对齐，到失败前末样本 `52.880 s`，共 `2.158 s`、44 个异步观测：杆中心下移 `5.999 mm`，左/右手分别下移 `4.681/4.234 mm`。前半段杆下沉速率约 `4.66 mm/s`，后半段约 `0.84 mm/s`；左右手对应由 `3.99/3.45` 降为 `0.29/0.43 mm/s`。曲线显示速度减小，尚不能据不足 3 s 的故障中断数据认定已稳定。杆相对两手也有额外下移，故观测既含双臂整体垂向位移，也含闭链/接触变形。简单模型预测 `7.85 mm` 与这段不完整动态下沉同量级，但不得由此认定唯一原因就是缺少杆载荷前馈。

图：[B_12 历史中断运行](../experiment_results/a1_runs/contact_hover_mmc_hold_lidio_20261020_12/mmc_vertical_droop_historical_interrupted.png)，显示首条已接收 MMC 参考后杆与双手的竖直变化；该次 HOLD 未完成，不作为固定增益基线。

为单独比较水平柔顺，候选方向定为 world `+X`。MuJoCo 两个 `mj_*_link0` 都是单位朝向（MJCF `quat="1 0 0 0"`），故该物理方向在两臂控制坐标中均是 `+x`，无需直接套用一侧轴到另一侧。拟议方向性参数仅改 `Kx:400→200 N/m`，保持 `Ky=Kz=400 N/m`、姿态 `K=20` 和 nullspace `10` 不变；各方向阻尼比仍为 `0.8`，由现有操作空间阻尼设计自动重算，标量近似下新 X 阻尼约为原来的 `sqrt(0.5)`。预期在等效线性并联弹簧假设下，对杆中心施加总计 `+2 N` world-X 力时，固定增益静态位移约 `2.5 mm`，方向性增益约 `5 mm`；这是设计量级预测，不是实验结果。

当时的阶段记录（定时外力入口实现前）：尚未执行方向性对照，也未施加扰动力；B_12 固定增益组在约 `2.16 s` 后因 MMC 状态查询失败、参考 watchdog 锁存而中止。把力直接从 ROS 任务线程写入 MuJoCo 状态会并发改写物理状态，因此该旧阶段只保存了不完整的无扰动观测。其后已新增由任务请求、硬件物理线程执行的实验力入口，并在六次完整 HOLD 中运行；最终结果见本文开头指向的对照记录。B_12 仍保持历史失败结论。

## MMC 状态查询修复与完整 B（2026-10-20）

### B_12 直接错误归类

旧 `checkControllerHealth()` 在每个 100 Hz 参考周期内同步串行查询 `/sim_multi_mode_controller/get_controllers` 和 `/controller_manager/list_controllers`，两个 future 各只等 50 ms；`wait_for_service(0)` 失败或任一请求 50 ms 未返回即永久锁存 `controller fault status unavailable`。B_12 日志没有健康查询分项，无法从旧记录再区分 service 未就绪与单次请求超时；可确认的是：失败前 0.102 s 还有有效 `current_robot_states` 读取，MMC 在 710.77 s 已核实 active，日志没有 MMC fault/watchdog 报错，因此该锁存属于“健康查询未完成”，不是控制器故障。旧 HOLD 实际约 2.11 s 后停止参考发布。

### 最小修复

- 新增 `async_service_query.hpp`：每服务最多一个在途请求，`poll()` 只用 `wait_for(0)`；单请求预算 100 ms（2×50 ms 查询周期），超时放弃并重试，参考发布不再等待健康查询。
- `ControllerFaultLatch` 增加 `observeQueryMiss()`：单次未返回不锁存；自最后一次完整观察起超过 250 ms 才锁存。250 ms 沿用既有 `kMaxStatusAgeNs`，等于 5×查询周期、25×10 ms 参考周期、0.5×500 ms MMC watchdog。明确 `faulted`、MMC 非 active、JTC/MMC 状态冲突或状态时间戳非法仍立即锁存。
- bridge 启动先做有界连续性探针：3 s 服务发现宽限，3 s 首完整观察窗口（单请求 1 s；新进程 DDS 端点匹配可超过图上的 ready 信号），成功后用 100 ms/250 ms 策略连续查询 1 s，再进入切换。HOLD 循环异步 poll；激活与回切握手使用同步有界检查。
- `test_mmc_state_service` 用 RAII 回收 executor/spin 线程，断言早退不再析构 joinable thread；延迟服务不再在 spin 中 `add_node`。新增延迟响应不阻塞轮询、单次超时容忍、持续失联按 250 ms 失败、明确故障/状态冲突立即锁存测试；`test_mmc_state_service` 13/13、`test_controller_fault_latch` 8/8、`test_mmc_activation_handshake` 6/6、`test_bridge_process` 2/2 通过。
- `mmc_hold_trace.csv` 增加参考发布计数/间隔和健康查询计数、耗时、年龄、在途标志；同时修正旧行只写 6 个 twist 值而表头有 12 列的错位。

### 运行结果

B_13（第一次尝试）在 prepare 后、切换前的探针失败：服务图在 0.020 s 报告 ready，但新进程首个请求 1.0 s 内未返回，旧探针在 250 ms 锁存。该失败与原始日志保留在 `contact_hover_mmc_hold_lidio_20261020_13/`，未重试原配置。

B_14（`contact_hover_mmc_hold_lidio_20261020_14`）完整通过，`task_state=DONE`：

| 项目 | 结果 |
|---|---|
| 健康查询启动 | discovery 0.020 s；首请求 1.0 s 超时 1 次，第二请求 0.056 s 成功；随后 1 s 窗口 17/17 成功、0 失败 |
| HOLD 健康查询 | 全程 67 次（含探针），trace 期间新增约 47 次；成功 66、失败 1（仅启动首查）、恢复 1、无 latched |
| 查询耗时 / 成功年龄 | 耗时 median 10.27 / p95 21.08 / max 41.13 ms（在下次轮询观测，含轮询节拍）；最后成功年龄 median 30.9 / p95 52.6 / max 62.3 ms |
| MMC 实际保持 | 3.01 s（bridge `impedance sequence complete after 3.01 s`）；trace sim span 2.998 s |
| 参考发布 | median 10.12 / p95 10.34 / max 10.45 ms，约 96.7 Hz；首参考 accepted seq 1，elapsed 0.043 s |
| 实际 MMC 保持窗口 | 杆 z −7.386 mm；左手 z −6.489 mm、右手 z −5.921 mm |
| 杆相对手位姿 | 左 (+0.335, −5.385, −0.897) mm；右 (−1.016, +5.167, −1.465) mm（同行异步读数） |
| 原抓取参考滑移 / 姿态 | 窗口内左 6.082→11.256 mm、右 7.167→13.170 mm；最大姿态 1.538°/0.456°，在 20 mm/2° 用户授权窗口内；5 mm 研究目标仍未满足 |
| 回切 | STRICT 回切 JTC，`seeded from the measured joint state`，`returned to the trajectory controller`；bridge exit 0 |
| 释放 | `[CONTACT_HOVER_RELEASED] xyz=(0.487547,0.001195,0.020655) open=true force_hold=OFF weld=OFF attach=none` |

task 的 `CONTACT_HOVER_RESULT hold=5.798 s` 是整个 HOLD 相位（含启动探针、切换和回切），不是 MMC 实际保持时间；其峰值 46.123 mm、末值 32.418 mm、峰后下降 13.705 mm 是不同时间口径的描述量，不能与 MMC 实际保持窗口的杆 z 变化相减来算滑移。接触 occupancy 为 64.7%（sample-count），不证明连续接触。B_14 的 `mmc_hold_trace.csv` 仍由修正前的二进制生成（每行只写 6 个 twist 值），本文健康统计按字段位置 64–73 读取；修正后的源码下一次运行会写出与表头一致的 12 个 twist 值。未修改夹持力、摩擦、模型或阻抗增益，未提交、未推送。原始数据在 `experiment_results/a1_runs/contact_hover_mmc_hold_lidio_20261020_14/` 与同名 `launch_logs/`。

同一版本、场景和抓持参数下的刚性 JTC / 固定增益 MMC 3+3 对照见 [CONTACT_HOVER_AB_COMPARISON_20261020.md](CONTACT_HOVER_AB_COMPARISON_20261020.md)。

