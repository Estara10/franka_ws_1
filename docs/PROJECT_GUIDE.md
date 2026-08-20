# 双臂 Franka 协作搬运项目指南

本文是仓库当前架构、开发优先级和能力边界的公共说明。`AGENTS.md`、`CLAUDE.md`、
根 `README.md` 和包级文档应引用本文，不再各自维护一份容易漂移的架构副本。

## 项目目标和当前范围

项目基于 ROS 2 Humble、MoveIt 2、MuJoCo 和 ros2_control，研究两台 Franka Panda
协同夹持并搬运一根 40 cm × 4 cm × 4 cm 铝杆的轨迹规划与柔顺控制。

当前唯一开发主线位于 `src/multipanda_ros2`。根 `src/dual_arm_*` 四个早期包已移至
`archive/legacy_dual_arm_stack`，不再参与 colcon 枚举或运行。

开发顺序已经确定：

1. A0 固定场景回归。
2. 模式 A 安全与可靠性修复。
3. 物体空间轨迹、协调 IK、完整碰撞检查、真实轨迹优化和统一时间参数化。
4. 在仿真中完成外部笛卡尔阻抗链路，并与内部柔顺对比后决定默认方案。
5. A1 已知真值的可复现随机场景。
6. B 视觉估计场景。

视觉目前只是实验性框架，不是现阶段验收主线。

## 当前活动架构

### 启动和仿真层

- `start_dual_arm_task.sh`：当前一键入口，负责模式选择、环境加载、必要构建检查、启动和日志。
- `franka_moveit_config/launch/sim_dual_moveit.launch.py`：启动 MoveIt，并自动包含 MuJoCo；
  不得再并行单独启动第二个 MuJoCo 实例。
- `franka_hardware/src/sim/franka_mj_hardware_system.cpp`：MuJoCo 与 ros2_control 的硬件接口，
  同时实现当前内部柔顺和抓取 weld 约束。

### 任务层

`src/multipanda_ros2/dual_arm_carry_task` 是当前任务包：

- `carry_task_state_machine.cpp`：节点、参数和同步 FSM 循环。
- `dual_arm_controller.cpp`：MoveIt 接口、IK、轨迹、PlanningScene 和抓放逻辑。
- `gripper_controller.cpp`：左右夹爪 action 客户端。
- `adaptive_compliance_supervisor.cpp`：外部阻抗参数监督器，默认关闭。
- `communication_latency_monitor.cpp`、`task_metrics_monitor.cpp`：实验指标节点，均为 C++。

状态顺序为：

```text
INIT → APPROACH → GRASP → WAITING_PHYSICS → LIFT → TRANSPORT
     → ROTATE → DESCEND → PLACE → RETREAT → DONE
```

任一动作失败应进入 `ERROR`，不得以 `DONE` 掩盖失败。

### 抓取与对象状态

夹爪闭合后，任务发布 `/grasp_weld_active=true`，MuJoCo 硬件插件启用左右手与杆之间的
weld；同时任务把杆作为 AttachedCollisionObject 加入 MoveIt。放置时逆序解除 weld、
打开夹爪、detach 并把杆恢复到世界场景。仿真场景和 PlanningScene 必须保持一致。

## 轨迹规划的真实状态

当前接近和部分抓取动作使用 Pilz PTP；运输、旋转和下降主要由逐点 IK 手工构造轨迹，
随后使用 TOTG，部分路径再使用 Ruckig。MoveIt 配置虽然注册了 CHOMP，但核心任务代码
目前没有真正调用 CHOMP。因此 `chomp_only` 和 `compliant_chomp` 仍是历史模式名，不能
把它们描述成已经完成 CHOMP 优化的实验组。

后续目标链路是：

```text
物体空间杆轨迹
  → 左右固定抓取变换
  → 协调 IK
  → 关节限制、环境碰撞、自碰撞与双臂相对约束检查
  → 真实轨迹优化
  → Ruckig 统一时间参数化
  → 位置控制或阻抗控制执行
```

## 柔顺控制的真实状态

### 内部柔顺

内部柔顺位于 MuJoCo 硬件插件的位置伺服环。它根据双腕受力估计降低关节位置增益、提高
阻尼，同时继续跟踪 `dual_panda_arm_controller` 的关节位置轨迹。准确称呼是“基于内力的
自适应关节伺服增益”，不是完整笛卡尔阻抗或导纳控制。它目前可用于仿真基线。

### 外部柔顺

`adaptive_compliance_supervisor` 只根据力反馈调节笛卡尔刚度和阻尼；它本身不生成连续的
期望末端轨迹。完整外部阻抗还需要轨迹到末端目标的桥接、无跳变控制器切换、力矩限制、
超时保护和恢复策略。

`dual_panda_arm_controller` 与 `sim_multi_mode_controller` 不能同时 active。外部监督器默认
关闭是为了避免控制资源冲突，而不是已经完成集成。外部阻抗应先在 MuJoCo 中验证：若成功率、
精度和内力/振动指标均可靠，再作为正式方案；否则保留为 experimental，仿真默认继续使用
内部柔顺。

长期实验模式计划为：

| 模式 | 轨迹优化 | 柔顺控制 |
|---|---|---|
| `baseline` | 无 | 无 |
| `optimized` | 有 | 无 |
| `compliant` | 无或同一基础轨迹 | 笛卡尔阻抗；完成前由内部柔顺作基线 |
| `optimized_compliant` | 有 | 笛卡尔阻抗 |

当前命令行仍使用旧模式名，重命名属于后续功能阶段。

## A0、A1 和 B

- A0：固定初始位姿，用于快速回归构建、状态机、规划和控制。
- A1：MuJoCo 按固定 seed 采样已知杆位姿；任务使用真值。样本必须通过桌面边界、双臂 IK、
  组合碰撞和预抓取路径检查后才接受。
- B：任务只使用视觉估计位姿，并用仿真真值评价位置和角度误差。

当前 `scripts/vision_core.py` 和 `/detected_rod_pose` 只构成框架。它尚无可信的完整成功记录，也没有
完成 CameraInfo/TF 标定、角度消歧、稳定窗口和位姿冻结。

## 已确认的验收方向

- 模式 A 先于模式 B。
- 仿真优先，但控制接口和安全策略要为未来真机迁移保留空间。
- 末端误差 2 mm 是优化目标；阶段性功能门槛可以为 3 mm，但必须如实报告。
- 铝杆目标质量为 0.5 kg。当前 MuJoCo 模型按尺寸和密度计算约为 0.64 kg，本阶段不修改，
  后续仿真阶段统一修正并重新验证。
- 实验原始数据保留；单次自动报告未来按正式结果与原始运行分层管理。

## 必须跟踪的技术风险

1. 手工逐点 IK 轨迹没有完整的碰撞有效性保证。
2. 夹爪 action 回调捕获栈变量，存在超时后的生命周期和数据竞争风险。
3. MultiThreadedExecutor 与同步 FSM 共享状态缺少明确同步。
4. SRDF/ACM 大范围禁碰及重复规则可能隐藏真实碰撞。
5. 当前一键脚本在模式 B 中会修改源码，并会复制生成配置回源码树。
6. 一键脚本使用较宽泛的进程清理，可能影响同用户其他 ROS 会话。
7. MuJoCo、PlanningScene、attach/detach 和 ACM 的对象状态有不一致日志。
8. jerk limit 不完整，旋转段没有统一经过 Ruckig。
9. 相机、`world` 与 `base_link` TF 链及标定尚不完整。
10. 同步误差、能量、末端误差和通信延迟目前包含代理量或异常样本，不能过度解释。

上述风险在阶段 0 只记录，不修改核心代码。

## 构建与启动约束

常用构建：

```bash
colcon build --packages-select dual_arm_carry_task franka_hardware franka_moveit_config
source install/setup.bash
```

推荐启动：

```bash
./start_dual_arm_task.sh
```

手动调试见 `MANUAL_STARTUP.md`。一键脚本目前可能自动构建并产生日志；模式 B 还可能修改
场景源码，因此运行前必须检查 Git 状态。`check_environment.sh` 和 `diagnose.sh` 是不被主流程调用的
遗留脚本，仍含旧路径和失效入口，不能作为权威工具；用户可确认后删除。

## 文档和归档规则

- 用户入口：根 `README.md`。
- 手动运行：`MANUAL_STARTUP.md`。
- 包参数和节点：`src/multipanda_ros2/dual_arm_carry_task/README.md`。
- 共同架构和能力边界：本文。
- 历史材料：`archive/README.md`。
- 项目交付物：`docs/deliverables/`。
- 上游组件文档保持原位，不按本项目文档规则重写。
- `experiment_results` 本阶段原样保留；报告分层留待后续处理。
