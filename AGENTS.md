# AGENTS.md

这是 Codex 在本仓库中的项目入口。开始工作前必须完整阅读
[`docs/PROJECT_GUIDE.md`](docs/PROJECT_GUIDE.md)，并以其中的当前主链、能力边界、阶段顺序和
技术风险为准。

`docs/PROJECT_GUIDE.md` 是所有 AI Agent 共用的项目规范 Source of Truth。项目属于大学生
创新创业训练，结题仅做仿真；必须遵守其中“项目定位与长期开发准则”的功能优先、适度验证、
问题分级和完成标准。阶段计划与旧提示词不得覆盖这些准则；历史审阅和实验记录不自动成为
新的验收关卡。

## Codex 工作约束

- 当前唯一开发主线是 `src/multipanda_ros2`；`archive/` 是只读历史材料，不参与构建。
- 保留用户已有修改，不清理或覆盖无关工作树内容。
- 修改前先核对调用链和配置来源；不要把旧 `src/dual_arm_*` 归档包当作当前实现。
- 构建、测试或启动 ROS/MuJoCo/RViz 前，先说明命令、目的及会产生的文件。
- 未经明确授权不得安装依赖、推送远端、改写 Git 历史或执行大范围进程清理。
- `sim_dual_moveit.launch.py` 已包含 MuJoCo，不得重复启动仿真实例。
- `dual_panda_arm_controller` 与 `sim_multi_mode_controller` 不得同时 active。
- 当前 CHOMP、外部笛卡尔阻抗和模式 B 视觉均未完成，文档、日志和验收不得把框架描述成成品。
- 核心算法、运动、碰撞、控制器和接口变更须做与影响范围相称的验证。仅对真实硬件风险、
  控制资源冲突、并发写仿真状态等高风险变更，或用户明确要求的任务安排独立审阅；
  简单文档、参数和低风险仿真修改不强制多轮审查或全系统验收。
- 不强制全量哈希、复杂审计链或重复报告；非阻断项记录后允许继续，达到事先明确的
  验收要求即可结束，不无限追加检查。

## 当前常用入口

```bash
colcon build --packages-select dual_arm_carry_task franka_hardware franka_moveit_config
source install/setup.bash
./start_dual_arm_task.sh
```

手动分步启动见 [`MANUAL_STARTUP.md`](MANUAL_STARTUP.md)，任务包参数见
[`src/multipanda_ros2/dual_arm_carry_task/README.md`](src/multipanda_ros2/dual_arm_carry_task/README.md)。
