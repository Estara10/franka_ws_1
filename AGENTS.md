# AGENTS.md

这是 Codex 在本仓库中的项目入口。开始工作前必须完整阅读
[`docs/PROJECT_GUIDE.md`](docs/PROJECT_GUIDE.md)，并以其中的当前主链、能力边界、阶段顺序和
技术风险为准。

## Codex 工作约束

- 当前唯一开发主线是 `src/multipanda_ros2`；`archive/` 是只读历史材料，不参与构建。
- 保留用户已有修改，不清理或覆盖无关工作树内容。
- 修改前先核对调用链和配置来源；不要把旧 `src/dual_arm_*` 归档包当作当前实现。
- 构建、测试或启动 ROS/MuJoCo/RViz 前，先说明命令、目的及会产生的文件。
- 未经明确授权不得安装依赖、推送远端、改写 Git 历史或执行大范围进程清理。
- `sim_dual_moveit.launch.py` 已包含 MuJoCo，不得重复启动仿真实例。
- `dual_panda_arm_controller` 与 `sim_multi_mode_controller` 不得同时 active。
- 当前 CHOMP、外部笛卡尔阻抗和模式 B 视觉均未完成，文档、日志和验收不得把框架描述成成品。
- 核心运动、碰撞、控制器和硬件接口变更必须包含验证方案，并接受独立审查。

## 当前常用入口

```bash
colcon build --packages-select dual_arm_carry_task franka_hardware franka_moveit_config
source install/setup.bash
./start_dual_arm_task.sh
```

手动分步启动见 [`MANUAL_STARTUP.md`](MANUAL_STARTUP.md)，任务包参数见
[`src/multipanda_ros2/dual_arm_carry_task/README.md`](src/multipanda_ros2/dual_arm_carry_task/README.md)。
