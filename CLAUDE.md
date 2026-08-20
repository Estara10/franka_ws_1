# CLAUDE.md

这是 Claude Code 在本仓库中的工具入口。公共项目背景不在此重复维护；开始工作前请完整阅读
[`docs/PROJECT_GUIDE.md`](docs/PROJECT_GUIDE.md)。Codex 专属约束见 `AGENTS.md`。

## Claude Code 工作约束

- 当前实现只使用 `src/multipanda_ros2`；`archive/` 为历史材料，不参与构建。
- 不覆盖用户未提交修改，不把归档的旧 `src/dual_arm_*` 包当作当前运行入口。
- 执行构建、测试、ROS、MuJoCo 或 RViz 前先说明影响。
- 未经明确授权不安装依赖、不推送、不改写 Git 历史、不使用宽泛 `pkill`。
- MoveIt 仿真 launch 已自动包含 MuJoCo，禁止重复启动。
- 位置轨迹控制器与多模式阻抗控制器不能同时 active。
- 对 CHOMP、外部阻抗和视觉框架的完成度必须如实描述。

用户入口见 [`README.md`](README.md)，手动启动见 [`MANUAL_STARTUP.md`](MANUAL_STARTUP.md)。
