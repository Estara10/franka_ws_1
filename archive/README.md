# 历史归档

本目录保存已经退出当前开发主线、但仍需保留来源和恢复能力的文件。整个目录由
`COLCON_IGNORE` 隔离，不能作为当前构建、启动或依赖解析入口。

## 归档内容

| 当前路径 | 原路径 | 归档原因 | 当前替代项 |
|---|---|---|---|
| `legacy_dual_arm_stack/dual_arm_description` | `src/dual_arm_description` | 早期双臂描述包，不属于当前一键启动链 | `src/multipanda_ros2/franka_description` |
| `legacy_dual_arm_stack/dual_arm_moveit_config` | `src/dual_arm_moveit_config` | 早期 MoveIt 配置，与当前规划组及控制器配置不同 | `src/multipanda_ros2/franka_moveit_config` |
| `legacy_dual_arm_stack/dual_arm_control` | `src/dual_arm_control` | 早期控制演示，不属于当前任务状态机 | `src/multipanda_ros2/dual_arm_carry_task` 及当前控制器包 |
| `legacy_dual_arm_stack/dual_arm_task` | `src/dual_arm_task` | 未完成的 Python 包脚手架 | `src/multipanda_ros2/dual_arm_carry_task` |
| `legacy_dual_arm_stack/generated/dual_panda.urdf` | `dual_panda.urdf` | 旧描述栈生成物 | 当前 xacro/launch 生成的机器人描述 |
| `docs/history/ISSUE_RESOLVED.md` | `ISSUE_RESOLVED.md` | 已解决问题的阶段性记录 | Git 历史和当前项目指南 |
| `docs/history/interactive_rviz/*` | `src/multipanda_ros2/{QUICK_START,CONTROLLER_TROUBLESHOOTING,RVIZ_INTERACTIVE_GUIDE}.md` | 包含旧路径和已不存在的启动入口 | 根 `README.md`、`MANUAL_STARTUP.md` |
| `code/history/patches` | `scripts/patches` | 一次性历史修补脚本，不是正式构建链 | 当前受版本控制的源文件 |
| `code/history/dual_arm_carry_task_src_backup` | `src/multipanda_ros2/dual_arm_carry_task/src_backup` | 旧单体实现及备份，不由当前 CMake 编译 | 当前模块化任务源码 |
| `backups/codex_backups` | `codex_backups` | 历史补丁备份 | Git 历史 |
| `logs/MUJOCO_LOG.TXT` | `MUJOCO_LOG.TXT` | 历史运行日志 | 新运行产生的日志目录 |
| `notes/huatu` | `huatu` | 无扩展名的历史绘图笔记 | 根目录现有分析脚本 |
| `tooling/legacy_empty_codex_file` | `.codex` | 零字节占位文件，阻止创建项目 Agent 配置目录 | 根 `.codex/` 目录 |

归档操作只改变路径，不改动上述文件内容。`experiment_results`、诊断脚本、核心源码、
MoveIt/MuJoCo 配置和视觉框架均未归档。

## 恢复方式

需要恢复历史包时，请从归档前基线 `2e4371a` 或本地备份分支
`backup/pre-multi-agent-20260820` 新建实验分支，再将所需旧路径恢复到 `src/`。
不要直接移除本目录的 `COLCON_IGNORE` 后构建，因为这会让旧包和当前包同时被 colcon
发现，重新引入包名、入口和依赖歧义。

示例（仅作为恢复说明，不应在当前主线执行）：

```bash
git switch -c restore/legacy-stack backup/pre-multi-agent-20260820
```

## 旧构建产物风险

归档源码不会清除已有的 `build/`、`install/` 和 `log/`。旧包可能仍残留在已安装环境中，
使 `ros2 pkg` 看起来仍能发现它们。需要验证归档效果时，应使用新的干净构建目录；不要把
旧 `install/setup.bash` 中的可见包误认为当前源码仍在参与构建。
