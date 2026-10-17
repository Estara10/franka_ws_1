# 本机更新与首次运行交接（2026-10-15）

> 运行约定更新（2026-10-17）：用户已授权AI在 `/home/lidio/franka_ws_1` 本机自主执行
> 测试与仿真，详见 [项目指南](PROJECT_GUIDE.md)。下文“仅用户运行”等旧限制在本机已被覆盖；
> 历史未运行/失败事实保持不变，队友机器不自动适用本授权。

本次已从 `https://github.com/Estara10/franka_ws_1` 的 main 快进更新至
`20a08ae25c9c2e7e7560619504f0399dcd6d478e`。本机 MuJoCo 3.2.0、DQ Robotics
的兼容修复、VS Code 配置、已有日志与实验数据保留。同步前的本地文件和补丁备份在
`.deps/sync_backups/20261015_162821/`。

依赖检查通过，全量编译 `33 packages finished [2min 14s]`，进程退出码 0。
7 个包有 stderr，主要为既有编译/CMake 警告，没有 failed 或 aborted。
新版核心节点与插件的 `ldd` 无缺失项，launch 的 Python AST 和脚本语法检查通过。
`compile_commands.json` 包含 690 条记录，完整包含 21 个包编译数据库的条目。
这些检查不代表已在本机运行验证；本轮没有启动 ROS 节点、仿真、dry-run、测试或实验。

本轮编译日志在 `log/build_2026-10-15_16-29-00/`；合并终端日志在
`/tmp/franka_sync_20261015_build.log`，后者可能被系统清理。

## 当前技术路线

以 [PROJECT_GUIDE.md](PROJECT_GUIDE.md) 的 2026-10-14 主线收敛和
[S1 最后续办记录](deliverables/S1_理想抓持搬运基线记录.md) 为准。
根 README 和部分历史段落仍有旧状态，不能只看文档开头判定最终结果。

已完成的 S1 是 ideal_weld 理想刚性抓持、保持 yaw 的闭环对照基线：
初始杆 `(0.50,0,+10°)` 到 `(0.40,0,+10°)`，另有 yaw=0 场景。
这条链使用 A1 真值准备与有效性检查、预抓取和下降、捕获 object→hand 变换、
物体空间路径、连续 seeded 双臂 KDL IK、独立 criterion 模型碰撞/FK/插值检查、
IPTP 赋时及统一时间伸缩、位置轨迹执行、释放和上撤退。
S1 搬运路径的赋时应与旧 A0 的 TOTG/Ruckig 分支区分。

同学记录的两场景终态均为 DONE；最终物理杆误差分别为
0.766 mm/0.314° 和 0.789 mm/0.259°，均为历史机器结果。
90°旋转及旧侧向目标 `(0.40,-0.20)` 曾被碰撞检查拒绝，不在本次复现命令中使用。
旧默认 static/A0 在新版夹爪协议下也有失败记录，不能直接沿用旧入口假定成功。

当前研究优先级是无 weld 的接触夹持与 5 cm 抬升承载，再接通同目标抓放；之后开展真实
几何轨迹优化、同轨迹柔顺对比和结题实验。CHOMP、完整外部笛卡尔阻抗与视觉仍未完成。
不要把 ideal_weld 成功当成摩擦抓取成功，也不要将文档中的执行器提示词自动视作
本轮已授权的开发或实验任务。

## 用户执行：首次单场景完整复现

所有运行与实验由用户亲自执行和监督。以下命令没有由 Codex 执行。
运行前先在已有仿真终端用 Ctrl+C 正常结束旧会话；本命令会自行启动唯一的
MoveIt + MuJoCo + RViz 实例，不要预先另外启动模拟器或任务节点。

在本机有桌面的 Ubuntu 终端执行：

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
export RUN_ID="franz_s1_$(date +%Y%m%d_%H%M%S)"
export ROS_LOG_DIR="$PWD/launch_logs/$RUN_ID"
mkdir -p "$ROS_LOG_DIR"

bash scripts/a1_run_scene.sh \
  --run-id "$RUN_ID" --domain-id 87 --seed 20261011 \
  --prepare-mode --grasp-model ideal_weld \
  --x 0.50 --y 0.0 --yaw-deg 10.0 \
  --target-x 0.40 --target-y 0.0 --target-yaw-deg 10.0 \
  --use-rviz --task-timeout 240 --hold-sec 300
```

`--prepare-mode` 在此是进入 A1 真值链的必要参数；显式 `ideal_weld` 会继续执行抓放，
不是只准备不运动。浮点参数保留 `.0`，避免 double 参数收到整数值后启动失败。
GUI 有问题时先保留失败日志，不直接换模式或重复运行；根据具体日志给出下一条命令。

当前 `franka_dual_arm` 已初始化为独立 Git 仓库，仓库元数据位于本目录 `.git`，
脚本会直接通过 `git -C "$PWD"` 记录版本和 dirty 状态。当前尚未自动提交或推送。
手动查看仓库状态可以用：

```bash
git status --short
git log -1 --oneline
```

脚本创建 `experiment_results/a1_runs/$RUN_ID/`，负责单次场景、真值、任务和子进程收尾。
初始化阶段允许写入一次初始杆位姿；搬运阶段 ideal_weld 是本次显式选择的理想抓持模型。
不要添加 `--grasp-contact-close`，该接触分支目前仍有未通过记录。

## 观察与取回结果

观察：双臂预抓取→下降→理想固定抓持→抬升→向 x=0.40 搬运→放下→释放→向上撤退。
本场景保持 +10°，不期待 90°旋转。脚本完成后会关闭其启动的会话。

成功需同时包含最终结果 `S1_IDEAL_WELD_RESULT state=DONE`、物理杆释放后误差，
以及 weld 关闭、detach、开爪和撤退确认。只看到 PREPARED、PREGRASP_REACHED
或 GRASP_POSE_REACHED 不是完整搬运成功。异常时 Ctrl+C，保留该次目录，不重复扫描参数。

同一个终端在脚本返回后执行：

```bash
cat "experiment_results/a1_runs/$RUN_ID/prepare_status.json"
rg -n 'S1_IDEAL_WELD_RESULT|S1_RELEASED_TRUTH|ERROR|FAILED|E_S1_' \
  "experiment_results/a1_runs/$RUN_ID/task_launch.log"
tail -n 60 "experiment_results/a1_runs/$RUN_ID/task_launch.log"
```

若 task_launch.log 尚未生成，说明失败发生在任务启动前，查看该目录的
`runner.log`、`sim_dual_moveit.log`、`script_status.json` 和终端输出。
每次使用新的 RUN_ID；不要覆盖已有数据。先交回这一场景结果，再安排 yaw=0 或接触验证。

## 后续无 weld 承载开发

2026-10-15 新增待运行 `contact_probe`。原文的首次 ideal_weld 运行记录保留。
下一项实验使用 [CONTACT_LOAD_PROBE.md](CONTACT_LOAD_PROBE.md) 中的新命令，
只做 5 cm 抬升承载，不将未验证的接触分支描述成已完成。
