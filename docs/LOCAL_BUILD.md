# Ubuntu 22.04 本地编译配置

工作区：`/home/franz/my-projects/franka_dual_arm`，ROS 2 Humble。

2026-10-19 本地已编译接触悬空诊断、Franka 硬件和任务包；最新代码状态、运行命令和
实验边界见 [HANDOFF_20261019.md](HANDOFF_20261019.md)。

2026-10-13 验证结果：全量 `33 packages finished [58.4s]`，退出码 0，无 failed/aborted。
`rosdep check --from-paths src --ignore-src` 通过；任务、MuJoCo、Franka 硬件和运动
生成器包均可被 ROS 发现，关键可执行文件及硬件插件的 `ldd` 无缺失动态库。
构建日志：`/tmp/franka_ws_1_colcon_build_dq.log`（临时目录文件可能被系统清理）；
colcon 也保留工作区 `log/` 下的日志。剩余 stderr 是编译警告，不影响本次构建。

本次迁移采用 MuJoCo 3.2.0（`~/.local/mujoco-3.2.0`），避免系统 ROS vendor 的
MuJoCo 3.12.0 与旧 viewer API 不兼容。`Findmujoco.cmake` 已让显式 `MUJOCO_DIR`
优先，另外修复了生成头文件安装的自链接及下游错误查找 `mujocoConfig.cmake` 的问题。

`panda_motion_generators` 需要额外的 DQ Robotics C++ 库。该依赖没有被现有
`package.xml` 完整声明，因此 `rosdep check` 通过不能证明它已安装。现已安装到
工作区 `.deps/dqrobotics`；CMake 显式查找头文件和动态库，使用导入目标传递路径。
本机保留系统 Eigen，已验证这套组合可以编译，无需照旧 README 降级。
本机 libfranka 为 ROS Humble 提供的 0.20；本次仅验证构建和动态库解析，
没有验证真实机器人固件兼容性，当前目标仍是仿真复现。

DQ Robotics 来源：<https://github.com/dqrobotics/cpp>，本次使用提交
`d4fd2838fe26cccaa4b6e45ad9c7e5e628d10213`。如需重新安装，在工作区执行：

```bash
mkdir -p .deps
touch .deps/COLCON_IGNORE
git clone https://github.com/dqrobotics/cpp.git .deps/dqrobotics-src
git -C .deps/dqrobotics-src checkout d4fd2838fe26cccaa4b6e45ad9c7e5e628d10213
cmake -S .deps/dqrobotics-src -B .deps/dqrobotics-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/.deps/dqrobotics"
cmake --build .deps/dqrobotics-build --parallel 2
cmake --install .deps/dqrobotics-build
```

以后全量编译：

```bash
cd /home/franz/my-projects/franka_dual_arm
bash scripts/build_local.sh
source install/setup.bash
```

脚本加载 Humble、选择上述 MuJoCo 和 DQ Robotics，并生成 `build/`、`install/`、
`log/`。可以传入 colcon 包选择参数，例如 `--packages-select panda_motion_generators`。
MuJoCo 安装位置不同时，可通过环境变量 `MUJOCO_DIR` 指定另一个 **3.2.0** 安装。

`aborted` 意为因本次构建中其他包失败而中止，不表示包被废弃。`stderr output`
也可能只是警告，应以首个编译错误和最终 `failed`/`aborted` 汇总判断。

编译通过只证明构建链路成立；双臂搬运的运行复现仍需单独进行 A0 仿真验证。
