# 启动异常定位

2026-10-26名义场景M0预试验前两次均未下发预抓取运动，`a1_run_scene.sh`在120 s内未发现 `/set_body_state`、`/get_body_state`、`/get_sim_info`。MuJoCo日志显示控制器已加载，但FastDDS反复报告 `getifaddrs: Operation not permitted`、`All whitelist interfaces were filtered out`、`Error creating socket: Operation not permitted`；服务列表为空。这表明本次运行环境的沙箱网络权限阻止本地DDS接口枚举/UDP socket，直接原因是执行环境权限，不是模拟场景、算法或控制器参数。

没有改动DDS配置、控制或物理参数。第三个受限启动在下发动作前被本任务中止并保留原始目录。随后在本机允许本地DDS socket的执行上下文中进行有界恢复，服务正常出现，M0/M1预试验均完成。因此已验证这是沙箱执行权限限制；未更改项目通信配置或安装依赖。
