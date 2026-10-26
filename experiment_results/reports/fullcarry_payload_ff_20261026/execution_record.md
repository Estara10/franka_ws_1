# 执行记录

- 基础提交：`81b7204`；未提交或推送Git。
- 本地改动：仅 `contact_load_probe.cpp` 增加运输名义/实际杆参考和双手参考、请求响应时刻、前馈命令时序记录。未改运动、赋时、刚度、力、保护或判据。
- 定向构建：`colcon build --packages-select dual_arm_carry_task`，最终通过。出现工作区原有metapackage及CMake/Boost兼容性提示。
- 静态核对：`run_batch.py`、`analyze_batch.py` Python编译通过；20行正式顺序、10个双条件区组及5/5先后平衡核对通过。
- 仿真命令：`python3 experiment_results/reports/fullcarry_payload_ff_20261026/run_batch.py pretrial`，headless、单实例、先M0后M1。前两次M0及第三次动作前中止均因沙箱阻断FastDDS本地UDP服务发现；使用允许本地DDS通信的执行权限后，M0第4次启动及M1第1次启动完成。
- 预试验：M0 1/1完成、M1 1/1完成；完整抓取、抬升、运输、下降、支撑确认、释放和撤退。名义运输开始距离MMC激活约4.43 s，未加承载等待。两条件运输事件中位移真值请求/响应时间使用中点对齐。
- 进入判断：运输计划时长3.037 s对2.244 s，实际运输3.048 s对2.248 s；计划时间相差0.792 s（26.1%），实际运输时间相差0.800 s。按归一化进度比较，实际杆参考Z相差8.302 mm。虽然共同名义高度指标可计算，但运输期间新增偏差无法与状态依赖参考和时长差区分，故门槛未通过。
- 正式动作：0/20；没有创建 `PRETRIAL_GATE_PASS`，正式运行器保持锁定。
- 重现预试验分析：`python3 experiment_results/reports/fullcarry_payload_ff_20261026/analyze_batch.py`。
- 当前批次正式命令（会因门槛文件缺失立即拒绝，不会启动仿真）：`python3 experiment_results/reports/fullcarry_payload_ff_20261026/run_batch.py formal`。
