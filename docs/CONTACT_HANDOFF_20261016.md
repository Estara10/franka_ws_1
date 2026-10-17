# 真实接触抓取阶段交接说明（2026-10-16）

> 运行约定更新（2026-10-17）：用户已授权AI在 `/home/lidio/franka_ws_1` 本机自主执行
> 测试与仿真，详见 [项目指南](PROJECT_GUIDE.md)。下文“仅用户运行”等旧限制在本机已被覆盖；
> 历史未运行/失败事实保持不变，队友机器不自动适用本授权。

本文面向下一位继续开发者，记录本轮代码、诊断证据、当前边界和下一步计划。
公共架构与项目定位仍以 [`PROJECT_GUIDE.md`](../PROJECT_GUIDE.md) 为准。

## 已完成的工作

- `franka_dual_arm` 已初始化为独立 Git 仓库，`origin` 为 `Estara10/franka_ws_1`。
- 保留并编译通过 `ideal_weld` 基线；该入口仍使用原始 weld/equality 逻辑，未改义。
- 新增 `contact_probe` 无 weld 实验入口：双侧接触闭合、force-hold、5 cm 抬升、2 s 保持、下降和释放。
- 建立了从 MuJoCo 物理线程到任务 CSV/日志的接触诊断链，记录：
  - 指位、命令、force-hold 请求；
  - 每侧指垫接触数和接触布尔量；
  - 法向/切向力、杆竖直受力；
  - feedback sequence、仿真时间、weld/equality 状态。
- 修复了左右夹爪 trace 混写问题，当前输出为：
  - `gripper_fingers_mj_left.csv`
  - `gripper_fingers_mj_right.csv`
- 接触 action 的时间预算、接近速度和 force-hold 结果等待已调整；当前可成功进入 LIFT。
- `contact_probe` 默认关闭内部 MuJoCo 自适应柔顺并使用刚性位置验证配置，避免与摩擦承载诊断混杂。
- 抬升轨迹已改为 2.5 mm 物体空间步长、每段至少 1 s，减少轨迹离散造成的接触冲击。
- `contact_probe` 已增加运行时单主指垫 profile：辅助指垫碰撞关闭，主指垫尺寸和法向接触时间常数做有界调整；`ideal_weld` 不启用该 profile。

## 关键实验事实

截至 `franz_contact_probe_fix8_20261016_222657`：

- A1 P2a/P2b、预抓取和下降均通过；
- `ideal_weld` 路径此前可以完成理想刚性抓持；
- contact-probe 的 single-primary-pad profile 已生效（禁用 16 个辅助 pad，调整 4 个主 pad）；
- `contact_probe` 已通过双侧有效力接触建立：
  - `bilateral_force_established`
  - 左右约 5–8 N 法向力；
  - weld/equality 均为关闭；
- fix8 的 LIFT 仍失败，状态为：
  - `E_CONTACT_SLIP_OR_DROP`；
  - 接触 action 已报告 `bilateral_force_established`，所以不是闭合阶段失败；
  - LIFT 中杆最高约只到 `z=0.02070 m`，基本没有脱离支撑面；
  - 最终左右滑移约 `5.0 mm`；
  - 接触数量通常为 8，但失败前会降到 3，且一侧接触布尔量变为 `(0,1)`；
  - 失败时出现单侧法向力接近 0、另一侧法向力/切向力突增的接触切换。

因此当前已证明：

```text
真实接触闭合和有限 force-hold 可以建立
但无 weld 的摩擦承载抬升尚未通过
```

当前不能声称真实接触搬运完成，也不能把 `CONTACT_PROBE` 失败归因于状态机未连通。
状态机已进入 `LIFT`，失败发生在物理承载阶段。

## 当前核心问题

当前最重要的技术问题不是简单的“夹持力不够”，而是抬升过程中接触稳定性不足：

1. 主指垫仍会出现单侧接触或接触数量变化；
2. 夹爪与杆之间的有效接触法向会突变；
3. 杆仍受到支撑面约束，初始脱离阶段对摩擦和冲击最敏感；
4. 5 N/指已经达到当前 force-hold 实现的上限，继续无依据增大不可取；
5. 当前杆模型约 0.64 kg，与目标报告中的 0.5 kg 不同，实验结论必须注明模型质量。

日志中的视觉/MoveIt/关闭期进程错误（如 octomap 未配置、关闭时 process died）不是本轮真实接触首因；
应与任务过程错误分开记录。

## 下一步建议顺序

### 1. 先做单主指垫 profile 的 fix8/fix9 对比

确认日志包含：

```text
single-primary-pad profile: disabled 16 auxiliary pad geoms, tuned 4 primary pads
bilateral_force_established
```

并比较 LIFT 中：

- `rod_pad_contact_count` 是否稳定；
- 左右法向力是否同时非零；
- `rod_force_z_n` 是否具有持续向上的合力；
- 5 mm 滑移发生的仿真时间和杆高度。

### 2. fix8 后的当前判断：主 pad 接触仍会丢失

fix8 说明仅减少辅助 pad 还不够；主 pad 在 LIFT 中仍会出现单侧接触和冲击。下一步应增加每个杆—pad 接触的 geom 名称、接触法向方向和接触点位置记录，确认当前主 pad 是否真正位于杆的相对两侧，还是只擦到杆边缘。不要先无限增加摩擦。

### 3. 再做有限摩擦对比

只有在几何接触方向确认正确后，才做小范围、可解释的对比，例如保持杆质量不变，仅比较：

```text
friction[0] = 3.0（当前基线）
friction[0] = 4.0（单一对比）
```

每次只改一个物理因素，不混合不同质量、不同 pad profile 和不同轨迹数据。

### 4. 若仍不能抬升，改成“脱离支撑面”两阶段动作

可研究先极小幅、低冲击的预抬升/预压动作，确认杆底面脱离支撑后，再进入 5 cm 连续抬升。
但任何预抬升仍必须由真实接触和物理真值验证，不能用 set_body_state、weld 或瞬移替代。

## 静态力诊断核对（2026-10-17，接续 44543dc）

沿物理线程→互斥共享快照→`GripperContactFeedback`→action trace / `contact_probe.csv` 核对后，确认两处诊断错误并作最小修复：MuJoCo contact frame 的轴向量存放在矩阵行，接触力转世界系应使用 `mju_mulMatTVec3`；旧 `rod_force_xyz` 使用 `mju_mulMatVec3`，与其单独计算的 `rod_force_z` 投影不一致。现在 xyz 与 z 使用同一转置投影。接触法向从 geom1 指向 geom2，代码以 rod 为 geom2 时取正、rod 为 geom1 时取负，得到“作用在杆上”的力，符号规则正确。

`gripper_fingers_mj_left/right.csv` 旧 writer 漏写 `rod_force_z_n`，导致 sim_time 和后续列错位；现已补齐。字段口径：每条 typed feedback 属于一个夹爪/一侧机械臂；其 `normal_left/right_n`、`tangent_left/right_n` 分别是该夹爪左/右指体所对应所有 pad—杆接触点的力幅值和，不能与另一夹爪重复计数；`rod_force_xyz_n` 是当前夹爪两指共同作用在杆上的世界系净力。两侧杆竖直合力用左右夹爪各自 `rod_force_z_n` 相加一次；单指法向力、驱动的 `hold_force_per_finger_n` 请求值和杆净力不是同一量。`contact_probe.csv` 中 left/right 前缀指机械臂夹爪，normal1/2 指该夹爪两指。

当前反馈没有逐接触 geom 名、接触点和每指三维杆力，所以仅靠这些聚合字段仍不能断言四指是否在杆的相对两侧形成非擦边夹持；要做该几何结论需补这些观测，不能从两个 contact 布尔量猜测。已按本机已有配置 `/home/lidio/Libraries/mujoco`（MuJoCo 3.2.0）构建 `dual_arm_a1_msgs`、`franka_hardware`、`dual_arm_carry_task`，3 包成功；有既有 MuJoCo/MoveIt CMake warning 和 hardware signedness warning。本轮遵照用户约定没有运行测试、ROS 或仿真。编译只确认接口和实现可构建，力诊断修复仍待用户的一次 +10° `contact_probe` 运行数据验证。

## 代码和实验边界

- `ideal_weld` 必须持续作为理想模型回归入口保留。
- `contact_probe` 的 `CONTACT_LOAD_VALIDATED` 仍需同时满足抬升高度、保持时间、滑移、受力占空比和释放条件。
- 不得把接触 action 成功、双指闭合或单次接触帧当作承载成功。
- 不得直接把摩擦无限增大、减轻杆质量、放宽滑移阈值来制造成功。
- 所有 ROS/MuJoCo/RViz 运行和实验由用户执行；Codex 只提供命令、编译、静态检查和日志分析。

本机单次运行包装脚本：`../scripts/run_contact_probe_local.sh`。它保留调用者已设置的 `MUJOCO_DIR`（未设置时使用本机现有安装），固定 `(0.50, 0, +10°)` 和当前刚性 `contact_probe` 配置，为运行与 ROS 日志分配独立目录，只调用一次现有 runner，不构建、不重试、不清理其他进程。用户执行 `./scripts/run_contact_probe_local.sh` 后请回传该脚本打印的运行目录中的 `contact_probe.csv`、两侧 `gripper_fingers_mj_*.csv`、`task_launch.log`、`sim_dual_moveit.log`、`runner.log`、`prepare_status.json` 和 `controllers.txt`。初版脚本误传 `--headless`；现已改为 `--use-rviz` 且不传 headless 参数，使后续用户启动显示可视化。

## 10 N无weld承载调试（2026-10-17，本轮有界试验）

本轮实际物理试验预算已用完：`094120`、`094526`、`094822` 三次均到达抓取位并发送接触 grasp；`093959` 在任务目标发送前启动失败，不计为物理试验。三次有效试验均未通过接触动作稳定窗，因此没有进入抬升，本轮没有验证新的承载能力。

`094822` 在显式保持期把每指附加dof阻尼从15.6临时增至31.2 N·s/m（总阻尼约32.2），其余力限、摩擦、质量和判据不变。数据否定“单纯加倍该阻尼即可稳定10 N夹持”：驱动到约−20 N腱力上限后，末端gap仍以约0.96–1.13 mm波动，末窗双侧接触占空仅13–14/20；各手掌在相邻0.2 s内位姿变化约0.1 mm量级，杆仍贴支撑面。失败诊断时左侧约11.5/11.6 N法向且双指接触，右侧出现单指接触与约−3.57 N杆竖直合力；杆未被抬起。夹爪action以 deadline/`E_CONTACT_GRASP_ACTION_FAILED_OR_UNKNOWN` 失败，weld及杆固定约束关闭。`094120` 与 `094526` 同样未过稳定窗；所有运行目录及逐接触CSV均保留。

本轮临时阻尼加倍已撤回；默认与已有16.6 N·s/m总阻尼基线一致。当前证据支持的结论是：10 N驱动下存在指关节/接触持续振荡及间歇单侧失接触，动作失败并未真正进入承载测试；不能据此把原先“接触动作成功但杆不离地”的滑移问题判为已解决，也不能仅以0.5 mm gap标准过严解释。既有 `085803` 的约5.1 mm双侧滑移和不足0.1 mm净升高仍是物理承载失败证据。下一步应先针对饱和腱位置伺服的闭合保持动力学提出一个可检验修复，再用单次有界运行确认；若仍无法达到真实离地、2 s保持、落回和释放条件，应报告阻塞，不扫描参数或放宽判据。`run_contact_probe_local.sh` 的只读诊断数据和历史记录不覆盖。

## 首次本机 contact_probe 数据（2026-10-17，run `lidio_contact_probe_yaw10_20261017_085140`）

该次确已启动 MuJoCo 3.2.0 并运行物理线程，但包装脚本传入 `--headless`，所以无屏幕窗口；日志明确记录 `Running headless`、offscreen render thread，仿真服务、控制器和动作均已工作。任务完成预抓取、开爪、下降及接触闭合，接触反馈返回成功；这只代表动作协议完成，不代表承载。5 cm 抬升开始约 40.924 s，因左右抓取参考滑移分别越过 5 mm 限值（终点 5.064 / 5.031 mm）而以 `E_CONTACT_SLIP_OR_DROP` 中止。杆 z 从抬升首样本约 20.606 mm 到终点约 20.687 mm（净升高约 0.08 mm；整个采样最大起伏约 0.18 mm），仍由支撑面承托。

修正后的“作用在杆上”世界系竖直力在抬升样本中净值均值约 +0.29 N、中位数 +0.09 N，仅 8/119 个样本达到或超过 6.28 N；最大 +26.44 N 是短暂样本，终点约 −0.78 N，不能视为持续承载。两夹爪每指接触布尔量有间歇丢失，末端左侧 `pads=0, contacts=(0,0)`，右侧仍有 7 个 pad 接触、`contacts=(1,1)`；两侧滑移同步累积。现有数据最支持“接触不能形成持续向上的承载合力并伴随接触中断”，但不能区分擦边几何与驱动保持/负载分配：CSV 缺少逐接触点/geom 法向和逐指作用在杆上的三维力，且 `gripper_fingers_mj_left/right.csv` 记录在抬升开始前即结束（约 40.3 s），没有抬升期 actuator 力/速度轨迹。当前不据此调摩擦、预压或质量；下一步如需定位，应先补齐每接触几何与抬升期驱动观测，再只改变一个有证据支持的因素。

## 可视模式启动异常（2026-10-17，run `lidio_contact_probe_yaw10_20261017_085514`）

可视模式下 MuJoCo 窗口实际启动并连接，但在插件加载阶段退出：`mujoco_ros2_control` 收到 robot_description 后抛出 `spin_some() called while already spinning`；随后 controller_manager 服务消失，truth runner 以 `E_SERVICES_UNAVAILABLE` 退出，未发布 truth、未启动任务，也没有接触/物理试验数据。调用链已静态确认：`mujoco_ros` 主进程线程持续对共享 MultiThreadedExecutor 调用 `spin_some()`，而插件 `getURDF()` 又在同一 executor 上调用 `spin_until_future_complete()`，存在嵌套/并发 spin。

已在 `mujoco_ros2_control::getURDF()` 改为等待异步参数 future，由已有 executor 线程处理请求；未启动或重试仿真。仅构建 `mujoco_ros2_control` 成功（本机 MuJoCo 3.2.0），出现已有 viewer initializer warning 与 colcon underlay override warning；没有运行测试、ROS 或仿真。此修复尚需用户下一次可视启动确认；本次失败目录和日志完整保留。

## 接触保持动力学有界验证（2026-10-17）

按用户授权，本轮改为本机运行。实现选择固定预压位置参考加显式 actuator 力限幅：MuJoCo 位置 actuator 为 `F=kp*(ctrl-tendon_length)`，`kp=1200 N/m`，左右指通过0.5 tendon gear分力；故10 N/指对应约20 N actuator/tendon幅值上限，并非20 N垫面法向力。显式保持参考锁存在接触时的tendon长度基础上预压15 mm（`0.9*20/1200`），接触保持期间由物理线程把 actuator 范围限至±20 N，退出时恢复模型原范围；普通开爪/位置命令和 `ideal_weld` 不走该保持分支。没有改质量、摩擦、接触几何、阻尼或验收阈值。针对性单测中共享保持/协议/接触判据通过；两个受影响包构建通过。客户端故障测试的断言日志通过，但进程在DDS权限报错后以信号退出，单列为环境性测试异常；全套指定测试未全绿。

唯一机制工况为 `lidio_contact_probe_yaw10_20261017_100700`；另一个 `100451` 仅在truth/任务启动前因沙箱DDS/UDP权限导致仿真退出，不计有效物理工况，目录保留。有效运行完成预抓取和下降并触发双臂显式抓取，但12 s内没有通过原有接触稳定窗，未进入隔离保持观测，更未执行抬升。左右action均以deadline失败；最后20个新样本里双指占用为14/20，gap范围分别约0.996 mm与0.857 mm，未满足现有窗条件。逐接触trace显示保持期actuator限幅为20 N，实际力约−17.5至−18.2 N，未触到力限边界；指位置/接触仍振荡并有间歇失接触。该证据不支持“本次因达到力饱和而失败”，但也不足以单独区分离散反馈、接触求解或接触几何切换。杆全程仍由支撑面承托；无weld/杆固定约束启用。机制验证失败，依用户预算规则停止，未启动第二个完整contact_probe工况；因此本轮没有无weld抬升、2 s保持、落回/释放成功结论，也未简化指标或把action失败改判成功。所有构建、测试及两个运行目录保留，未提交或推送。

## 直接腱力对照与两工况结果（2026-10-17）

几何核查使用 `lidio_contact_probe_yaw10_20261017_100700/contact_probe_contacts.csv`。以杆真值姿态把接触点与法向转到杆局部系后，四个主 pad 均落在杆的相对侧面，接触 x 约 ±19.4 mm（杆半宽20 mm）、z 近0（不在上下底面），指端沿杆长方向约 ±150 mm；法向相反并沿局部 x。未见明确擦底、顶边或接触面切换错误。该数据接触距离约 −0.44…−0.79 mm、有效 margin 1 mm，运行时混合摩擦为 `(3,3,0.5,0.005,0.005)`。本轮没有修改接触几何、质量、摩擦、求解器或阻尼。

保持对照只在显式 `contact_probe` 中临时把位置 actuator 切到直接腱力：原编译 actuator 为 fixed-tendon position servo，`kp=1200 N/m`、`gear0=1`、split tendon 系数 `(0.5,0.5)`；每指10 N请求乘0.9后以 ctrl≈−18 N 输出，约对应每指−9 N广义力，不等同垫面法向力。物理线程保存并恢复原 dyn/gain/bias 类型及参数、gear、ctrl/force范围与限位标记；普通位置命令和 `ideal_weld` 不进入此路径。

启动目录 `103452` 只因 sandbox 禁止 DDS socket 而未到任务阶段；有效原地保持数据在 `lidio_contact_hold_directforce_A_20261017_1036_retry`。两只夹爪均进入直接力保持并达到10 N/指请求、actuator约−18 N；窗口末端占用为左夹爪19/20、右夹爪18/20，gap散布分别0.670与0.768 mm，未满足原0.5 mm窗而以deadline失败。对照 `100700` 的14/20和0.857/0.996 mm，本轮数据支持直接腱力减少接触丢失并改善gap稳定，但尚不证明承载。按用户允许的接触窗简化，显式力保持现在冻结为20个新鲜样本、每指至少16/20接触、整窗gap散布≤1.0 mm、原12 s deadline不变；承载高度、2 s保持、5 mm/2°滑移、受力占空和释放条件均未放宽。单侧持续失接触仍失败。

工况 B `lidio_contact_directforce_lift_B_20261017_103801` 中，双臂 action 均按新窗成功（gap散布0.758/0.879 mm，接触占用16/20），预抬升观测通过；随后真实 lift 失败为 `E_CONTACT_SLIP_OR_DROP`。杆最高仅约0.020941 m，相对初始约升0.27 mm，仍在支撑面；左右相对抓取参考最大滑移约5.135/4.913 mm，右夹爪出现一指失接触，末态杆向上净接触力不足以承担约6.28 N重力。全程 weld/equality 与杆固定约束为关闭。没有通过抬升、保持、落回或释放验收；不报告承载通过。

失败日志也暴露关机时 controller manager 已停止调用物理 `write()`，单靠 `on_deactivate` 不能恢复正在使用的直接力模式。随后增加了失败收尾的双侧 measured-position Stop 服务请求，并让硬件停用等待物理线程确认恢复；任务失败原因保持不变，不开爪、不重试。该收尾补丁构建成功，但因本轮物理预算已用尽，没有再做仿真验证。最终 `franka_hardware` 与 `dual_arm_carry_task` 构建通过；共享状态定向测试36/36、夹爪 client 故障传播测试11/11通过。工作区其他历史 lint 结果未作为本轮验收。

## 103801 逐接触滑移核查与唯一 NoSlip 对照（2026-10-17）

仅分析 `lidio_contact_directforce_lift_B_20261017_103801` 的原始数据后，确认杆在 LIFT 的 7.678 s 内仍未离开支撑：中心 z 最大仅增加 0.263 mm；两臂抓取参考滑移从 0.081/0.196 mm 累积至 5.135/4.913 mm，平均净增长约 0.66/0.61 mm/s。由手、指关节和杆位姿有限差分、投影到接触切平面的活动接触相对速度估计，中位约 15.9/17.3 mm/s，p90 约 44.9/42.9 mm/s；这是运动学估计，不是 MuJoCo 接触约束的直接滑移速度。任务采样帧中两指同时有接触的比例为左 86/124、右 98/124；接触在持续渐进滑移期间反复切换，失败末端出现单指失接触（基线诊断时右夹爪一指法向力归零）。因此软接触渐进滑移与接触切换均有直接支持，接触切换/局部卸载很可能促成最后失败。

摩擦不足证据较弱但不能完全排除：编译后 friction 为 `(3,3,0.5,0.005,0.005)`、condim=6；双指均接触时，任务记录的聚合切向力/法向力中位数左/右为 0.104/0.081，p90 为 0.694/0.579，低于切向系数 3 的单独切向上限。把单条解码 wrench 的两个切向力、扭转力矩和两个滚动力矩分别按五个摩擦系数归一化后求 L1 预算指标，基线活动接触中位数约 0.064、p90 约 0.362；少量近零法向力/切换样本显著超 1（最大约 14.4），因此该值只能作 wrench 级诊断，不能当作严格可行的约束占用率。condim=6 且 pyramidal cone 下，切向、扭转和滚动维度共享摩擦预算；CSV 只存 `mj_contactForce` 解码后的 6D wrench，没有保存金字塔内部的 10 维约束力坐标/`efc_force`，无法还原精确求解器空间占用率。该数据不支持把整体物理摩擦不足定为首因，但也不能排除失去一指接触时的局部容量不足。

运行模型为 MuJoCo 3.2.0，`mj_dual.xml` 设 timestep 0.002 s、implicitfast、Newton、pyramidal、50 iterations、tolerance 1e-9；未设 `impratio` 或 `noslip_iterations`，运行代码也未改这两个选项，因此实际值是默认 `impratio=1`、`noslip_iterations=0`（关闭），`noslip_tolerance=1e-6`。contact_probe 运行时把主 pad `solref` 设为 `(0.02,1)`、`solimp` 为 `(0.95,0.99,0.001,0.5,2)`；杆为 `(0.005,1)`、`(0.99,0.999,0.001,0.5,2)`。逐接触记录的实际混合值为 `solref=(0.0125,1)`、`solimp=(0.97,0.9945,0.001,0.5,2)`、margin 1 mm、接触距离约 −0.37…−0.82 mm。软接触不等于低阻抗，但无 NoSlip 后处理且摩擦维度仍有非零速度，与观察到的持续滑移相符。

据此只做了一个官方 NoSlip 后处理诊断：除 `noslip_iterations=2` 外不变，MuJoCo 3.2.0 默认容差保持 1e-6。首次启动目录 `lidio_contact_probe_noslip2_yaw10_20261017_103801` 仅因沙箱禁止 DDS socket 在任务启动前失败并保留；唯一有效物理对照为 `lidio_contact_probe_noslip2_yaw10_20261017_105300`。它仍以 `E_CONTACT_SLIP_OR_DROP` 失败；在 83 个 LIFT 帧、5.18 s 内杆 z 最大只升 0.223 mm，左/右最大滑移为 1.918/5.098 mm，右侧末端一指失接触。与基线相比没有离地或滑移改善，故 NoSlip 对照不支持“软接触正则化单独主导承载失败”；接触切换/局部卸载仍是强证据。运行目录保留，安装模型已恢复，weld 与杆固定约束关闭。按用户要求，不再安排其他接触参数或物理试验；NoSlip 结果也不证明原模型或真实硬件抓持成立。

## 2026-10-17 最小夹爪复现与一次保持驱动修复

NoSlip 对照后未再改摩擦、阻尼或求解器。逐步复现使用当前 `dual_scene.xml`、原杆质量/接触参数、单主pad运行副本和 +10° 实测夹取状态；weld/connect 对杆约束保持关闭。第一份快照误把 `p2b_path.csv` 的预抓取末点当成抓取状态，静态 FK 显示 pad 离杆约 13 cm；该无接触运行保留于 `l idio_contact_minirepro_force_limited_position_yaw10_20261017_121500`（目录名为 `lidio_contact_minirepro_force_limited_position_yaw10_20261017_121500`），不计为有效夹持工况。修正快照后，以记录的抓取 IK 为 seed、匹配 LIFT 样本双腕 link8 FK，并确认四指在初始状态均接触杆，再完成唯一有效最小物理复现。

针对旧数据中恒定腱力持续饱和、两指同相速度脉冲而同步差很小，接触时保持参考腱长和原 90% 力水平，改用原生位置伺服的固定目标并保留原执行器力限幅；普通位置命令和 ideal_weld 入口不变。夹爪包构建与 `test_gripper_shared_state` 通过。该驱动修复未通过物理复现：固定手掌 1 s 内左右双指接触占比分别约 82.8%/77.0%，每臂最大 q1-q2 差约 0.060/0.082 mm，最大单指速度约 0.122/0.128 m/s；驱动实际输出约 −18.6 至 −17.8 N，仍接近饱和。第一次接触中断约在仿真 0.012 s，手掌尚未移动，q 速度脉冲与接触切换同步出现；没有证据显示两指同步约束先失配。随后双手以 5 mm/s 同步上移 50 mm，接触占比降至约 43–45%，杆高度相对初始仅 −0.82 至 +0.36 mm，未离开支撑面；松脱后手指收至约 4.1 mm。最小复现逐步数据为该目录的 `physics_steps.csv`，输入重建依据为 `grasp_state_provenance.txt`。这是物理功能失败，未达到回到 ROS contact_probe 的前置条件；本轮未运行集成探针，也不再追加第二个驱动方案。现有证据支持共同模态指位振荡/接触切换先发生，反对两指明显失同步为首因；接触求解响应是否为振荡源仍不能单凭该记录区分。

## 2026-10-17 最小复现方法与夹爪模型静态核查

本轮停止物理试验。最小复现中固定阶段以每个 2 ms 步重写两臂 arm qpos/qvel（位置复位、速度清零），抬升阶段再逐步覆盖 qpos 并由有限差分指定 qvel；这不是模型中的物理固定约束或闭环机械臂运动，可能通过每步状态投影改变接触求解。约 12 ms 接触切换和指速脉冲是该隔离诊断中的观测，不能外推成真实控制链下的根因。记录中 q1−q2 最大差仅约 0.06/0.082 mm，且两个指速脉冲近同相；当前数据不支持 ROS 运动链或明显指间失同步为首因。该最小复现没有通过离地承载验证。

模型基准采用 Google DeepMind MuJoCo Menagerie 的 `franka_emika_panda/panda.xml`（该模型说明其由 Franka 公开 URDF 派生；不是用仓库内另一份同源 `panda.xml` 自证）。指体质量/惯量（每指 0.015 kg、对角惯量约 2.375e-6/2.375e-6/7.5e-7 kg·m²）、继承关节 armature=0.1、damping=1、行程 0–0.04 m、0.5/0.5 split tendon、指间 joint equality，以及五个 pad box 的位置/尺寸与参考相同。这些项不是相对参考的差异；参考包含相同做法也不证明它适合本项目。

重要差异与编译检查：

- 每个本项目手指同时保留 `finger_0` 碰撞 mesh 和五个 pad box。编译后的 mesh 接触参数为 MuJoCo 默认 friction `(1,0.005,0.0001)`、condim=3、solref `(0.02,1)`、solimp `(0.9,0.95,0.001,0.5,2)`、margin 0；pad 明确设置 friction `(3,0.5,0.005)`、condim=6、solref `(0.01,1)`、solimp `(0.95,0.99,0.001,0.5,2)`、margin 1 mm。杆显式设置相同摩擦、condim=6、solref `(0.005,1)`、solimp `(0.99,0.999,0.001,0.5,2)`、margin 1 mm。`CONTACT_PROBE_SINGLE_PAD` 只匹配 BOX：关掉四个小 pad、把主 pad 放大到 `(0.010,0.006,0.010)` 并将其 solref 改为 `(0.02,1)`；原始 mesh collision 未禁用，故运行配置仍是 mesh+主 pad 两种接触表面/参数。两者可能发生接触切换，但最小复现 CSV 未记录具体 geom 身份，不能把它写成已证实根因。
- MuJoCo 3.2.0 静态编译 `dual_scene.xml` 得到 finger joint `(armature=0.1,damping=1,frictionloss=0,range=0…0.04)`；gripper actuator 为 `(dyntype=none,gaintype=fixed,biastype=affine,gain=1200,bias=(0,-1200,0),gear=1,ctrlrange=0…0.08,forcerange=±150)`。源 XML 的 defaults 没有重复声明或额外覆盖；运行时 contact profile 会继续改 pad collision flags/size/solref。MuJoCo 默认 solver 值为 `impratio=1,noslip_iterations=0`。夹持保持中叠加的软件指速阻尼以及新固定位置目标都是仅显式 probe 分支的实验改动；其中固定位置目标已失败，硬件 CSV/log 模式和辅助函数注释现明确包含 `EXPERIMENTAL`，不提升为正式基线。普通位置命令及 ideal_weld 语义不变。
- Menagerie 参考使用不同的 `general` tendon actuator：ctrl 0–255 映射 0–0.04 m、gain 0.015686、bias `0 -100 -10`、forcerange ±100；本项目是 kp=1200 N/m 的 position actuator。这里是驱动和标定差异，不意味着参考 drive 可直接替换。

收敛方案：不再叠加保持机制。下一次代码修复先让 probe 的“单主 pad”语义准确：只对 finger-mesh ↔ `obj_rod_01` 这一对接触做局部过滤，同时保留 mesh 对地面、手掌、跨臂及其他环境的碰撞；不关闭整指/整手碰撞、不改杆质量、摩擦或固定约束。为相关接触 trace 保存 geom 身份后，才判断此前接触切换是否来自 mesh 与 pad 交替。固定位置保持实验继续留档但不得作为默认/正式基线。若之后需要替换子模型，以 Menagerie 的手指惯性、关节和 actuator 作为对照起点，只改夹爪子树及直接依赖的接触几何，机器人、真实杆质量、真实接触和无手—杆固定约束保持不变。

## Contact-probe 单主 pad 过滤修复与一次启动结果（2026-10-17）

在 `CONTACT_PROBE_SINGLE_PAD=true` 时，硬件插件现对当前 `mjModel` 的 `obj_rod_01` geom 与四个 finger collision mesh geom 安装窄接触过滤。实现包装 MuJoCo 全局 `mjcb_contactfilter`：先调用既有 callback 并保留它的排除结果，再过滤指定 mesh—杆对；其他模型和几何对维持原过滤行为。mesh 的 `contype/conaffinity` 未改，因此 mesh—地面、手掌、跨臂及其他环境碰撞仍按原规则处理；主 pad—杆不命中过滤规则。非 contact_probe 不安装该规则；硬件 deactivate 成功后停用本规则。初始化日志输出目标/mesh geom ID 与角色，逐接触 CSV 增加 `finger_collision_mesh` / `finger_pad_box` 等角色字段，deactivate 日志记录过滤候选数量。

定向单测覆盖 geom 顺序对称性、mesh—杆被过滤、pad—杆和 mesh—环境保留、其他模型不受影响，以及既有 callback 的排除决定不被覆盖。`franka_hardware` 构建通过，定向 `test_gripper_shared_state` 1/1 通过。没有改保持策略、接触窗口、摩擦、质量、阻尼、求解器或运动轨迹。

唯一集成运行 `lidio_contact_probe_padfilter_yaw10_20261017_01` 在 runner 启动阶段记录 `E_SIM_COMPONENT_DIED_AT_STARTUP`，发生于 truth/任务动作前；未生成 `prepare_status.json` 或接触 CSV，也没有过滤器初始化日志和物理试验数据。首批异常为 Fast-DDS UDP socket `Operation not permitted`，RViz Qt platform 初始化失败，随后 mujoco_node 和 move_group 崩溃。故该次运行不能确认过滤器已在插件中启用，也没有接触、抬升、保持、释放或承载结论。本轮按“一次运行”约束未重试，所有启动文件保留。生产过滤谓词由定向测试验证，实际安装插件启动和真实接触仍待可用环境运行确认。未提交或推送。

## 2026-10-17 接触过滤器掩码语义修复（本轮）

静态与数据核对发现：`CONTACT_PROBE_SINGLE_PAD` 下安装的 mesh-rod 接触过滤器会绕过
MuJoCo 内置的 contype/conaffinity 过滤（3.2.0 `engine_collision_driver.c` 中为
`if (mjcb_contactfilter) ... else if (filterBitmask(...))`）。因此 `initSim` 把辅助
pad 的 contype/conaffinity 置 0 后，它们在 `lidio_contact_probe_yaw10_20261017_113058`
中仍参与碰撞：逐接触 CSV 显示杆接触全部来自辅助 pad（32 点/夹爪），主 pad
69/77/150/158 从未接触；而此前所有运行只有 4 个主 pad 接触。该运行不能作为单主 pad
profile 的结论数据。

修复：回调内复现内置位掩码规则（掩码不交叉即丢弃），mesh-rod 规则保留；新增单测覆盖
"掩码置 0 的 pad 必须被丢弃"。物理线程在首次手指-杆接触时打印 geom id/role，出现非
`finger_pad_box` 时打印 ERROR。50 N/指请求撤回为 10 N：固定位置保持在 ref≈0.021 m 时，
请求超过约 14 N 会使目标越出 ctrlrange 并以 actuator fault 中止，ctrl=0 时每指上限约
12.5 N，50 N 不可达。未改摩擦、质量、阻尼、接触几何尺寸、求解器、控制器、轨迹或验收阈值。

## 2026-10-17 Phase D：掩码修复后的一次 contact_probe（run `lidio_contact_probe_yaw10_20261017_130603`）

- 接触 geom 集合：杆（163）接触只来自 `69/77/150/158`，`geom1_role` 全部为 `finger_pad_box`；无 `disabled_aux_pad_box`、无 mesh 行。7919 行采样中四 pad 各约 1916–1960 行。
- `pad_contact_count`：每夹爪最小 0 / 中位 8 / 最大 8（4 个主 pad × 2 个接触流形点），不是此前辅助 pad 的 32。
- 日志：`single-primary-pad profile: disabled 16 auxiliary pad geoms, tuned 4 primary pads`；`[contact_probe_filter] active rod_geom=163 finger_mesh_geom_ids=[68,76,149,157] previous_callback=none`；`first finger-rod contact arm=mj_left geom=77 role=finger_pad_box`、`arm=mj_right geom=150 role=finger_pad_box`（INFO，无 INVALID）；deactivate 时 filtered mesh-rod candidate pairs=27740。
- 任务结果：`[CONTACT_PROBE_RESULT] state=FAILED detail=E_CONTACT_SLIP_OR_DROP`；LIFT 131 行内杆 z 仅 0.020643→0.020838 m（最大抬升 0.194 mm），左/右滑移最大 5.061/4.863 mm，左指超 5 mm 判失败。`weld_left/weld_right/rod_constraint` 全程 0。
- 判定：预测成立——掩码修复后杆–指接触回到 4 个主 pad；承载失败（滑移/未离地）属正常物理结果。未改任何物理参数，未重跑，未提交。runner 因任务 ERROR 以退出码 2 结束，但探测数据完整（prepare 模式仍执行了完整 CONTACT_PROBE 流程）。

## 2026-10-17 P1-a：impratio=10 单因素试验（run `lidio_contact_probe_yaw10_20261017_131332`）

- 唯一改动：contact_probe 运行时 `opt.impratio` 1.0→10.0；日志 `[contact_probe] friction impedance ratio impratio=10.000 (default 1.000 ...)`，deactivate 时 `impratio restored to 1.000`。其余参数与 130603 完全相同。
- 几何无回归：杆（163）接触仍只来自 `69/77/150/158`，role 全为 `finger_pad_box`，无 aux/mesh；`pad_contact_count` 中位 8（4 pad × 2 流形点）。
- 摩擦利用率 `|t|/(μN)`：中位 **9.26%**、p90 **51.95%**（130603 基线 1.65%/3.71%）；p99=1.00、2.29% 样本超锥、最大 77.5（基线 p99 0.23、0.09%、最大 1.58）。impratio 确实放大了切向约束力，但同时进入持续近锥/超锥与冲击区。
- 承载：LIFT 114 行内杆 z 0.020990→0.021053 m（最大抬升 **0.097 mm**，低于基线的 0.194 mm）；左/右最大滑移 5.008/5.066 mm，失败时右指失接触；`weld_left/weld_right/rod_constraint` 全程 0；`[CONTACT_PROBE_RESULT] state=FAILED detail=E_CONTACT_SLIP_OR_DROP`。
- 判定：**“切向约束阻抗不足”假设不成立**——利用率显著上升但抬升未改善、滑移仍撞线并掉指；且超锥/冲击比例大幅升高，触发停止条件（impratio 过高）。已按计划回滚源码中的 impratio 覆盖并重建 `franka_hardware`，安装产物恢复默认 1.0；未试第二个值，未改其他物理参数，未提交。

## 2026-10-17 P1-b：主 pad solref 0.02→0.01 单因素试验（run `lidio_contact_probe_yaw10_20261017_132414`）

- 唯一改动：contact_probe 主 pad `geom_solref[0]` 0.02→0.01；逐接触 CSV 中 `contact_solref0` 实测 **0.0075**（130603 为 0.0125），确认混合 solref 已按预期下降。几何无回归：杆接触仍只有 `69/77/150/158`，role 全为 `finger_pad_box`。
- 失败点在抓取接触窗，未进入 LIFT：两夹爪 `[CONTACT_FORCE] failed reason=deadline`（hold 12 s 超时），窗口占用 (11,11)/(10,10)（未达 16/20），左指最终 `final_contacts=(0,0)`、`measured_pad_normal=(0.000,0.000)`，右指 12.5/11.0 N；任务 `[CONTACT_PROBE_RESULT] state=FAILED detail=E_CONTACT_GRASP_ACTION_FAILED_OR_UNKNOWN`。
- 因未进入 LIFT，`contact_probe.csv` 只有表头，dFz/dslip 与 5 mm 净力**无法测量**（按计划口径：剔除前 0.5 s 线性回归；另注：用该口径重算 130603 得 734 N/m，与任务书引用的 ~0.5 kN/m 不一致，疑似规划器口径更窄，已如实记录）。
- 判定：触发“接触失稳（闭合窗失败、单指失接触）→ 0.01 过硬”停止条件。已回滚 solref 到 0.02、重建 `franka_hardware`（install 已更新，单测 41/41 通过）；未试第二个值，未改其他物理参数，未提交。

## 2026-10-17 P1-b 重跑：主 pad solref 0.02→0.01（run `lidio_contact_probe_yaw10_20261017_133244`）

- 接触 geom：仅 `69/77/150/158`，role 全为 `finger_pad_box`；`contact_solref0=0.0075`（1288 行，基线 0.0125），改动确认生效。
- 闭合窗：左夹爪 `bilateral_force_established occupancy=(16,16) gap_spread=0.000944 m`、法向 11.06/15.62 N；右夹爪 `failed reason=deadline last_window_occupancy=(12,12)`、`final_contacts=(1,0)`、法向 18.93/0.000 N —— 单指失接触，窗口未达 16/20。
- 未进入 LIFT：`contact_probe.csv` 仅表头，`dFz/dslip`、截距与 5 mm 净力**无法测量**（130603 同口径基线 522 N/m、1.31 N、5 mm 处 3.92 N）。
- 冲击：逐接触 `wrench_n` max/p99 = 8.086/6.462 N（基线 5.76/3.82 N），高于基线但未达 50 N；`[CONTACT_PROBE_RESULT] state=FAILED detail=E_CONTACT_GRASP_ACTION_FAILED_OR_UNKNOWN`，weld/constraint 全程 0。
- 判定：触发“闭合窗失败（占用 <16/20）→ 变硬带来接触失稳”停止条件。已回滚 solref 到 0.02 并重建（install 13:34:09，单测 41/41）；未试 0.005，未改其他参数，未提交。该配置与 `..._132414` 相同，本次为按新任务书重跑，结果同样停在抓取接触窗。

## 2026-10-17 P1-c：抬升阶段分段提高主 pad 刚度（run `lidio_contact_probe_yaw10_20261017_134301`）

- 首次尝试 `..._134117` 在启动阶段失败（`/get_sim_info` 5 s 超时，mujoco_node 尚未初始化完；脚本因 manifest 已写而空转），SIGINT 后目录保留，按计划重试一次。
- 闭合窗两侧均通过：右 `occupancy=(18,18) gap_spread=0.000900 m`，左 `(16,16) 0.000991 m`，随后 `lift contact stiffness ON: primary pad solref=0.0050`；失败时 `OFF ... 0.0200`。逐接触 `contact_solref0` 分段实测 0.0125（1344 行）→ 0.005（1364 行）；杆接触仍只有 `69/77/150/158`，role 全为 `finger_pad_box`。
- 斜率 `dFz/dslip=188 N/m`、截距 `-1.041 N`、5 mm 处净力 `-0.10 N`（基线 522 N/m、1.31 N、3.92 N）；LIFT 56 行内杆 z 20.7070→20.6305 mm（最大 20.768 mm），滑移 5.011/5.036 mm。
- 稳定性：切换后 `pads` 在 0/4/8 间跳变、`rod_force_z` 在 ±12 N 与 0 间跳变；逐接触 `wrench_n` max/p99=13.66/10.53 N（基线 5.76/3.82 N），逐指法向峰值达 28.9/33.3 N；失败时左指 `pads=0 contacts=(0,0)`。`[CONTACT_PROBE_RESULT] state=FAILED detail=E_CONTACT_SLIP_OR_DROP`，weld/constraint 全程 0。
- 判定：触发“抬升阶段也吃不住 0.005（切换后接触失稳、单指失接触）”停止条件，斜率假设无法成立。已回滚整个分段刚度机制（共享状态、action server 钩子、硬件切换），主 pad solref 恢复固定 0.02；重建两包，单测 41/41；未试第二个值，未改其他参数，未提交。

## 2026-10-17 E1：cone=elliptic 单因素试验（run `lidio_contact_probe_yaw10_20261017_135303`）

- 唯一改动：`mj_dual.xml` 第 6 行 `cone="pyramidal"→"elliptic"`；安装文件已核实为 elliptic 后运行。
- 闭合窗两侧均通过：左 `(16,16) gap_spread=0.000782 m`、右 `(16,16) 0.000954 m`；杆接触仍只有 `69/77/150/158`，role 全为 `finger_pad_box`。
- 接触点数并未稳定为每臂 8：LIFT 中 `pad_contact_count` 在 0/4/8 间跳变；`wrench_n` 有 94/6748 行 >100 N、45 行 >1000 N，t=41.4 出现单点 1.65e6 N 尖峰，逐指法向峰值 157.9/210.5 N（基线约 10 N、wrench_n max 5.76 N）。
- 净向上力：起始段 L/R 之和由约 1.1 N 增至约 3.9 N，随后接触断续；失败时 `rod_force_z=(2.944,-1.195)` N。按同口径拟合 `dFz/dslip=3796 N/m`、5 mm 处 16.1 N，但该拟合被失接触瞬态污染，不能视为有效刚度。
- 滑移/漂移：LIFT 内手–杆相对 z 变化约 3.4 mm、相对 y 约 3.0/3.4 mm；左/右最大滑移 4.461/5.085 mm。杆 z 20.9765→21.0123 mm（最大 21.1078 mm，+0.131 mm），未升起。
- `[CONTACT_PROBE_RESULT] state=FAILED detail=E_CONTACT_SLIP_OR_DROP`；weld/constraint 全程 0。判定：接触明显失稳 → 触发停止条件，**不执行 E2**；已回退 `cone="pyramidal"` 并重建 franka_description（install 已核实），未改其他参数，未提交。

## 2026-10-17 D1：放宽抬升滑移中止到 15 mm 的诊断运行（run `lidio_contact_probe_yaw10_20261017_135746`）

- 唯一改动：抬升监视器 `lift_slip_tol=15 mm`（诊断），抓取/闭合/一致性检查仍为 5 mm；`[CONTACT_PROBE_CONFIG]` 如实打印 `lift_slip_limit=15mm(diagnostic)/2deg; grasp_slip_tol=5mm; acceptance unchanged`。锥模型 pyramidal、pad solref=0.02、10 N/指不变。
- 滑移最终达 15.033/15.158 mm 触发中止；杆 z 20.6505→20.8093 mm，全程最大仅 **+0.377 mm**，滑移 0.7 mm 时首次升 0.1 mm，但从未达到 0.5 mm，更未达 5 mm/45 mm。
- 净向上力 `left+right rod_force_z` 在 −11.8…+23 N 间振荡，仅 41/208 个 LIFT 样本 ≥6.28 N（瞬时），无持续承载；失败时 L/R 为 (−3.563,−6.213) N。
- 接触并非成片消失：滑移 >5 mm 的 79 个样本中 67 个（85%）两侧 pad 均 >0，但 `pad_contact_count` 在 0/3/4/5/7/8 间跳变，单指偶发失接触，法向约 7–16 N 持续存在。
- `[CONTACT_PROBE_RESULT] state=FAILED detail=E_CONTACT_SLIP_OR_DROP`；weld/constraint 全程 0。判定：滑移到 15 mm 杆仍无明显抬升（<5 mm），不是 5 mm 判据的问题，接触本身无法形成持续向上合力 → 停止，不再放宽阈值。监视器已回退到 5 mm 并重建（install 13:59:22），未改其他参数，未提交。

## 2026-10-17 E3：接触 condim 6→3 单因素试验（run `lidio_contact_probe_yaw10_20261017_140247`）

- 唯一改动：`obj_rod_01` 与主 pad `fingertip_pad_collision_1` 的 `condim=6→3`；安装文件已核实，逐接触 `contact_dim` 全部为 3。锥模型 pyramidal、摩擦、质量、solref/solimp、力、轨迹、判据不变。
- 几何与闭合窗：杆接触仍只有 `69/77/150/158`，role 全为 `finger_pad_box`；左 `(16,16) gap_spread=0.000870`、右 `(17,17) 0.000876`，两侧通过。
- 净竖直力：均值 **0.60 N**、中位 0.54、范围 **−5.88…+11.52 N**、≥6.28 N 仅 **7/122** 样本（D1 基线：均值 2.69、范围 −11.8…23、41/209）。振荡幅度减小，但均值与承载能力没有上升。
- 杆 z 20.2876→20.5278 mm，全程最大 **+0.305 mm**（基线 0.377 mm）；左/右最大滑移 4.969/5.026 mm，5 mm 判据中止。
- 姿态：左/右抓取参考旋转误差最大 **0.410°/0.374°**，远低于 2°，未出现滚转失稳；`pad_contact_count` 仍在 0/4/8 间跳变，法向峰值 ≤16.5 N、切向 ≤10.6 N。
- `[CONTACT_PROBE_RESULT] state=FAILED detail=E_CONTACT_SLIP_OR_DROP`；weld/constraint 全程 0。判定：振荡略减但杆升 <1 mm、均值更低 → condim 不是承载失败主因；已回滚两个文件到 `condim=6` 并重建核实（install 14:04），未改其他参数，未提交。

## 2026-10-17 E4：抬升速度/加速度缩放 ×4（run `lidio_contact_probe_yaw10_20261017_140713`）

- 唯一改动：抬升缩放 0.05/0.03→**0.20/0.12**（具名常量，IPTP、scaleTimedTrajectory、五阶后验限幅三处一致）；`[CONTACT_PROBE_CONFIG]/[CONTACT_PROBE_PLAN]` 已打印实际值。
- **提速未生效**：`[CONTACT_PROBE_PLAN] duration=37.905 s`（基线 31.916 s，反而更长）；手 z 速度中位约 0.51/0.46 mm/s，手–杆相对滑移速度中位仅 **0.49/0.45 mm/s**（任务预期 2–3 mm/s）。原因是既有的“每段 ≥1 s（垂直步 ≤2.5 mm/s）”拉伸逻辑压过了速度缩放；按任务书未改该约束。
- 同滑移净力（首个到达该滑移的样本）：1 mm 2.36 N、2 mm 4.89 N、3 mm 4.65 N、4 mm 5.41 N、5 mm **6.97 N**（基线约 1.8/2.9/3.5/3.9 N）。
- 杆 z 20.6420→20.9490 mm，全程最大 **+0.307 mm**，仍未升起；左/右最大滑移 5.060/5.075 mm，5 mm 中止；姿态误差最大 0.131°/0.074°。
- 稳定性好于基线：`pad_contact_count` 仅 0/8 跳变，法向峰值 ≤10.2 N，无冲击尖峰；`[CONTACT_PROBE_RESULT] state=FAILED detail=E_CONTACT_SLIP_OR_DROP`，weld/constraint 全程 0。
- 判定：操作变量未被实际施加（实际抬升更慢），**不能据此判定“速度不是主因”**；杆仍未升起、滑移仍撞 5 mm。已回滚缩放常量到 0.05/0.03 并重建（install 14:08:53），未改每段最小时间或其它参数，未提交。

## 2026-10-17 E5：真正施加 10× 抬升速度（run `lidio_contact_probe_yaw10_20261017_141604`）

- 唯一改动：抬升缩放 0.05/0.03→**0.50/0.30**，并把 stretch 下限从“总 5 s / 每段 1 s”放宽为“总 1 s / 每段 0.1 s”；`[CONTACT_PROBE_PLAN] duration=3.860 s`（基线 31.9 s、E4 37.9 s），10× 缩放首次真正生效。
- 实际速度：手 z 速度中位 **16.13/16.15 mm/s**（约基线 10×）；手–杆相对滑移速度中位 1.27/1.08 mm/s、p90 3.07/3.13 mm/s。
- 同滑移净力：1 mm −2.62 N、2 mm 6.29 N、3 mm −0.43 N、4 mm 2.68 N、5 mm 5.41 N（振荡，未呈单调上升）。
- **杆首次被实际抬起**：rod z 20.6745→52.0747 mm，最大上升 **+31.40 mm**；同期左/右滑移仅 4.661/5.140 mm，右指在 5 mm 判据处中止。手–杆相对 z 在抬升中保持同步（rod 跟随 hand 上升）。
- 稳定性：`pad_contact_count` 在 0/4/8 间跳变，法向 ≤16.6 N、切向 ≤18.8 N；抬升起始处出现瞬态接触力尖峰 19897.6/13266.7 N（左，同一时刻）与 132.6 N（右），全 CSV 仅 3 行 >50 N；姿态误差最大 0.768°/0.312°（≤2°）。
- `[CONTACT_PROBE_RESULT] state=FAILED detail=E_CONTACT_SLIP_OR_DROP`；weld/constraint 全程 0。判定：**速度因素成立**（杆在 5 mm 滑移内被抬起 31.4 mm，远超此前 <1 mm），但仍受 5 mm 滑移判据中止，且起始处有瞬态冲击；按矩阵第一行保留 0.50/0.30 + 0.1 s 配置，交规划器决定是否放宽滑移或加大抬升；未提交。

## 2026-10-17 E6：10× 抬升 + 10 mm 抬升滑移上限，跑完整 probe（run `lidio_contact_probe_yaw10_20261017_142616`）

- 配置：保留 E5 的 `0.50/0.30` 速度缩放与 stretch“总 1 s / 每段 0.1 s”；仅抬升/保持/下落监视器滑移上限改为 **10 mm（用户授权）**，抓取类检查仍 5 mm，45 mm 高度与 2 s 保持判据未放宽。`[CONTACT_PROBE_PLAN] duration=3.860 s`。
- 抬升结束：rod z 20.634→60.299 mm，全程最大 **60.510 mm（上升 39.88 mm）**，未达到 45 mm；左/右滑移 6.78/8.08 mm（未触及 10 mm）。
- 保持阶段未真正实施：高度检查在抬升结束时返回 `E_CONTACT_ROD_NOT_LIFTED`，`contact_probe.csv` 仅 1 行 HOLD（rod_z 60.285 mm），无 2 s 保持、无落回/释放。
- 接触稳定性好于 E5：`wrench_n` max/p99 = 6.2/5.05 N，**无尖峰**（E5 为 19.9 kN）；法向 ≤12.4 N、切向 ≤18.5 N，`pad_contact_count` 仍在 0/4/8 间跳变，姿态误差最大 0.913°/0.430°。
- `[CONTACT_PROBE_RESULT] state=FAILED detail=E_CONTACT_ROD_NOT_LIFTED`；weld/constraint 全程 0。判定：**差一点**（39.9 mm vs 45 mm，滑移 6.8/8.1 mm），保持/释放未被检验；按矩阵第二行记录，下一轮由规划器决定加大抬升到 5.5–6 cm 并处理 45 mm 判据。未改其它参数，未提交。

## 2026-10-17 E7：6 cm 抬升 + 固定 45 mm 判据，跑完整 probe（run `lidio_contact_probe_yaw10_20261017_143348`）

- 首次尝试 `..._143218` 在启动阶段 `/get_sim_info` 超时（同前，runner 退出后脚本空转），SIGINT 保留目录后重试一次；本记录为有效重试。
- 配置：`height=0.06`、`rod_rise_target=0.045`（与行程解耦）、`lift_slip_tol=0.012`；保留 10× 速度（0.50/0.30）与 stretch 0.1 s。`[CONTACT_PROBE_PLAN] delta=0.060m duration=3.813s contact_step=3.0mm`。
- **抬升通过**：rod z 20.647→69.681 mm（最大 69.780），上升 **49.13 mm ≥45 mm**；抬升末段滑移 7.449/8.621 mm（≤12 mm）。
- **保持失败**：HOLD 仅 1.42 s（25 行）后右指滑移达 **12.003 mm** 触发 `E_CONTACT_HOLD: E_CONTACT_SLIP_OR_DROP`；保持期间杆从 69.62 下滑到 66.17 mm（−3.45 mm），净向上力均值 4.64 N（15/25 样本 ≥6.28 N），接触在 8/8、8/0、0/8 间切换。未进入落回与释放。
- 稳定性：`wrench_n` max 8244.8 N（瞬态尖峰）、p99 5.58 N、18 行 >50 N；法向存在时约 10 N；姿态误差最大 1.049°/0.724°（≤2°）；weld/constraint 全程 0。
- `[CONTACT_PROBE_RESULT] state=FAILED detail=E_CONTACT_HOLD: E_CONTACT_SLIP_OR_DROP`。判定：**抬升达标、保持阶段失败**（矩阵第二行）；不调参，交规划器。实际滑移 7.45/8.62 mm（抬升）→ 12.00 mm（保持中止），非零滑移。未提交。

## 2026-10-17 E8：接触点滑移预算修复，首次完成抬升—保持—落回—释放（run `lidio_contact_probe_yaw10_20261017_150226`）

**结果：`[CONTACT_PROBE_RESULT] state=DONE`，无 weld 真实接触抓取在本固定场景（0.50, 0, +10°）首次完成抬升、2 s 保持、落回与释放。**

- `[CONTACT_LOAD_VALIDATED] samples=33 sim_hold=2.002s lift=0.047m max_slip=11.037mm max_angle=0.882deg weld=false rod_constraint=false`。
- 杆 z：20.65 → 最高 71.43 → 松爪后 20.45 mm（回到支撑面）；x/y 释放后 `(0.498733, 0.001453, 0.020654)`。
- 保持受力：左指 31/34、30/34，右指 32/34、32/34（均 ≥80%），法向约 10 N；净向上力均值 5.70 N。验收日志右夹爪 `(0.000,0.000)` 为单采样瞬态。
- 全程 weld_left/weld_right/rod_constraint = 0。

本轮为让抓持成立所做的三项改动（均在未提交工作树中）：

1. **抓取高度下移**：`contact_probe_grasp_z_offset` 0.0073 → **0.0123 m**。接触点在杆侧面的初始 z 从约 −10.8…+9.2 mm 移到更低位置，抬升+保持的约 11 mm 滑移不再把接触点推到杆顶边（此前 E7 失败时 rod-local z 已到 +20.0 mm）。
2. **落回前重建立抓持参考**：保持验证完成后，用当前实测杆位姿与双手 FK 重算 `object→hand`，避免双手反推杆位姿差 >5 mm 触发 `E_CONTACT_DUAL_GRASP_RESIDUAL`。抬升/保持的滑移已在此之前记录（11.037 mm），重建立不掩盖它。
3. **按实际升高落回**：落回量改为 `当前杆 z − 初始杆 z`（0.047 m），而不是指令行程 0.06 m；否则杆会被压低 10 mm 触发 `E_CONTACT_ROD_SUPPORT_PENETRATION`。

**必须如实记录的代价**：竖直滑移 **11.04 mm**，超过原验收的 5 mm；本轮使用的是用户授权的 12 mm 抬升滑移上限。这是“有滑移的接触抓取”，不是零滑移抓取。另：`contact_probe` 只做原地抬升/落回/释放，未做横向运输；全成功率与泛化未验证。

当前工作树配置（未提交）：pad z 0.010（0.018 试验因闭合失稳已回退）、10× 速度 0.50/0.30、6 cm 行程、45 mm 固定高度判据、12 mm 抬升滑移上限、抓取偏移 0.0123 m、落回重建立 + 按实际升高。E5 的 10× 与 stretch 0.1 s 保留。

**复跑确认（run `lidio_contact_probe_yaw10_20261017_150853`）**：同一构建、同一配置再次 `state=DONE`。`CONTACT_LOAD_VALIDATED samples=35 sim_hold=2.062s lift=0.047m max_slip=11.390mm max_angle=0.903deg`；杆 20.69→71.15→20.40 mm；释放后 `(0.499346,0.000825,0.020654)`；每指受力占比 32/36、34/36、35/36、34/36；weld/constraint 全程 0。**当前配置 2/2 成功**；样本数仍小，不代表成功率或泛化。


## 2026-10-17 E9：最大速度 + 50 cm/3 s，完整 probe 通过

- 速度缩放改为 **1.00/0.80**（关节限值内），stretch 改为全程≥0.5 s、每段≥0.02 s；`noslip_iterations` 恢复默认 0；pad margin 恢复模型值 0.001。抓取偏移 0.0123 m、落回重建立、按实际升高落回均保留。
- 6 cm / 2 s 最大速度：`[CONTACT_PROBE_PLAN] IPTP duration=0.803s`；`[CONTACT_LOAD_VALIDATED] samples=33 sim_hold=2.002s lift=0.051m max_slip=5.945mm max_angle=0.676deg`；`state=DONE`。整段仿真 4.05 s（抬升 0.94 + 保持 2.00 + 落回 0.98）。
- 50 cm / 3 s 最大速度（run `lidio_contact_probe_yaw10_20261017_153128`）：`IPTP duration=3.967s`；`samples=49 sim_hold=3.012s lift=0.483m max_slip=12.856mm max_angle=0.970deg`；杆 20.64→509.36→20.15 mm；释放后 `(0.501422,0.002013,0.020654)`；`state=DONE`。整段仿真 10.99 s（抬升 4.16 + 保持 3.00 + 落回 3.56）。
- 结论：**提高速度会提高切向摩擦并降低滑移**。同样 50 cm，10× 在约 34 cm 处滑到 20 mm 脱落；最大速度下全程约 12.9 mm 滑移（落回段最大 15.4 mm），完整抬到 48.3 cm、保持 3 s、落回并释放。滑移仍非零，属用户接受的代价。
- 顺手修正成功信息中硬编码的 “5cm/2s” 为实际 height/hold。未提交。


## 2026-10-17 E10：完整流程（抬升 50 cm→保持 3 s→前进 10 cm→落回→释放）首次通过

- 配置：抓取偏移 0.0123 m；瞄准/下降 0.5/0.5；夹爪闭合 0.015 m/s（3×）；抬升 1.0/0.6；运输 1.0/0.3、最短 2 s；落回 1.0/0.8；`noslip_iterations=20`；10 N/指；滑移窗口 20 mm。
- 无窗口 run `..._155319`：`CONTACT_LOAD_VALIDATED samples=50 sim_hold=3.012s lift=0.491m max_slip=11.362mm`；运输 x 0.500→0.400；落回滑移 7.88 mm；释放 `(0.396408,0.001894,0.020655)`；`state=DONE`。
- **可视化 run `..._155952`（本次）**：MuJoCo viewer 连接（非 headless）、RViz 启动；`CONTACT_LOAD_VALIDATED samples=50 sim_hold=3.060s lift=0.491m max_slip=10.822mm max_angle=1.320deg`；`CONTACT_PROBE_TRANSPORT` 前进 0.10 m；`CONTACT_PROBE_LOWER` 按倾斜感知高度落回 0.486 m；释放 `(0.396215,0.001562,0.020655)`；`CONTACT_PROBE_RESULT state=DONE`。放置误差约 x 3.8 mm / y 1.6 mm。
- 阶段滑移（155952/155319 量级）：抬升约 6–8 mm；保持约 3–4 mm（3 s）；运输约 4 mm；落回约 4–8 mm。滑移仍非零；只前进 10 cm，未做旋转/返回。
- 期间修正：释放位姿检查由“与初始抓取位姿比较”改为“与松爪前实测位姿比较”（前进后初始位姿已不适用）；配置日志改为打印实际 height/hold/slip。

完整逐次运行索引（今天的全部 60 次 contact_probe 运行，含配置与实际结果）见 [CONTACT_RUN_INDEX_20261017.md](CONTACT_RUN_INDEX_20261017.md)。

**无 weld 核查（155952）**：任务 CSV 238 个采样的 `weld_left/weld_right/rod_constraint` 全为 0；逐接触 CSV 12,252 个采样同样全为 0；日志无 weld 激活记录，`CONTACT_PROBE_RELEASED` 为 `weld=OFF`。物理上杆相对手滑移 10.82 mm，若启用了手–杆 weld 不可能出现该相对运动。手指间同步 joint equality 按模型保留，不属于杆约束。


## 2026-10-17 E11：完整搬运 + 旋转（前进 10 cm、偏航 +10°）

- 目标 `(0.40, 0.00, +20°)`；`makePlan` 扩展为物体空间平移 + 偏航插值，运输段 1.0/0.3、最短 2 s。
- run `..._161453`：`CONTACT_LOAD_VALIDATED samples=49 sim_hold=3.040s lift=0.491m max_slip=10.889mm max_angle=1.303deg`；`CONTACT_PROBE_TRANSPORT dx=-0.100 dy=0.000 dyaw=10.5 deg`；`CONTACT_PROBE_RELEASED xyz=(0.3983,-0.0044,0.0205)`；`state=DONE`。
- 最终杆 yaw 18.71°（目标 20°）；放置误差 x 1.7 mm / y 4.4 mm / yaw 1.29°。阶段滑移：抬升 9.45、保持 9.85、运输 5.50、落回 8.31 mm。
- `noslip_iterations=30` 对照 run `..._161330`：保持期杆下滑降到约 1.8 mm，但右指受力占比不过 80%（`E_CONTACT_HOLD_PAD_FORCE_NOT_SUSTAINED`），最大滑移未明显下降；保留 noslip=20。
- 未实现松爪后的手臂撤退/返回；任务在释放确认处结束。


## 2026-10-17 E12：40 cm / 2 s 完整流程与五阶检查修正

- 按用户要求改为抬升 40 cm、保持 2 s（50 cm/3 s 保留为上一版）；其余（前进 10 cm + 旋转、落回、释放）不变。
- 抬升段 1.0/0.4、最短 1 s；40 cm 路径在 0.6/0.4 加速度下曾被五阶插值检查拒绝（`mj_left_joint2` acc −2.82 超过 0.4×5=2.0）；改为按关节硬限值 ±5 校验后通过，缩放因子继续只用于规划速度偏好。
- run `..._164743`：`CONTACT_LOAD_VALIDATED samples=35 sim_hold=2.060s lift=0.392m max_slip=9.512mm max_angle=1.414deg`；运输 `dx=-0.098 dy=-0.001 dyaw=10.3 deg`；落回 0.386 m；释放 `(0.3963,-0.0027,0.0205)`、yaw 18.65°；`state=DONE`。
- 放置误差 x 3.7 mm / y 2.7 mm / yaw 1.35°；阶段滑移：抬升 5.70、保持 7.32、运输 4.21、落回 7.51 mm。
- 启动失败 run `..._164125`（`E_SIM_INFO_UNAVAILABLE`）不计物理结果。
