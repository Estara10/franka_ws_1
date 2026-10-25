# 相同运输前保持条件下的补充对照

本批独立于80次正式主实验。四条件各3次；所有条件均计算候选，P/I0/I1仍选基础参考。运输前以抬升完成为起点设置8 s仿真窗口。物理参数、控制增益和既有保护不变。

P=基础＋JTC；T=时间优化＋JTC；I0=基础＋阻抗、物体前馈关闭；I1=基础＋同阻抗、前馈开启。

| 条件 | 完成/尝试 | 运输高度净变化中位数/mm | 高度范围中位数/mm | 释放位置误差均值/mm | 偏航误差均值/° |
|---|---|---|---|---|---|
| P | 3/3 | -6.387 | 6.928 | 6.814 | 1.213 |
| T | 3/3 | -6.650 | 7.240 | 6.918 | 1.219 |
| I0 | 0/3 | 未测 | 未测 | 未测 | 未测 |
| I1 | 0/3 | 未测 | 未测 | 未测 | 未测 |

## 结果解释

P、T各完成3/3。T选用候选的连续参考jerk RMS相对同次基础参考平均下降5.64%；运输高度净变化、变化范围及释放误差未显示一致优势。跨次规划由各自实测起点生成，不能声称P/T跨运行的关节路径完全相同。
I0三次在运输派发起点偏移检查中停止；I1三次在起点连接的抓持参考检查中停止。两条件均已抬升并经过8秒窗口，但没有有效运输窗口，因此没有完整运输的前馈因果比较。停止不等于已证明物体掉落；也不能把该长保持工况与原80次通常流程的完成率混合。
I0实际发布的每臂物体前馈最大值为0 N，I1为3.1392 N；机器人本体补偿、阻抗增益和夹持参数未改变。

## 时间与采样

GetBodyState响应没有物理采样时间戳，记录了请求发送与响应接收的仿真时间。本分析以二者中点估计采样时刻，并报告该区间宽度；未将其称为精确物理步。仅对实际运输阶段的不同真值样本，在覆盖范围内按50 ms插值，不外推。高度范围不是目标误差、振荡幅值或实测jerk。

本轮是探索性小样本对照，不能据此估计总体可靠率或声称统计显著。正常部署中候选计算仍是算法开销，匹配保持不消除这一成本。P/I0/I1计算的候选未选用，其候选jerk变化只能描述计算结果，不能写成执行优化收益。

## 逐次状态

- P1: DONE；run=explanatory_20261025_P1_attempt4；
- T1: DONE；run=explanatory_20261025_T1_attempt1；
- I01: FAILED；run=explanatory_20261025_I01_attempt1；E_ABCD_MMC_DISPATCH_START_MOVED
- I11: FAILED；run=explanatory_20261025_I11_attempt2；E_CONTACT_DUAL_GRASP_RESIDUAL
- I12: FAILED；run=explanatory_20261025_I12_attempt1；E_CONTACT_DUAL_GRASP_RESIDUAL
- I02: FAILED；run=explanatory_20261025_I02_attempt1；E_ABCD_MMC_DISPATCH_START_MOVED
- T2: DONE；run=explanatory_20261025_T2_attempt1；
- P2: DONE；run=explanatory_20261025_P2_attempt1；
- T3: DONE；run=explanatory_20261025_T3_attempt1；
- P3: DONE；run=explanatory_20261025_P3_attempt1；
- I13: FAILED；run=explanatory_20261025_I13_attempt1；E_CONTACT_DUAL_GRASP_RESIDUAL
- I03: FAILED；run=explanatory_20261025_I03_attempt1；E_ABCD_MMC_DISPATCH_START_MOVED
