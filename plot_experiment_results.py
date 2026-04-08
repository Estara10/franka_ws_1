#!/usr/bin/env python3
import argparse
import csv
import sys
import glob
import os
import matplotlib
# Use a non-interactive backend by default to avoid issues when ending the task,
# can also be changed to 'TkAgg' if we want it to block and show. 
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def has_data_rows(csv_path: str) -> bool:
    try:
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            next(reader, None)
            for row in reader:
                if row and any(cell.strip() for cell in row):
                    return True
        return False
    except Exception:
        return False


def classify_csv_header(header):
    normalized = [h.strip() for h in header]
    header_set = set(normalized)

    # Full experiment CSV exported by sensor_dashboard / recorder
    if {
        "Time",
        "Left_Y_Force",
        "Right_Y_Force",
        "Internal_Stress",
        "Joint_Accel_Norm",
    }.issubset(header_set):
        return "full"

    # Task metrics sync samples exported by task_metrics monitor
    if {"time_sec", "left_step_mm", "right_step_mm", "sync_error_mm"}.issubset(header_set):
        return "sync"

    return "unknown"

def parse_args():
    parser = argparse.ArgumentParser(
        description="绘制实验CSV的离线分析图（受力/内应力/平滑度/阻抗缩放）"
    )
    parser.add_argument(
        "--csv",
        default="",
        help="指定输入CSV文件路径；为空时自动查找最新CSV"
    )
    parser.add_argument(
        "--data-dir",
        default="",
        help="CSV搜索目录；为空时依次搜索脚本目录下 experiment_data 与当前目录 experiment_data"
    )
    parser.add_argument(
        "--output",
        default="",
        help="输出PNG路径；为空时默认与CSV同目录同名 .png"
    )
    parser.add_argument(
        "--plot-dir",
        default="",
        help="图表输出目录；当未指定 --output 时生效，默认输出到 experiment_results/plots"
    )
    return parser.parse_args()


def discover_latest_csv(explicit_data_dir: str) -> str:
    if explicit_data_dir:
        search_dirs = [explicit_data_dir]
        candidates = []
        for directory in search_dirs:
            if not os.path.isdir(directory):
                continue
            candidates.extend(glob.glob(os.path.join(directory, "*.csv")))
        non_empty = [c for c in candidates if has_data_rows(c)]
        if non_empty:
            return max(non_empty, key=os.path.getctime)
        if candidates:
            print("⚠ 找到CSV文件，但都为空数据（仅表头/无数据行），跳过图表绘制。")
            print(f"  目录: {explicit_data_dir}")
            return ""
        if not candidates:
            print("⚠ 未找到实验数据CSV文件，跳过图表绘制。")
            print("  可选：传入 --csv 指定文件，或传入 --data-dir 指定目录。")
            return ""
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        full_data_dirs = [
            os.path.join(script_dir, "experiment_results", "data"),
            os.path.join(os.getcwd(), "experiment_results", "data"),
            os.path.join(script_dir, "experiment_data"),
            os.path.join(os.getcwd(), "experiment_data"),
        ]
        report_dirs = [
            os.path.join(script_dir, "experiment_results", "reports"),
            os.path.join(os.getcwd(), "experiment_results", "reports"),
        ]

    candidates = []
    for directory in full_data_dirs:
        if not os.path.isdir(directory):
            continue
        candidates.extend(glob.glob(os.path.join(directory, "*.csv")))

    # Priority 1: standard full experiment CSV
    non_empty_full = [c for c in candidates if has_data_rows(c)]
    if non_empty_full:
        return max(non_empty_full, key=os.path.getctime)

    # Priority 2: task_metrics sync samples CSV
    sync_candidates = []
    for directory in report_dirs:
        if not os.path.isdir(directory):
            continue
        sync_candidates.extend(glob.glob(os.path.join(directory, "task_metrics_*_sync_samples.csv")))

    non_empty_sync = [c for c in sync_candidates if has_data_rows(c)]
    if non_empty_sync:
        return max(non_empty_sync, key=os.path.getctime)

    if candidates or sync_candidates:
        print("⚠ 找到CSV文件，但都为空数据（仅表头/无数据行），跳过图表绘制。")
        print("  可能原因：任务监控未采到样本或任务提前结束。")
        print("  建议：完整跑完任务后重试，或开启 ENABLE_SENSOR_GUI=true / ENABLE_EXPERIMENT_RECORDER=true 重新采集。")
        return ""

    print("⚠ 未找到实验数据CSV文件，跳过图表绘制。")
    print("  已搜索: experiment_results/data, experiment_data, experiment_results/reports(task_metrics_*_sync_samples.csv)")
    print("  可选：传入 --csv 指定文件，或传入 --data-dir 指定目录。")
    return ""


def plot_full_experiment(times, left_y, right_y, stress, accel, k_scale, d_scale):
    # =============== 顶部子图：双臂Y轴接触力 ===============
    plt.subplot(4, 1, 1)
    plt.plot(times, left_y, label='左臂 Y向受力', color='g')
    plt.plot(times, right_y, label='右臂 Y向受力', color='b')
    plt.title("全局双臂Y轴接触力 (N)")
    plt.xlabel("任务完整时间流 (s)")
    plt.ylabel("受力值 (N)")
    plt.grid(True)
    plt.legend(loc='upper right')

    # =============== 子图2：内应力 ===============
    plt.subplot(4, 1, 2)
    plt.plot(times, stress, label='夹击内应力', color='r', linewidth=2)
    plt.axhline(y=5.0, color='grey', linestyle='--', label='应力安全阈值参考')
    plt.title("全局内应力评估 [柔顺性核心指标 - 值越低、突变越少越好]")
    plt.xlabel("任务完整时间流 (s)")
    plt.ylabel("内应力大小 (N)")
    plt.grid(True)
    plt.legend(loc='upper right')

    max_stress = max(stress) if stress else 0
    plt.ylim(0, max_stress + 5.0)

    # =============== 子图3：平滑度 ===============
    plt.subplot(4, 1, 3)
    plt.plot(times, accel, label='关节加速度范数', color='m', linewidth=2)
    plt.title("全阶段运动平滑度 [CHOMP规划评估指标 - 起伏越小越平顺]")
    plt.xlabel("任务完整时间流 (s)")
    plt.ylabel("加速度范数强度")
    plt.grid(True)
    plt.legend(loc='upper right')

    # =============== 底部子图：自适应阻抗缩放 ===============
    plt.subplot(4, 1, 4)
    plt.plot(times, k_scale, label='刚度 K 缩放 (%)', color='k', linewidth=2)
    plt.plot(times, d_scale, label='阻尼 D 缩放 (%)', color='orange', linewidth=2)
    plt.title("全局自适应阻抗参数变化率")
    plt.xlabel("任务完整时间流 (s)")
    plt.ylabel("参数缩放率 (%)")
    plt.grid(True)
    plt.legend(loc='upper right')


def plot_sync_metrics(times, left_step, right_step, sync_error, dt_sec, stage_names):
    # =============== 顶部子图：双臂步进位移 ===============
    plt.subplot(4, 1, 1)
    plt.plot(times, left_step, label='左臂步进位移 (mm)', color='g')
    plt.plot(times, right_step, label='右臂步进位移 (mm)', color='b')
    plt.title("双臂末端步进位移 (来自 task_metrics sync_samples)")
    plt.xlabel("任务完整时间流 (s)")
    plt.ylabel("位移 (mm)")
    plt.grid(True)
    plt.legend(loc='upper right')

    # =============== 子图2：同步误差 ===============
    plt.subplot(4, 1, 2)
    plt.plot(times, sync_error, label='双臂同步误差 (mm)', color='r', linewidth=2)
    plt.axhline(y=5.0, color='grey', linestyle='--', label='同步目标阈值 5mm')
    plt.title("双臂同步误差评估 [越低越好]")
    plt.xlabel("任务完整时间流 (s)")
    plt.ylabel("同步误差 (mm)")
    plt.grid(True)
    plt.legend(loc='upper right')

    max_sync = max(sync_error) if sync_error else 0
    plt.ylim(0, max_sync + 2.0)

    # =============== 子图3：采样间隔 ===============
    plt.subplot(4, 1, 3)
    plt.plot(times, dt_sec, label='采样间隔 dt (s)', color='m', linewidth=2)
    plt.title("任务监控采样间隔稳定性")
    plt.xlabel("任务完整时间流 (s)")
    plt.ylabel("dt (s)")
    plt.grid(True)
    plt.legend(loc='upper right')

    # =============== 底部子图：任务阶段索引 ===============
    plt.subplot(4, 1, 4)
    plt.step(times, stage_names, where='post', label='任务阶段索引', color='k', linewidth=2)
    plt.title("任务阶段演化")
    plt.xlabel("任务完整时间流 (s)")
    plt.ylabel("阶段索引")
    plt.grid(True)
    plt.legend(loc='upper right')


def main():
    args = parse_args()

    latest_csv = args.csv if args.csv else discover_latest_csv(args.data_dir)
    if not latest_csv:
        return 0

    if not os.path.isfile(latest_csv):
        print(f"⚠ 输入CSV不存在，跳过图表绘制: {latest_csv}")
        return 0

    print(f"📊 正在读取数据并绘制图表: {latest_csv}")

    times = []
    left_y = []
    right_y = []
    stress = []
    accel = []
    k_scale = []
    d_scale = []

    left_step = []
    right_step = []
    sync_error = []
    dt_sec = []
    stage_numeric = []
    stage_map = {}

    csv_kind = "unknown"

    try:
        with open(latest_csv, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            csv_kind = classify_csv_header(header)

            if csv_kind == "unknown":
                print(f"⚠ 未识别的CSV列格式: {header}")
                print("  该脚本当前支持: 标准实验CSV(Time, Left_Y_Force, ...), 或 task_metrics_*_sync_samples.csv")
                return 0

            header_index = {name.strip(): i for i, name in enumerate(header)}
            for row in reader:
                if not row:
                    continue

                if csv_kind == "full":
                    times.append(float(row[0]))
                    left_y.append(float(row[1]))
                    right_y.append(float(row[2]))
                    stress.append(float(row[3]))
                    accel.append(float(row[4]))
                    if len(row) >= 7:
                        k_scale.append(float(row[5]))
                        d_scale.append(float(row[6]))
                    else:
                        k_scale.append(100.0)
                        d_scale.append(100.0)
                else:
                    t = float(row[header_index["time_sec"]])
                    ls = float(row[header_index["left_step_mm"]])
                    rs = float(row[header_index["right_step_mm"]])
                    se = float(row[header_index["sync_error_mm"]])
                    dt = float(row[header_index["dt_sec"]]) if "dt_sec" in header_index else 0.0
                    stage_name = row[header_index["stage"]].strip() if "stage" in header_index else "unknown"

                    if stage_name not in stage_map:
                        stage_map[stage_name] = len(stage_map)

                    times.append(t)
                    left_step.append(ls)
                    right_step.append(rs)
                    sync_error.append(se)
                    dt_sec.append(dt)
                    stage_numeric.append(stage_map[stage_name])
    except Exception as e:
        print(f"读取 CSV 出错: {e}")
        sys.exit(1)

    if not times:
        print("CSV 文件内容为空数据！不生成图表。")
        sys.exit(0)

    # 开始离线高质量绘图
    plt.figure(figsize=(10, 12))
    plt.rcParams["font.sans-serif"] = ["WenQuanYi Zen Hei", "Noto Sans CJK SC", "Microsoft YaHei"]
    plt.rcParams["axes.unicode_minus"] = False

    if csv_kind == "full":
        plot_full_experiment(times, left_y, right_y, stress, accel, k_scale, d_scale)
    else:
        print("ℹ 检测到 task_metrics sync_samples CSV，绘制同步误差版图表。")
        plot_sync_metrics(times, left_step, right_step, sync_error, dt_sec, stage_numeric)

    plt.tight_layout()
    
    # 默认保存到专门图表目录；可由 --output 或 --plot-dir 覆盖
    if args.output:
        png_name = args.output
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        plot_dir = args.plot_dir if args.plot_dir else os.path.join(script_dir, "experiment_results", "plots")
        os.makedirs(plot_dir, exist_ok=True)
        csv_root, _ = os.path.splitext(latest_csv)
        png_name = os.path.join(plot_dir, os.path.basename(csv_root) + '.png')
    plt.savefig(png_name, dpi=300)
    print(f"✅ 【完美】全阶段数据高精度分析曲线已生成，并保存至: {png_name}")
    print("   你现在可以直接打开该PNG查看结果。")
    
    # 退出前释放内存
    plt.close()

if __name__ == "__main__":
    main()
