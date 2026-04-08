#!/usr/bin/env python3
import argparse
import os
import glob
import csv
import statistics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
def parse_args():
    parser = argparse.ArgumentParser(
        description="统计多次通信实验的延迟平均值（默认最近10次）并输出图表/CSV"
    )
    parser.add_argument(
        "--input-dir",
        default="experiment_results/reports",
        help="通信测试CSV目录",
    )
    parser.add_argument(
        "--output-dir",
        default="experiment_results/plots",
        help="统计图输出目录",
    )
    parser.add_argument(
        "--max-files",
        type=int,
        default=10,
        help="最多统计最近N次实验（默认: 10）",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="统计全部实验，忽略 --max-files",
    )
    parser.add_argument(
        "--pattern",
        default="comm_latency_*_samples.csv",
        help="输入文件匹配模式",
    )
    parser.add_argument(
        "--stats-csv",
        default="",
        help="统计CSV输出路径；不指定时自动命名",
    )
    parser.add_argument(
        "--chart-name",
        default="",
        help="统计图文件名；不指定时自动命名",
    )
    return parser.parse_args()


def percentile(values, p):
    if not values:
        return 0.0
    sorted_vals = sorted(values)
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    k = (len(sorted_vals) - 1) * p
    floor_idx = int(k)
    ceil_idx = min(floor_idx + 1, len(sorted_vals) - 1)
    if floor_idx == ceil_idx:
        return sorted_vals[floor_idx]
    ratio = k - floor_idx
    return sorted_vals[floor_idx] + (sorted_vals[ceil_idx] - sorted_vals[floor_idx]) * ratio


def build_label(file_path, default_label):
    filename = os.path.basename(file_path)
    parts = filename.split('_')
    if len(parts) >= 4:
        date_str = parts[2]
        time_str = parts[3]
        if len(date_str) == 8 and len(time_str) >= 4:
            return f"{date_str[-4:]}\n{time_str[:2]}:{time_str[2:4]}"
    return default_label


def read_latency_series(file_path):
    latencies = []
    with open(file_path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames or 'latency_ms' not in reader.fieldnames:
            return latencies
        for row in reader:
            raw = (row.get('latency_ms') or '').strip()
            if not raw:
                continue
            try:
                latencies.append(float(raw))
            except ValueError:
                continue
    return latencies


def main():
    args = parse_args()

    if args.max_files <= 0:
        print("--max-files 需要大于0")
        return 1

    os.makedirs(args.output_dir, exist_ok=True)

    pattern = os.path.join(args.input_dir, args.pattern)
    csv_files = glob.glob(pattern)
    csv_files.sort(key=os.path.getctime, reverse=True)

    if not csv_files:
        print(f"未找到任何通信延迟CSV: {pattern}")
        return 0

    if args.all:
        files_to_process = csv_files
    else:
        files_to_process = csv_files[:args.max_files]

    # 反转为时间从旧到新，便于横轴阅读
    files_to_process = list(reversed(files_to_process))

    scope_text = "all" if args.all else f"top{len(files_to_process)}"
    print(f"Start aggregating {len(files_to_process)} experiments ({scope_text}).")

    rows = []
    all_latencies = []

    for idx, file_path in enumerate(files_to_process):
        latencies = read_latency_series(file_path)
        if not latencies:
            print(f"Skip invalid/empty file: {os.path.basename(file_path)}")
            continue

        exp_label = build_label(file_path, f"Exp {idx + 1}")
        mean_ms = statistics.mean(latencies)
        median_ms = statistics.median(latencies)
        p95_ms = percentile(latencies, 0.95)
        min_ms = min(latencies)
        max_ms = max(latencies)
        std_ms = statistics.stdev(latencies) if len(latencies) > 1 else 0.0

        rows.append(
            {
                "label": exp_label,
                "source_file": os.path.basename(file_path),
                "samples": len(latencies),
                "mean_ms": mean_ms,
                "median_ms": median_ms,
                "p95_ms": p95_ms,
                "min_ms": min_ms,
                "max_ms": max_ms,
                "std_ms": std_ms,
            }
        )
        all_latencies.extend(latencies)
        print(
            f"{os.path.basename(file_path)} -> mean={mean_ms:.3f} ms, "
            f"p95={p95_ms:.3f} ms, samples={len(latencies)}"
        )

    if not rows:
        print("没有可用于统计的有效通信实验数据。")
        return 0

    weighted_mean = statistics.mean(all_latencies)
    weighted_median = statistics.median(all_latencies)
    weighted_p95 = percentile(all_latencies, 0.95)
    mean_of_means = statistics.mean([r["mean_ms"] for r in rows])

    labels = [r["label"] for r in rows]
    means = [r["mean_ms"] for r in rows]
    p95_vals = [r["p95_ms"] for r in rows]

    fig_width = max(12.0, len(rows) * 1.1)
    plt.figure(figsize=(fig_width, 6.5))

    bars = plt.bar(labels, means, color="#4C78A8", edgecolor="black", alpha=0.85, label="Mean")
    plt.plot(labels, p95_vals, color="#E45756", marker="o", linewidth=2.0, label="P95")
    plt.axhline(
        y=weighted_mean,
        color="#2E8B57",
        linestyle="--",
        linewidth=1.8,
        label=f"Weighted mean ({weighted_mean:.3f} ms)",
    )

    for bar in bars:
        yval = bar.get_height()
        plt.text(
            bar.get_x() + bar.get_width() / 2,
            yval + 0.02,
            f"{yval:.2f}",
            ha="center",
            va="bottom",
            fontsize=10,
            fontweight="bold",
        )

    y_max = max(max(means), max(p95_vals), weighted_mean)
    plt.ylim(0, y_max * 1.25 if y_max > 0 else 1.0)
    plt.title(f"Communication Latency Stats ({len(rows)} Experiments)")
    plt.xlabel("Experiment Date/Time")
    plt.ylabel("Latency (ms)")
    plt.grid(axis="y", linestyle="--", alpha=0.7)
    plt.legend()
    plt.tight_layout()

    if args.chart_name:
        chart_filename = args.chart_name
    else:
        chart_filename = f"comm_latency_average_stats_{scope_text}.png"
    chart_path = os.path.join(args.output_dir, chart_filename)
    plt.savefig(chart_path, dpi=130, bbox_inches="tight")
    plt.close()

    if args.stats_csv:
        stats_csv_path = args.stats_csv
    else:
        stats_csv_path = os.path.join(args.output_dir, f"comm_latency_average_stats_{scope_text}.csv")

    with open(stats_csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "label",
            "source_file",
            "samples",
            "mean_ms",
            "median_ms",
            "p95_ms",
            "min_ms",
            "max_ms",
            "std_ms",
        ])
        for r in rows:
            writer.writerow([
                r["label"],
                r["source_file"],
                r["samples"],
                f"{r['mean_ms']:.6f}",
                f"{r['median_ms']:.6f}",
                f"{r['p95_ms']:.6f}",
                f"{r['min_ms']:.6f}",
                f"{r['max_ms']:.6f}",
                f"{r['std_ms']:.6f}",
            ])
        writer.writerow([])
        writer.writerow(["summary", "value_ms"])
        writer.writerow(["weighted_mean", f"{weighted_mean:.6f}"])
        writer.writerow(["weighted_median", f"{weighted_median:.6f}"])
        writer.writerow(["weighted_p95", f"{weighted_p95:.6f}"])
        writer.writerow(["mean_of_means", f"{mean_of_means:.6f}"])

    print("\nSummary:")
    print(f"- Experiments: {len(rows)}")
    print(f"- Weighted mean latency: {weighted_mean:.3f} ms")
    print(f"- Weighted median latency: {weighted_median:.3f} ms")
    print(f"- Weighted p95 latency: {weighted_p95:.3f} ms")
    print(f"- Mean of experiment means: {mean_of_means:.3f} ms")
    print(f"- Chart saved: {chart_path}")
    print(f"- Stats CSV saved: {stats_csv_path}")
    return 0

if __name__ == '__main__':
    raise SystemExit(main())