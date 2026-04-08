#!/usr/bin/env python3
import os
import glob
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    input_dir = 'experiment_results/reports'
    output_dir = 'experiment_results/plots'
    
    # 确保输出目录存在
    os.makedirs(output_dir, exist_ok=True)
    
    # 查找所有通信测试数据(samples文件)
    pattern = os.path.join(input_dir, 'comm_latency_*_samples.csv')
    csv_files = glob.glob(pattern)
    
    # 按照文件创建时间排序，获取最新的文件
    csv_files.sort(key=os.path.getctime, reverse=True)
    
    # 根据用户要求，最多只生成前 10 个数据文件的图表
    max_files = 10
    files_to_process = csv_files[:max_files]
    
    if not files_to_process:
        print("未找到任何通信延迟的CSV数据(.csv)。")
        return
        
    print(f"📂 共找到 {len(csv_files)} 个通信测试数据文件。将仅处理最新的 {len(files_to_process)} 个：")
    
    for file in files_to_process:
        times = []
        latencies = []
        try:
            with open(file, 'r') as f:
                reader = csv.reader(f)
                header = next(reader)
                
                # 预期的表头包含 receive_time_sec 和 latency_ms
                try:
                    time_idx = header.index('receive_time_sec')
                    lat_idx = header.index('latency_ms')
                except ValueError:
                    print(f"⚠️ 跳过 {file}, 表头格式不符合预期: {header}")
                    continue
                    
                for row in reader:
                    if not row: continue
                    times.append(float(row[time_idx]))
                    latencies.append(float(row[lat_idx]))
            
            if not times:
                print(f"⚠️ 跳过 {file}, 数据为空。")
                continue
                
            # 开始准备画布并画图
            plt.figure(figsize=(10, 5))
            plt.plot(times, latencies, label='Latency (ms)', color='#1f77b4', linewidth=1.5)
            plt.title(f"Communication Latency over Time\n{os.path.basename(file)}")
            plt.xlabel("Receive Time (sec)")
            plt.ylabel("Latency (ms)")
            plt.grid(True, linestyle='--', alpha=0.7)
            plt.legend(loc='upper right')
            
            # 另存为图片
            filename_base = os.path.basename(file).replace('.csv', '.png')
            output_path = os.path.join(output_dir, filename_base)
            plt.savefig(output_path, dpi=120, bbox_inches='tight')
            plt.close()
            print(f"✅ 已生成图表: {output_path}")
            
        except Exception as e:
            print(f"❌ 处理 {os.path.basename(file)} 时出错: {e}")

if __name__ == '__main__':
    main()
