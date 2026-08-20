import re

with open("sensor_dashboard.py", "r") as f:
    py_code = f.read()

old_print = '            print(f"\\\\n✅ 实验已完成，图表已自动保存至: {filename}")'
new_print = old_print + '\n            import sys\n            sys.stdout.flush()'

if 'sys.stdout.flush()' not in py_code:
    py_code = py_code.replace(old_print, new_print)
    with open("sensor_dashboard.py", "w") as f:
        f.write(py_code)
