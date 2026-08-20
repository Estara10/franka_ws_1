import re

with open('sensor_dashboard.py', 'r') as f:
    text = f.read()

text = text.replace('def update_loop():\n        update_ui(root, node, vars, canvas, ax1, ax2, line_l, line_r, line_diff)',
                    'def update_loop():\n        update_ui(root, node, vars, canvas, ax1, ax2, ax3, line_l, line_r, line_diff, line_smooth)')

with open('sensor_dashboard.py', 'w') as f:
    f.write(text)
