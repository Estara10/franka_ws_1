with open("sensor_dashboard.py", "r") as f:
    code = f.read()

code = code.replace("collections.deque(maxlen=100)", "collections.deque()")

with open("sensor_dashboard.py", "w") as f:
    f.write(code)
