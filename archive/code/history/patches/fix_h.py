with open("src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp", "r") as f:
    cpp_code = f.read()

if "<std_msgs/msg/string.hpp>" not in cpp_code:
    cpp_code = cpp_code.replace("#include <std_msgs/msg/bool.hpp>", "#include <std_msgs/msg/bool.hpp>\n#include <std_msgs/msg/string.hpp>")
    with open("src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp", "w") as f:
        f.write(cpp_code)
