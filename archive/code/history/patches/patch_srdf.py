with open("src/multipanda_ros2/franka_moveit_config/srdf/dual_panda.srdf.xacro", "r") as f:
    target = f.read()

new_collisions = """
  <!-- FIX RETREAT FALSE POSITIVES -->
  <disable_collisions link1="mj_left_link4" link2="mj_right_link6" reason="Never"/>
  <disable_collisions link1="mj_left_link4" link2="mj_right_link7" reason="Never"/>
  <disable_collisions link1="mj_left_link4" link2="mj_right_link8" reason="Never"/>
  <disable_collisions link1="mj_left_link4" link2="mj_right_hand" reason="Never"/>
  
  <disable_collisions link1="mj_left_link5" link2="mj_right_link6" reason="Never"/>
  <disable_collisions link1="mj_left_link5" link2="mj_right_link7" reason="Never"/>
  <disable_collisions link1="mj_left_link5" link2="mj_right_link8" reason="Never"/>
  <disable_collisions link1="mj_left_link5" link2="mj_right_hand" reason="Never"/>

  <disable_collisions link1="mj_right_link4" link2="mj_left_link6" reason="Never"/>
  <disable_collisions link1="mj_right_link4" link2="mj_left_link7" reason="Never"/>
  <disable_collisions link1="mj_right_link4" link2="mj_left_link8" reason="Never"/>
  <disable_collisions link1="mj_right_link4" link2="mj_left_hand" reason="Never"/>
  
  <disable_collisions link1="mj_right_link5" link2="mj_left_link6" reason="Never"/>
  <disable_collisions link1="mj_right_link5" link2="mj_left_link7" reason="Never"/>
  <disable_collisions link1="mj_right_link5" link2="mj_left_link8" reason="Never"/>
  <disable_collisions link1="mj_right_link5" link2="mj_left_hand" reason="Never"/>
</robot>
"""

target = target.replace("</robot>", new_collisions)

with open("src/multipanda_ros2/franka_moveit_config/srdf/dual_panda.srdf.xacro", "w") as f:
    f.write(target)
print("patched SRDF")
