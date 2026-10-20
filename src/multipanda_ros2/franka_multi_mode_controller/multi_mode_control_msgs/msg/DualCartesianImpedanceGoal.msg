geometry_msgs/Pose l_pose
geometry_msgs/Pose r_pose
geometry_msgs/Twist l_twist
geometry_msgs/Twist r_twist
float64[7] l_q_n
float64[7] r_q_n
# Explicit payload feedforward, expressed in each arm's link0 (zero-Jacobian)
# frame. Zero by default: an absent field leaves the original control law
# unchanged.
geometry_msgs/Vector3 l_ff_force
geometry_msgs/Vector3 r_ff_force
