import csv, time, sys
from datetime import datetime, timezone
import rclpy
from dual_arm_a1_msgs.msg import GripperContactFeedback
rclpy.init()
node=rclpy.create_node("p3a2c_contact_feedback_recorder")
f=open(sys.argv[1],"w",newline="",buffering=1)
w=csv.writer(f)
w.writerow(["receive_unix_ns","receive_utc","header_sec","header_nanosec","arm","finger_q1_m","finger_q2_m","total_gap_m","left_finger_obj_rod_01","right_finger_obj_rod_01","update_seq"])
def cb(m):
    now=time.time_ns()
    utc=datetime.fromtimestamp(now/1e9,tz=timezone.utc).isoformat(timespec="microseconds")
    w.writerow([now,utc,m.header.stamp.sec,m.header.stamp.nanosec,m.arm,m.finger_q1,m.finger_q2,m.finger_q1+m.finger_q2,int(m.left_finger_rod_contact),int(m.right_finger_rod_contact),m.update_seq])
node.create_subscription(GripperContactFeedback,"/gripper_contact_feedback",cb,20)
try: rclpy.spin(node)
except KeyboardInterrupt: pass
finally:
    f.flush(); f.close(); node.destroy_node(); rclpy.shutdown()
