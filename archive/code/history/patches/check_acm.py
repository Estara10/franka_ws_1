#!/usr/bin/env python3
"""诊断 MoveIt2 运行时的 ACM 状态"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
import sys

class ACMChecker(Node):
    def __init__(self):
        super().__init__('acm_checker')
        self.client = self.create_client(GetPlanningScene, '/get_planning_scene')
        
    def check_acm(self):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('服务 /get_planning_scene 不可用')
            return False
            
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        acm = response.scene.allowed_collision_matrix
        
        print(f"\n=== ACM 状态 ===")
        print(f"Entry names 数量: {len(acm.entry_names)}")
        print(f"Default entry names 数量: {len(acm.default_entry_names)}")
        
        if len(acm.entry_names) == 0:
            print("\n⚠️  ACM 为空！SRDF 规则未被加载到 Planning Scene")
            return False
        else:
            print(f"\n✓ ACM 包含 {len(acm.entry_names)} 个链接")
            print(f"链接列表: {acm.entry_names[:10]}...")
            return True

def main():
    rclpy.init()
    checker = ACMChecker()
    result = checker.check_acm()
    checker.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if result else 1)

if __name__ == '__main__':
    main()
