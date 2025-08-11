import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
import numpy as np
import math

class TwistToPose(Node):
    def __init__(self):
        super().__init__('twist_to_pose')
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/current_pose', self.pose_callback, 10)

        # 目标姿态
        self.target_pos = np.array([0.3, 0.0, 0.2])  

        # 参数
        self.pos_tol = 0.005      # 位置容忍度 (5mm)
        self.max_speed = 0.2      # 最大速度 (m/s)
        self.min_speed = 0.05     # 最低速度 (m/s)

        self.timer = self.create_timer(0.02, self.control_loop)
        self.current_pos = None

    def pose_callback(self, msg: PoseStamped):
        self.current_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

    def control_loop(self):
        if self.current_pos is None:
            self.get_logger().info("等待当前位置...")
            return

        delta = self.target_pos - self.current_pos
        distance = np.linalg.norm(delta)

        if distance < self.pos_tol:
            self.get_logger().info("✅ 到达目标位置，停止！")
            self.publish_twist(np.zeros(3))
            rclpy.shutdown()
            return

        direction = delta / distance
        desired_speed = distance  # 按距离缩放
        # 限制在 [min_speed, max_speed] 之间
        speed = max(self.min_speed, min(self.max_speed, desired_speed))
        vel = direction * speed

        self.publish_twist(vel)

    def publish_twist(self, linear_vel):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear_vel[0])
        msg.twist.linear.y = float(linear_vel[1])
        msg.twist.linear.z = float(linear_vel[2])
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.twist_pub.publish(msg)


def main():
    rclpy.init()
    node = TwistToPose()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
