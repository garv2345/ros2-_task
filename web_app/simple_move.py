import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class ArmDancer(Node):
    def __init__(self):
        super().__init__('arm_dancer_node')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.step = 0

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()

        if self.step == 0:
            # POSE 1: Stretch Out
            point.positions = [1.5, -0.5, 0.5, 0.0, 0.0, 0.0]
            point.time_from_start = Duration(sec=2)
            self.get_logger().info('DANCE MOVE 1: Stretching Out...')
            self.step = 1
        elif self.step == 1:
            # POSE 2: Go Home
            point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            point.time_from_start = Duration(sec=2)
            self.get_logger().info('DANCE MOVE 2: Going Home...')
            self.step = 2
        else:
            # Done
            return

        msg.points.append(point)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmDancer()

    # Run loop for 5 seconds to allow both moves
    start_time = time.time()
    while rclpy.ok() and (time.time() - start_time) < 6:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
