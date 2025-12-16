import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ArmMover(Node):
    def __init__(self):
        super().__init__('arm_mover_node')
        # This topic is the "Input Pipe" for the robot driver
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.moved = False

    def timer_callback(self):
        if self.moved:
            return
            
        msg = JointTrajectory()
        # The names of the 6 motors in the UR5 arm
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        # POINT 1: Move to "Home" position
        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        point.time_from_start = Duration(sec=2)
        msg.points.append(point)

        # Send the message
        self.publisher_.publish(msg)
        self.get_logger().info('Sending command: Move to Home Position...')
        self.moved = True

def main(args=None):
    rclpy.init(args=args)
    node = ArmMover()
    
    # Keep the node alive for 3 seconds to ensure message is sent
    import time
    time.sleep(0.5) 
    rclpy.spin_once(node)
    time.sleep(2)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
