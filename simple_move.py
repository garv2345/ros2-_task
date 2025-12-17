import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('simple_mover')
    
    # Connect to the robot controller
    publisher = node.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
    
    # Wait for connection to be established
    time.sleep(1)
    print("Sending Double-Move Command...")
    
    msg = JointTrajectory()
    msg.joint_names = [
        'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
    ]
    
    # --- MOVE 1: Swing to the Side (at 2 seconds) ---
    point1 = JointTrajectoryPoint()
    point1.positions = [1.57, -1.57, 1.57, 0.0, 0.0, 0.0]
    point1.time_from_start.sec = 2
    msg.points.append(point1)

    # --- MOVE 2: Return to Upright/Home (at 5 seconds) ---
    point2 = JointTrajectoryPoint()
    point2.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    point2.time_from_start.sec = 5
    msg.points.append(point2)
    
    publisher.publish(msg)
    print("Command Sent! Watch the robot dance.")
    
    # Wait for the robot to finish moving before closing
    time.sleep(6)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
