import rclpy, argparse
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

from linkattacher_msgs.srv import AttachLink, DetachLink

class GripperController(Node):
    def __init__(self, model=None):
        super().__init__('gripper_controller')

        # Setup gripper action client
        self.gripper_joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        self.gripper_action_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        # Setup service clients for attach/detach
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')

        for client, name in [(self.attach_client, 'ATTACHLINK'), (self.detach_client, 'DETACHLINK')]:
            while not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().info(f'⏳ Waiting for /{name} service...')
            
            if not client.service_is_ready():
                self.get_logger().error(f"❌ Service /{name} is not ready")
                return
            
            self.get_logger().info(f"✅ Service /{name} is ready")

        # Model & link names
        self.model1 = 'tiago'
        self.link1 = 'gripper_left_finger_link'
        self.link2 = 'link'
        
        self.model2 = None
        
        if model == '582':
            self.model2 = 'aruco_cube_exam_id582'
        elif model == '63':
            self.model2 = 'aruco_cube_exam_id63'
         

    def control_gripper(self, open):
        """Controls the gripper (open or close) and attaches/detaches the object."""
        self.get_logger().info(f"👐 {'Opening' if open else 'Closing'} gripper")
        
        # Create a JointTrajectory message for the gripper
        msg = JointTrajectory()
        msg.joint_names = self.gripper_joint_names

        point = JointTrajectoryPoint()
        
        #  The maxium gripper opening is 0.065 m but it gives issues,
        #  so we set it to 0.060 m
        #  Close gripper to 0.030 m since aruco_cube_exam_id582 is 0.06 m
        point.positions = [0.064, 0.064] if open else [0.037, 0.037]
        point.time_from_start = Duration(sec=1, nanosec=0)
        msg.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = msg

        # Wait for action server
        if not self.gripper_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ Gripper action server not available")
            return False

        # Send goal
        goal_future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=10.0)

        if not goal_future.done():
            self.get_logger().error("❌ Gripper goal timed out (send)")
            return False

        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Gripper goal was rejected")
            return False

        self.get_logger().info("✅ Gripper goal accepted, waiting for result...")

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        if not result_future.done():
            self.get_logger().error("❌ Gripper result timeout")
            return False

        result = result_future.result()

        success = result.result.error_code == 0
        if success:
            self.get_logger().info("✅ Gripper movement succeeded")
            if open:
                self.detach_all_models()
            else:
                self.attach()
        else:
            self.get_logger().error(f"❌ Gripper movement failed with error code: {result.result.error_code}")

        return success
    

    def attach(self):
        self.get_logger().info("🔗 Attaching object...")
        req = AttachLink.Request()
        req.model1_name = self.model1
        req.link1_name = self.link1
        req.model2_name = self.model2
        req.link2_name = self.link2
        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info("✅ Object attached successfully.")
        else:
            self.get_logger().error("❌ Failed to attach object.")

    def detach(self):
        self.get_logger().info("🧩 Detaching object...")
        req = DetachLink.Request()
        req.model1_name = self.model1
        req.link1_name = self.link1
        req.model2_name = self.model2
        req.link2_name = self.link2
        future = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info("✅ Object detached successfully.")
        else:
            self.get_logger().error("❌ Failed to detach object.")
            
    def detach_all_models(self):
        """Detach all possible models to ensure clean state when opening gripper."""
        self.get_logger().info("🧩 Detaching all potential objects...")
        
        # Try to detach model 63
        temp_model2 = self.model2  # Store current model
        self.model2 = "aruco_cube_exam_id63"
        self.detach()
        
        # Try to detach model 582
        self.model2 = "aruco_cube_exam_id582"
        self.detach()
        
        # Restore original model setting
        self.model2 = temp_model2

def main(args=None):
    parser = argparse.ArgumentParser(description='Gripper controller Node')
    parser.add_argument('--input_string', type=str, required=True, 
                        help='possible values: "OPEN", "CLOSE63", or "CLOSE582"')
    parsed_args, other_args = parser.parse_known_args()
    
    rclpy.init(args=other_args)
    
    command = parsed_args.input_string.strip().upper()
    
    success = False
    while not success:
        if command == "OPEN":
            node = GripperController()  # No specific model needed for open
            success = node.control_gripper(open=True)
        elif command == "CLOSE63":
            node = GripperController(model="63")
            success = node.control_gripper(open=False)
        elif command == "CLOSE582":
            node = GripperController(model="582")
            success = node.control_gripper(open=False)
        else:
            print(f"Invalid input_string: {parsed_args.input_string} (use 'OPEN', 'CLOSE63', 'CLOSE582')")
            rclpy.shutdown()
            return
        
        if not success:
            node.get_logger().error("❌ Failed to open gripper, retrying...")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
