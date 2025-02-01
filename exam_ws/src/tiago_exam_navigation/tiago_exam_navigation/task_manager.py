#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread
from tiago_exam_navigation.navigation_client import EnhancedNavigationClient
from tiago_exam_navigation.manipulation_client import ManipulationClient

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        
        # Simple numeric state instead of enum
        self.state = 0
        self.current_marker_index = 0
        self.retry_count = 0
        
        # Marker sequence from requirements
        self.marker_sequence = [
            {'id': 63, 'place_frame': 'place_zone_1'},
            {'id': 582, 'place_frame': 'place_zone_2'}
        ]
        
        # Configuration parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_retries', 3),
                ('nav_timeout', 300),
                ('manip_timeout', 120),
                ('recovery_wait', 5.0)
            ]
        )
        
        # Initialize clients with callback groups
        self.cb_group = ReentrantCallbackGroup()
        self.nav_client = EnhancedNavigationClient()
        self.manip_client = ManipulationClient()
        
        # Set up executor in background thread
        executor = MultiThreadedExecutor(4)
        executor.add_node(self)
        self.executor_thread = Thread(target=executor.spin, daemon=True)
        self.executor_thread.start()
        
        # Sleep briefly to allow initialization
        self.create_rate(10.0).sleep()

    def run_state_machine(self):
        """Main state machine loop"""
        while rclpy.ok():
            current_marker = self.marker_sequence[self.current_marker_index]
            pick_frame = f"aruco_marker_{current_marker['id']}"
            place_frame = current_marker['place_frame']

            if self.state == 0:
                # Navigate to pick location
                self.get_logger().info("State 0: Navigating to pick location")
                success = self._execute_with_timeout(
                    self.nav_client.navigate_to_frame(pick_frame),
                    self.get_parameter('nav_timeout').value
                )
                if success:
                    self.state = 1
                else:
                    self.state = 5  # Recovery state

            elif self.state == 1:
                # Pick object
                self.get_logger().info("State 1: Picking object")
                success = self._execute_with_timeout(
                    self.manip_client.pick_object(pick_frame),
                    self.get_parameter('manip_timeout').value
                )
                if success:
                    self.state = 2
                else:
                    self.state = 5

            elif self.state == 2:
                # Navigate to place location
                self.get_logger().info("State 2: Navigating to place location")
                success = self._execute_with_timeout(
                    self.nav_client.navigate_to_frame(place_frame),
                    self.get_parameter('nav_timeout').value
                )
                if success:
                    self.state = 3
                else:
                    self.state = 5

            elif self.state == 3:
                # Place object
                self.get_logger().info("State 3: Placing object")
                success = self._execute_with_timeout(
                    self.manip_client.place_object(place_frame),
                    self.get_parameter('manip_timeout').value
                )
                if success:
                    self.state = 4
                else:
                    self.state = 5

            elif self.state == 4:
                # Task completion check
                self.get_logger().info("State 4: Checking task completion")
                self.current_marker_index += 1
                self.retry_count = 0
                
                if self.current_marker_index >= len(self.marker_sequence):
                    self.get_logger().info("All markers processed!")
                    break
                else:
                    self.state = 0  # Start next marker

            elif self.state == 5:
                # Recovery state
                self.get_logger().warn("State 5: Recovery")
                self._recovery_procedure()
                
                if self.retry_count < self.get_parameter('max_retries').value:
                    self.retry_count += 1
                    self.state = 0  # Retry from beginning
                else:
                    self.get_logger().error(f"Failed marker after {self.retry_count} retries")
                    self.current_marker_index += 1
                    self.retry_count = 0
                    if self.current_marker_index >= len(self.marker_sequence):
                        break
                    self.state = 0

    async def _execute_with_timeout(self, coroutine, timeout):
        """Execute coroutine with timeout and error handling"""
        try:
            return await rclpy.task.wait_for_future_complete(
                coroutine,
                timeout_sec=timeout
            )
        except TimeoutError:
            self.get_logger().error(f"Operation timed out after {timeout}s")
            return False
        except Exception as e:
            self.get_logger().error(f"Operation failed: {str(e)}")
            return False

    def _recovery_procedure(self):
        """Execute system-wide recovery actions"""
        self.get_logger().warn("Executing recovery procedure...")
        
        # Cancel current operations
        self.nav_client.cancel_navigation()
        self.manip_client.home_arm()
        
        # Wait recovery time
        self.create_rate(1/self.get_parameter('recovery_wait').value).sleep()
        
        self.get_logger().info("Recovery completed")

    def _shutdown(self):
        """Clean shutdown procedure"""
        self.get_logger().info("Shutting down...")
        self.nav_client.destroy()
        self.manip_client.destroy()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        task_manager = TaskManager()
        task_manager.run_state_machine()
        
    except KeyboardInterrupt:
        task_manager.get_logger().info("Task interrupted by user")
    finally:
        if rclpy.ok():
            task_manager._shutdown()

if __name__ == '__main__':
    main()