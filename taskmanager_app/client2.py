import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self._feedback_response = None
        self._send_goal_future = None

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.sequence))
        self._feedback_response = feedback

    def cancel_goal(self):
        if self._send_goal_future and not self._send_goal_future.done():
            goal_handle = self._send_goal_future.result()
            if goal_handle and goal_handle.is_active:
                goal_handle.cancel_goal()
                self.get_logger().info('Goal canceled.')

    def get_feedback_response(self):
        return self._feedback_response

def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    # Wait for a moment (simulating some time passing)
    rclpy.spin_once(action_client, timeout_sec=1.0)

    # Cancel the goal after some time (simulating user intervention)
    action_client.cancel_goal()

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
