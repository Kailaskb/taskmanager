import rclpy
from rclpy.action import ActionServer, GoalResponse
from example_interfaces.action import Fibonacci
import time

class FibonacciServer:

    def __init__(self):
        self.node = rclpy.create_node('fibonacci_action_server')
        self.action_server = ActionServer(
            self.node,
            Fibonacci,
            'fibonacci',
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback
        )
        self.request_received_time = None  # Variable to store the request time

    def goal_callback(self, goal_request):
        # Optionally, you can implement checks before accepting a goal
        self.request_received_time = time.time()  # Store the time when the goal request is received
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """
        Callback function called when a cancellation request is received.
        """
        self.node.get_logger().info('Cancellation requested. Stopping execution...')
        goal_handle.canceled()

    def execute_callback(self, goal_handle):
        self.node.get_logger().info('Executing goal...')

        # Access the stored request time
        if self.request_received_time is not None:
            self.node.get_logger().info(f'Request received time: {self.request_received_time}')

            # Specify the duration to wait (e.g., 5 seconds)
            wait_duration = 5
            end_time = self.request_received_time + wait_duration

            # Add a while loop to wait for the specified duration
            while time.time() < end_time:
                remaining_time = end_time - time.time()
                self.node.get_logger().info(f'Waiting for {remaining_time:.2f} seconds...')

                # Create a feedback message
                feedback_msg = Fibonacci.Feedback()
                feedback_msg.sequence = [0, 1]

                for _ in range(goal_handle.request.order):
                    next_number = feedback_msg.sequence[-1] + feedback_msg.sequence[-2]
                    feedback_msg.sequence.append(next_number)

                    # Publish feedback
                    goal_handle.publish_feedback(feedback_msg)

                    # Simulate doing some work
                    self.node.get_logger().info(f'Generated number: {next_number}')
                    self.node.get_logger().info('Doing work...')
                    self.node.get_logger().info('Work done.')

                    # Check for cancellation requests
                    if goal_handle.is_cancel_requested:
                        self.cancel_callback(goal_handle)
                        return

                    # Sleep for a short duration (adjust as needed)
                    time.sleep(1)

            # Goal successfully completed
            result = Fibonacci.Result()
            result.sequence = feedback_msg.sequence
            goal_handle.succeed()
            return result

def main(args=None):
    rclpy.init(args=args)

    fibonacci_server = FibonacciServer()
    rclpy.spin(fibonacci_server.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
