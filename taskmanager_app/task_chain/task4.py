import rclpy
from celery import shared_task
from example_interfaces.action import Fibonacci
from taskmanager_app.client2 import FibonacciActionClient
import logging

logger = logging.getLogger(__name__)

cancel_task = False  # Initialize cancel_task flag

@shared_task
def fibonacci(order):
    global cancel_task  # Refer to the global variable

    try:
        rclpy.init()
        logger.info('ROS 2 initialized successfully.')

        # Create an instance of the FibonacciActionClient
        action_client = FibonacciActionClient()

        logger.info('Action started')

        # Send the Fibonacci goal with the specified order
        action_client.send_goal(order)

        # Periodically check and cancel the goal if cancel_task is True
        while not action_client.get_feedback_response() and not cancel_task:
            rclpy.spin_once(action_client, timeout_sec=1)
            # You can adjust the sleep duration based on your requirements
            # For example, time.sleep(0.1) for a shorter delay

        # Check if the task was canceled
        if cancel_task:
            action_client.cancel_goal()
            logger.info('Fibonacci task canceled.')
            return {'status': 'Task canceled'}

        # Get the feedback response if the goal is completed
        feedback_response = action_client.get_feedback_response()

        # Log and return the feedback response
        logger.info(f'Feedback response: {feedback_response.sequence}')
        return {'feedback_response': feedback_response.sequence}

    except Exception as e:
        # Log an error and retry the task
        logger.error(f'Error during Fibonacci task: {e}')
        raise fibonacci.retry(exc=e)

    finally:
        # Shutdown the ROS node
        rclpy.shutdown()
        logger.info('ROS 2 shutdown completed.')

# Function to set the cancel_task flag
# def set_cancel_task_flag(flag):
#     global cancel_task
#     cancel_task = flag
