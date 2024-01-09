# taskmanager_project/taskmanager_app/task_chain/task3.py
import json
import rclpy
from taskmanager_app.client import BasicNavigator, PoseStamped, TaskResult
from rclpy.duration import Duration
from celery import shared_task

goal_pose_config = {}

@shared_task
def execute_navigation_task(x,y,orientation_w):
    if not rclpy.ok():
        rclpy.init()

    navigator = BasicNavigator()

    try:
        print("Navigation started")
        navigator.waitUntilNav2Active()
        print('Navigation server started')

        # Use the goal_pose configuration from the JSON file
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x 
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = orientation_w

        navigator.goToPose(goal_pose)

        i = 0
        while not navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            # Inside the while loop where feedback is obtained
            feedback = navigator.getFeedback()
            if feedback and hasattr(feedback, 'estimated_time_remaining') and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    goal_pose.pose.position.x = -3.0
                    navigator.goToPose(goal_pose)

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        navigator.lifecycleShutdown()
        return result

        # exit(0)
    
    except Exception as e:
        print(f'Error during navigation: {e}')
        raise execute_navigation_task.retry(exc=e)
    
    finally:
        try:
            if hasattr(navigator, 'shutdown'):
                navigator.shutdown()
        except Exception as shutdown_error:
            print(f'Error during shutdown: {shutdown_error}')
