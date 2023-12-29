# taskmanager_project/taskmanager_app/task_chain/task3.py
import json
import rclpy
from taskmanager_app.client import BasicNavigator, PoseStamped, TaskResult
from rclpy.duration import Duration
from celery import shared_task

# Load the configuration from task3_config.json
with open('taskmanager_app/task_chain/task_configs/task3_config.json', 'r') as config_file:
    config_data = json.load(config_file)

goal_pose_config = config_data.get('task3', {}).get('execute_navigation_task', {})

@shared_task
def execute_navigation_task(json_data):
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
        goal_pose.pose.position.x = goal_pose_config.get('x', 0.0)
        goal_pose.pose.position.y = goal_pose_config.get('y', 0.0)
        goal_pose.pose.orientation.w = goal_pose_config.get('orientation_w', 1.0)

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
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
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
        navigator.shutdown()
