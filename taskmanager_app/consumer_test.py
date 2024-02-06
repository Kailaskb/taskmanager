import json
import asyncio
import rclpy
from channels.generic.websocket import AsyncWebsocketConsumer
from celery.result import AsyncResult
from taskmanager_app.task_chain.task3 import execute_navigation_task
from taskmanager_app import client
from channels.db import database_sync_to_async
from taskmanager_app.models import TaskConfig, FlagModel, BitMapModels, TaskResponseModels, MapBackupModels
from taskmanager_app.task_chain.task3 import task_result 
from asgiref.sync import sync_to_async
from taskmanager_app.client import TaskResult
# from taskmanager_app.models import FlagModel
flag_instance = FlagModel()

active_tasks = {}  # Dictionary to store active task IDs and corresponding Celery task objects
feedback_group = "feedback_group"

TASK_RESPONSE = {
    'SUCCEEDED': 1,
    'CANCELED' : 2,
    'FAILED' : 3
}

class TestCombinedConsumer(AsyncWebsocketConsumer):
    
    async def connect(self):
        await self.accept()

    async def disconnect(self, close_code):
        print(f"WebSocket connection closed with code: {close_code}")

        await super().disconnect(close_code)

    @database_sync_to_async
    def get_live_bitmap_instance(self):
        try:
            return BitMapModels.objects.filter(is_live=True).first()
        except Exception as e:
            print(f"Error fetching live BitMapModels instance: {e}")
            return None

    @database_sync_to_async
    def get_task_config(self, name):
        try:
            return TaskConfig.objects.filter(name=name).first()
        except Exception as e:
            print(f"Error fetching TaskConfig instance: {e}")
            return None

    async def response_feedback(self, actionName, task_name, actionGroupName, actionGroupId):
        feedback_data = {
            'Action name': actionName,
            'Task name': task_name,
            'Action group name': actionGroupName,
            'Action group Id': actionGroupId
        }
        await self.channel_layer.group_send(
            feedback_group,
            {
                'type': 'send_feedback',
                'Action status': feedback_data
            }
        )


    async def send_feedback(self, event):
        feedback_data = event['data']
        await self.send(text_data=json.dumps(feedback_data))

    
    async def handle_error(self, error_message):
        await self.send(text_data=f"Error: {error_message}")

    async def process_instance(self, instance_name_to_check, dir_value, pos_x, pos_y):
        if dir_value is not None and pos_x is not None and pos_y is not None:
            # Pass dir_value, pos_x, and pos_y to the Celery task
            execute_navigation_task.delay(orientation_w=dir_value, x=pos_x, y=pos_y)

            # await self.send(text_data=json.dumps({'status': 'Task has been triggered'}))
            await self.send(text_data=f"Instance {instance_name_to_check} found in live BitMapModels!\n"
                                      f"dir: {dir_value}\n"
                                      f"pos: x={pos_x}, y={pos_y}")
        else:
            # Handle missing values
            await self.handle_error(f"Incomplete data for instance {instance_name_to_check}.")

    async def receive(self, text_data):
        try:
            name = text_data
            task_config = await self.get_task_config(name)

            # Extract instance_name_to_check from task_config
            if task_config:
                for task in task_config.task:
                    for action_group in task.get("actionGroups", []):
                        if action_group.get("actionName") == "navigation_action":
                            for param in action_group["params"]:
                                if param.get("key") == "target_name" and "stringValue" in param:
                                    instance_name_to_check = param["stringValue"]
                                    await self.response_feedback(task_name=task_config.name, actionName=action_group.get("actionName"), actionGroupName=task.get("actionGroupName"), actionGroupId=task.get("actionGroupId"))

                                    # Get the live BitMapModels instance
                                    live_bitmap_instance = await self.get_live_bitmap_instance()

                                    # Check if the instance exists and has the necessary fields
                                    if live_bitmap_instance:
                                        advanced_point_list = live_bitmap_instance.map.get("advancedPointList", [])

                                        # Check if instance_name_to_check is present in advancedPointList.instanceName
                                        for point_data in advanced_point_list:
                                            if "instanceName" in point_data and point_data["instanceName"] == instance_name_to_check:
                                                dir_value = point_data.get("dir")
                                                pos_x = point_data.get("pos", {}).get("x")
                                                pos_y = point_data.get("pos", {}).get("y")

                                                # Execute navigation task and await the result
                                                task_result = execute_navigation_task.delay(
                                                    x=pos_x, y=pos_y, orientation_w=dir_value
                                                )

                                                # Check the result using the get method
                                                result = task_result.get()
                                                if result['status'] == 'completed':
                                                    await self.process_instance(instance_name_to_check, dir_value, pos_x, pos_y)
                                                    await self.send(text_data=f"Task {task_config.name} completed: {instance_name_to_check}")
                                                else:
                                                    await self.send(f"{instance_name_to_check} has been finished")

                                    else:
                                        await self.handle_error("Unable to check instanceName in live BitMapModels.")

                                    # Break from the loop once a navigation_action is found in the current action group
                                    break

                            # Check if there are more tasks to process
                            if task_config.task.index(task) < len(task_config.task) - 1:
                                await self.send(text_data=f"Moving to the next task in {task_config.name}")

                            else:
                                await self.send(text_data=f"All tasks in {task_config.name} completed.")

                else:
                    await self.send(text_data="No task found")

        except Exception as e:
            print(f"Error in receive method: {e}")
            await self.handle_error(str(e))


class NavResponseConsumer(AsyncWebsocketConsumer):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stored_serialized_data = None

    async def connect(self):
        await self.accept()
        await self.channel_layer.group_add(feedback_group, self.channel_name)

    async def disconnect(self, close_code):
        await self.channel_layer.group_discard(feedback_group, self.channel_name)

    async def receive(self, text_data):
        if text_data == "status":
            # Your logic here
            if self.stored_serialized_data:
                # Send the stored serialized data
                await self.send(text_data=self.stored_serialized_data)
            else:
                await self.send("No Action response found")
        else:
            await self.send("Invalid action")

    async def send_feedback_data(self, event):
        feedback_data = event['Action status']
        print(f"Received feedback data: {feedback_data}")
        # await self.send(f"Received feedback data: {feedback_data}")

        # Store the serialized data in a variable
        self.stored_serialized_data = json.dumps(feedback_data)

        # Send the data
        # await self.send_feedback_response(self.stored_serialized_data)

    async def send_feedback_response(self, serialized_data):
        # You can now use the serialized data variable as needed
        await self.send(text_data=serialized_data)

    async def send_feedback(self, feedback_data):
        await self.channel_layer.group_send(
            feedback_group,
            {
                'type': 'send_feedback_data',
                'Action status': feedback_data
            }
        )
        
class CancelGoalConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.accept()
        await self.channel_layer.group_add("feedback_group", self.channel_name)
        

    async def disconnect(self, close_code):
        await self.channel_layer.group_discard("feedback_group", self.channel_name)

    async def receive(self, text_data):
        if text_data == "cancel":
            await self.set_cancel_flag()
            await self.send(text_data=json.dumps({'message': 'Cancel flag changed to true'}))
        else:
            await self.send(text_data=json.dumps({'message': 'Invalid action'}))

    @database_sync_to_async
    def set_cancel_flag(self):
        try:
            # Replace with your actual model and initialization logic
            flag_instance = FlagModel.objects.get(pk=1)
            flag_instance.cancel_flag = True
            flag_instance.save()
            print("Cancel flag changed to true")
        except FlagModel.DoesNotExist:
            print("Flag instance not found")
        except Exception as e:
            print(f"Error: {str(e)}")