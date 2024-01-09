# taskmanager_project/taskmanager_app/consumers.py
import json
import asyncio
import rclpy
from channels.generic.websocket import AsyncWebsocketConsumer
from celery.result import AsyncResult
from taskmanager_app.task_chain.task3 import execute_navigation_task
from taskmanager_app import client
from channels.db import database_sync_to_async
from taskmanager_app.models import TaskConfig, QueueFlagModel, BitMapModels



active_tasks = {}  # Dictionary to store active task IDs and corresponding Celery task objects

class TaskManagerConsumer(AsyncWebsocketConsumer):

    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._instance.groups = set()
            rclpy.init()
            cls._instance.basic_navigator = client.BasicNavigator()
        return cls._instance
    
    async def connect(self):
        await self.accept()

    async def disconnect(self):
        # Clean up tasks when the WebSocket connection is closed
        for task_id, celery_task in active_tasks.items():
            celery_task.revoke(terminate=True)
        active_tasks.clear()

    async def receive(self, text_data):
        try:
            # Assuming text_data is a JSON string, parse it
            data = json.loads(text_data)
            name = data.get('name')
            task_config = await self.get_task_config(name)
            
            queue_flag = await self.get_queue_flag_status()
            # if task_queue_enabled:
            if queue_flag == True:
                await self.run_task_with_queue(task_config)
            else:
                await self.run_task_without_queue(task_config)

        except json.JSONDecodeError:
            # Handle JSON decoding error
            await self.send(text_data=json.dumps({'error': 'Invalid JSON format'}))

        except Exception as e:
            # Handle other exceptions
            await self.send(text_data=json.dumps({'error': f'An error occurred: {str(e)}'}))

        data = json.loads(text_data)

        task_id = data.get('task_id')
        task_name = data.get('task_name')
        json_data = data.get('json_data')

        task = None  # Define task outside the if block

        if task_name == 'task3':
            if task_id in active_tasks:
                # Task with the same ID is already active, delay execution
                await self.send(text_data=json.dumps({'status': 'Task already active with the same ID'}))
                return
            else:
                # Queue the task for execution
                task = execute_navigation_task.delay(json_data)
                active_tasks[task_id] = task
        else:
            await self.send(text_data=json.dumps({'status': f'Task {task_name} not found'}))
            return

        try:
            # Keep the WebSocket connection alive until the task is canceled or completed
            while not AsyncResult(task.id).ready():
                # Send task status only if connected to the specific WebSocket
                if self.scope.get('path') == '/3001/':
                    await self.send_task_status(task_id, 'in_progress', result=None)
                await asyncio.sleep(1)

            result = task.result
            await self.send_task_status(task_id, 'completed', result)
        except asyncio.CancelledError:
            # Task was canceled by the client
            await self.send_task_status(task_id, 'canceled', result=None)
        finally:
            # Clean up tasks when the task is completed or canceled
            active_tasks.pop(task_id, None)
            task.revoke(terminate=True)

    async def send_task_status(self, task_id, status, result=None):
        response = {
            'action': 'task_status',
            'task_id': task_id,
            'status': status,
            'result': result,
        }
        await self.send(text_data=json.dumps(response))


class CancelTaskConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.accept()

        # Retrieve the BasicNavigator instance stored in the TaskManagerConsumer
        task_manager_consumer = TaskManagerConsumer()
        self.basic_navigator = task_manager_consumer.basic_navigator

        # Cancel the task using BasicNavigator instance
        self.basic_navigator.cancelTask()

    async def disconnect(self, close_code):
        # Clean up tasks when the WebSocket connection is closed
        for task_id, celery_task in active_tasks.items():
            celery_task.revoke(terminate=True)
        active_tasks.clear()


class NavConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.accept()

    async def disconnect(self, close_code):
        # Perform actions when the WebSocket connection is disconnected
        print(f"WebSocket connection closed with code: {close_code}")

        # Additional cleanup or actions can be added here

        # Call the parent class's disconnect method to ensure proper cleanup
        await super().disconnect(close_code)

    @database_sync_to_async
    def get_live_bitmap_instance(self):
        try:
            return BitMapModels.objects.filter(is_live=True).first()
        except Exception as e:
            print(f"Error fetching live BitMapModels instance: {e}")
            return None

    async def receive(self, text_data):
        try:
            # Assuming text_data is the instanceName you want to check
            instance_name_to_check = text_data

            # Get the live BitMapModels instance
            live_bitmap_instance = await self.get_live_bitmap_instance()

            # Check if the instance exists and has the necessary fields
            if live_bitmap_instance:
                advanced_point_list = live_bitmap_instance.map.get("advancedPointList", [])

                # Check if instance_name_to_check is present in advancedPointList.instanceName
                for point_data in advanced_point_list:
                    if "instanceName" in point_data and point_data["instanceName"] == instance_name_to_check:
                        # InstanceName found
                        dir_value = point_data.get("dir")
                        pos_x = point_data.get("pos", {}).get("x")
                        pos_y = point_data.get("pos", {}).get("y")

                        if dir_value is not None and pos_x is not None and pos_y is not None:
                            # Pass dir_value, pos_x, and pos_y to the Celery task
                            execute_navigation_task.delay(orientation_w=dir_value, x=pos_x, y=pos_y)

                            await self.send(text_data=json.dumps({'status': 'Task has been triggered'}))
                            await self.send(text_data=f"Instance {instance_name_to_check} found in live BitMapModels!\n"
                                                      f"dir: {dir_value}\n"
                                                      f"pos: x={pos_x}, y={pos_y}")
                        else:
                            # Handle missing values
                            await self.send(text_data=f"Error: Incomplete data for instance {instance_name_to_check}.")
                        
                        break
                else:
                    # InstanceName not found
                    await self.send(text_data=f"Instance {instance_name_to_check} not found in live BitMapModels.")
            else:
                # Handle the case where the necessary keys are not present in the map
                await self.send(text_data="Error: Unable to check instanceName in live BitMapModels.")

        except Exception as e:
            print(f"Error in receive method: {e}")
            # Handle the exception and send an error message to the client if needed
            await self.send(text_data=f"Error: {str(e)}")
