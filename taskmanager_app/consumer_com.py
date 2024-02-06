import json
from channels.generic.websocket import AsyncWebsocketConsumer
from celery.result import AsyncResult
from taskmanager_app.task_chain.task3 import execute_navigation_task
from taskmanager_app.models import TaskConfig, FlagModel, BitMapModels, TaskResponseModels, MapBackupModels
from asgiref.sync import sync_to_async
from channels.db import database_sync_to_async
from asgiref.sync import async_to_sync
import asyncio


class CombinedConsumer(AsyncWebsocketConsumer):
    active_tasks = {}
    flag_instance = FlagModel()
    feedback_group = "feedback_group"
    TASK_RESPONSE = {
        'SUCCEEDED': 1,
        'CANCELED': 2,
        'FAILED': 3
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stored_serialized_data = None

    async def connect(self):
        await self.accept()
        await self.channel_layer.group_add(self.feedback_group, self.channel_name)

    async def disconnect(self, close_code):
        await self.channel_layer.group_discard(self.feedback_group, self.channel_name)

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
            self.feedback_group,
            {
                'type': 'send_feedback',
                'Action status': feedback_data
            }
        )
        
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
            
    @sync_to_async
    def save_task_config(self, name, task):
        existing_task = TaskConfig.objects.filter(name=name).first()
        if existing_task:
            existing_task.task = task
            existing_task.save()
        else:
            TaskConfig.objects.create(
                name=name,
                task=task
            )
    
    async def send_feedback_data(self, event):
        feedback_data = event['Action status']
        await self.send(feedback_data)
        print(f"Received feedback data: {feedback_data}")
        
        
    async def send_feedback_response(self, serialized_data):
        await self.send(text_data=serialized_data)


    async def send_feedback(self, feedback_data):
        await self.channel_layer.group_send(
            self.feedback_group,
            {
                'type': 'send_feedback_data',
                'Action status': feedback_data
            }
        )
        
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

            if name == "cancel":
                await self.set_cancel_flag()
                await self.send(text_data=json.dumps({'message': 'Cancel flag changed to true'}))
                
            elif name == "status":
                if self.stored_serialized_data:
                    await self.send(text_data=self.stored_serialized_data)
                else:
                    await self.send(text_data="No Action response found")


            elif task_config:
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

                                                # task_result = execute_navigation_task.apply_async(
                                                # args=(),
                                                # kwargs={'x': pos_x, 'y': pos_y, 'orientation_w': dir_value},
                                                # )
                                                
                                                # while not task_result.ready():
                                                #     # Check for cancellation
                                                #     if await database_sync_to_async(lambda: FlagModel.objects.get(pk=1).cancel_flag)():
                                                #         task_result.revoke(terminate=True)
                                                #         await self.send(text_data=json.dumps({'message': 'Task canceled.'}))
                                                #         break
                                                #     await asyncio.sleep(1)

                                                # Check the result using the get method
                                                result = task_result.get()

                                                if result['status'] == 'TaskResult.SUCCEEDED':
                                                    await self.process_instance(instance_name_to_check, dir_value, pos_x, pos_y)
                                                    await self.send(text_data=f"Task {task_config.name} completed: {instance_name_to_check}")
                                                    await sync_to_async(TaskResponseModels.objects.create)(
                                                        name=task_config.name,
                                                        status=1,
                                                    )
                                                elif result['status'] == 'TaskResult.CANCELED':
                                                    # Check the external cancel flag (e.g., in the database)
                                                    external_cancel_flag = await database_sync_to_async(
                                                        lambda: FlagModel.objects.get(pk=1).cancel_flag
                                                    )()

                                                    # If the external cancel flag is still True, update it to False
                                                    if external_cancel_flag:
                                                        await database_sync_to_async(
                                                            lambda: FlagModel.objects.filter(pk=1).update(cancel_flag=False)
                                                        )()
                                                        await self.send(f"Cancel flag changed to False")

                                                    # Process the instance and send appropriate messages
                                                    await self.process_instance(instance_name_to_check, dir_value, pos_x, pos_y)
                                                    await self.send(text_data=f"Task {task_config.name} was canceled: {instance_name_to_check}")
                                                    await sync_to_async(TaskResponseModels.objects.create)(
                                                        name=task_config.name,
                                                        status=2,
                                                    )

                                                elif result['status'] == 'TaskResult.FAILED':
                                                    await self.process_instance(instance_name_to_check, dir_value, pos_x, pos_y)
                                                    await self.send(text_data=f"Task {task_config.name} was failed: {instance_name_to_check}")
                                                    await sync_to_async(TaskResponseModels.objects.create)(
                                                        name=task_config.name,
                                                        status=3,
                                                    )
                                                    
                                                else:
                                                    await self.send(f"{instance_name_to_check} has been finished")
                                                    # await sync_to_async(TaskResponseModels.objects.create)(
                                                    #     name=task_config.name,
                                                    #     status=1,
                                                    # )

                                    else:
                                        await self.handle_error("Unable to check instanceName in live BitMapModels.")
                                        await sync_to_async(TaskResponseModels.objects.create)(
                                                        name=task_config.name,
                                                        status=3,
                                                    )

                                    # Break from the loop once a navigation_action is found in the current action group
                                    break

                            # Check if there are more tasks to process
                            if task_config.task.index(task) < len(task_config.task) - 1:
                                await self.send(text_data=f"Moving to the next task in {task_config.name}")

                            else:
                                await self.send(text_data=f"All tasks in {task_config.name} completed.")

                else:
                    await self.send(text_data="All done !!!")
                    
            else:
                    await self.send(text_data="No action found")
        except Exception as e:
            await self.handle_error(f"An error occurred: {str(e)}")
            