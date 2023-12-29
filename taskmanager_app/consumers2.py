import json
import aioredis
import asyncio
from celery import Celery
from celery.result import AsyncResult
from channels.generic.websocket import AsyncWebsocketConsumer
from channels.db import database_sync_to_async
from taskmanager_app.task_chain.task4 import fibonacci  # Assuming your tasks module is named 'tasks'
from taskmanager_app.task_chain.task4 import set_cancel_task_flag
from taskmanager_app.task_chain.task1 import operands 
from taskmanager_app.models import TaskConfig, QueueFlagModel
import redis


class FibonacciConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.accept()

    async def disconnect(self, close_code):
        # Clean up tasks when the WebSocket connection is closed
        # Implement cleanup logic if needed
        pass

    async def receive(self, text_data):
        data = json.loads(text_data)
        task_name = data.get('task_name')

        if task_name == 'fibonacci':
            # You can extract additional parameters from 'data' if needed
            order = data.get('order', 10)  # Default order is set to 10 if not provided
        
            # Execute the Celery task asynchronously with the provided order
            result = fibonacci.delay(order)

            task_result = AsyncResult(result.id)
            feedback_response = None
            
            while not task_result.ready():
                await asyncio.sleep(1)  # Add a small delay to avoid busy-waiting
            if task_result.successful():
                feedback_response = task_result.result.get('feedback_response')

            
            await self.send(text_data=json.dumps({'status': 'Fibonacci task completed', 'feedback_response': feedback_response}))
        
        else:
            await self.send(text_data=json.dumps({'status': f'Task {task_name} not found'}))
        
class JsonLoadConsumer(AsyncWebsocketConsumer):
    @database_sync_to_async
    def save_task_config(self, task_name, task_id, json_data):
        existing_task = TaskConfig.objects.filter(task_name=task_name).first()
        if existing_task:
            existing_task.json_data = json_data
            existing_task.save()
        else:
            TaskConfig.objects.create(
                task_name=task_name,
                task_id=task_id,
                json_data=json_data
            )
    
    
    async def connect(self):
        await self.accept()

    async def disconnect(self, close_code):
        # Clean up if needed
        pass

    async def receive(self, text_data):
        try:
            json_data = json.loads(text_data)

            task_name = json_data.get('task_name', '')
            task_id = json_data.get('task_id', '')
            json_data_value = json_data.get('json_data', {})

            await self.save_task_config(task_name, task_id, json_data_value)

            await self.send(text_data=json.dumps({'status': 'Config updated successfully'}))
        except json.JSONDecodeError:
            await self.send(text_data=json.dumps({'error': 'Invalid JSON format'}))
        except Exception as e:
            await self.send(text_data=json.dumps({'error': f'An error occurred: {str(e)}'}))
            
class CancelTaskConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.accept()

    async def disconnect(self, close_code):
        pass  # Clean up if needed

    async def receive(self, text_data):
        data = json.loads(text_data)
        action = data.get('action')

        if action == 'cancel_fibonacci_task':
            print('cancel called')
            set_cancel_task_flag(True)
            await self.send(text_data=json.dumps({'status': 'Fibonacci task cancellation requested'}))
        else:
            await self.send(text_data=json.dumps({'error': 'Invalid action'}))
            
            
redis_client = redis.StrictRedis(host='localhost', port=6379, db=0)
# task_queue_enabled = True  # Global switch to control the task queue state

class OperandsTaskConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.accept()

    async def disconnect(self, close_code):
        # Clean up tasks when the WebSocket connection is closed
        pass

    @database_sync_to_async
    def get_task_config(self, task_name):
        return TaskConfig.objects.filter(task_name=task_name).first()
    
    @database_sync_to_async
    def get_queue_flag_status(self):
        return QueueFlagModel.objects.first().queue_flag

    async def receive(self, text_data):
        try:
            # Assuming text_data is a JSON string, parse it
            data = json.loads(text_data)
            task_name = data.get('task_name')
            task_config = await self.get_task_config(task_name)
            
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

    async def run_task_with_queue(self, task_config):
        if task_config:
            operand1 = task_config.json_data.get('operand1', 0)
            operand2 = task_config.json_data.get('operand2', 0)

            task_result = operands.delay(operand1, operand2)
            result = task_result.get()

            response_data = {
                "action": "Task Status",
                "task_name": task_config.task_name,
                "task_status": result,
            }

            response_data_string = json.dumps(response_data)
            redis_client.set('redis_key', response_data_string)
            print("JSON data stored in Redis.")

            # await self.send(text_data=json.dumps(response_data))
        else:
            await self.send(text_data=json.dumps({'error': 'Task not found'}))


    async def run_task_without_queue(self, task_config):
        if task_config:
            operand1 = task_config.json_data.get('operand1', 0)
            operand2 = task_config.json_data.get('operand2', 0)

            # Run the task synchronously without queuing
            result = operands(operand1, operand2)

            response_data = {
                "action": "Task Status",
                "task_name": task_config.task_name,
                "task_status": result,
            }

            response_data_string = json.dumps(response_data)
            redis_client.set('redis_key', response_data_string)
            print("JSON data stored in Redis.")

            # await self.send(text_data=json.dumps(response_data))
        else:
            await self.send(text_data=json.dumps({'error': 'Task not found'}))
            
class SetQueueFlagConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.accept()

    async def disconnect(self, close_code):
        pass

    @database_sync_to_async
    def update_queue_flag(self, new_queue_flag_value):
        # Assuming there's only one instance, adjust the logic if needed
        queue_flag_model, created = QueueFlagModel.objects.get_or_create(pk=1)

        queue_flag_model.queue_flag = new_queue_flag_value
        queue_flag_model.save()

    async def receive(self, text_data):
        try:
            data = json.loads(text_data)
            new_queue_flag_value = data.get('queue_flag', None)

            if new_queue_flag_value is not None and isinstance(new_queue_flag_value, bool):
                await self.update_queue_flag(new_queue_flag_value)

                # Respond to the client indicating successful update
                await self.send(text_data=json.dumps({'message': 'Queue flag updated successfully'}))
            else:
                await self.send(text_data=json.dumps({'error': 'Invalid data. Missing or invalid queue_flag value'}))

        except json.JSONDecodeError:
            await self.send(text_data=json.dumps({'error': 'Invalid JSON format'}))

        except Exception as e:
            await self.send(text_data=json.dumps({'error': f'An error occurred: {str(e)}'}))
