import json
from channels.generic.websocket import AsyncWebsocketConsumer
from celery.result import AsyncResult
from taskmanager_app.task_chain.task2 import dummy_task
from taskmanager_app.task_chain.task1 import operands
from taskmanager_app.task_chain.task4 import fibonacci
from queue import Queue
import asyncio


TASK_STATUS_CODES = {
    'StatusNone': 0,
    'Waiting': 1,
    'Running': 2,
    'Completed': 3,
    'Failed': 4,
    'Canceled':5
}

active_tasks = {}  # Dictionary to store active task IDs and corresponding Celery task objects
with open('taskmanager_app/task_chain/task_configs/task3_config.json', 'r') as config_file:
    config_data = json.load(config_file)

operand1 = config_data.get('task3', {}).get('operands', {}).get('operand1', 0)
operand2 = config_data.get('task3', {}).get('operands', {}).get('operand2', 0)
order = config_data.get('task3', {}).get('order', {}).get('order', 5)



active_tasks = {}  # Dictionary to store active task IDs and corresponding Celery task objects
task_queue = Queue()  # Queue to hold tasks to be executed in order

class RunTaskConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.accept()

    async def disconnect(self, close_code):
        # Clean up tasks when the WebSocket connection is closed
        for task_id, celery_task in active_tasks.items():
            celery_task.revoke(terminate=True)
        active_tasks.clear()

    async def receive(self, text_data):
        data = json.loads(text_data)

        task_id = data.get('task_id')
        task_name = data.get('task_name')
        json_data = data.get('json_data')

        # first_task = dummy_task.delay(task_id)
        # operand = operands.delay(operand1, operand2)
        # first_task_result = AsyncResult(first_task.id)
        # operand_result = AsyncResult(operand.id)

        # if task_name == 'task3':
        #     if task_id in active_tasks:
        #         # Task with the same ID is already active, delay execution
        #         task_queue.put((task_id, task_name))
        #         return
        #     elif first_task:
        #         active_tasks[task_id] = first_task
        #         if first_task_result.successful():
        #             # Task succeeded
        #             operand = operands.delay(task_id)
        #             active_tasks[task_id] = operand
        #     else:
        #         await self.send(text_data=json.dumps({"No task found to be executed"}))
        
        #fibonacci action server
        if task_name == 'fibonacci':
            # Assume json_data contains the 'order' parameter
        
            # Execute the Celery task asynchronously with the provided order
            fibonacci.delay(order)
    
            # Send a response to the client
            await self.send(text_data=json.dumps({'status': 'Task submitted successfully'}))


        else:
            await self.send(text_data=json.dumps({'status': f'Task {task_name} not found'}))
            return

        # if self.scope.get('path') == '/3001/':
        #     await self.send_task_list_status(task_name, task_id, first_task_result.status)

    async def send_task_list_status(self, task_name, task_id, status_code):
        response = {
            'taskListName': task_name,
            'taskListStatus': status_code,  # Fix here, use status_code directly
            'actionids': task_id
        }
        await self.send(text_data=json.dumps(response))

        
class JsonLoadConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.accept()

    async def disconnect(self, close_code):
        # Clean up if needed
        pass

    async def receive(self, text_data):
        try:
            json_data = json.loads(text_data)

            with open('taskmanager_app/task_chain/task_configs/task3_config.json', 'w') as config_file:
                json.dump(json_data, config_file, indent=2)

            # Send a response to the client
            await self.send(text_data=json.dumps({'status': 'Config updated successfully'}))
        except json.JSONDecodeError:
            # Handle JSON decoding error
            await self.send(text_data=json.dumps({'error': 'Invalid JSON format'}))
        except Exception as e:
            # Handle other exceptions
            await self.send(text_data=json.dumps({'error': f'An error occurred: {str(e)}'}))
            


