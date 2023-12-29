# taskmanager_project/taskmanager_app/consumers.py
import json
import asyncio
import rclpy
from channels.generic.websocket import AsyncWebsocketConsumer
from celery.result import AsyncResult
from taskmanager_app.task_chain.task3 import execute_navigation_task
from taskmanager_app import client

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

        # Send initial task status to the client only if connected to the specific WebSocket
        if self.scope.get('path') == '/3001/':
            await self.send_task_status(task_id, 'in_progress')

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
