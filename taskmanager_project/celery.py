# taskmanager_project/celery.py
from __future__ import absolute_import, unicode_literals
import os
from celery import Celery

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'taskmanager_project.settings')

app = Celery('taskmanager_project')
app.config_from_object('django.conf:settings', namespace='CELERY')
app.conf.broker_url = 'redis://localhost:6379/0'  # Update with your broker URL
app.conf.result_backend = 'redis://localhost:6379/0'


# Import and register your tasks
# from taskmanager_app.task_chain.task1 import operands
# from taskmanager_app.task_chain.task2 import dummy_task
# from taskmanager_app.task_chain.task3 import execute_navigation_task
# from taskmanager_app.task_chain.task4 import fibonacci

app.autodiscover_tasks()
