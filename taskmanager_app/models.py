# taskmanager_app/models.py
from django.db import models

class TaskConfig(models.Model):
    task_name = models.CharField(max_length=255)
    task_id = models.CharField(max_length=255)
    task = models.JSONField()

    def __str__(self):
        return f'{self.task_name} - {self.task_id}'

class QueueFlagModel(models.Model):
    queue_flag = models.BooleanField(default=False)
    
    
    
    
#     {
#     "task_name": "task3_config",
#     "task_id": 1,
#     "json_data":{
#       "operand1": 10,
#       "operand2": 0
#     }
# }
