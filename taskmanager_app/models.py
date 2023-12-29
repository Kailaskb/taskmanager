# taskmanager_app/models.py
from django.db import models

class TaskConfig(models.Model):
    task_name = models.CharField(max_length=255)
    task_id = models.CharField(max_length=255)
    json_data = models.JSONField()

    def __str__(self):
        return f'{self.task_name} - {self.task_id}'
