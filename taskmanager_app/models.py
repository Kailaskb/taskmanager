# taskmanager_app/models.py
from django.db import models

class TaskConfig(models.Model):
    name = models.CharField(max_length=255)
    task = models.JSONField()

    def __str__(self):
        return f'{self.name} - {self.id}'


class QueueFlagModel(models.Model):
    queue_flag = models.BooleanField(default=False)
    
class BitMapModels(models.Model):
    is_live = models.BooleanField(null = True, default=False)
    map = models.JSONField()
    
    def save(self, *args, **kwargs):
        # Check if there is another instance with is_live set to True
        if self.is_live:
            BitMapModels.objects.filter(is_live=True).exclude(pk=self.pk).update(is_live=False)
        super(BitMapModels, self).save(*args, **kwargs)
