# taskmanager_app/models.py
from django.db import models
from django.core.exceptions import ValidationError

def validate_task_config_count():
    # Check if the total number of TaskConfig entries is less than or equal to 6
    if TaskConfig.objects.count() >= 1000:
        raise ValidationError('The maximum number of TaskConfig entries (1000) has been reached.')

class TaskConfig(models.Model):
    name = models.CharField(max_length=50, unique=True)
    task = models.JSONField()
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)
    created_user = models.CharField(max_length=50, default='admin')
    updated_user = models.CharField(max_length=50, default='admin')


    def clean(self):
        validate_task_config_count()

    def save(self, *args, **kwargs):
        self.clean()
        super().save(*args, **kwargs)

    def __str__(self):
        return f'{self.name} - {self.id}'


class FlagModel(models.Model):
    queue_flag = models.BooleanField(default=False)
    cancel_flag = models.BooleanField(default = False)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)
    created_user = models.CharField(max_length=50, default='admin')
    updated_user = models.CharField(max_length=50, default='admin')
    
    
class BitMapModels(models.Model):
    is_live = models.BooleanField(null = True, default=False)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)
    created_user = models.CharField(max_length=50, default='admin')
    updated_user = models.CharField(max_length=50, default='admin')
    map = models.JSONField()
    
    def save(self, *args, **kwargs):
        # Check if there is another instance with is_live set to True
        if self.is_live:
            BitMapModels.objects.filter(is_live=True).exclude(pk=self.pk).update(is_live=False)
        super(BitMapModels, self).save(*args, **kwargs)
        
class MapBackupModels(models.Model):
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)
    created_user = models.CharField(max_length=50, default='admin')
    updated_user = models.CharField(max_length=50, default='admin')
    map_backup = models.JSONField()
    

class TaskResponseModels(models.Model):
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)
    created_user = models.CharField(max_length=50, default='admin')
    updated_user = models.CharField(max_length=50, default='admin')
    name  = models.CharField(max_length=50)
    status = models.IntegerField()
    canceled_by = models.CharField(max_length=50, default='user/server')