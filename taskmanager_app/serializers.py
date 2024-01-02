from rest_framework import serializers
from .models import TaskConfig

class TaskConfigSerializer(serializers.ModelSerializer):
    class Meta:
        model = TaskConfig
        fields = ['name', 'task']

    def update(self, instance, validated_data):
        instance.name = validated_data.get('name', instance.name)
        instance.task = validated_data.get('task', instance.task)
        instance.save()
        return instance
