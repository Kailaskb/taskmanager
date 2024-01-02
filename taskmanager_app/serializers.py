from rest_framework import serializers
from .models import TaskConfig




class TaskConfigSerializer(serializers.Serializer):
    name = serializers.CharField()
    task = serializers.JSONField()