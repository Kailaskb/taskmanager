from rest_framework import serializers
from .models import TaskConfig, BitMapModels

class TaskConfigSerializer(serializers.ModelSerializer):
    class Meta:
        model = TaskConfig
        fields = ['name', 'task']

    def update(self, instance, validated_data):
        instance.name = validated_data.get('name', instance.name)
        instance.task = validated_data.get('task', instance.task)
        instance.save()
        return instance

class BitMapSerializer(serializers.ModelSerializer):
    class Meta:
        model = BitMapModels
        fields = '__all__'