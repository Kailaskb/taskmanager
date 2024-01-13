from rest_framework import serializers
from .models import TaskConfig, BitMapModels, MapBackupModels

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
        
class MapBackupModelsSerializer(serializers.ModelSerializer):
    class Meta:
        model = MapBackupModels
        fields = ['created_at', 'updated_at', 'created_user', 'updated_user', 'map_backup']
