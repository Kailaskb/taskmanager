from django.shortcuts import render
from rest_framework.views import APIView
from .serializers import TaskConfigSerializer
from .models import TaskConfig
from rest_framework.response import Response
from taskmanager_project.utils import fail, success
from rest_framework import status


# Create your views here.


class Taskfile(APIView):
    
    def post(self, request):
        try:
            serializer = TaskConfigSerializer(data=request.data)
            if not serializer.is_valid():
                return Response(fail(serializer.errors), status=400)
            name = serializer.validated_data['name']
            task = serializer.validated_data['task']
            obj = TaskConfig.objects.create(
                name=name,
                task=task
            )    
            data = TaskConfigSerializer(obj, many=False)
            return Response(success(data.data), status=201)
        except Exception as e:
            return Response(fail(str(e)), status=500)
    
    
    def get(self, request):
        try:
            obj = TaskConfig.objects.all()
            data = TaskConfigSerializer(obj, many=True)
            return Response(success(data.data), status=200)
        except Exception as e:
            return Response(fail(str(e)), status=500)
        
    def put(self, request, id):
        try:
            obj = TaskConfig.objects.filter(id=id)
            serializer = TaskConfigSerializer(data= request.data)
            if not serializer.is_valid():
                return Response(fail(serializer.errors), status=400)
            if not obj.exists():
                return Response(fail('obj does not exits'), status=400)
            obj.update(
                name=serializer.validated_data['name'],
                task=serializer.validated_data['task'],
            )
            data = TaskConfig(obj.first(), many=False)
            return Response(success(data.data), status=200)
        except Exception as e:
            return Response(fail(str(e)), status=500)
        
    def delete(self, request, id):
        try:
            obj = TaskConfig.objects.filter(id=id)
            if obj.exists():
                obj.delete()
                return Response(success(), status=204)
            return Response(fail('Task file does not exists'), status=400)
        except Exception as e:
            return Response(fail(str(e)), status=500)