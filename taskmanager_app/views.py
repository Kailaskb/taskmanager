from django.shortcuts import get_object_or_404
from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework import status
from .serializers import TaskConfigSerializer
from .models import TaskConfig
from taskmanager_project.utils import fail, success

class Taskfile(APIView):

    def post(self, request):
        try:
            serializer = TaskConfigSerializer(data=request.data)
            if serializer.is_valid():
                obj = serializer.save()
                data = TaskConfigSerializer(obj).data
                return Response(success(data), status=status.HTTP_201_CREATED)
            return Response(fail(serializer.errors), status=status.HTTP_400_BAD_REQUEST)
        except Exception as e:
            return Response(fail(str(e)), status=status.HTTP_500_INTERNAL_SERVER_ERROR)

    def get(self, request):
        try:
            objects = TaskConfig.objects.all()
            data = TaskConfigSerializer(objects, many=True).data
            return Response(success(data), status=status.HTTP_200_OK)
        except Exception as e:
            return Response(fail(str(e)), status=status.HTTP_500_INTERNAL_SERVER_ERROR)

    def put(self, request, id):
        try:
            obj = get_object_or_404(TaskConfig, id=id)
            serializer = TaskConfigSerializer(obj, data=request.data)
            if serializer.is_valid():
                serializer.save()
                data = TaskConfigSerializer(obj).data
                return Response(success(data), status=status.HTTP_200_OK)
            return Response(fail(serializer.errors), status=status.HTTP_400_BAD_REQUEST)
        except Exception as e:
            return Response(fail(str(e)), status=status.HTTP_500_INTERNAL_SERVER_ERROR)

    def delete(self, request, id):
        try:
            obj = get_object_or_404(TaskConfig, id=id)
            obj.delete()
            return Response(success(), status=status.HTTP_204_NO_CONTENT)
        except Exception as e:
            return Response(fail(str(e)), status=status.HTTP_500_INTERNAL_SERVER_ERROR)
