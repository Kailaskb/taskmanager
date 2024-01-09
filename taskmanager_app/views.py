from django.shortcuts import get_object_or_404
from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework import status
from .serializers import TaskConfigSerializer, BitMapSerializer
from .models import TaskConfig, BitMapModels
from taskmanager_project.utils import fail, success
import json
import os

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


class Mapview(APIView):
    
    MAP_DATA_FILE = "mapdata.json"  # File name to store the data locally

    def post(self, request):
        try:
            serializer = BitMapSerializer(data=request.data)
            if serializer.is_valid():
                obj = serializer.save()

                # Ensure only one instance is live
                if obj.is_live:
                    BitMapModels.objects.filter(is_live=True).exclude(pk=obj.pk).update(is_live=False)

                data = BitMapSerializer(obj).data
                return Response(success(data), status=status.HTTP_201_CREATED)
            return Response(fail(serializer.errors), status=status.HTTP_400_BAD_REQUEST)
        except Exception as e:
            return Response(fail(str(e)), status=status.HTTP_500_INTERNAL_SERVER_ERROR)
    
    def get(self, request):
        try:
            # Retrieve all instances
            all_objects = BitMapModels.objects.all()
            
            # Filter instances with is_live=True
            live_objects = BitMapModels.objects.filter(is_live=True)
            
            # Extract the 'map' attribute from each live instance
            data_live = [obj.map for obj in live_objects]

            # Serialize all instances
            data_all = BitMapSerializer(all_objects, many=True).data

            # Save or update the data to a local JSON file
            self.save_or_update_local_data(data_live)

            return Response(success(data_all), status=status.HTTP_200_OK)
        except Exception as e:
            return Response(fail(str(e)), status=status.HTTP_500_INTERNAL_SERVER_ERROR)

    
    def save_or_update_local_data(self, data):
        temp_file_path = self.MAP_DATA_FILE + '.temp'

        try:
            # Ensure there is at least one item in the data list
            if data:
                # Select the first item from the data list
                selected_data = data[0]

                with open(temp_file_path, 'w') as temp_file:
                    # Use json.dump to save the selected item as a JSON object
                    json.dump(selected_data, temp_file, indent=None)
                
                # Atomically replace the existing file with the temporary file
                os.replace(temp_file_path, self.MAP_DATA_FILE)
            else:
                print("Warning: Data list is empty. No data saved to the local file.")
        except (TypeError, ValueError) as e:
            # Handle JSON serialization errors
            print(f"Error serializing data to JSON: {str(e)}")
        except Exception as e:
            # Handle other exceptions (e.g., log an error) based on your application's needs
            print(f"Error saving data to local file: {str(e)}")
        finally:
            # Clean up the temporary file if it exists
            if os.path.exists(temp_file_path):
                os.remove(temp_file_path)

                
    def put(self, request,id):
        try:
            # Assuming you are updating an existing instance based on the provided ID
            instance = get_object_or_404(BitMapModels, id=id)

            # Update only the is_live field
            instance.is_live = request.data.get('is_live', instance.is_live)
            instance.save()

            # Ensure only one instance is live
            if instance.is_live:
                BitMapModels.objects.filter(is_live=True).exclude(pk=instance.pk).update(is_live=False)

            data = BitMapSerializer(instance).data
            return Response(success(data), status=status.HTTP_200_OK)
        except BitMapModels.DoesNotExist:
            return Response(fail("Instance not found"), status=status.HTTP_404_NOT_FOUND)
        except Exception as e:
            return Response(fail(str(e)), status=status.HTTP_500_INTERNAL_SERVER_ERROR)


    def delete(self, request, id):
        try:
            instance = get_object_or_404(BitMapModels, id=id)
            instance.delete()
            return Response(success("Instance deleted successfully"), status=status.HTTP_204_NO_CONTENT)
        except BitMapModels.DoesNotExist:
            return Response(fail("Instance not found"), status=status.HTTP_404_NOT_FOUND)
        except Exception as e:
            return Response(fail(str(e)), status=status.HTTP_500_INTERNAL_SERVER_ERROR)
