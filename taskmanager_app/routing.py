# taskmanager_project/taskmanager_app/routing.py
from django.urls import re_path
# from taskmanager_app.consumers import TaskManagerConsumer, CancelTaskConsumer
# from taskmanager_app.consumer1 import RunTaskConsumer, JsonLoadConsumer
from taskmanager_app.consumers2 import FibonacciConsumer, JsonLoadConsumer, CancelTaskConsumer, OperandsTaskConsumer
from taskmanager_app.consumer_redis import RedisDataConsumer



# websocket_urlpatterns = [
#     re_path(r'3000/$', TaskManagerConsumer.as_asgi()),
#     re_path(r'3001/$', TaskManagerConsumer.as_asgi()),
#     re_path(r'3002/$', CancelTaskConsumer.as_asgi()),
# ]

websocket_urlpatterns = [
    # re_path(r'3000/$', FibonacciConsumer.as_asgi()),
    re_path(r'3001/$', FibonacciConsumer.as_asgi()),
    re_path(r'3002/$', CancelTaskConsumer.as_asgi()),
    re_path(r'3003/$', JsonLoadConsumer.as_asgi()),
    re_path(r'3004/$', OperandsTaskConsumer.as_asgi()),
    re_path(r'3005/$', RedisDataConsumer.as_asgi()),
    
]
