# taskmanager_project/taskmanager_project/asgi.py

import os
from django.core.asgi import get_asgi_application
from channels.routing import ProtocolTypeRouter, URLRouter
from channels.auth import AuthMiddlewareStack
import taskmanager_app.routing  # import your routing configuration
os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'taskmanager_project.settings')

application = ProtocolTypeRouter({
    "http": get_asgi_application(),
    "websocket": AuthMiddlewareStack(
        URLRouter(
            taskmanager_app.routing.websocket_urlpatterns
        )
    ),
})
