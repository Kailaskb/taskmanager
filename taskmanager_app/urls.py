from django.urls import path
from .import views



urlpatterns = [
    path("task/json",views.Taskfile.as_view()),
    path('task/json/<int:id>', views.Taskfile.as_view()),
    path('map/json', views.Mapview.as_view()),
    path('map/json/<int:id>', views.Mapview.as_view())
]