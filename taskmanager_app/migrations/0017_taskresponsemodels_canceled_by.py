# Generated by Django 3.2.23 on 2024-01-18 09:36

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('taskmanager_app', '0016_mapbackupmodels'),
    ]

    operations = [
        migrations.AddField(
            model_name='taskresponsemodels',
            name='canceled_by',
            field=models.CharField(default='user/server', max_length=50),
        ),
    ]
