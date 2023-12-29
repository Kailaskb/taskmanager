# Generated by Django 3.2.23 on 2023-12-21 11:01

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('taskmanager_app', '0001_initial'),
    ]

    operations = [
        migrations.CreateModel(
            name='TaskConfig',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('task_name', models.CharField(max_length=255)),
                ('json_data', models.JSONField()),
            ],
        ),
        migrations.DeleteModel(
            name='Task',
        ),
    ]
