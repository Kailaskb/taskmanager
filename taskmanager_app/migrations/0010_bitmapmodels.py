# Generated by Django 3.2.23 on 2024-01-08 09:27

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('taskmanager_app', '0009_rename_task_name_taskconfig_name'),
    ]

    operations = [
        migrations.CreateModel(
            name='BitMapModels',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('is_live', models.BooleanField(default=False, null=True)),
                ('map', models.JSONField()),
            ],
        ),
    ]
