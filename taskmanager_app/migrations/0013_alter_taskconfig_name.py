# Generated by Django 3.2.23 on 2024-01-12 10:07

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('taskmanager_app', '0012_auto_20240111_1257'),
    ]

    operations = [
        migrations.AlterField(
            model_name='taskconfig',
            name='name',
            field=models.CharField(max_length=50, unique=True),
        ),
    ]
