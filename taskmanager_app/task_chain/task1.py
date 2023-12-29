# taskmanager_project/taskmanager_app/task_chain/task1.py
import json
from celery import shared_task
from time import sleep

@shared_task
def operands(operand1, operand2):
    # Read task configuration from JSON file
    # with open('task_configs/task1_config.json') as config_file:
    #     config = json.load(config_file)

    # sleep(10)
    # Simulate a time-consuming operation (e.g., addition) with sleep

    result = operand1 + operand2
    # Return the result as a JSON object
    # result

    return result
