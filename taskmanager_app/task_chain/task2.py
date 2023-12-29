# taskmanager_project/taskmanager_app/task_chain/task2.py


from celery import shared_task
from time import sleep

@shared_task
def dummy_task(task_id):
    # Task logic for task2
    sleep(10)
    print(f'Task 2 completed for task ID: {task_id}')
    result = "Task completed"
    result_json = {
        'result': result
    }

    return result_json