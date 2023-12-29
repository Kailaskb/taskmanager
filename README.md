# django_taskmanager
### After installing the required files
> to install required files
```
pip install -r requirements.txt
```
### Run these codes
> Create a virtual environment
```
python3 -m venv virtual_environment_name
```
> Activate your virtual environment
```
source virtual_environment_name/bin/activate
```
> Run
```
python manage.py makemigrations
python manage.py migrate
```
> To run python server
```
python manage.py runserver
```
> To run celery
```
celery -A taskmanager_project worker --loglevel=info
```
## Websocket urls
#### Fibonacci task
> Client request format
```
{"task_name": "fibonacci"}
```
>Url
```
ws://127.0.0.1:8000/3001/
```
#### Cancel Fibonacci
> Client request format
```
{"action": "cancel_fibonacci_task"}
```
> Url
```
ws://127.0.0.1:8000/3002/
```
#### Update/Create json file
> Client request format ( example json )
```
{
    "task_name": "task3_config",
    "task_id": 1,
    "json_data":{
      "operand1": 10,
      "operand2": 30
    }
```
> Url
```
ws://127.0.0.1:8000/3003/
```
#### Operands task (add)
> Client request format
```
{"task_name": "task3_config"}
```
> Url
```
ws://127.0.0.1:8000/3004/
```
#### Get result response from redis (Operands)
> Client request format
```
{
  "key": "redis_key"
}
```
> Url
```
ws://127.0.0.1:8000/3005/
``` 
### Check result data stored on redis (Operands)
```
redis-cli
```
> Get updated result response 
```
Get redis_key
```
#### Mysql commands
> Run mysql in a terminal and enter your password
```
mysql -u root -p
```
> Use your database
```
USE db_name(taskmanager_db);
```
> View tables
```
USE TABLES;
```
> Select table
```
SELECT * FROM your_table_name (taskmanager_app_taskconfig);
```
