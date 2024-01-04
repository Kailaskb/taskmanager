# Django Taskmanager 

## After installing the required files
> to install required files
```
pip install -r requirements.txt
```

## Run these codes
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
> To run the Python server
```
python manage.py runserver
```
> To run celery
```
celery -A taskmanager_project worker --loglevel=info
```

## Websocket URLs
#### Fibonacci task
> Client request format
```json
{"task_name": "fibonacci"}
```
>Url
```
ws://127.0.0.1:8000/3001/
```
#### Cancel Fibonacci
> Client request format
```json
{"action": "cancel_fibonacci_task"}
```
> Url
```
ws://127.0.0.1:8000/3002/
```
#### Update/Create a JSON file
> Client request format ( example JSON )
```json
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
```json
{"task_name": "task3_config"}
```
> Url
```
ws://127.0.0.1:8000/3004/
```
#### Operand task response
> Client request format
```json
{
  "key": "redis_key"
}
```
> Url
```
ws://127.0.0.1:8000/3005/
```
#### Set queue flag 1/0
> Client request format
```json
{
  "queue_flag": true
}
```
> Url
```
ws://127.0.0.1:8000/3006/
```
#### Get result response from Redis (Operands)
> Client request format
```json
{
  "key": "redis_key"
}
```
> Url
```
ws://127.0.0.1:8000/3005/
``` 

## Check result data stored on Redis (Operands)
```
redis-cli
```
> Get updated result response 
```
Get redis_key
```

## Mysql commands
> Run MySQL in a terminal and enter your password
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

## Files and Definitions 
* celery.py – celery configurations 
* task_chain – contains Tasks/Actions (tasks.py, …)
* task4.py – fibonacci task 
* task3.py –  navigation task
* client.py – Basicnavigator Action client
* client2.py – Fibonacci Action client
* consumer_redis.py – Consumer for response stored in redis
* consumer1.py & consumers.py – reference file (dummy)
* consumer2.py – consumer for current routs
* models.py – TaskConfig & QueueFlagModel – Model to store data in MySQL bd
* routing.py – Websocket URLs

## Work Flow

> Route /3001/  - Calls FibonacciConsumer  in consumer2.py 
* Checks if the task_name = fibonacci (received from the client)
* fibonacci.delay(order) trigger the task/action fibonacci in task4.py 

> Route /3002/- Calls CancelTaskConsumer in consumer2.py 
* Triggers the set_cancel_task_flag in task4.py 
* Set the cancel_task flag to True

> Route /3003/- Calls JsonLoadConsumer in consumer2.py 
* Receives json data from client
* Checks if task_name is existing or not in save_task_config 
* save_task_config checks if the task_name exists in TaskConfig models in models.py
* If task_name exists it will overwrite the json else a new file with the given task name will be saved in the models

> Route /3004/ - Calls OperandsTaskConsumer in consumer2.py
* Checks if the task_name = “task_name” (received from the client) is existing in the TaskConfig models and runs the Operand task with that json file
* get_task_config filters the task_name from Task_Config models
* get_queue_flag_status gets the value of queue_flag from QueueFlagModel in models.py
* If queue_flag is True runs the task with queue else without queue
* operands.delay(operand1, operand2) triggers the operands task in task1.py
* operand1 & operand2 gets value from the TaskConfig models with the received task_name in its table
* redis_client.set(‘redis_key’, response_data_string) saves the task response to redis db with the key “redis_key”
* redis_client connects to the host, port and db where data is to be stored

> Route /3005/ - Calls RedisDataConsumer in consumer_redis.py
* Checks if the key exists (received from the client) & sends the last response received in redis with key (eg: “redis_key”)
* redis_client connects to the redis host, port and bd  where data is stored

> Route /3006/ - Calls SetQueueFlagConsumer in consumer2.py
* Receives queue_flag as true / false (Received from client)
* And updates the queue_flag value in the QueueFlagModel in models.py
* Set the queue_flag value to true/false to turn off / on the queue_flag in OperandsTaskConsumer

## Crud (json file upload)
### Create
#### URL
```
[Post] http://127.0.0.1:8000/task/json
```
#### Request payload
```json

{
    "name": "task_1",
    "task": [
      {
        "actionGroups": [
          {
            "actionName": "move_action",
            "pluginName": "MoveFactory",
            "params": [
              {
                "key": "skill_name",
                "stringValue": "GotoSpecifiedPose"
              },
              {
                "key": "target_name",
                "stringValue": "LM2"
              }
            ],
            "ignoreReturn": false,
            "overtime": 0,
            "externalOverId": -1,
            "needResult": false,
            "sleepTime": 0,
            "actionId": 0
          },
          {
            "actionName": "move_action",
            "pluginName": "MoveFactory",
            "params": [
              {
                "key": "skill_name",
                "stringValue": "GotoSpecifiedPose"
              },
              {
                "key": "target_name",
                "stringValue": "LM2"
              }
            ],
            "ignoreReturn": false,
            "overtime": 0,
            "externalOverId": -1,
            "needResult": false,
            "sleepTime": 0,
            "actionId": 0
          }
        ],
        "actionGroupName": "group 1",
        "actionGroupId": 0,
        "loop": true,
        "condition": "Infinite"
      },
      {
        "actionGroups": [
          {
            "actionName": "move_action",
            "pluginName": "MoveFactory",
            "params": [
              {
                "key": "skill_name",
                "stringValue": "Translate"
              },
              {
                "key": "move_dist",
                "doubleValue": 1
              },
              {
                "key": "speed_x",
                "doubleValue": 1
              },
              {
                "key": "speed_y",
                "doubleValue": 10
              }
            ],
            "ignoreReturn": false,
            "overtime": 0,
            "externalOverId": -1,
            "needResult": false,
            "sleepTime": 0,
            "actionId": 0
          }
        ],
        "actionGroupName": "group 2",
        "actionGroupId": 1,
        "loop": false,
        "condition": "Infinite"
      },
      {
        "actionGroups": [
          {
            "actionName": "move_action",
            "pluginName": "MoveFactory",
            "params": [
              {
                "key": "skill_name",
                "stringValue": "Translate"
              },
              {
                "key": "move_dist",
                "doubleValue": 1
              },
              {
                "key": "speed_x",
                "doubleValue": 1
              },
              {
                "key": "speed_y",
                "doubleValue": 10
              }
            ],
            "ignoreReturn": false,
            "overtime": 0,
            "externalOverId": -1,
            "needResult": false,
            "sleepTime": 0,
            "actionId": 0
          }
        ],
        "actionGroupName": "group 2",
        "actionGroupId": 1,
        "loop": false,
        "condition": "Infinite"
      },
      {
        "actionGroups": [
          {
            "actionName": "move_action",
            "pluginName": "MoveFactory",
            "params": [
              {
                "key": "skill_name",
                "stringValue": "Translate"
              },
              {
                "key": "move_dist",
                "doubleValue": 1
              },
              {
                "key": "speed_x",
                "doubleValue": 1
              },
              {
                "key": "speed_y",
                "doubleValue": 10
              }
            ],
            "ignoreReturn": false,
            "overtime": 0,
            "externalOverId": -1,
            "needResult": false,
            "sleepTime": 0,
            "actionId": 0
          }
        ],
        "actionGroupName": "group 2",
        "actionGroupId": 1,
        "loop": false,
        "condition": "Infinite"
      }
    ]
  }
```
#### Response payload
```json
{
    "status": true,
    "message": "success",
    "data": {
        "name": "task_1",
        "task": [
            {
                "actionGroups": [
                    {
                        "actionName": "move_action",
                        "pluginName": "MoveFactory",
                        "params": [
                            {
                                "key": "skill_name",
                                "stringValue": "GotoSpecifiedPose"
                            },
                            {
                                "key": "target_name",
                                "stringValue": "LM2"
                            }
                        ],
                        "ignoreReturn": false,
                        "overtime": 0,
                        "externalOverId": -1,
                        "needResult": false,
                        "sleepTime": 0,
                        "actionId": 0
                    },
                    {
                        "actionName": "move_action",
                        "pluginName": "MoveFactory",
                        "params": [
                            {
                                "key": "skill_name",
                                "stringValue": "GotoSpecifiedPose"
                            },
                            {
                                "key": "target_name",
                                "stringValue": "LM2"
                            }
                        ],
                        "ignoreReturn": false,
                        "overtime": 0,
                        "externalOverId": -1,
                        "needResult": false,
                        "sleepTime": 0,
                        "actionId": 0
                    }
                ],
                "actionGroupName": "group 1",
                "actionGroupId": 0,
                "loop": true,
                "condition": "Infinite"
            },
            {
                "actionGroups": [
                    {
                        "actionName": "move_action",
                        "pluginName": "MoveFactory",
                        "params": [
                            {
                                "key": "skill_name",
                                "stringValue": "Translate"
                            },
                            {
                                "key": "move_dist",
                                "doubleValue": 1
                            },
                            {
                                "key": "speed_x",
                                "doubleValue": 1
                            },
                            {
                                "key": "speed_y",
                                "doubleValue": 10
                            }
                        ],
                        "ignoreReturn": false,
                        "overtime": 0,
                        "externalOverId": -1,
                        "needResult": false,
                        "sleepTime": 0,
                        "actionId": 0
                    }
                ],
                "actionGroupName": "group 2",
                "actionGroupId": 1,
                "loop": false,
                "condition": "Infinite"
            },
            {
                "actionGroups": [
                    {
                        "actionName": "move_action",
                        "pluginName": "MoveFactory",
                        "params": [
                            {
                                "key": "skill_name",
                                "stringValue": "Translate"
                            },
                            {
                                "key": "move_dist",
                                "doubleValue": 1
                            },
                            {
                                "key": "speed_x",
                                "doubleValue": 1
                            },
                            {
                                "key": "speed_y",
                                "doubleValue": 10
                            }
                        ],
                        "ignoreReturn": false,
                        "overtime": 0,
                        "externalOverId": -1,
                        "needResult": false,
                        "sleepTime": 0,
                        "actionId": 0
                    }
                ],
                "actionGroupName": "group 2",
                "actionGroupId": 1,
                "loop": false,
                "condition": "Infinite"
            },
            {
                "actionGroups": [
                    {
                        "actionName": "move_action",
                        "pluginName": "MoveFactory",
                        "params": [
                            {
                                "key": "skill_name",
                                "stringValue": "Translate"
                            },
                            {
                                "key": "move_dist",
                                "doubleValue": 1
                            },
                            {
                                "key": "speed_x",
                                "doubleValue": 1
                            },
                            {
                                "key": "speed_y",
                                "doubleValue": 10
                            }
                        ],
                        "ignoreReturn": false,
                        "overtime": 0,
                        "externalOverId": -1,
                        "needResult": false,
                        "sleepTime": 0,
                        "actionId": 0
                    }
                ],
                "actionGroupName": "group 2",
                "actionGroupId": 1,
                "loop": false,
                "condition": "Infinite"
            }
        ]
    }
}
```
### List
#### URL
```
[Get] http://127.0.0.1:8000/task/json
```
#### Response payload
```json
{
    "status": true,
    "message": "success",
    "data": {
        "name": "task_1",
        "task": [
            {
                "actionGroups": [
                    {
                        "actionName": "move_action",
                        "pluginName": "MoveFactory",
                        "params": [
                            {
                                "key": "skill_name",
                                "stringValue": "GotoSpecifiedPose"
                            },
                            {
                                "key": "target_name",
                                "stringValue": "LM2"
                            }
                        ],
                        "ignoreReturn": false,
                        "overtime": 0,
                        "externalOverId": -1,
                        "needResult": false,
                        "sleepTime": 0,
                        "actionId": 0
                    },
                    {
                        "actionName": "move_action",
                        "pluginName": "MoveFactory",
                        "params": [
                            {
                                "key": "skill_name",
                                "stringValue": "GotoSpecifiedPose"
                            },
                            {
                                "key": "target_name",
                                "stringValue": "LM2"
                            }
                        ],
                        "ignoreReturn": false,
                        "overtime": 0,
                        "externalOverId": -1,
                        "needResult": false,
                        "sleepTime": 0,
                        "actionId": 0
                    }
                ],
                "actionGroupName": "group 1",
                "actionGroupId": 0,
                "loop": true,
                "condition": "Infinite"
            },
            {
                "actionGroups": [
                    {
                        "actionName": "move_action",
                        "pluginName": "MoveFactory",
                        "params": [
                            {
                                "key": "skill_name",
                                "stringValue": "Translate"
                            },
                            {
                                "key": "move_dist",
                                "doubleValue": 1
                            },
                            {
                                "key": "speed_x",
                                "doubleValue": 1
                            },
                            {
                                "key": "speed_y",
                                "doubleValue": 10
                            }
                        ],
                        "ignoreReturn": false,
                        "overtime": 0,
                        "externalOverId": -1,
                        "needResult": false,
                        "sleepTime": 0,
                        "actionId": 0
                    }
                ],
                "actionGroupName": "group 2",
                "actionGroupId": 1,
                "loop": false,
                "condition": "Infinite"
            },
            {
                "actionGroups": [
                    {
                        "actionName": "move_action",
                        "pluginName": "MoveFactory",
                        "params": [
                            {
                                "key": "skill_name",
                                "stringValue": "Translate"
                            },
                            {
                                "key": "move_dist",
                                "doubleValue": 1
                            },
                            {
                                "key": "speed_x",
                                "doubleValue": 1
                            },
                            {
                                "key": "speed_y",
                                "doubleValue": 10
                            }
                        ],
                        "ignoreReturn": false,
                        "overtime": 0,
                        "externalOverId": -1,
                        "needResult": false,
                        "sleepTime": 0,
                        "actionId": 0
                    }
                ],
                "actionGroupName": "group 2",
                "actionGroupId": 1,
                "loop": false,
                "condition": "Infinite"
            },
            {
                "actionGroups": [
                    {
                        "actionName": "move_action",
                        "pluginName": "MoveFactory",
                        "params": [
                            {
                                "key": "skill_name",
                                "stringValue": "Translate"
                            },
                            {
                                "key": "move_dist",
                                "doubleValue": 1
                            },
                            {
                                "key": "speed_x",
                                "doubleValue": 1
                            },
                            {
                                "key": "speed_y",
                                "doubleValue": 10
                            }
                        ],
                        "ignoreReturn": false,
                        "overtime": 0,
                        "externalOverId": -1,
                        "needResult": false,
                        "sleepTime": 0,
                        "actionId": 0
                    }
                ],
                "actionGroupName": "group 2",
                "actionGroupId": 1,
                "loop": false,
                "condition": "Infinite"
            }
        ]
    }
}
```
### Update
#### Url
```
[put] http://127.0.0.1:8000/task/json/id
```
#### Request payload
```json

{
    "name": "task_6",
    "task": [
      {
        "actionGroups": [
          {
            "actionName": "move_action",
            "pluginName": "MoveFactory",
            "params": [
              {
                "key": "skill_name",
                "stringValue": "GotoSpecifiedPose"
              },
              {
                "key": "target_name",
                "stringValue": "LM2"
              }
            ],
            "ignoreReturn": false,
            "overtime": 0,
            "externalOverId": -1,
            "needResult": false,
            "sleepTime": 0,
            "actionId": 0
          },
          {
            "actionName": "move_action",
            "pluginName": "MoveFactory",
            "params": [
              {
                "key": "skill_name",
                "stringValue": "GotoSpecifiedPose"
              },
              {
                "key": "target_name",
                "stringValue": "LM2"
              }
            ],
            "ignoreReturn": false,
            "overtime": 0,
            "externalOverId": -1,
            "needResult": false,
            "sleepTime": 0,
            "actionId": 0
          }
        ],
        "actionGroupName": "group 1",
        "actionGroupId": 0,
        "loop": true,
        "condition": "Infinite"
      },
      {
        "actionGroups": [
          {
            "actionName": "move_action",
            "pluginName": "MoveFactory",
            "params": [
              {
                "key": "skill_name",
                "stringValue": "Translate"
              },
              {
                "key": "move_dist",
                "doubleValue": 1
              },
              {
                "key": "speed_x",
                "doubleValue": 1
              },
              {
                "key": "speed_y",
                "doubleValue": 10
              }
            ],
            "ignoreReturn": false,
            "overtime": 0,
            "externalOverId": -1,
            "needResult": false,
            "sleepTime": 0,
            "actionId": 0
          }
        ],
        "actionGroupName": "group 2",
        "actionGroupId": 1,
        "loop": false,
        "condition": "Infinite"
      },
      {
        "actionGroups": [
          {
            "actionName": "move_action",
            "pluginName": "MoveFactory",
            "params": [
              {
                "key": "skill_name",
                "stringValue": "Translate"
              },
              {
                "key": "move_dist",
                "doubleValue": 1
              },
              {
                "key": "speed_x",
                "doubleValue": 1
              },
              {
                "key": "speed_y",
                "doubleValue": 10
              }
            ],
            "ignoreReturn": false,
            "overtime": 0,
            "externalOverId": -1,
            "needResult": false,
            "sleepTime": 0,
            "actionId": 0
          }
        ],
        "actionGroupName": "group 2",
        "actionGroupId": 1,
        "loop": false,
        "condition": "Infinite"
      },
      {
        "actionGroups": [
          {
            "actionName": "move_action",
            "pluginName": "MoveFactory",
            "params": [
              {
                "key": "skill_name",
                "stringValue": "Translate"
              },
              {
                "key": "move_dist",
                "doubleValue": 1
              },
              {
                "key": "speed_x",
                "doubleValue": 1
              },
              {
                "key": "speed_y",
                "doubleValue": 10
              }
            ],
            "ignoreReturn": false,
            "overtime": 0,
            "externalOverId": -1,
            "needResult": false,
            "sleepTime": 0,
            "actionId": 0
          }
        ],
        "actionGroupName": "group 2",
        "actionGroupId": 1,
        "loop": false,
        "condition": "Infinite"
      }
    ]
  }
```
### Delete
#### Url
```
[delete] http://127.0.0.1:8000/task/json/id
```


