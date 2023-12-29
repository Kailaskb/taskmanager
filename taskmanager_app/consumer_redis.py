import json
from channels.generic.websocket import AsyncWebsocketConsumer
import redis

redis_client = redis.StrictRedis(host='localhost', port=6379, db=0)

class RedisDataConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.accept()

    async def disconnect(self, close_code):
        pass

    async def receive(self, text_data):
        try:
            # Assuming text_data is a JSON string, parse it
            data = json.loads(text_data)

            # Extract the key from the client's request
            key_to_retrieve = data.get('key')
            
            if key_to_retrieve:
                # Construct the Redis key based on the received key
                redis_key = key_to_retrieve

                # Retrieve the latest data from Redis
                latest_data = redis_client.get(redis_key)

                # If data is found, send it to the client
                if latest_data:
                    latest_data_dict = json.loads(latest_data)
                    await self.send(text_data=json.dumps(latest_data_dict))
                else:
                    await self.send(text_data=json.dumps({'error': f'Data not found for key: {redis_key}'}))
            else:
                await self.send(text_data=json.dumps({'error': 'Key not provided in the request'}))

        except json.JSONDecodeError:
            # Handle JSON decoding error
            await self.send(text_data=json.dumps({'error': 'Invalid JSON format'}))
        except Exception as e:
            # Handle other exceptions
            await self.send(text_data=json.dumps({'error': f'An error occurred: {str(e)}'}))
