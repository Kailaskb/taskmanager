import asyncio
import websockets
import json

async def connect_to_websocket(uri):
    async with websockets.connect(uri) as websocket:
        while True:
            task_name = input("Enter task name (or 'exit' to disconnect): ")

            if task_name.lower() == 'exit':
                break

            await websocket.send(task_name)

            response = await websocket.recv()
            print(f"Received response: {response}")

            # Check status
            await websocket.send("status")
            status_response = await websocket.recv()
            print(f"Received status response: {status_response}")

if __name__ == "__main__":
    server_uri = "ws://127.0.0.1:8000/3010/"
    asyncio.get_event_loop().run_until_complete(connect_to_websocket(server_uri))
