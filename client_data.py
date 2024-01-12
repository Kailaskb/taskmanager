async def receive(self, text_data):
        try:
            name = text_data
            task_config = await self.get_task_config(name)

            # Extract instance_name_to_check from task_config
            if task_config:
                for task in task_config.task:
                    for action_group in task.get("actionGroups", []):
                        if action_group.get("actionName") == "navigation_action":
                            for param in action_group["params"]:
                                if param.get("key") == "target_name" and "stringValue" in param:
                                    instance_name_to_check = param["stringValue"]
                                    await self.response_feedback(task_name=task_config.name, actionName=action_group.get("actionName"), actionGroupName=task.get("actionGroupName"), actionGroupId=task.get("actionGroupId"))

                                    # Get the live BitMapModels instance
                                    live_bitmap_instance = await self.get_live_bitmap_instance()

                                    # Check if the instance exists and has the necessary fields
                                    if live_bitmap_instance:
                                        advanced_point_list = live_bitmap_instance.map.get("advancedPointList", [])

                                        # Check if instance_name_to_check is present in advancedPointList.instanceName
                                        for point_data in advanced_point_list:
                                            if "instanceName" in point_data and point_data["instanceName"] == instance_name_to_check:
                                                dir_value = point_data.get("dir")
                                                pos_x = point_data.get("pos", {}).get("x")
                                                pos_y = point_data.get("pos", {}).get("y")

                                                # Execute navigation task and await the result
                                                task_result = execute_navigation_task.delay(
                                                    x=pos_x, y=pos_y, orientation_w=dir_value
                                                )

                                                # Check the result using the get method
                                                result = task_result.get()
                                                if result['status'] == 'completed':
                                                    await self.process_instance(instance_name_to_check, dir_value, pos_x, pos_y)
                                                    await self.send(text_data=f"Task {task_config.name} completed: {instance_name_to_check}")
                                                else:
                                                    await self.send(f"{instance_name_to_check} has been finished")

                                    else:
                                        await self.handle_error("Unable to check instanceName in live BitMapModels.")

                                    # Break from the loop once a navigation_action is found in the current action group
                                    break

                            # Check if there are more tasks to process
                            if task_config.task.index(task) < len(task_config.task) - 1:
                                await self.send(text_data=f"Moving to the next task in {task_config.name}")

                            else:
                                await self.send(text_data=f"All tasks in {task_config.name} completed.")

                else:
                    await self.send(text_data="No task found")

        except Exception as e:
            print(f"Error in receive method: {e}")
            await self.handle_error(str(e))
            
            
            
            
            await sync_to_async(TaskResponseModels.objects.create)(
                                                        name=task_config.name,
                                                        status=2,
                                                    )
            
            async def set_cancel_flag(self):
                await database_sync_to_async(self.update_cancel_flag)()

            def update_cancel_flag(self):
                # Update the cancel_flag field
                flag_instance.cancel_flag = True
                flag_instance.save()
                
            await self.response_feedback(task_name=task_config.name, actionName=action_group.get("actionName"), actionGroupName=task.get("actionGroupName"), actionGroupId=task.get("actionGroupId"))

            async def response_feedback(self, actionName, task_name, actionGroupName, actionGroupId):
                feedback_data = {
                    'Action name': actionName,
                    'Task name': task_name,
                    'Action group name': actionGroupName,
                    'Action group Id': actionGroupId
                }
                await self.channel_layer.group_send(
                    feedback_group,
                    {
                        'type': 'send_feedback',
                        'Action status': feedback_data
                    }
                )