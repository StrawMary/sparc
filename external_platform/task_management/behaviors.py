def get_go_to_behavior(task_manager, target):
	say1 = task_manager.create_task_say("Going to " + target)
	move1 = task_manager.create_task_go_to(target)
	if not move1:
		return task_manager.create_task_say("Sorry, I don't know the target")

	say2 = task_manager.create_task_say("Reached " + target)
	say3 = task_manager.create_task_say("Stopped")

	say1.success_child = move1

	move1.success_child = say2
	move1.fail_child = say3
	return say1


def get_say_behavior(task_manager, text):
	say1 = task_manager.create_task_say(text)

	return say1


def get_find_behavior(task_manager, target):
	return task_manager.create_task_say("Not implemented yet. I'm sorry!")