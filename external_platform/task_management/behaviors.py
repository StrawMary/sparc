def get_say_behavior(task_manager, text):
	say1 = task_manager.create_task_say(text)

	return say1


def get_search_behavior(task_manager, target):
	say1 = task_manager.create_task_say("Looking for " + target)
	search1 = task_manager.create_task_search(target)
	if not search1:
		return task_manager.create_task_say("No target")

	say2 = task_manager.create_task_say("Found " + target)
	say3 = task_manager.create_task_say("I didn't find the target")

	say1.success_child = search1

	search1.success_child = say2
	search1.fail_child = say3
	return say1


def get_go_to_behavior(task_manager, target):
	say1 = task_manager.create_task_say("Going to " + target)
	move1 = task_manager.create_task_go_to(target)
	if not move1:
		return task_manager.create_task_say("Sorry, I don't know the target's position.")

	say2 = task_manager.create_task_say("Reached " + target)
	say3 = task_manager.create_task_say("Sorry. I failed to reach " + target)

	say1.success_child = move1

	move1.success_child = say2
	move1.fail_child = say3
	return say1


def get_reminders_behavior(task_manager, target):
	say1 = task_manager.create_task_say("Displaying reminders for " + target)
	reminders1 = task_manager.create_task_show_reminders(target)
	if not reminders1:
		return task_manager.create_task_say("Sorry, I can't do that.")

	say1.success_child = reminders1
	return say1


def get_health_behaviour(task_manager, target):
	say1 = task_manager.create_task_say("Showing health status")
	reminders1 = task_manager.create_task_show_health_measurements(target)
	if not reminders1:
		return task_manager.create_task_say("Sorry, I can't do that.")

	say1.success_child = reminders1
	return say1


def get_find_behavior(task_manager, target):
	say1 = task_manager.create_task_say("Looking for " + target)
	say2 = task_manager.create_task_say("I've found " + target)
	say4 = task_manager.create_task_say("Going to " + target + " home")
	say3 = task_manager.create_task_say("Sorry. I couldn't find " + target)
	move1 = task_manager.create_task_go_to(target)
	move2 = task_manager.create_task_go_to(target + "@")
	search1 = task_manager.create_task_search(target)
	search2 = task_manager.create_task_search(target)

	if not move1 and not move2:
		return task_manager.create_task_say("Sorry I don't know anything about " + target)

	if move2:
		move2.success_child = search2
		search2.success_child = say2
		search2.fail_child = say3
		move2.fail_child = say3
		say4.success_child = move2

	if move1:
		move1.success_child = search1
		search1.success_child = say2
		if move2:
			move1.fail_child = say4
			search1.fail_child = say4
		else:
			move1.fail_child = say3
			search1.fail_child = say3
		say1.success_child = move1
		return say1
	else:
		say1.success_child = say4
		return say1


def get_simple_listen_behavior(task_manager):
	say1 = task_manager.create_task_say("Is this a listen test?")
	say2 = task_manager.create_task_say("Yes, it is a test.")
	say3 = task_manager.create_task_say("No, it is not a test.")
	say4 = task_manager.create_task_say("Sorry, I didn't hear you.")
	listen_task = task_manager.create_task_listen(['da', 'nu'])
	success_child = {'da': say2, 'nu': say3}

	listen_task.success_child = success_child
	listen_task.fail_child = say4
	say1.success_child = listen_task
	return say1

