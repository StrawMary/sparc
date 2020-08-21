import config as cfg

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
	listen_task = task_manager.create_task_listen(["da", "nu"])
	success_child = {"da": say2, "nu": say3}

	listen_task.success_child = success_child
	listen_task.fail_child = say4
	say1.success_child = listen_task
	return say1


def get_actuators_behaviour(task_manager, target, optional_entities=None):
	say1 = task_manager.create_task_say("Ok. Will do.")
	say2 = task_manager.create_task_say("Done.")
	say3 = task_manager.create_task_say("Sorry, I failed.")

	actuation_task = task_manager.create_task_actuation(target, optional_entities)
	actuation_task.success_child = say2
	actuation_task.fail_child = say3

	say1.success_child = actuation_task
	return say1

def get_serve_behaviour(task_manager):

	show_logo = task_manager.create_task_show_URL("https://alexawada.com/aimas/")
	say_presentation = task_manager.create_task_say("Hello. I'm Pepper. Thank you for your visit. "
													"We are in the AI MAS laboratory of the "
													"Faculty of Computer Science and Automatic Control in the "
													"Politehnica University")
	say_presentation_2 = task_manager.create_task_say("The lab is coordinated by Professor Adina Florea.")
	say_presentation_3 = task_manager.create_task_say("We will now present you a short demo from our work. ...")


	say1 = task_manager.create_task_say("Ok. I will start serving people. Please hand me my tray. "
										"Touch my head when you are done.")
	hand_movement_task = task_manager.create_task_pose("serve")
	wait_task = task_manager.create_task_wait('head')
	say_what_im_doing = task_manager.create_task_say("I will start looking for orders.")
	look_for_hand_up_task = task_manager.create_task_search("hand_up", "#person_id")
	say_found_person = task_manager.create_task_say("I found a person.")
	say_not_found_person = task_manager.create_task_say("I found a person.")

	#cum luam rezultatul?
	move_to_task = task_manager.create_task_go_to("#person_id")
	say3 = task_manager.create_task_say("Hello. I'm Pepper, what do you want to drink?")
	say4 = task_manager.create_task_say("We have coke and water. Say cancel if you don't want anything.")


	sayCoke = task_manager.create_task_say("Coke. Coming Right up.")
	sayWater = task_manager.create_task_say("Water. Coming Right up.")

	listen_task = task_manager.create_task_listen(["coke", "water", "cancel"])
	success_child = {"coke": sayCoke, "water": sayWater, "cancel": None}

	move_to_home_coke = task_manager.create_task_go_to("home")
	move_to_home_water = task_manager.create_task_go_to("home")

	say_give_me_coke = task_manager.create_task_say("I have an order for a coke")
	say_give_me_water = task_manager.create_task_say("I have an order for a water")

	say_place_order = task_manager.create_task_say("Place the item into my tray and touch my head when you are done. Thank you!")
	wait_place_order = task_manager.create_task_wait('head')
	move_back_to_person = task_manager.create_task_go_to("last_pos")
	say_take_order = task_manager.create_task_say("Hi. Please take your order and touch my head when you are done")
	wait_take_order = task_manager.create_task_wait('head')
	move_back_home = task_manager.create_task_go_to("home")

	say_done = task_manager.create_task_say("I have delivered the order.")



	show_logo.success_child = say_presentation
	say_presentation.success_child = say_presentation_2
	say_presentation_2.success_child = say_presentation_3

	say_presentation_3.success_child = say1
	say1.success_child = hand_movement_task
	hand_movement_task.success_child = wait_task

	wait_task.success_child = say_what_im_doing
	say_what_im_doing.success_child = look_for_hand_up_task
	look_for_hand_up_task.success_child = say_found_person
	look_for_hand_up_task.fail_child = say_not_found_person
	say_found_person.success_child = move_to_task
	move_to_task.success_child = say3
	say3.success_child = say4

	say4.success_child = listen_task
	listen_task.success_child = success_child
	listen_task.fail_child = say3

	sayCoke.success_child = move_to_home_coke
	sayWater.success_child = move_to_home_water

	move_to_home_coke.success_child = say_give_me_coke
	move_to_home_water.success_child = say_give_me_water

	say_give_me_coke.success_child = say_place_order
	say_give_me_water.success_child = say_place_order
	say_place_order.success_child = wait_place_order
	wait_place_order.success_child = move_back_to_person
	move_back_to_person.success_child = say_take_order
	say_take_order.success_child = wait_take_order
	wait_take_order.success_child = move_back_home
	move_back_home.success_child = say_done

	return show_logo


def get_presentation_behaviour(task_manager, optional_entities={}):
	say1 = task_manager.create_task_say("Dear Guests")
	say2 = task_manager.create_task_say("Thank you for your visit. "
										"We are in the AI MAS laboratory of the "
										"Faculty of Computer Science and Automatic Control in the Politehnica "
										"University")
	say3 = task_manager.create_task_say("The lab is coordinated by Professor Adina Florea.")
	say4 = task_manager.create_task_say("Let me introduce you the projects of the lab.")
	show_logo = task_manager.create_task_show_URL("https://alexawada.com/aimas/")

	go_to_andrei = task_manager.create_task_go_to('andrei@')
	say5 = task_manager.create_task_say("I will invite Andrei to talk about the Nemo Drive project.")

	go_home = task_manager.create_task_go_to("home")
	go_home2 = task_manager.create_task_go_to("home")
	go_home3 = task_manager.create_task_go_to("home")



	wait_task = task_manager.create_task_wait('head')
	wait_task2 = task_manager.create_task_wait('head')
	wait_task3 = task_manager.create_task_wait('head')

	say_thank_task1 = task_manager.create_task_say("Thank you Andrei!")
	say6 = task_manager.create_task_say("I will now invite Alex to speak about human robot interaction")
	go_to_alex = task_manager.create_task_go_to('alexg@')

	go_to_sorici = task_manager.create_task_go_to('alexsorici@')
	say_thank_task2 = task_manager.create_task_say("Thank you Alex!")

	say7 = task_manager.create_task_say("I will now invite the other Alex to talk about the CAMI project")
	say8 = task_manager.create_task_say("Thank you for your visit. Hope you've enjoyed our presentations!")


	say1.success_child = show_logo
	show_logo.success_child = say2
	say2.success_child = say3
	say3.success_child = say4

	say4.success_child = go_to_andrei
	go_to_andrei.success_child = say5
	go_to_andrei.fail_child = say5
	say5.success_child = wait_task
	wait_task.success_child = say_thank_task1

	say_thank_task1.success_child = go_to_alex
	go_to_alex.success_child = say6
	go_to_alex.fail_child = say6
	say6.success_child = wait_task2

	wait_task2.success_child = say_thank_task2
	say_thank_task2.success_child = go_to_sorici
	go_to_sorici.success_child = say7
	go_to_sorici.fail_child = say7
	say7.success_child = go_home3

	go_home3.success_child = wait_task3

	wait_task3.success_child = say8

	return say1


def get_remember_behaviour(task_manager, optional_entities={}):
	if 'target' in optional_entities:
		say1 = task_manager.create_task_say("Ok, %s. Look at me for 5 seconds." % optional_entities['target'])
		remember_task = task_manager.create_task_remember(optional_entities['target'], "person")
		say2 = task_manager.create_task_say("Done.")
		say3 = task_manager.create_task_say("Sorry, I failed.")

		say1.success_child = remember_task
		remember_task.success_child = say2
		remember_task.fail_child = say3
		return say1

	say1 = task_manager.create_task_say("Ok. What is the target name?")
	listen = task_manager.create_task_listen()
	say3 = task_manager.create_task_say(listen.get_result)
	say4 = task_manager.create_task_say("Look at me for 5 seconds.")
	say5 = task_manager.create_task_say("Sorry, I didn't hear the target name.")
	remember_task = task_manager.create_task_remember(listen.get_result, "person")
	say6 = task_manager.create_task_say("Done.")
	say7 = task_manager.create_task_say("Sorry, I failed.")

	say1.success_child = listen
	listen.success_child = say3
	say3.success_child = say4
	listen.fail_child = say5
	say4.success_child = remember_task
	remember_task.success_child = say6
	remember_task.fail_child = say7
	return say1


def get_demo1_behaviour(task_manager, optional_parameters={}):
	say1 = task_manager.create_task_say("Hello. I'm Pepper. Thank you for your visit. "
										"We are in the AI MAS laboratory, of the "
										"Faculty of Computer Science and Automatic Control, in the "
										"University Politehnica of Bucharest. The lab, is coordinated by Professor Adina Florea. "
										"Let me show you what I can do. ")

	sayheart = task_manager.create_task_say("I can show you your health status.")
	heart = task_manager.create_task_show_health_measurements("heart")
	say2 = task_manager.create_task_say("I can raise the blinds")

	actuation = task_manager.create_task_actuation("blinds", {"command": "raise"})

	say3 = task_manager.create_task_say("I can turn on the light, and change it's color")
	actuation2 = task_manager.create_task_actuation("lights", {"command": "on"})
	actuation3 = task_manager.create_task_actuation("lights", {"color": "green"})

	say4 = task_manager.create_task_say("I can recognise people")
	find_stephanie = task_manager.create_task_search("Stephanie")
	say5 = task_manager.create_task_say("Hello Stephanie!")

	say1.success_child = sayheart
	sayheart.success_child = heart
	heart.success_child = say2
	say2.success_child = actuation
	actuation.success_child = say3
	say3.success_child = actuation2
	actuation2.success_child = actuation3
	actuation3.success_child = say4
	say4.success_child = find_stephanie
	find_stephanie.success_child = say5

	return say1


def get_demo2_behaviour(task_manager, optional_parameters={}):
	show_logo = task_manager.create_task_show_URL("https://alexawada.com/aimas/")
	show_logo2 = task_manager.create_task_show_URL("https://alexawada.com/aimas/")
	say1 = task_manager.create_task_say("Hello. I'm Pepper. Thank you for your visit. "
										"We are in the AI MAS laboratory, of the "
										"Faculty of Computer Science and Automatic Control, in the "
										"University Politehnica of Bucharest. The lab, is coordinated by Professor Adina Florea. ")
	say2 = task_manager.create_task_say("I can do multiple things, to help you. For example, I can command the environmental "
										"sensors, I can show your different health measurements, and remind you of your "
										"appointments. I can even recognize people, and objects, and go to them.")
	head_left = task_manager.create_task_choregraphe_behaviour("move-head-left/behavior_1")
	say3 = task_manager.create_task_say("Hello Andrei! I see you are next to a plant. Did you water it?")
	say4 = task_manager.create_task_say("Great!")
	listen1 = task_manager.create_task_listen(["yes"])
	listen1_success_child = {"yes": say4}
	head_right = task_manager.create_task_choregraphe_behaviour("move-head-right/behavior_1")
	head_right2 = task_manager.create_task_choregraphe_behaviour("move-head-right/behavior_1")

	listen2 = task_manager.create_task_listen(["thank you"])
	listen2_success_child = {"thank you": show_logo2}
	say5 = task_manager.create_task_say("Hello Stephanie! Can I help you with anything?")
	follow_me = task_manager.create_task_choregraphe_behaviour("follow_me-29b9c8/behavior_1")
	listen3 = task_manager.create_task_listen(["follow me"])
	listen3_success_child = {"follow me": head_right2}

	show_logo.success_child = say1
	say1.success_child = say2
	say2.success_child = head_left
	head_left.success_child = say3
	say3.success_child = listen1
	listen1.success_child = listen1_success_child

	say4.success_child = listen2
	listen2.success_child = listen2_success_child

	show_logo2.success_child = head_right
	head_right.success_child = say5
	say5.success_child = listen3
	listen3.success_child = listen3_success_child
	head_right2.success_child = follow_me

	return show_logo


def get_demo_behaviour2(task_manager, optional_parameters={}):
	show_logo = task_manager.create_task_show_URL("https://alexawada.com/aimas/")
	say1 = task_manager.create_task_say("Hello. I'm Pepper. Thank you for your visit. "
										"Faculty of Computer Science and Automatic Control, in the "
										"University Politehnica of Bucharest. The lab, is coordinated by Professor Adina Florea. \\pau=500\\")
	say2 = task_manager.create_task_say("I love artificial intelligence \\pau=500\\")
	say3 = task_manager.create_task_say("Andrei will present now the first project")

	show_logo.success_child = say1
	say1.success_child = say2
	say2.success_child = say3

	return say1


def get_activity_test_behaviour(task_manager, optional_entities={}):
	say1 = task_manager.create_task_say("Do you want to recognize a specific activity?")
	say2 = task_manager.create_task_say("What activity do you want to recognize?")
	say31 = task_manager.create_task_say("Ok")
	say32 = task_manager.create_task_say("Ok")
	say33 = task_manager.create_task_say("Ok")
	say4 = task_manager.create_task_say("Sorry, I didn't hear you.")

	listen_task1 = task_manager.create_task_listen(["yes", "no"])
	listen_task2 = task_manager.create_task_listen(["hand waving", "drink water"])

	recognize1 = task_manager.create_task_recognize_activity("hand waving")
	recognize2 = task_manager.create_task_recognize_activity("drink water")
	recognize3 = task_manager.create_task_recognize_activity()

	say5 = task_manager.create_task_say(recognize3.get_result)
	say6 = task_manager.create_task_say("Action recognized correctly.")
	say7 = task_manager.create_task_say("Wrong recognized action.")
	say8 = task_manager.create_task_say("Wrong recognized action.")
	say9 = task_manager.create_task_say(recognize1.get_result)
	say10 = task_manager.create_task_say(recognize2.get_result)

	success_child1 = {"yes": say2, "no": say33}
	success_child2 = {"hand waving": say31, "drink water": say32}

	listen_task1.success_child = success_child1
	listen_task1.fail_child = say4

	listen_task2.success_child = success_child2
	listen_task2.fail_child = say4

	say1.success_child = listen_task1
	say2.success_child = listen_task2
	say31.success_child = recognize1
	say32.success_child = recognize2
	say33.success_child = recognize3

	recognize1.success_child = say6
	recognize1.fail_child = say7
	say7.success_child = say9

	recognize2.success_child = say6
	recognize2.fail_child = say8
	say8.success_child = say10

	recognize3.success_child = say5

	return say1
