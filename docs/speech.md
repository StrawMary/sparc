# Speech Component

The speech component is based on a publisher-subscriber paradigm. The speech detected by the publisher is transmitted to the subscriber through a ROS topic. This component is implemented in **sparc/external_platform/robot_interaction** folder:
 - speech_recognition_publisher.py
 - speech_recognition_subscriber.py

The publisher and subscriber communicate through *speech_text* channel. If the topic name is changed, it must be changed in both files:

    # Speech_recognition_subscriber.py
    self.recognition_subscriber = rospy.Subscriber('speech_text', String, self.callback_text)
    
    # Speech_recognition_publisher.py
    self.publisher = rospy.Publisher('speech_text', String, queue_size=10)

## Speech recognition publisher

The publisher file uses a Google speech recognition engine, for both English and Romanian languages. The script can be configured to work in either language using the *language* parameter in the file. Moreover, the implementation allows the user to input the sentences from keyboard, through the *audio_stream* parameter.

As the publisher uses a ROS topic to send the recognized sentences, a *roscore* instance must be started. To run the script:

    python speech_recognition_publisher.py

The script will prompt a question *"Audio stream? [y/n]:"* to which the user can respond with *"y"* (yes) or *"n"* (no), which represent if the script should use the input from the microphone or not.

## Speech recognition subscriber

The subscriber file contains a class that subscribes to the ROS topic and tries to interpret the recognized speeches using [wit.ai](https://wit.ai/). The wit.ai application extracts the action to be done and the parameters, if possible, and passes the result to the rest of the application through the argument method received in the class' constructor.

To run the script (if you run it as a standalone application, please make sure that the robot_stream flag is false and ):

    python speech_recognition_subscriber.py

### Wit.ai configuration

To work for both languages English and Romanian, two applications were created: [sparc](https://wit.ai/stefaniag/sparc/entities) and [sparc-ro](https://wit.ai/stefaniag/sparc-ro). Each application is trained with a set of queries specific to the associated language.

The wit.ai application extracts relevant entities from the queries. The user-defined entities used in the project are:
 - **intent** = specifies which type of action is intended. The *intent* entity is seen as a trait of the sentence and considers the sentence as a whole. The application is trained to recognize the following intents:
   - **hello** = greet someone
   - **stop** = stop the current behavior
   - **say** = say something
   - **go to** = go to a position
   - **find** = find a target in the environment
   - **look** = look around for the target
   - **health** = display health statistics
   - **reminders** = display and read out reminders for a person
     -  **next**/**previous** = intents required for voice interaction in reminders behavior (the user can vocally switch to next or previous reminder)
 - **target** = parameter for *say*, *go to*, *find*, *look* and *reminders* intents. The entity is attached to a specific part of the sentence, which represent the target. Each keyword of the target entity can have multiple synonyms which can be configured.
 - **health_entity** = parameter for *health* intent. Similar to *target* entity.

To use a wit.ai application, an access key must be provided. The access keys for this project are defined in **sparc/external_folder/config.py** file, in the *access_keys* parameter.

The *mandatory_intent_entities* parameter in **sparc/external_folder/config.py** file is a dictionary which contains all the existing intents with the associated list of mandatory entities. That means that if an entity from the mandatory list is not extracted from the sentence when that specific intent is recognized, the command is invalid.  To add a new intent, a new *(key, value)* pair needs to be added in the dictionary, where the key represent the intent name and the value the list of mandatory entities.

### Understanding new phrases

To test with new sentences the wit.ai application, follow the next steps:
1. Go to the [application page](https://wit.ai/stefaniag/sparc/entities)
2. In the *"User says"* box introduce your sentence (e.g.: go to Stephanie)
3. If previously trained, the application will extract some entities
![Wit.ai response example](https://lh3.googleusercontent.com/5aXubssD1NW6vRKIG017VImwnJZ8Ds3Jrr6OhxPlelwpOZpgB0G-ZajcTH4f6TnhcXjKCkJkAAZb83U007eCH4JlsrgtByN7UEmzKDyKBSqxK5aEoiOYQaSTGLi1qW05LJ-UmQd7D8yiRPH-dBZic4YpyF2k5FsZE2ihdkDmHaulzdq-4dDJ7jncXufa6BtUHzAI8KJvIz6Re8MJ_H5tADoexXsY-eAXZzzV2KBvTHTu9Wype_Xg0ZkTnXZKpHerH_XM1RaX4aM7TRy7WV8LvJsCXv7_gC_iKlKq5A2t-8ay7tccM3GeAQGLWUpov0uYsAW-4r6edT8UKjPP0TvbeiJ-VfNk5-aWzndmGEEI6TX9eY8Dy6eN9SUXIbxfCC_xolDxR4S6O8hWVx5WdLz0BsT8prcpFbbC21k8buXaXBe1vkPh3l4ytjf1fR4R7BVIVcqCynJgzpoOkeSQ1uh7WYSxdGrwvmXzoXv_LUZ-gCEKUh9x6davBtO-5-A3qoJynnDrK1bmOBA505-yY7XUYKaHmr86-yqET_sByKrCXSQWKBphjOLyHxv_hx2rqPskIYTfVupwwGpJwm7XO1tQV0mzoqnkUX6r=w1855-h990)

If you want to train the application with the new sentence, select the right entities and click *Validate*. This will start the re-training process (the training status, *Done*, *On-going* or *Scheduled*, can be seen in the top-left corner). To associate a *target* or *health_entity* entity to the input sentence:
1. Select the part of the sentence associated with the entity
2. Click "*Add a new entity*"
3. Choose the appropiate entity
4. Select the associated value (keyword)