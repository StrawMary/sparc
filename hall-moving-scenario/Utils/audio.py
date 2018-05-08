import time
import speech_recognition as sr


# this is called from the background thread
def callback(recognizer, audio):

    try:
        print("Google Speech Recognition thinks you said " + recognizer.recognize_google(audio))
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))


r = sr.Recognizer()
m = sr.Microphone()
with m as source:
    r.adjust_for_ambient_noise(source)  # we only need to calibrate once, before we start listening

# start listening in the background (note that we don't have to do this inside a `with` statement)
stop_listening = r.listen_in_background(m, callback, phrase_time_limit=2.0)
# `stop_listening` is now a function that, when called, stops background listening

# do some unrelated computations for 5 seconds
for _ in range(100): time.sleep(1)  # we're still listening even though the main thread is doing other things

# calling this function requests that the background listener stop listening
stop_listening(wait_for_stop=False)
