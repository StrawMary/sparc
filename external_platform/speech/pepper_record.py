from pepper_microphone import PepperSpeechRecognitionEngine
import pickle
import sys, getopt

def main(argv, rec_cnt = 0, end_cnt = 0):
	one_shot = False
	if rec_cnt != 0 and end_cnt == rec_cnt:
		one_shot = True

	print(rec_cnt)
	while True:
		speechRec = PepperSpeechRecognitionEngine(False)
		speechRec.talking = True

		speechRec.first_speech_frame = 10
		audio_data_new, speech_data = speechRec.listen()
		speechRec.microphone_buffer = []
		if len(speech_data) < 150000:
			print("Not enough data")
			continue
		#result = speechRec.recognize_google(audio_data_new, "ro-RO")
		result = speechRec.recognize_google(audio_data_new, "en-EN")
		print(result)
		audio_result = {
			"audio": audio_data_new,
			"recognition": result
		}
		pickle.dump(audio_result, open(argv[0] + "_" + str(rec_cnt) + ".p", "wb"))
		with open(argv[0] + "_" + str(rec_cnt) + ".wav", "wb") as f:
			rec_cnt += 1
			f.write(audio_data_new.get_wav_data())
		speechRec.talking = True

		if one_shot:
			exit(0)





if __name__ == "__main__":
	if len(sys.argv) < 2:
		print("noth enough args")
		exit(0)
	if len(sys.argv) >= 3:
		cnt = sys.argv[2]
	else:
		cnt = 0

	if len(sys.argv) >= 4:
		end_cnt = sys.argv[3]
	else:
		end_cnt = cnt

	main(sys.argv[1:], int(cnt), int(end_cnt))


