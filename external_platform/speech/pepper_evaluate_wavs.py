import os
import pickle
import speech_recognition as sr

class PepperEval:
	def __init__(self, folder_path):
		print("Reading content from:", folder_path)
		sub_folders = [os.path.join(folder_path, o) for o in os.listdir(folder_path)
                    if os.path.isdir(os.path.join(folder_path,o))]
		all_files = []
		for sub_folder in sub_folders:
			print("Reading subfolder from:", sub_folder)
			wav_files = [os.path.join(sub_folder, o) for o in os.listdir(sub_folder)
			 if os.path.join(sub_folder, o).endswith('.wav')]
			all_files.append((sub_folder, wav_files))
		for sub_folder, s_files in all_files:
			print(sub_folder, len(s_files))

		self._all_files = all_files

	def eval(self):
		r = sr.Recognizer()
		for subfolder, files in self._all_files:
			for file in files:
				print(file)
				with sr.WavFile(file) as source:
					audio = r.record(source)

					wav_data = audio.get_wav_data(48000, 1)
					ad = sr.AudioData(wav_data, 48000, 1)

					print(ad.sample_rate)
					print(ad.sample_width)

					try:
						print("Google Speech Recognition thinks you said:" + r.recognize_google(ad, language='ro-RO'))
					except Exception as e:
						print("Error ", e)
			break





pe = PepperEval("/home/amiro/workspace/sparc/external_platform/results")
pe.eval()