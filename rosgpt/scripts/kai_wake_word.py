#!/usr/bin/python3

from datetime import datetime
import pvporcupine
from pvrecorder import PvRecorder
import argparse
import rospy
from threading import Thread
from std_msgs.msg import String, Bool
import struct
import wave
import os
import time
import pyaudio
import speech_recognition as sr

class PorcupineDemo(Thread):
    def __init__(
            self,
            access_key,
            library_path,
            model_path,
            keyword_paths,
            sensitivities,
            input_device_index=None,
            output_path=None):
        
        super(PorcupineDemo, self).__init__()

        self._access_key = access_key
        self._library_path = library_path
        self._model_path = model_path
        self._keyword_paths = keyword_paths
        self._sensitivities = sensitivities
        self._input_device_index = input_device_index

        self._output_path = output_path

    def listen(self):
        """
        Here we need to read the input from the user and convert it into string and publish it.
        """
        rospy.loginfo("Here we will start to listen")
        r = sr.Recognizer()
        with sr.Microphone() as source:
            audio=r.listen(source)
            query = ''
            try:
                r.adjust_for_ambient_noise(source, duration=0.2)
                query = r.recognize_google(audio, language='en-US')
                return query
            except Exception as e:
                print("[ERROR] Invalid text, try again " )
        time.sleep(2)

    def run(self, data):
        """
         Creates an input audio stream, instantiates an instance of Porcupine object, and monitors the audio stream for
         occurrences of the wake word(s). It prints the time of detection for each occurrence and the wake word.
         """
        pub = rospy.Publisher('wake_word', String, queue_size=10)
        session_pub = rospy.Publisher('/stt_session_key', Bool, queue_size=10)
        user_text_pub = rospy.Publisher('user_text', String, queue_size=10)
        keywords = list()
        for x in self._keyword_paths:
            keyword_phrase_part = os.path.basename(
                x).replace('.ppn', '').split('_')
            if len(keyword_phrase_part) > 6:
                keywords.append(' '.join(keyword_phrase_part[0:-6]))
            else:
                keywords.append(keyword_phrase_part[0])

        porcupine = None
        recorder = None
        wav_file = None
        pa=None
        try:
            porcupine = pvporcupine.create(
                access_key=self._access_key,
                library_path=self._library_path,
                model_path=self._model_path,
                keyword_paths=self._keyword_paths,
                sensitivities=self._sensitivities)
            session_key = data
            print(session_key)
            
            pa = pyaudio.PyAudio()
            recorder = pa.open(rate=porcupine.sample_rate, channels=1, format=pyaudio.paInt16, input=True, frames_per_buffer=porcupine.frame_length)

            if self._output_path is not None:
                wav_file = wave.open(self._output_path, "w")
                wav_file.setparams((1, 2, 16000, 512, "NONE", "NONE"))

            #print(f'Using device: {recorder.selected_device}')

            print('Listening {')
            for keyword, sensitivity in zip(keywords, self._sensitivities):
                print('  %s (%.2f)' % (keyword, sensitivity))
            print('}')
            if session_key == "on":

                while not rospy.is_shutdown(): 
                    #pcm = recorder.read()
                    pcm=recorder.read(porcupine.frame_length)
                    pcm2=struct.unpack_from("h" * porcupine.frame_length, pcm)

                    if wav_file is not None:
                        wav_file.writeframes(struct.pack("h" * len(pcm), *pcm))

                    result = porcupine.process(pcm2)
                    if result >= 0:
                        print('[%s] Detected %s' %
                            (str(datetime.now()), keywords[result]))
                        text = self.listen()
                        if text is not None:
                            user_text_pub.publish(text)
                            rospy.loginfo(text)
                        wake_str = "session_on"
                        rospy.loginfo(wake_str)
                        time.sleep(1)
                        session_pub.publish(True)
                    
        except pvporcupine.PorcupineInvalidArgumentError as e:
            print("One or more arguments provided to Porcupine is invalid: {\n" +
                  f"\t{self._access_key=}\n" +
                  f"\t{self._library_path=}\n" +
                  f"\t{self._model_path=}\n" +
                  f"\t{self._keyword_paths=}\n" +
                  f"\t{self._sensitivities=}\n" +
                  "}")
            print(
                f"If all other arguments seem valid, ensure that '{self._access_key}' is a valid AccessKey")
            raise e
        except pvporcupine.PorcupineActivationError as e:
            print("AccessKey activation error")
            raise e
        except pvporcupine.PorcupineActivationLimitError as e:
            print(
                f"AccessKey '{self._access_key}' has reached it's temporary device limit")
            raise e
        except pvporcupine.PorcupineActivationRefusedError as e:
            print(f"AccessKey '{self._access_key}' refused")
            raise e
        except pvporcupine.PorcupineActivationThrottledError as e:
            print(f"AccessKey '{self._access_key}' has been throttled")
            raise e
        except pvporcupine.PorcupineError as e:
            print(f"Failed to initialize Porcupine")
            raise e
        except KeyboardInterrupt:
            print('Stopping ...')
        finally:
            if porcupine is not None:
                porcupine.delete()

            if recorder is not None:
                recorder.close()

            if wav_file is not None:
                wav_file.close()

    
    @classmethod
    def show_audio_devices(cls):
        devices = PvRecorder.get_audio_devices()

        for i in range(len(devices)):
            print(f'index: {i}, device name: {devices[i]}')
    


def main():

    rospy.init_node('wake_up', anonymous=True)


    parser = argparse.ArgumentParser()

    parser.add_argument('--access_key',
                    help='AccessKey obtained from Picovoice Console (https://console.picovoice.ai/)')

    parser.add_argument(
    '--keywords',
    nargs='+',
    help='List of default keywords for detection. Available keywords: %s' % ', '.join(
        sorted(pvporcupine.KEYWORDS)),
    choices=sorted(pvporcupine.KEYWORDS),
    metavar='')

    parser.add_argument(
    '--keyword_paths',
    nargs='+',
    help="Absolute paths to keyword model files. If not set it will be populated from `--keywords` argument")

    parser.add_argument(
    '--library_path', help='Absolute path to dynamic library.', default=pvporcupine.LIBRARY_PATH)

    parser.add_argument(
    '--model_path',
    help='Absolute path to the file containing model parameters.',
    default=pvporcupine.MODEL_PATH)

    parser.add_argument(
    '--sensitivities',
    nargs='+',
    help="Sensitivities for detecting keywords. Each value should be a number within [0, 1]. A higher " +
            "sensitivity results in fewer misses at the cost of increasing the false alarm rate. If not set 0.5 " +
            "will be used.",
    type=float,
    default=None)

    parser.add_argument('--audio_device_index',
                    help='Index of input audio device.', type=int, default=-1)

    parser.add_argument(
    '--output_path', help='Absolute path to recorded audio for debugging.', default=None)

    parser.add_argument('--show_audio_devices', action='store_true')

    args = parser.parse_args()

    if args.show_audio_devices:
        PorcupineDemo.show_audio_devices()
    else:
        if args.access_key is None:
            raise ValueError("AccessKey (--access_key) is required")
        if args.keyword_paths is None:
            if args.keywords is None:
                raise ValueError(
                "Either `--keywords` or `--keyword_paths` must be set.")

            keyword_paths = [pvporcupine.KEYWORD_PATHS[x]
                            for x in args.keywords]
        else:
            keyword_paths = args.keyword_paths

        if args.sensitivities is None:
            args.sensitivities = [0.5] * len(keyword_paths)

        if len(keyword_paths) != len(args.sensitivities):
            raise ValueError(
            'Number of keywords does not match the number of sensitivities.')

        PorcupineDemo(
        access_key=args.access_key,
        library_path=args.library_path,
        model_path=args.model_path,
        keyword_paths=keyword_paths,
        sensitivities=args.sensitivities,
        output_path=args.output_path,
        input_device_index=args.audio_device_index).run("on")
        rospy.spin()


"""
 rosrun porcupine_wake_word kai_wake_word.py --access_key 1h/tb+FdmR/svwEt1dIsKnJ/g9F68W3hZuhDnKdMNg/g+j9VtXvAVw== --keyword_path /home/utsav/Hello-Kai_en_linux_v2_1_0.ppn
"""   
    

if __name__ == '__main__':
    main()
