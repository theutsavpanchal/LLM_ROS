#!/usr/bin/env python3

import speech_recognition as sr
from ctypes import *

import pyaudio

def get_audio2():
    r = sr.Recognizer()
    m = sr.Microphone()

    try:
        print("A moment of silence, please...")
        with m as source: r.adjust_for_ambient_noise(source)
        print("Set minimum energy threshold to {}".format(r.energy_threshold))
        while True:
            print("Say something!")
            with m as source: audio = r.listen(source)
            print("Got it! Now to recognize it...")
            try:
                # recognize speech using Google Speech Recognition
                value = r.recognize_google(audio)

                # we need some special handling here to correctly print unicode characters to standard output
                if str is bytes:  # this version of Python uses bytes for strings (Python 2)
                    print("You said {}".format(value).encode("utf-8"))
                else:  # this version of Python uses unicode for strings (Python 3+)
                    # print("You said {}".format(value))
                    return value
            except sr.UnknownValueError:
                print("Oops! Didn't catch that")
            except sr.RequestError as e:
                print("Uh oh! Couldn't request results from Google Speech Recognition service; {0}".format(e))
    except KeyboardInterrupt:
        pass


def get_audio():

    r = sr.Recognizer()
    with sr.Microphone() as source:
        audio = r.listen(source)
        said = ""
        try:
            
            said = r.recognize_google(audio)
            said=said.lower()
            print(f"Text Received:  {said}")
        except Exception as e:
            print("Exception: " + str(e))

    return said.lower()

if __name__ == "__main__":
    WAKE = "hello kai"

    while True:
        print("Say Hello Kai !")
        text = get_audio()
        if text.count(WAKE) > 0:
            print("Detected Wakeword!...Now Listening")
            request = get_audio()
            print(request)
        