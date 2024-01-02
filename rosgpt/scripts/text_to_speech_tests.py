#!/usr/bin/env python3
import pyttsx3
import edge_tts
import asyncio
from elevenlabs import generate, play


def eleven_labs_tts(text):
    audio = generate(
    text=text,
    voice="Rachel",
    model="eleven_multilingual_v2"
    )
    play(audio)


def pyttxx3_test(text):
    engine = pyttsx3.init()
    # convert this text to speech
    engine.setProperty("rate", 150)
    engine.say(text)
    # play the speech
    engine.runAndWait()

if __name__ == '__main__':
    text = "your request could not be processed ! please try again"
    eleven_labs_tts(text)
    #pyttxx3_test(text)
