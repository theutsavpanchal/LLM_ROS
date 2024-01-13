#!/usr/bin/env python3
import pyttsx3
import edge_tts
import asyncio
from elevenlabs import generate, play
import elevenlabs
import requests
import uuid
import os
from playsound import playsound

ELEVENLABS_API_KEY = "API_KEY"
ELEVENLABS_VOICE_STABILITY = 0.30
ELEVENLABS_VOICE_SIMILARITY = 0.75

# Choose your favorite ElevenLabs voice
ELEVENLABS_VOICE_NAME = "Hugh"
ELEVENLABS_ALL_VOICES = []




def get_voices() -> list:
    """Fetch the list of available ElevenLabs voices.

    :returns: A list of voice JSON dictionaries.
    :rtype: list

    """
    url = "https://api.elevenlabs.io/v1/voices"
    headers = {
        "xi-api-key": ELEVENLABS_API_KEY
    }
    response = requests.get(url, headers=headers)
    return response.json()["voices"]

def generate_audio(text: str, output_path: str = "") -> str:
    """Converts

    :param text: The text to convert to audio.
    :type text : str
    :param output_path: The location to save the finished mp3 file.
    :type output_path: str
    :returns: The output path for the successfully saved file.
    :rtype: str

    """
    voices = get_voices()
    try:
        voice_id = next(filter(lambda v: v["name"] == ELEVENLABS_VOICE_NAME, voices))["voice_id"]
    except StopIteration:
        voice_id = voices[0]["voice_id"]
    url = f"https://api.elevenlabs.io/v1/text-to-speech/{voice_id}"
    headers = {
        "xi-api-key": ELEVENLABS_API_KEY,
        "content-type": "application/json"
    }
    data = {
        "text": text,
        "voice_settings": {
            "stability": ELEVENLABS_VOICE_STABILITY,
            "similarity_boost": ELEVENLABS_VOICE_SIMILARITY,
        }
    }
    response = requests.post(url, json=data, headers=headers)
    with open(output_path, "wb") as output:
        output.write(response.content)
    return output_path

def pyttxx3_test(text):
    engine = pyttsx3.init()
    # convert this text to speech
    engine.setProperty("rate", 150)
    engine.say(text)
    # play the speech
    engine.runAndWait()


def eleven_labs_tts(text):
    reply_file = f"{uuid.uuid4()}.mp3"
    reply_path = f"/home/utsav/LLM_ROS/src/{reply_file}"
    output_path = generate_audio(text, output_path=reply_path)
    playsound(output_path)


if __name__ == '__main__':
    text = "your request could not be processed ! please try again"
    eleven_labs_tts(text)
    #pyttxx3_test(text)
