## LLM based conversational voice assistant for Robots

    author: Utsav Panchal
    email: panchalutsav274@gmail.com

    An idea to create a Voice assistant for robots with Chatgpt backend


### How to run

1) start roscore
```
roscore
```
2) Run Kai wake word detection node. 
Make sure to put the correct path of the .ppn file. In this case /home/your_username/Hello-Kai_en_linux_v2_1_0.ppn
```
rosrun rosgpt kai_wake_word.py --access_key 1h/tb+FdmR/svwEt1dIsKnJ/g9F68W3hZuhDnKdMNg/g+j9VtXvAVw== --keyword_path /home/utsav/Hello-Kai_en_linux_v2_1_0.ppn
```
3) Run GPT parser node
```
rosrun rosgpt gpt_parser.py
```
4) Run Command parser node
```
rosrun rosgpt cmd_parser.py
```


If the problem says: "package do not exists". Do the following


    source devel/setup.bash

    Either you must run the above source command each time you open a new terminal window or add it to your .bashrc file as follows.

    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

### Errors you might face
ALSA and speech recognition related issues: refer [here](https://stackoverflow.com/questions/7088672/pyaudio-working-but-spits-out-error-messages-each-time) and also [here](https://github.com/Uberi/speech_recognition/issues/526)


### References 

https://wiki.ros.org/catkin/Tutorials/create_a_workspace  
https://github.com/aniskoubaa/rosgpt/tree/main
https://www.emqx.com/en/blog/how-to-use-mqtt-in-python
