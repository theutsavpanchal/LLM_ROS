## LLM based conversational voice assistant for Robots

    author: Utsav Panchal
    email: panchalutsav274@gmail.com

    An idea to create a Voice assistant for robots with Chatgpt backend


### How to run

1) start roscore
2) Start Client Node
3) Start GPT parser node


```
roscore
rosrun rosgpt client_node.py
rosrun rosgpt gpt_parser.py
```

More ideas will be implemented...

If the problem says: "package do not exists". Do the following


    source devel/setup.bash

    Either you must run the above source command each time you open a new terminal window or add it to your .bashrc file as follows.

    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc


### References 

https://wiki.ros.org/catkin/Tutorials/create_a_workspace  
https://github.com/aniskoubaa/rosgpt/tree/main
