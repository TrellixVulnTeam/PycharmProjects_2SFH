The code is for teleoping a robot.
author: Jacqueline Zhou jianingzhou@brandeis.edu
=========================================================================
key "h" is for halt
key "f" is for go forward
key "b" is for go backward
key "l" is for rotating left
key "r" is for rotating right
key "s" is for spiral movement
key "z" is for zigzag movement
added feature:
key "o" is for going back to birth point
key "q" is for storing the next key strokes until the next "q" stroke
    hitting "q" for the third time will be carrying out the stored motion
    and withdraw from the program upon completion
=========================================================================
first run key_publisher.py to publish key strokes to the topic
then run JacquelineZhou_PA3.py
=========================================================================
JacquelineZhou_PA3.py uses two vectors (velocity_vectors, transform_vectors)
to manipulate the published velocity command in order to control the movement
of the robot.

I imporved the printing, having it only print out the state by each key stroke
made, and also explained what is each state doing. I added a state dictionary 
which contains the explaining string for each state (state_dict)
=========================================================================
Writing this program is much easier than PA2, I found using the buffer loop
really helpful