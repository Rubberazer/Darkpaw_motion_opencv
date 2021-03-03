# Darkpaw_motion_opencv
A little project in C/C++ for the Adeept Darkpaw robot.

Basically replaced the original maker's python code by my own C code -with a pinch of C++ for the OpenCV bit, this is an initial commit and still have to decide what to do with the camera e.g. chase objects or something. I added an ultrasound sensor (the usual HC-SR04) as there were empty slots in the robot HAT, so the robot doesn't hit the walls. At the moment the robot can move: forwards, backwards, turn right and left totally fine. In future developments will also try to incorporate the gyroscope that comes along with the kit somehow, probably at the end of the process I will be lacking computer power again, c'est la vie! What I won't probably do is add a web server, but if end up doing it, it is going to be something along the lines of Libmicrohttpd. I actually believe that remote control is overrated anyways, let robots be autonomous creatures.

The robot itself can be found here: https://www.adeept.com/adeept-darkpaw-bionic-quadruped-spider-robot-kit-for-raspberry-pi-4-3-model-b-b-2b-stem-crawling-robot-opencv-tracking-self-stabilizing_p0125_s0035.html

It is a cool piece of machinery and the provided HAT supports connecting some additional stuff if you wanted too, I equipped the toy with a Raspberry Pi 4 with 8 Gigas so it should be OKish even for the OpenCV stuff, as a basis to control the GPIO I am using the famous PIGPIO library wich always works first time, really good stuff that can be found here: http://abyz.me.uk/rpi/pigpio/
