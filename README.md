# Darkpaw_motion_opencv
A little project in C/C++ for the Adeept Darkpaw robot.

<video src='https://github.com/Rubberazer/Darkpaw_motion_opencv/assets/47650457/d74e66f9-015d-43ed-9e28-c8865e67b35d' width=352/>

Basically replaced the original maker's python code by my own C code -with a pinch of C++ for the OpenCV thread, I am using OpenCV to first detect and then follow objects. I added an ultrasound sensor (the usual HC-SR04) as there were empty slots in the robot HAT, so it doesn't hit the walls. At the moment the robot can move: forward, backward, turn right and left totally fine, is able to detect, track and follow objects. It also incorporates a MPU6050 gyroscope & accelerometer that comes with the kit, at the moment those values are stored along with the distance measured by the ultrasound sensor in a csv file with a time stamp. Version 3 also incorporates a web server.

08/03/2021: Added PID regulator to compensate drifting and to steer the robot, working with the gyroscope data as feedback to the PID. 

11/03/2021: Added an OpenCV MOG2 background substractor and a tracker, currently using a CSRT but it could be easily replaced, the background substractor detects movement and focuses on the biggest area to pass that rectangle to the tracker so it gets initialized, it works. FPS are rubbish but still is able to follow slow motion objects e.g passing hand.

14/03/2021: The robot is finally able to follow the detected object, the magic is broken when the CSRT threshold is crossed and from that point the robot falls back to motion detection until it finds a new target to track, in the mean time the robot stops.

28/03/2021: Added an embedded web server based on Mongoose, now the robot streams video straight from its camera marking (blue rectangle) the tracked object. Unfortunately the workload finally broke the camels back and it starts to show that the Rasperry 4 is just too short, future potential optimization is required, probably switch from a CSRT to a MOOSE tracker.

10/04/2021: Probably last version, some cleaning, closing threads properly and tidying up in general. Kept the CSRT tracker and solved some recurrent exceptions thrown by OpenCV, mostly to do with capturing out of frame.

The robot itself can be found here: https://www.adeept.com/adeept-darkpaw-bionic-quadruped-spider-robot-kit-for-raspberry-pi-4-3-model-b-b-2b-stem-crawling-robot-opencv-tracking-self-stabilizing_p0125_s0035.html

It is a cool toy and the provided HAT supports connecting some additional stuff if you wanted too, I equipped it with a Raspberry Pi 4 with 8 Gigas so it is (almost) enough for the OpenCV stuff, as a basis to control the GPIO I am using the famous PIGPIO library wich always works first time, really good stuff that can be found here: http://abyz.me.uk/rpi/pigpio/

Embedded web server is built using the famous Mongoose, version 7.2 that can be obtained here: https://github.com/cesanta/mongoose/releases

OpenCV 4.5.0 (or any other version) can be obtained here: https://github.com/opencv/opencv

[<h2 align="left">First Version</h2>](https://github.com/Rubberazer/Darkpaw_motion_opencv/tree/main/First_version)

  Detects moving objects and once detected it tracks (and follows) them
  
  Files: 2_darkpaw.cpp, makefile
  
  Compile: type 'make', pigpio and opencv libraries should be installed previously
  Execute: sudo ./darkpaw

[<h2 align="left">Second Version</h2>](https://github.com/Rubberazer/Darkpaw_motion_opencv/tree/main/Second_version)
  
  Detects moving objects and once detected it tracks (and follows) them, this version also incorporates a web server for video streaming.
  
  Files: 3_darkpaw.cpp, makefile, web_root folder
  
  Compile: type 'make', pigpio and opencv libraries should be installed previously, also mongoose.c and mongoose.h are required 
  
  Execute: sudo ./darkpaw
 
 [<h2 align="left">Latest @ main</h2>](https://github.com/Rubberazer/Darkpaw_motion_opencv)
  
  Same functionality as 2 but terminating threads properly, cleaning up comments and tidying up a bit, all files hanging from 'main' in the repository
  
  Files: 4_darkpaw.cpp, makefile, web_root folder
  
  Compile: type 'make', pigpio and opencv libraries should be installed previously, also mongoose.c and mongoose.h are required 
  
  Execute: sudo ./darkpaw
