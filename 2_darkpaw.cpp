#include <iostream>	/* for standard I/O in C++ */
#include <cstdio>	/* for printf, cstdio in C++ */
#include <cstdint>	/* for uint64 definition */
#include <cstdlib>	/* for exit() definition */
#define _BSD_SOURCE	/* this one makes time.h to work properly */
#include <sys/time.h>	/* for clock_gettime */
#include <cmath>	/* for mathematical funtions, cmath in C++ */
#include <pthread.h>    /* for threading out loud*/
#include <pigpio.h>     /* for handling the GPIO */
#include <csignal>	/* for catching exceptions e.g. control-C, csignal in C++ */
#include <unistd.h>	/* this one is to make usleep() work */
#include <opencv2/core/core.hpp>	/*this one and the ones that follow are the opencv stuff */
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

/*The following structure defines all the inputs from th sensorial parts , it is meant to act 
as a global variable for the threads*/

typedef struct
{
  double distance; /*distance to the obstacle from the ultrasounds sensor*/
  int ultrimpct; /*near frontal impact to the wall, from the ultrasounds */ 
  int block; /* this is to avoid the robot from getting stuck in obstacles not detected by the sensors */
  int servos; /*this one is the PCA9685 handler returned by I2Copen() */
  int global_position; /*this one is to store the legs global position from 1 to 8 */
  
} PARAM;

typedef struct
{
  int FLB_pulse_old;
  int FLM_pulse_old;
  int FLE_pulse_old;
  int HLB_pulse_old;
  int HLM_pulse_old;
  int HLE_pulse_old;
  int FRB_pulse_old;
  int FRM_pulse_old;
  int FRE_pulse_old;
  int HRB_pulse_old;
  int HRM_pulse_old;
  int HRE_pulse_old;
  int servos;
  
} SERVOS_PARAMETERS;

/* Define globally accessible variables no mutex */
PARAM parameters;
/* Define globally accessible variables no mutex for exclusive use of the servos*/
SERVOS_PARAMETERS parameters_servo;

/* This global variable is used to interrupt the infite while loops with signaction()
 this is important as if not used and trying to stop the program with Ctrl-c,
 the robot might go full speed against the  wall (it happened)*/
static volatile int interrupt = 1;

/* the next function is the signal() function handler, is critical to 
   avoid damage to the robot*/

void inthandler(int signum) 
{
  interrupt = 0;
  i2cWriteByteData(parameters_servo.servos, 0xFD, 0x10); /* Shutting down all channels */
  i2cWriteByteData(parameters_servo.servos, 0x00, 0x00); /* Reset */
  i2cClose(parameters_servo.servos);
  gpioTerminate();
  printf("Caught signal %d, coming out ...\n", signum);
  exit(1);
}

/* This function below is for the ultrasounds sensor */

void *Distance(void *arg)
{
  const int TRIGGER = 11; /*BCM pin 11, sending ultrasound cry*/
  const int ECHO = 8; /*BCM pin 8, receiving ultrasound*/
  gpioSetMode(ECHO, PI_INPUT); 
  gpioSetMode(TRIGGER, PI_OUTPUT); 
  struct timeval t1, t2, t3;
  double time_interval,distance, distbuff;
  int count;
  char s[100];
  time_t now;
  struct tm *t;
  FILE *f;

  f = fopen("record.csv", "w+");
  if (f == NULL)
    {
      printf("Error opening file!\n");
      exit(1);
    }
  while (interrupt)
    {
      time_interval = 0;
      distance = 0;
      gettimeofday(&t3, NULL);
      gpioWrite(TRIGGER, 1);
      usleep(15);
      gpioWrite(TRIGGER, 0);

    while (!gpioRead(ECHO))
      {
	gettimeofday(&t1, NULL);
	gettimeofday(&t2, NULL);
	time_interval = (t2.tv_sec-t3.tv_sec) * 1000000; 
	time_interval = (time_interval + (t2.tv_usec-t2.tv_usec))/1000000; 
	if (time_interval > 0.05)
	  break;
      }

    while (gpioRead(ECHO))
      {
	gettimeofday(&t2, NULL);
	if (time_interval > 0.05)
	  break;
      }
    if (time_interval < 0.05)
      {
	time_interval = (t2.tv_sec-t1.tv_sec) * 1000000; 
	time_interval = (time_interval + (t2.tv_usec-t1.tv_usec))/1000000; 
	distance = time_interval*17150;
	parameters.distance = distance;
	if ((abs(distance - distbuff)) < 2)
	  {
	    count++;	
	  }
	if (count>7)
	  {
	    parameters.block = 1;
	    count = 0;
	  }
	else
	  {
	    parameters.block = 0;
	  }
	distbuff = distance;
	if(distance<25) 
	  {
	    parameters.ultrimpct = 1;
	  }
	else
	  {
	    parameters.ultrimpct = 0;
	  }
	time(&now);
	t = localtime(&now);
	strftime(s, 100, "%H:%M:%S", t);
	fprintf(f,"%s, %f\n",s ,parameters.distance);
      }

      usleep(250000);
    }
  fclose(f);
  pthread_exit(NULL);
}
/* Next thread will manage the opencv stuff */

void *Camera(void *arg)
{
  while(interrupt)
    {
      Mat frame;
    //--- INITIALIZE VIDEOCAPTURE
    VideoCapture cap;
    // open the default camera using default API
    // cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID, apiID);
    // check if we succeeded
    if (!cap.isOpened()) 
      {
        cerr << "ERROR! Unable to open camera\n";
      }
    //--- GRAB AND WRITE LOOP
    cout << "Start grabbing" << endl
        << "Press any key to terminate" << endl;
    for (;;)
      {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        // show live and wait for a key with timeout long enough to show images
        imshow("Live", frame);
        if (waitKey(5) >= 0)
            break;
      }
    }
  pthread_exit(NULL);
}

/* Pigpio library initialization */

void Init_Pigpio(void)
{
  int Init; 
  Init = gpioInitialise();
  if (Init < 0)
    {
      /* pigpio initialisation failed */
      printf("Pigpio initialisation failed. Error code:  %d\n", Init);
      exit(Init);
    }
  else
    {
      /* pigpio initialised okay*/
      printf("Pigpio initialisation OK. Return code:  %d\n", Init);
    }
}

/* Next function is to create a connection to and initialize the PCA9685 drive */

int Restart_PCA9685(void)

{
  int Init =0;
  int oldmode;
  int newmode;
  int servos;
  servos = i2cOpen(1,0x40,0);
    if (servos >=  0)
    {
      /* PI connection to I2C slave 40 OK*/
      printf("Open I2C to slave 0x40 OK. Return code:  %d\n", Init);
      printf("PCA9685 number handler:  %d\n", servos);
    }
  else
    {
      /* No PI connection to I2C slave 40 */
      printf("Open I2C to slave 0x40 failed. Error code:  %d\n", Init);
      exit(servos);
    }
  oldmode = i2cReadByteData(servos, 0x00); /* getting current mode */
  newmode = (oldmode & 0xEF); /* wake up definition */
  i2cWriteByteData(servos, 0x00, newmode); /* wake up in case */
  usleep(5000);
  oldmode = i2cReadByteData(servos, 0x00); /* getting current mode */
  i2cWriteByteData(servos, 0x00, oldmode | 0x80); /* restart */
  return(servos);
}

/*Setting up the servo drive here (frequency) */

void Init_PCA9685(int servos) 
{
  float freq;
  int oldmode;
  int newmode;
  
  /* Setting the PCA9685 frequency, must be 50Hz for the SG-90 servos */
  //i2cWriteByteData(servo, 0x00, 0x00); /* Resetting the PCA9685 first thing */
	
  freq = (25000000/(4096*50)) - 0.5; /* now 25*10^6 is 25Mhz of the internal clock, 4096 is 12 bit resolution and 50hz is the wanted frequency setup) */
  freq = (int) freq;
  /* now there is a whole sequence to set up the frequency */

  oldmode = i2cReadByteData(servos, 0x00); /* getting current mode */
  newmode = (oldmode & 0x7F) | 0x10; /* sleep mode definition */
  i2cWriteByteData(servos, 0x00, newmode); /* going to sleep now */
  i2cWriteByteData(servos, 0xFE, freq); /* setting up the frequency now */
  i2cWriteByteData(servos, 0x00, oldmode); /* coming back to the old mode */
  usleep(5000);
  i2cWriteByteData(servos, 0x00, oldmode | 0x80); /* final step on frequency set up */
}

/* Next function will put all hte servos to its central position */

void Centre(int servos_handler)

{
  int i;
  float pulse_1;
  int pulse;
  int servos;
  servos = servos_handler;
  /* These ones are to calculate the central postion common to all of them */
  pulse_1 = 1500; /* 1500 should be the centered position, 1000 is up and 2000 is down */
  pulse_1 = (pulse_1*4096)/20000; /* this ends up being 307 so maybe more handy to use that number directly */
  pulse = (int) pulse_1;
  
  /* Looping through the 12 servos - from 0 to 11, servo n 0 four registers are: 0x06, 0x07, 0x08, 0x09*/
  //current = i2cReadByteData(servos, 0x00); /* getting current mode */
  //printf("MODE1 register status now: %d\n",current);
  
  for(i=0;i<12;i++) 
    {
      i2cWriteByteData(servos, (4*i)+6, 0x00);
      i2cWriteByteData(servos, (4*i)+7, 0x00);
      i2cWriteByteData(servos, (4*i)+8, pulse & 0xFF);
      i2cWriteByteData(servos, (4*i)+9, pulse >> 8);
      usleep(5000); 
    } 
  parameters_servo.FLB_pulse_old = 307;
  parameters_servo.FLM_pulse_old = 307;
  parameters_servo.FLE_pulse_old = 307;
  parameters_servo.HLB_pulse_old = 307;
  parameters_servo.HLM_pulse_old = 307;
  parameters_servo.HLE_pulse_old = 307;
  parameters_servo.FRB_pulse_old = 307;
  parameters_servo.FRM_pulse_old = 307;
  parameters_servo.FRE_pulse_old = 307;
  parameters_servo.HRB_pulse_old = 307;
  parameters_servo.HRM_pulse_old = 307;
  parameters_servo.HRE_pulse_old = 307;
  
}

/* Next function moves the robot forward */

void Forward(int servos_handler)

{
  int i;
  int FLB_pulse_old = parameters_servo.FLB_pulse_old;
  int FLM_pulse_old = parameters_servo.FLM_pulse_old;
  int FLE_pulse_old = parameters_servo.FLE_pulse_old;
  int HLB_pulse_old = parameters_servo.HLB_pulse_old;
  int HLM_pulse_old = parameters_servo.HLM_pulse_old;
  int HLE_pulse_old = parameters_servo.HLE_pulse_old;
  int FRB_pulse_old = parameters_servo.FRB_pulse_old;
  int FRM_pulse_old = parameters_servo.FRM_pulse_old;
  int FRE_pulse_old = parameters_servo.FRE_pulse_old;
  int HRB_pulse_old = parameters_servo.HRB_pulse_old;
  int HRM_pulse_old = parameters_servo.HRM_pulse_old;
  int HRE_pulse_old = parameters_servo.HRE_pulse_old;
  float FLB_pulse_now;
  float FLM_pulse_now;
  float FLE_pulse_now;
  float HLB_pulse_now;
  float HLM_pulse_now;
  float HLE_pulse_now;
  float FRB_pulse_now;
  float FRM_pulse_now;
  float FRE_pulse_now;
  float HRB_pulse_now;
  float HRM_pulse_now;
  float HRE_pulse_now;
  int global_position;
  int servos;
  float FLB_pulse;
  float FLM_pulse;
  float FLE_pulse;
  float FRB_pulse;
  float FRM_pulse;
  float FRE_pulse;
  float HLB_pulse;
  float HLM_pulse;
  float HLE_pulse;
  float HRB_pulse;
  float HRM_pulse;
  float HRE_pulse;
  int FLB_direction = 1;
  int FLM_direction = -1;
  int FLE_direction = -1;
  int FRB_direction = -1;
  int FRM_direction = 1;
  int FRE_direction = 1;
  int HLB_direction = -1;
  int HLM_direction = 1;
  int HLE_direction = 1;
  int HRB_direction = 1;
  int HRM_direction = -1;
  int HRE_direction = -1;
  int wiggle_h = 120;
  int wiggle_v = 200;
  int wiggle_middle = 30;
  servos = servos_handler;
  //global_position = parameters.global_position;
  
  /* Going from position 1 to 8 to walk forward*/
  
    for(global_position=parameters.global_position;global_position<9;global_position++) 
    {
      /* Calculate the pulse to send to each servomotor and determine each leg position*/
      parameters.global_position=global_position;

      switch (global_position)
      {
	case 1:
	  FLB_pulse = 307 + wiggle_middle*FLB_direction;
	  FLM_pulse = 307 + wiggle_v*FLM_direction;
	  FLE_pulse = 307 + wiggle_v*FLE_direction;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(3-2))/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(5-2))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(7-2))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 2:
	  FLB_pulse = 307 + (wiggle_middle + wiggle_h)*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(4-2))/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(6-2))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(8-2))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 3:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(3-2))/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(5-2))/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(7-2))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle)*HRB_direction;
	  HRM_pulse = 307 + wiggle_v*HRM_direction;
	  HRE_pulse = 307 + wiggle_v*HRE_direction;
	  break;
	  
	case 4:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(4-2))/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(6-2))/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(8-2))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + wiggle_h)*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 5:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(5-2))/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(7-2))/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + wiggle_middle*FRB_direction;
	  FRM_pulse = 307 + wiggle_v*FRM_direction;
	  FRE_pulse = 307 + wiggle_v*FRE_direction;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(3-2))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 6:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(6-2))/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(8-2))/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + wiggle_h)*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(4-2))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 7:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(7-2))/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle)*HLB_direction;
	  HLM_pulse = 307 + wiggle_v*HLM_direction;
	  HLE_pulse = 307 + wiggle_v*HLE_direction;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(3-2))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(5-2))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 8:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(8-2))/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + wiggle_h)*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(4-2))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(6-2))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;  
      }
      
      for(i=1;i<5;i++) 
      {
	FLB_pulse_now = FLB_pulse_old+(FLB_pulse-FLB_pulse_old)*i/4;
	FLM_pulse_now = FLM_pulse_old+(FLM_pulse-FLM_pulse_old)*i/4;
	FLE_pulse_now = FLE_pulse_old+(FLE_pulse-FLE_pulse_old)*i/4;
	HLB_pulse_now = HLB_pulse_old+(HLB_pulse-HLB_pulse_old)*i/4;
	HLM_pulse_now = HLM_pulse_old+(HLM_pulse-HLM_pulse_old)*i/4;
	HLE_pulse_now = HLE_pulse_old+(HLE_pulse-HLE_pulse_old)*i/4;
	FRB_pulse_now = FRB_pulse_old+(FRB_pulse-FRB_pulse_old)*i/4;
	FRM_pulse_now = FRM_pulse_old+(FRM_pulse-FRM_pulse_old)*i/4;
	FRE_pulse_now = FRE_pulse_old+(FRE_pulse-FRE_pulse_old)*i/4;
	HRB_pulse_now = HRB_pulse_old+(HRB_pulse-HRB_pulse_old)*i/4;
	HRM_pulse_now = HRM_pulse_old+(HRM_pulse-HRM_pulse_old)*i/4;
	HRE_pulse_now = HRE_pulse_old+(HRE_pulse-HRE_pulse_old)*i/4;
	
	/* Next 3 code blocks are for the front left leg (LF) number 1 */
	i2cWriteByteData(servos, 6, 0x00);
	i2cWriteByteData(servos, 7, 0x00);
	i2cWriteByteData(servos, 8, (int)FLB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 9, (int)FLB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 10, 0x00);
	i2cWriteByteData(servos, 11, 0x00);
	i2cWriteByteData(servos, 12, (int)FLM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 13, (int)FLM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 14, 0x00);
	i2cWriteByteData(servos, 15, 0x00);
	i2cWriteByteData(servos, 16, (int)FLE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 17, (int)FLE_pulse_now >> 8);
	usleep(500);
	/* Next 3 code blocks are for the front right leg (FR) number 3 */
	i2cWriteByteData(servos, 30, 0x00);
	i2cWriteByteData(servos, 31, 0x00);
	i2cWriteByteData(servos, 32, (int)FRB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 33, (int)FRB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 34, 0x00);
	i2cWriteByteData(servos, 35, 0x00);
	i2cWriteByteData(servos, 36, (int)FRM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 37, (int)FRM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 38, 0x00);
	i2cWriteByteData(servos, 39, 0x00);
	i2cWriteByteData(servos, 40, (int)FRE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 41, (int)FRE_pulse_now >> 8);
	usleep(500);
	/* Next 3 code blocks are for the front hind left leg (HL) number 2 */
	i2cWriteByteData(servos, 18, 0x00);
	i2cWriteByteData(servos, 19, 0x00);
	i2cWriteByteData(servos, 20, (int)HLB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 21, (int)HLB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 22, 0x00);
	i2cWriteByteData(servos, 23, 0x00);
	i2cWriteByteData(servos, 24, (int)HLM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 25, (int)HLM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 26, 0x00);
	i2cWriteByteData(servos, 27, 0x00);
	i2cWriteByteData(servos, 28, (int)HLE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 29, (int)HLE_pulse_now >> 8);
	usleep(500);
	/* Next 3 code blocks are for the front hind right leg (HR) number 4 */
	i2cWriteByteData(servos, 42, 0x00);
	i2cWriteByteData(servos, 43, 0x00);
	i2cWriteByteData(servos, 44, (int)HRB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 45, (int)HRB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 46, 0x00);
	i2cWriteByteData(servos, 47, 0x00);
	i2cWriteByteData(servos, 48, (int)HRM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 49, (int)HRM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 50, 0x00);
	i2cWriteByteData(servos, 51, 0x00);
	i2cWriteByteData(servos, 52, (int)HRE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 53, (int)HRE_pulse_now >> 8);
	usleep(500);
      }
      
      FLB_pulse_old = (int)FLB_pulse;
      FLM_pulse_old = (int)FLM_pulse;
      FLE_pulse_old = (int)FLE_pulse;
      HLB_pulse_old = (int)HLB_pulse;
      HLM_pulse_old = (int)HLM_pulse;
      HLE_pulse_old = (int)HLE_pulse;
      FRB_pulse_old = (int)FRB_pulse;
      FRM_pulse_old = (int)FRM_pulse;
      FRE_pulse_old = (int)FRE_pulse;
      HRB_pulse_old = (int)HRB_pulse;
      HRM_pulse_old = (int)HRM_pulse;
      HRE_pulse_old = (int)HRE_pulse;
      
      parameters_servo.FLB_pulse_old = (int)FLB_pulse;
      parameters_servo.FLM_pulse_old = (int)FLM_pulse;
      parameters_servo.FLE_pulse_old = (int)FLE_pulse;
      parameters_servo.HLB_pulse_old = (int)HLB_pulse;
      parameters_servo.HLM_pulse_old = (int)HLM_pulse;
      parameters_servo.HLE_pulse_old = (int)HLE_pulse;
      parameters_servo.FRB_pulse_old = (int)FRB_pulse;
      parameters_servo.FRM_pulse_old = (int)FRM_pulse;
      parameters_servo.FRE_pulse_old = (int)FRE_pulse;
      parameters_servo.HRB_pulse_old = (int)HRB_pulse;
      parameters_servo.HRM_pulse_old = (int)HRM_pulse;
      parameters_servo.HRE_pulse_old = (int)HRE_pulse;
      
      usleep(10000);
    }   
parameters.global_position = 1;  

}

/* Next function will move the robot backwards */

void Backward(int servos_handler)

{
  int i;
  int FLB_pulse_old = parameters_servo.FLB_pulse_old;
  int FLM_pulse_old = parameters_servo.FLM_pulse_old;
  int FLE_pulse_old = parameters_servo.FLE_pulse_old;
  int HLB_pulse_old = parameters_servo.HLB_pulse_old;
  int HLM_pulse_old = parameters_servo.HLM_pulse_old;
  int HLE_pulse_old = parameters_servo.HLE_pulse_old;
  int FRB_pulse_old = parameters_servo.FRB_pulse_old;
  int FRM_pulse_old = parameters_servo.FRM_pulse_old;
  int FRE_pulse_old = parameters_servo.FRE_pulse_old;
  int HRB_pulse_old = parameters_servo.HRB_pulse_old;
  int HRM_pulse_old = parameters_servo.HRM_pulse_old;
  int HRE_pulse_old = parameters_servo.HRE_pulse_old;
  float FLB_pulse_now;
  float FLM_pulse_now;
  float FLE_pulse_now;
  float HLB_pulse_now;
  float HLM_pulse_now;
  float HLE_pulse_now;
  float FRB_pulse_now;
  float FRM_pulse_now;
  float FRE_pulse_now;
  float HRB_pulse_now;
  float HRM_pulse_now;
  float HRE_pulse_now;
  int global_position;
  int servos;
  float FLB_pulse;
  float FLM_pulse;
  float FLE_pulse;
  float FRB_pulse;
  float FRM_pulse;
  float FRE_pulse;
  float HLB_pulse;
  float HLM_pulse;
  float HLE_pulse;
  float HRB_pulse;
  float HRM_pulse;
  float HRE_pulse;
  int FLB_direction = 1;
  int FLM_direction = -1;
  int FLE_direction = -1;
  int FRB_direction = -1;
  int FRM_direction = 1;
  int FRE_direction = 1;
  int HLB_direction = -1;
  int HLM_direction = 1;
  int HLE_direction = 1;
  int HRB_direction = 1;
  int HRM_direction = -1;
  int HRE_direction = -1;
  int wiggle_h = 120;
  int wiggle_v = 200;
  int wiggle_middle = 30;
  servos = servos_handler;
  //global_position = parameters.global_position;
  
  /* Going from position 8 to 1 to walk backwards*/
  
    for(global_position=parameters.global_position;global_position>0;global_position--) 
    {
      /* Calculate the pulse to send to each servomotor and determine each leg position*/
      parameters.global_position=global_position;

      switch (global_position)
      {
	case 1:
	  FLB_pulse = 307 + wiggle_middle*FLB_direction;
	  FLM_pulse = 307 + wiggle_v*FLM_direction;
	  FLE_pulse = 307 + wiggle_v*FLE_direction;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(1))/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(3))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(5))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 2:
	  FLB_pulse = 307 + (wiggle_middle + wiggle_h)*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(2))/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(4))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(6))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 3:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(1))/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(3))/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(5))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle)*HRB_direction;
	  HRM_pulse = 307 + wiggle_v*HRM_direction;
	  HRE_pulse = 307 + wiggle_v*HRE_direction;
	  break;
	  
	case 4:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(2))/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(4))/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(6))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + wiggle_h)*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 5:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(3))/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(5))/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + wiggle_middle*FRB_direction;
	  FRM_pulse = 307 + wiggle_v*FRM_direction;
	  FRE_pulse = 307 + wiggle_v*FRE_direction;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(1))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 6:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(4))/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(6))/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + wiggle_h)*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(2))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 7:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(5))/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle)*HLB_direction;
	  HLM_pulse = 307 + wiggle_v*HLM_direction;
	  HLE_pulse = 307 + wiggle_v*HLE_direction;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(1))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(3))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 8:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(6))/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + wiggle_h)*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(2))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(4))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;  
      }
      
      for(i=1;i<5;i++) 
      {
	FLB_pulse_now = FLB_pulse_old+(FLB_pulse-FLB_pulse_old)*i/4;
	FLM_pulse_now = FLM_pulse_old+(FLM_pulse-FLM_pulse_old)*i/4;
	FLE_pulse_now = FLE_pulse_old+(FLE_pulse-FLE_pulse_old)*i/4;
	HLB_pulse_now = HLB_pulse_old+(HLB_pulse-HLB_pulse_old)*i/4;
	HLM_pulse_now = HLM_pulse_old+(HLM_pulse-HLM_pulse_old)*i/4;
	HLE_pulse_now = HLE_pulse_old+(HLE_pulse-HLE_pulse_old)*i/4;
	FRB_pulse_now = FRB_pulse_old+(FRB_pulse-FRB_pulse_old)*i/4;
	FRM_pulse_now = FRM_pulse_old+(FRM_pulse-FRM_pulse_old)*i/4;
	FRE_pulse_now = FRE_pulse_old+(FRE_pulse-FRE_pulse_old)*i/4;
	HRB_pulse_now = HRB_pulse_old+(HRB_pulse-HRB_pulse_old)*i/4;
	HRM_pulse_now = HRM_pulse_old+(HRM_pulse-HRM_pulse_old)*i/4;
	HRE_pulse_now = HRE_pulse_old+(HRE_pulse-HRE_pulse_old)*i/4;
	
	/* Next 3 code blocks are for the front left leg (LF) number 1 */
	i2cWriteByteData(servos, 6, 0x00);
	i2cWriteByteData(servos, 7, 0x00);
	i2cWriteByteData(servos, 8, (int)FLB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 9, (int)FLB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 10, 0x00);
	i2cWriteByteData(servos, 11, 0x00);
	i2cWriteByteData(servos, 12, (int)FLM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 13, (int)FLM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 14, 0x00);
	i2cWriteByteData(servos, 15, 0x00);
	i2cWriteByteData(servos, 16, (int)FLE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 17, (int)FLE_pulse_now >> 8);
	usleep(500);
	/* Next 3 code blocks are for the front right leg (FR) number 3 */
	i2cWriteByteData(servos, 30, 0x00);
	i2cWriteByteData(servos, 31, 0x00);
	i2cWriteByteData(servos, 32, (int)FRB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 33, (int)FRB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 34, 0x00);
	i2cWriteByteData(servos, 35, 0x00);
	i2cWriteByteData(servos, 36, (int)FRM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 37, (int)FRM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 38, 0x00);
	i2cWriteByteData(servos, 39, 0x00);
	i2cWriteByteData(servos, 40, (int)FRE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 41, (int)FRE_pulse_now >> 8);
	usleep(500);
	/* Next 3 code blocks are for the front hind left leg (HL) number 2 */
	i2cWriteByteData(servos, 18, 0x00);
	i2cWriteByteData(servos, 19, 0x00);
	i2cWriteByteData(servos, 20, (int)HLB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 21, (int)HLB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 22, 0x00);
	i2cWriteByteData(servos, 23, 0x00);
	i2cWriteByteData(servos, 24, (int)HLM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 25, (int)HLM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 26, 0x00);
	i2cWriteByteData(servos, 27, 0x00);
	i2cWriteByteData(servos, 28, (int)HLE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 29, (int)HLE_pulse_now >> 8);
	usleep(500);
	/* Next 3 code blocks are for the front hind right leg (HR) number 4 */
	i2cWriteByteData(servos, 42, 0x00);
	i2cWriteByteData(servos, 43, 0x00);
	i2cWriteByteData(servos, 44, (int)HRB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 45, (int)HRB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 46, 0x00);
	i2cWriteByteData(servos, 47, 0x00);
	i2cWriteByteData(servos, 48, (int)HRM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 49, (int)HRM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 50, 0x00);
	i2cWriteByteData(servos, 51, 0x00);
	i2cWriteByteData(servos, 52, (int)HRE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 53, (int)HRE_pulse_now >> 8);
	usleep(500);
      }
      
      FLB_pulse_old = (int)FLB_pulse;
      FLM_pulse_old = (int)FLM_pulse;
      FLE_pulse_old = (int)FLE_pulse;
      HLB_pulse_old = (int)HLB_pulse;
      HLM_pulse_old = (int)HLM_pulse;
      HLE_pulse_old = (int)HLE_pulse;
      FRB_pulse_old = (int)FRB_pulse;
      FRM_pulse_old = (int)FRM_pulse;
      FRE_pulse_old = (int)FRE_pulse;
      HRB_pulse_old = (int)HRB_pulse;
      HRM_pulse_old = (int)HRM_pulse;
      HRE_pulse_old = (int)HRE_pulse;
      
      parameters_servo.FLB_pulse_old = (int)FLB_pulse;
      parameters_servo.FLM_pulse_old = (int)FLM_pulse;
      parameters_servo.FLE_pulse_old = (int)FLE_pulse;
      parameters_servo.HLB_pulse_old = (int)HLB_pulse;
      parameters_servo.HLM_pulse_old = (int)HLM_pulse;
      parameters_servo.HLE_pulse_old = (int)HLE_pulse;
      parameters_servo.FRB_pulse_old = (int)FRB_pulse;
      parameters_servo.FRM_pulse_old = (int)FRM_pulse;
      parameters_servo.FRE_pulse_old = (int)FRE_pulse;
      parameters_servo.HRB_pulse_old = (int)HRB_pulse;
      parameters_servo.HRM_pulse_old = (int)HRM_pulse;
      parameters_servo.HRE_pulse_old = (int)HRE_pulse;
      
      usleep(10000);
    }   
parameters.global_position = 8;  

}

/* Next function will turn the robot to the left */

void Left(int servos_handler)

{
  int i;
  int FLB_pulse_old = parameters_servo.FLB_pulse_old;
  int FLM_pulse_old = parameters_servo.FLM_pulse_old;
  int FLE_pulse_old = parameters_servo.FLE_pulse_old;
  int HLB_pulse_old = parameters_servo.HLB_pulse_old;
  int HLM_pulse_old = parameters_servo.HLM_pulse_old;
  int HLE_pulse_old = parameters_servo.HLE_pulse_old;
  int FRB_pulse_old = parameters_servo.FRB_pulse_old;
  int FRM_pulse_old = parameters_servo.FRM_pulse_old;
  int FRE_pulse_old = parameters_servo.FRE_pulse_old;
  int HRB_pulse_old = parameters_servo.HRB_pulse_old;
  int HRM_pulse_old = parameters_servo.HRM_pulse_old;
  int HRE_pulse_old = parameters_servo.HRE_pulse_old;
  float FLB_pulse_now;
  float FLM_pulse_now;
  float FLE_pulse_now;
  float HLB_pulse_now;
  float HLM_pulse_now;
  float HLE_pulse_now;
  float FRB_pulse_now;
  float FRM_pulse_now;
  float FRE_pulse_now;
  float HRB_pulse_now;
  float HRM_pulse_now;
  float HRE_pulse_now;
  int global_position;
  int servos;
  float FLB_pulse;
  float FLM_pulse;
  float FLE_pulse;
  float FRB_pulse;
  float FRM_pulse;
  float FRE_pulse;
  float HLB_pulse;
  float HLM_pulse;
  float HLE_pulse;
  float HRB_pulse;
  float HRM_pulse;
  float HRE_pulse;
  int FLB_direction = 1;
  int FLM_direction = -1;
  int FLE_direction = -1;
  int FRB_direction = -1;
  int FRM_direction = 1;
  int FRE_direction = 1;
  int HLB_direction = -1;
  int HLM_direction = 1;
  int HLE_direction = 1;
  int HRB_direction = 1;
  int HRM_direction = -1;
  int HRE_direction = -1;
  int wiggle_h = 120;
  int wiggle_v = 200;
  int wiggle_middle = 30;
  servos = servos_handler;
  //global_position = parameters.global_position;
  
  /* Going from position 1 to 4 to turn left*/
  
    for(global_position=parameters.global_position;global_position<5;global_position++) 
    {
      /* Calculate the pulse to send to each servomotor and determine each leg position*/
      parameters.global_position=global_position;

      switch (global_position)
      {
	case 1: 
	  FLB_pulse = 307 + wiggle_middle*FLB_direction;
	  FLM_pulse = 307 + wiggle_v*FLM_direction;
	  FLE_pulse = 307 + wiggle_v*FLE_direction;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(5-2))/3 - wiggle_h)*(-1))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(5-2))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle)*HRB_direction;
	  HRM_pulse = 307 + wiggle_v*HRM_direction;
	  HRE_pulse = 307 + wiggle_v*HRE_direction;
	  break;
	
	case 2: 	    
	  FLB_pulse = 307 + (wiggle_middle + wiggle_h*(-1))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(8-2))/3 - wiggle_h)*(-1))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(8-2))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + wiggle_h)*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 3: 	
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(5-2))/3 - wiggle_h)*(-1))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307;
	  HLB_pulse = 307 + (-wiggle_middle)*HLB_direction;
	  HLM_pulse = 307 + wiggle_v*HLM_direction;
	  HLE_pulse = 307 + wiggle_v*HLE_direction;
	  FRB_pulse = 307 + wiggle_middle*FRB_direction;
	  FRM_pulse = 307 + wiggle_v*FRM_direction;
	  FRE_pulse = 307 + wiggle_v*FRE_direction;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(5-2))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 4: 	
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(8-2))/3 - wiggle_h)*(-1))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + wiggle_h*(-1))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + wiggle_h)*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 +(-wiggle_middle + (wiggle_h*(6-(8-2))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
      }
      
      for(i=1;i<5;i++) 
      {
	FLB_pulse_now = FLB_pulse_old+(FLB_pulse-FLB_pulse_old)*i/4;
	FLM_pulse_now = FLM_pulse_old+(FLM_pulse-FLM_pulse_old)*i/4;
	FLE_pulse_now = FLE_pulse_old+(FLE_pulse-FLE_pulse_old)*i/4;
	HLB_pulse_now = HLB_pulse_old+(HLB_pulse-HLB_pulse_old)*i/4;
	HLM_pulse_now = HLM_pulse_old+(HLM_pulse-HLM_pulse_old)*i/4;
	HLE_pulse_now = HLE_pulse_old+(HLE_pulse-HLE_pulse_old)*i/4;
	FRB_pulse_now = FRB_pulse_old+(FRB_pulse-FRB_pulse_old)*i/4;
	FRM_pulse_now = FRM_pulse_old+(FRM_pulse-FRM_pulse_old)*i/4;
	FRE_pulse_now = FRE_pulse_old+(FRE_pulse-FRE_pulse_old)*i/4;
	HRB_pulse_now = HRB_pulse_old+(HRB_pulse-HRB_pulse_old)*i/4;
	HRM_pulse_now = HRM_pulse_old+(HRM_pulse-HRM_pulse_old)*i/4;
	HRE_pulse_now = HRE_pulse_old+(HRE_pulse-HRE_pulse_old)*i/4;
	
	/* Next 3 code blocks are for the front left leg (LF) number 1 */
	i2cWriteByteData(servos, 6, 0x00);
	i2cWriteByteData(servos, 7, 0x00);
	i2cWriteByteData(servos, 8, (int)FLB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 9, (int)FLB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 10, 0x00);
	i2cWriteByteData(servos, 11, 0x00);
	i2cWriteByteData(servos, 12, (int)FLM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 13, (int)FLM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 14, 0x00);
	i2cWriteByteData(servos, 15, 0x00);
	i2cWriteByteData(servos, 16, (int)FLE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 17, (int)FLE_pulse_now >> 8);
	usleep(500);
	/* Next 3 code blocks are for the front right leg (FR) number 3 */
	i2cWriteByteData(servos, 30, 0x00);
	i2cWriteByteData(servos, 31, 0x00);
	i2cWriteByteData(servos, 32, (int)FRB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 33, (int)FRB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 34, 0x00);
	i2cWriteByteData(servos, 35, 0x00);
	i2cWriteByteData(servos, 36, (int)FRM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 37, (int)FRM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 38, 0x00);
	i2cWriteByteData(servos, 39, 0x00);
	i2cWriteByteData(servos, 40, (int)FRE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 41, (int)FRE_pulse_now >> 8);
	usleep(500);
	/* Next 3 code blocks are for the front hind left leg (HL) number 2 */
	i2cWriteByteData(servos, 18, 0x00);
	i2cWriteByteData(servos, 19, 0x00);
	i2cWriteByteData(servos, 20, (int)HLB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 21, (int)HLB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 22, 0x00);
	i2cWriteByteData(servos, 23, 0x00);
	i2cWriteByteData(servos, 24, (int)HLM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 25, (int)HLM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 26, 0x00);
	i2cWriteByteData(servos, 27, 0x00);
	i2cWriteByteData(servos, 28, (int)HLE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 29, (int)HLE_pulse_now >> 8);
	usleep(500);
	/* Next 3 code blocks are for the front hind right leg (HR) number 4 */
	i2cWriteByteData(servos, 42, 0x00);
	i2cWriteByteData(servos, 43, 0x00);
	i2cWriteByteData(servos, 44, (int)HRB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 45, (int)HRB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 46, 0x00);
	i2cWriteByteData(servos, 47, 0x00);
	i2cWriteByteData(servos, 48, (int)HRM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 49, (int)HRM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 50, 0x00);
	i2cWriteByteData(servos, 51, 0x00);
	i2cWriteByteData(servos, 52, (int)HRE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 53, (int)HRE_pulse_now >> 8);
	usleep(500);
      }
      
      FLB_pulse_old = (int)FLB_pulse;
      FLM_pulse_old = (int)FLM_pulse;
      FLE_pulse_old = (int)FLE_pulse;
      HLB_pulse_old = (int)HLB_pulse;
      HLM_pulse_old = (int)HLM_pulse;
      HLE_pulse_old = (int)HLE_pulse;
      FRB_pulse_old = (int)FRB_pulse;
      FRM_pulse_old = (int)FRM_pulse;
      FRE_pulse_old = (int)FRE_pulse;
      HRB_pulse_old = (int)HRB_pulse;
      HRM_pulse_old = (int)HRM_pulse;
      HRE_pulse_old = (int)HRE_pulse;
      
      parameters_servo.FLB_pulse_old = (int)FLB_pulse;
      parameters_servo.FLM_pulse_old = (int)FLM_pulse;
      parameters_servo.FLE_pulse_old = (int)FLE_pulse;
      parameters_servo.HLB_pulse_old = (int)HLB_pulse;
      parameters_servo.HLM_pulse_old = (int)HLM_pulse;
      parameters_servo.HLE_pulse_old = (int)HLE_pulse;
      parameters_servo.FRB_pulse_old = (int)FRB_pulse;
      parameters_servo.FRM_pulse_old = (int)FRM_pulse;
      parameters_servo.FRE_pulse_old = (int)FRE_pulse;
      parameters_servo.HRB_pulse_old = (int)HRB_pulse;
      parameters_servo.HRM_pulse_old = (int)HRM_pulse;
      parameters_servo.HRE_pulse_old = (int)HRE_pulse;
      
      usleep(10000);
    }   
parameters.global_position = 1;  

}

/* Next function will turn the robot to the right */ 

void Right(int servos_handler)

{
  int i;
  int FLB_pulse_old = parameters_servo.FLB_pulse_old;
  int FLM_pulse_old = parameters_servo.FLM_pulse_old;
  int FLE_pulse_old = parameters_servo.FLE_pulse_old;
  int HLB_pulse_old = parameters_servo.HLB_pulse_old;
  int HLM_pulse_old = parameters_servo.HLM_pulse_old;
  int HLE_pulse_old = parameters_servo.HLE_pulse_old;
  int FRB_pulse_old = parameters_servo.FRB_pulse_old;
  int FRM_pulse_old = parameters_servo.FRM_pulse_old;
  int FRE_pulse_old = parameters_servo.FRE_pulse_old;
  int HRB_pulse_old = parameters_servo.HRB_pulse_old;
  int HRM_pulse_old = parameters_servo.HRM_pulse_old;
  int HRE_pulse_old = parameters_servo.HRE_pulse_old;
  float FLB_pulse_now;
  float FLM_pulse_now;
  float FLE_pulse_now;
  float HLB_pulse_now;
  float HLM_pulse_now;
  float HLE_pulse_now;
  float FRB_pulse_now;
  float FRM_pulse_now;
  float FRE_pulse_now;
  float HRB_pulse_now;
  float HRM_pulse_now;
  float HRE_pulse_now;
  int global_position;
  int servos;
  float FLB_pulse;
  float FLM_pulse;
  float FLE_pulse;
  float FRB_pulse;
  float FRM_pulse;
  float FRE_pulse;
  float HLB_pulse;
  float HLM_pulse;
  float HLE_pulse;
  float HRB_pulse;
  float HRM_pulse;
  float HRE_pulse;
  int FLB_direction = 1;
  int FLM_direction = -1;
  int FLE_direction = -1;
  int FRB_direction = -1;
  int FRM_direction = 1;
  int FRE_direction = 1;
  int HLB_direction = -1;
  int HLM_direction = 1;
  int HLE_direction = 1;
  int HRB_direction = 1;
  int HRM_direction = -1;
  int HRE_direction = -1;
  int wiggle_h = 120;
  int wiggle_v = 200;
  int wiggle_middle = 30;
  servos = servos_handler;
  //global_position = parameters.global_position;
  
  /* Going from position 4 to 1 to turn right*/
  
    for(global_position=parameters.global_position;global_position>0;global_position--) 
    {
      /* Calculate the pulse to send to each servomotor and determine each leg position*/
      parameters.global_position=global_position;

      switch (global_position)
      {
	case 1: 
	  FLB_pulse = 307 + wiggle_middle*FLB_direction;
	  FLM_pulse = 307 + wiggle_v*FLM_direction;
	  FLE_pulse = 307 + wiggle_v*FLE_direction;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(5-2))/3 - wiggle_h)*(-1))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(5-2))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle)*HRB_direction;
	  HRM_pulse = 307 + wiggle_v*HRM_direction;
	  HRE_pulse = 307 + wiggle_v*HRE_direction;
	  break;
	
	case 2: 	    
	  FLB_pulse = 307 + (wiggle_middle + wiggle_h*(-1))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(8-2))/3 - wiggle_h)*(-1))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(8-2))/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + wiggle_h)*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 3: 	
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(5-2))/3 - wiggle_h)*(-1))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307;
	  HLB_pulse = 307 + (-wiggle_middle)*HLB_direction;
	  HLM_pulse = 307 + wiggle_v*HLM_direction;
	  HLE_pulse = 307 + wiggle_v*HLE_direction;
	  FRB_pulse = 307 + wiggle_middle*FRB_direction;
	  FRM_pulse = 307 + wiggle_v*FRM_direction;
	  FRE_pulse = 307 + wiggle_v*FRE_direction;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(6-(5-2))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 4: 	
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(6-(8-2))/3 - wiggle_h)*(-1))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + wiggle_h*(-1))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + wiggle_h)*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 +(-wiggle_middle + (wiggle_h*(6-(8-2))/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
      }
      
      for(i=1;i<5;i++) 
      {
	FLB_pulse_now = FLB_pulse_old+(FLB_pulse-FLB_pulse_old)*i/4;
	FLM_pulse_now = FLM_pulse_old+(FLM_pulse-FLM_pulse_old)*i/4;
	FLE_pulse_now = FLE_pulse_old+(FLE_pulse-FLE_pulse_old)*i/4;
	HLB_pulse_now = HLB_pulse_old+(HLB_pulse-HLB_pulse_old)*i/4;
	HLM_pulse_now = HLM_pulse_old+(HLM_pulse-HLM_pulse_old)*i/4;
	HLE_pulse_now = HLE_pulse_old+(HLE_pulse-HLE_pulse_old)*i/4;
	FRB_pulse_now = FRB_pulse_old+(FRB_pulse-FRB_pulse_old)*i/4;
	FRM_pulse_now = FRM_pulse_old+(FRM_pulse-FRM_pulse_old)*i/4;
	FRE_pulse_now = FRE_pulse_old+(FRE_pulse-FRE_pulse_old)*i/4;
	HRB_pulse_now = HRB_pulse_old+(HRB_pulse-HRB_pulse_old)*i/4;
	HRM_pulse_now = HRM_pulse_old+(HRM_pulse-HRM_pulse_old)*i/4;
	HRE_pulse_now = HRE_pulse_old+(HRE_pulse-HRE_pulse_old)*i/4;
	
	/* Next 3 code blocks are for the front left leg (LF) number 1 */
	i2cWriteByteData(servos, 6, 0x00);
	i2cWriteByteData(servos, 7, 0x00);
	i2cWriteByteData(servos, 8, (int)FLB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 9, (int)FLB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 10, 0x00);
	i2cWriteByteData(servos, 11, 0x00);
	i2cWriteByteData(servos, 12, (int)FLM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 13, (int)FLM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 14, 0x00);
	i2cWriteByteData(servos, 15, 0x00);
	i2cWriteByteData(servos, 16, (int)FLE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 17, (int)FLE_pulse_now >> 8);
	usleep(500);
	/* Next 3 code blocks are for the front right leg (FR) number 3 */
	i2cWriteByteData(servos, 30, 0x00);
	i2cWriteByteData(servos, 31, 0x00);
	i2cWriteByteData(servos, 32, (int)FRB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 33, (int)FRB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 34, 0x00);
	i2cWriteByteData(servos, 35, 0x00);
	i2cWriteByteData(servos, 36, (int)FRM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 37, (int)FRM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 38, 0x00);
	i2cWriteByteData(servos, 39, 0x00);
	i2cWriteByteData(servos, 40, (int)FRE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 41, (int)FRE_pulse_now >> 8);
	usleep(500);
	/* Next 3 code blocks are for the front hind left leg (HL) number 2 */
	i2cWriteByteData(servos, 18, 0x00);
	i2cWriteByteData(servos, 19, 0x00);
	i2cWriteByteData(servos, 20, (int)HLB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 21, (int)HLB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 22, 0x00);
	i2cWriteByteData(servos, 23, 0x00);
	i2cWriteByteData(servos, 24, (int)HLM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 25, (int)HLM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 26, 0x00);
	i2cWriteByteData(servos, 27, 0x00);
	i2cWriteByteData(servos, 28, (int)HLE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 29, (int)HLE_pulse_now >> 8);
	usleep(500);
	/* Next 3 code blocks are for the front hind right leg (HR) number 4 */
	i2cWriteByteData(servos, 42, 0x00);
	i2cWriteByteData(servos, 43, 0x00);
	i2cWriteByteData(servos, 44, (int)HRB_pulse_now & 0xFF);
	i2cWriteByteData(servos, 45, (int)HRB_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 46, 0x00);
	i2cWriteByteData(servos, 47, 0x00);
	i2cWriteByteData(servos, 48, (int)HRM_pulse_now & 0xFF);
	i2cWriteByteData(servos, 49, (int)HRM_pulse_now >> 8);
	usleep(500);
	i2cWriteByteData(servos, 50, 0x00);
	i2cWriteByteData(servos, 51, 0x00);
	i2cWriteByteData(servos, 52, (int)HRE_pulse_now & 0xFF);
	i2cWriteByteData(servos, 53, (int)HRE_pulse_now >> 8);
	usleep(500);
      }
      
      FLB_pulse_old = (int)FLB_pulse;
      FLM_pulse_old = (int)FLM_pulse;
      FLE_pulse_old = (int)FLE_pulse;
      HLB_pulse_old = (int)HLB_pulse;
      HLM_pulse_old = (int)HLM_pulse;
      HLE_pulse_old = (int)HLE_pulse;
      FRB_pulse_old = (int)FRB_pulse;
      FRM_pulse_old = (int)FRM_pulse;
      FRE_pulse_old = (int)FRE_pulse;
      HRB_pulse_old = (int)HRB_pulse;
      HRM_pulse_old = (int)HRM_pulse;
      HRE_pulse_old = (int)HRE_pulse;
      
      parameters_servo.FLB_pulse_old = (int)FLB_pulse;
      parameters_servo.FLM_pulse_old = (int)FLM_pulse;
      parameters_servo.FLE_pulse_old = (int)FLE_pulse;
      parameters_servo.HLB_pulse_old = (int)HLB_pulse;
      parameters_servo.HLM_pulse_old = (int)HLM_pulse;
      parameters_servo.HLE_pulse_old = (int)HLE_pulse;
      parameters_servo.FRB_pulse_old = (int)FRB_pulse;
      parameters_servo.FRM_pulse_old = (int)FRM_pulse;
      parameters_servo.FRE_pulse_old = (int)FRE_pulse;
      parameters_servo.HRB_pulse_old = (int)HRB_pulse;
      parameters_servo.HRM_pulse_old = (int)HRM_pulse;
      parameters_servo.HRE_pulse_old = (int)HRE_pulse;
      
      usleep(10000);
    }   
parameters.global_position = 4;  

}

/* Next function is the main thread to manage robot movement */

void *Walking(void *arg)
{
  int servo_handler;
  servo_handler = (int)arg;
  
  while(interrupt)
  {
    Centre(servo_handler);
    sleep(1);
    /*while(!parameters.ultrimpct)
    {
      Forward(servo_handler);
      sleep(0.05);
    }
    sleep (1);
    while(parameters.ultrimpct)
    {
      Right((int)arg);
      sleep(0.05);
    }*/
  }
  
  pthread_exit(NULL);
}

int main(int argc, char *argv[])
{
pthread_t callThd[3]; /*Number of threads to be used is defined here*/
pthread_attr_t attr;
void *status;
int i;
int pth_err;

/* Initializing the Pigpio here */

Init_Pigpio();

/* control signal() here IMPORTANT, if this is called before initializing the pigpio it will NOT work*/
signal(SIGINT, inthandler);

/* Restart the PCA9685 here */
parameters_servo.servos = Restart_PCA9685();

/* Setting up frequency the PCA9685 here */
Init_PCA9685(parameters_servo.servos);

/* Initializing the parameters.global_position to 1 */
parameters.global_position =1;
	
/* Create threads to start seeing, will not use attributes on this occasion*/  
pthread_attr_init(&attr);
pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

pth_err = pthread_create(&callThd[0], &attr, Distance, NULL); 
if (pth_err !=0)
  {
    printf("Thread 1 not created, exiting the program with error: %d\n", pth_err);
    exit(1);
  }

pth_err = pthread_create(&callThd[1], &attr, Walking, (void *) parameters_servo.servos); 
if (pth_err !=0)
  {
    printf("Thread 2 not created, exiting the program with error: %d\n", pth_err);
    exit(1);
  }

pth_err = pthread_create(&callThd[2], &attr, Camera, NULL); 
if (pth_err !=0)
  {
    printf("Thread 3 not created, exiting the program with error: %d\n", pth_err);
    exit(1);
  }

while(interrupt)
{
/*Printing parameters*/
/*printf("Parameters.dr == %d\n", parameters.dr);
printf("Parameters.dl == %d\n", parameters.dl);
printf("Parameters.distance == %lf\n ", parameters.distance);
printf("Parameters.ultrimpct == %d\n ", parameters.ultrimpct);
printf("Parameters.block == %d\n ", parameters.block);*/

  sleep(100);

}
for(i=0;i<3;i++) 
    {
      pthread_join(callThd[i], &status);
    }
  interrupt = 0;
  i2cWriteByteData(parameters_servo.servos, 0xFD, 0x10); /* Shutting down all channels */
  i2cWriteByteData(parameters_servo.servos, 0x00, 0x00); /* Reset */
  i2cClose(parameters_servo.servos);
  gpioTerminate();
    
  exit(0);
}
