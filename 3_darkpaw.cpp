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

#include "mongoose.h"	/* This one is for the web server */

#include <opencv2/core/core.hpp>	/*this one and the ones that follow are the opencv stuff */
#include <opencv2/core/core_c.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/datasets/track_alov.hpp>


using namespace std;
using namespace cv;
using namespace cv::datasets;

/*The following structure(s) define all the inputs from the sensorial parts and interaction
 with servos, they are meant to act as global variables for the threads*/

typedef struct
{
  double distance; /*distance to the obstacle from the ultrasounds sensor*/
  int ultrimpct; /*near frontal impact to the wall, from the ultrasounds */ 
  int block; /* this is to avoid the robot from getting stuck in obstacles not detected by the sensors */
  int global_position; /*this one is to store the legs global position from 1 to 8 */
  int MPU6050;
  float acce_x; /* Accelerometer X axis */ 
  float acce_y; /* Accelerometer y axis */ 
  float acce_z; /* Accelerometer z axis */ 
  float gyro_x; /* Gyroscope X axis */ 
  float gyro_y; /* Gyroscope y axis */ 
  float gyro_z; /* Gyroscope z axis */ 
  float camera_x; /* This is the x axis of the target coming from the camera */
  int seek; /* This is the flag to start seeking */
  int enable_walking; /* This just enables the robot capacity to move */
  FILE *f;
  
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
  int servos; /*this one is the PCA9685 handler returned by I2Copen() */
  int forward;
  int PID_start;
  float PID_out;
  float PID_left;
  float PID_right;
  float PID_measurement[20]; /* For angle rolling average */
  
} SERVOS_PARAMETERS;

/* Define globally accessible variables no mutex */
volatile PARAM parameters;
/* Define globally accessible variables no mutex for exclusive use of the servos*/
volatile SERVOS_PARAMETERS parameters_servo;

/* Buffer for jpeg streaming*/
static std::vector<uchar> outbuf;

/* This global variable is used to interrupt the infite while loops with signal()
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
  i2cClose(parameters.MPU6050);
  gpioTerminate();
  //fclose(parameters.f);
  printf("Caught signal %d, coming out ...\n", signum);
  //exit(1);
}

void inthandler1(int signum) 
{
  printf("Caught signal %d, coming out ...\n", signum);
  exit(1);
}
/* Web server functions below */

// HTTP request handler function. It implements the following endpoints:
//   /video - hangs forever, returns MJPEG video stream
//   all other URI - serves web_root/ directory
static void cb(struct mg_connection *c, int ev, void *ev_data, void *fn_data) 
{
  /*char result[5];
  if (mg_http_get_var(&hm->query, "stop", result, sizeof(result))>0)
  {printf("%s\n",result);} */
  
  if (ev == MG_EV_HTTP_MSG) {
    struct mg_http_message *hm = (struct mg_http_message *) ev_data; //(struct mg_http_message *) ev_data;
    
    if (mg_http_match_uri(hm, "/video")) { 
      c->label[0] = 'S';  // Mark that connection as live streamer
      mg_printf(
          c, "%s",
          "HTTP/1.0 200 OK\r\n"
          "Cache-Control: no-cache\r\n"
          "Pragma: no-cache\r\nExpires: Thu, 01 Dec 1994 16:00:00 GMT\r\n"
          "Content-Type: multipart/x-mixed-replace; boundary=--foo\r\n\r\n");
      }  
      
      else if (mg_http_match_uri(hm, "/start")) {
	parameters.enable_walking = 1; 
	hm->uri = mg_str("/");
	struct mg_http_serve_opts opts = {.root_dir = "web_root"};
	mg_http_serve_dir(c, hm, &opts);
      }
      else if (mg_http_match_uri(hm, "/stop")) {
	parameters.enable_walking = 0;
	hm->uri = mg_str("/");
	struct mg_http_serve_opts opts = {.root_dir = "web_root"};
	mg_http_serve_dir(c, hm, &opts);
      }
       else if (mg_http_match_uri(hm, "/reset")) {
	parameters.seek = 0;
	hm->uri = mg_str("/");
	struct mg_http_serve_opts opts = {.root_dir = "web_root"};
	mg_http_serve_dir(c, hm, &opts);
      }
      else { 
      struct mg_http_serve_opts opts = {.root_dir = "web_root"};
      mg_http_serve_dir(c, hm, &opts);
    }
  } 
}

static void broadcast_mjpeg_frame(struct mg_mgr *mgr) 
{
  if( outbuf.size()<4 || 
            (outbuf[0]!=0xff && outbuf[0]!=0xd8) ||
            (outbuf[outbuf.size()-2]!=0xff && outbuf[outbuf.size()-1]!=0xd9))
        {
            usleep(10000);
        }
  uchar *data = outbuf.data();
  size_t size = outbuf.size();
  struct mg_connection *c;
  for (c = mgr->conns; c != NULL; c = c->next) {
    if (c->label[0] != 'S') continue;         // Skip non-stream connections
    if (outbuf.data() == NULL || size == 0) continue;  // Skip on buffer read error f (outbuf.data() == NULL || size == 0) continue;
    mg_printf(c,
              "--foo\r\nContent-Type: image/jpeg\r\n"
              "Content-Length: %lu\r\n\r\n",
              (unsigned long) size);
    mg_send(c, data, size); // mg_send(c, &outbuf[0], size) mg_send(c, outbuf.data(), size);
    mg_send(c, "\r\n", 2);
  }
}

static void timer_callback(void *arg) 
{
  struct mg_mgr *mgr = (struct mg_mgr *) arg;
  broadcast_mjpeg_frame(mgr);
}

/* PID regulator thread */

void *PID(void *arg)

{
  /* Controller gains */
  const float Kp = 3.5;
  const float Ki = 0.5;
  const float Kd = 0.25;
  
  /* Sample time (in seconds) */
  const float T = 0.1;
  
  /* Derivative filter tau (in seconds) */
  const float Tau = 0.02;
  
  /* Controller "memory" */
  float prevError = 0;			/* Required for integrator */
  float prevIntegral = 0;		/* Required for integrator */
  //float prevMeasurement = 0;		/* This could be used for derivative instead of error/prevError */
  float prevDerivative = 0;		/* Required for differentiator */
  float proportional = 0;
  float integral = 0;
  float derivative = 0;
  
  /* Controller intput */
  float measurement[20] = {0};
  
  /* Controller output */
  float out = 0;
  
  /* Controller setpoint */
  const float setpoint = 320;
  
  /* Controller error */
  float error = 0;
  
  /* Flags */
  int left_flag = 0;
  
  while(interrupt)
  {
    while(parameters_servo.PID_start)
    {
      /*for (int i = 0; i < 20; i++) 
       {
	measurement[i] = parameters_servo.PID_measurement[i];
	 if (i>0)
	  {
	    measurement[0] += measurement[i];
	  }
       } 
      measurement[0] = measurement[0]/20; */
      measurement[0] = parameters.camera_x; 
      error = setpoint - measurement[0];
      if (error>0)
	{
	  left_flag = 1;
	}
      else
	{
	  left_flag = 0;
	}
      /* Proportional */
      proportional = Kp * error;
      
      /* Integral */
      integral = prevIntegral + 0.5 * Ki * T * (error + prevError);
      
      
      /* Derivative */
      derivative = (2.0*Kd*(error-prevError)+(2.0*Tau-T)*prevDerivative)/(2.0*Tau+T);
      
      /* Saving for next iteration */
      prevDerivative = derivative;
      prevIntegral = integral;
      //prevMeasurement = measurement[0];
      prevError = error;
      
      out = fabs(proportional + integral + derivative);
      out = 1-out/320;
      
      if (out < 0.50)
	{
	out = 0.50;
	}
      if (out > 1)
	{
	out = 1;
	}
      
      parameters_servo.PID_out = out;
      if ((-20 > error) || (error > 20))
	{
	  if (left_flag == 1)
	    {
	      parameters_servo.PID_right = out;
	      parameters_servo.PID_left = 1;
	    }
	  else 
	    {
	      parameters_servo.PID_left = out;
	      parameters_servo.PID_right = 1;
	    }
	}
      else
	{
	  parameters_servo.PID_left = 1;
	  parameters_servo.PID_right = 1;
	}
      usleep(100000);
    }
      
      /*printf("Ultraimpct: %d \n", parameters.ultrimpct); 
      printf("PID_measurement: %f \n", measurement[0]);
      printf("PID_error: %f \n", error);
      printf("PID_out: %f \n", out);
      printf("PID_out_left: %f \n",parameters_servo.PID_left);
      printf("PID_out_right: %f \n",parameters_servo.PID_right); */
  }
  pthread_exit(NULL);
}

void *Book_keeping(void *arg)

{
  char s[100];
  time_t now;
  struct tm *t;
  FILE *f;
  
  f = fopen("record.csv", "w+");
  if (f == NULL)
    {
      printf("Error opening file! Quitting Book_keeping thread\n");
      interrupt = 0;
      //exit(1);
    }
  parameters.f = f;
  while(interrupt)
  {
    time(&now);
    t = localtime(&now);
    strftime(s, 100, "%H:%M:%S", t);
    fprintf(f,"%s, %f, %f, %f, %f, %f, %f, %f, %f\n",s ,parameters.distance, parameters_servo.PID_out, parameters.acce_x, 
	    parameters.acce_y, parameters.acce_z, parameters.gyro_x, parameters.gyro_y, parameters.gyro_z);
    usleep(250000);
  }
  fclose(f);
  pthread_exit(NULL);
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
      }

      usleep(250000);
    }
    
  pthread_exit(NULL);
}
/* Next thread will read the gyroscope & accelerometer data MPU6050 */

void *MPU6050_DATA(void *arg)

{
  //const float GRAVITIY_MS2 = 9.80665;
  
  /* Scale Modifiers */
  //const float ACCEL_SCALE_MODIFIER_2G = 16384.0;
  const float ACCEL_SCALE_MODIFIER_4G = 8192.0;
  //const float ACCEL_SCALE_MODIFIER_8G = 4096.0;
  //const float ACCEL_SCALE_MODIFIER_16G = 2048.0;
  const float GYRO_SCALE_MODIFIER_250DEG = 131.0;
  //const float GYRO_SCALE_MODIFIER_500DEG = 65.5;
  //const float GYRO_SCALE_MODIFIER_1000DEG = 32.8;
  //const float GYRO_SCALE_MODIFIER_2000DEG = 16.4;
  
  /* Pre-defined ranges */
  //const int ACCEL_RANGE_2G = 0x00;
  const int ACCEL_RANGE_4G = 0x08;
  //const int ACCEL_RANGE_8G = 0x10;
  //const int ACCEL_RANGE_16G = 0x18;
  const int GYRO_RANGE_250DEG = 0x00;
  //const int GYRO_RANGE_500DEG = 0x08;
  //const int GYRO_RANGE_1000DEG = 0x10;
  //const int GYRO_RANGE_2000DEG = 0x18;
  
  /* MPU-6050 Registers */
  const int PWR_MGMT_1 = 0x6B;
  //const int PWR_MGMT_2 = 0x6C;
  const int ACCEL_XOUT0 = 0x3B;
  const int ACCEL_YOUT0 = 0x3D;
  const int ACCEL_ZOUT0 = 0x3F;
  //const int TEMP_OUT0 = 0x41;
  const int GYRO_XOUT0 = 0x43;
  const int GYRO_YOUT0 = 0x45;
  const int GYRO_ZOUT0 = 0x47;
  const int ACCEL_CONFIG = 0x1C;
  const int GYRO_CONFIG = 0x1B;
 
  int MPU6050 = 0;
  int acce_x_H = 0;
  int acce_x_L = 0;
  float acce_x = 0;
  int acce_y_H = 0;
  int acce_y_L = 0;
  float acce_y = 0;
  int acce_z_H = 0;
  int acce_z_L = 0;
  float acce_z = 0;
  int gyro_x_H = 0;
  int gyro_x_L = 0;
  float gyro_x = 0;
  int gyro_y_H = 0;
  int gyro_y_L = 0;
  float gyro_y = 0;
  int gyro_z_H = 0;
  int gyro_z_L = 0;
  float gyro_z = 0;
  int count_20 = 0;

  /* Opening the conenction to the I2C slave */
  MPU6050 = i2cOpen(1,0x68,0);
  
    if (MPU6050 >=  0)
    {
      /* PI connection to I2C slave 0x68 OK*/
      printf("Open I2C to slave 0x68 OK. Return code:  %d\n", MPU6050);
      printf("MPU6050 number handler:  %d\n", MPU6050);
    }
  else
    {
      /* No PI connection to I2C slave 0x68 */
      printf("Open I2C to slave 0x68 failed. Quitting MPU6050 thread Error code:  %d\n", MPU6050);
      interrupt = 0;
      //exit(MPU6050);
    }
  parameters.MPU6050 = MPU6050;
  
  /* Wake up the MPU-6050 since it starts in sleep mode */
  i2cWriteByteData(MPU6050, PWR_MGMT_1, 0x00); 
  usleep(1000);
  
  /* Now set up the accelerator  range to 4G */
  i2cWriteByteData(MPU6050, ACCEL_CONFIG, ACCEL_RANGE_4G); 
  usleep(1000);
  
  /* Now set up the gyroscope  range to 500 deg/second */
  i2cWriteByteData(MPU6050, GYRO_CONFIG, GYRO_RANGE_250DEG);
  usleep(1000);
  
  while (interrupt)
  {
    /* Reading accelerometer values */
    acce_x_H = i2cReadByteData(MPU6050, ACCEL_XOUT0); /* getting the H register 15:8 */
    acce_x_L = i2cReadByteData(MPU6050, ACCEL_XOUT0+1); /* getting the H register 7:0 */
    acce_x = (acce_x_H << 8) + acce_x_L;
    if (acce_x >= 0x8000)
    {
      acce_x = -(65535 - acce_x) + 1;
    }
    acce_x = acce_x/ACCEL_SCALE_MODIFIER_4G;
    parameters.acce_x = acce_x;
    
    acce_y_H = i2cReadByteData(MPU6050, ACCEL_YOUT0); /* getting the H register 15:8 */
    acce_y_L = i2cReadByteData(MPU6050, ACCEL_YOUT0+1); /* getting the H register 7:0 */
    acce_y = (acce_y_H << 8) + acce_y_L;
    if (acce_y >= 0x8000)
    {
      acce_y = -(65535 - acce_y) + 1;
    }
    acce_y = acce_y/ACCEL_SCALE_MODIFIER_4G;
    parameters.acce_y = acce_y;
    
    acce_z_H = i2cReadByteData(MPU6050, ACCEL_ZOUT0); /* getting the H register 15:8 */
    acce_z_L = i2cReadByteData(MPU6050, ACCEL_ZOUT0+1); /* getting the H register 7:0 */
    acce_z = (acce_z_H << 8) + acce_z_L;
    if (acce_z >= 0x8000)
    {
      acce_z = -(65535 - acce_z) + 1;
    }
    acce_z = acce_z/ACCEL_SCALE_MODIFIER_4G;
    parameters.acce_z = acce_z;
    
    /* Reading gyroscope values */
    gyro_x_H = i2cReadByteData(MPU6050, GYRO_XOUT0); /* getting the H register 15:8 */
    gyro_x_L = i2cReadByteData(MPU6050, GYRO_XOUT0+1); /* getting the H register 7:0 */
    gyro_x = (gyro_x_H << 8) + gyro_x_L;
    if (gyro_x >= 0x8000)
    {
      gyro_x = -(65535 - gyro_x) + 1;
    }
    gyro_x = gyro_x/GYRO_SCALE_MODIFIER_250DEG;
    parameters.gyro_x = gyro_x;
    
    gyro_y_H = i2cReadByteData(MPU6050, GYRO_YOUT0); /* getting the H register 15:8 */
    gyro_y_L = i2cReadByteData(MPU6050, GYRO_YOUT0+1); /* getting the H register 7:0 */
    gyro_y = (gyro_y_H << 8) + gyro_y_L;
    if (gyro_y >= 0x8000)
    {
      gyro_y = -(65535 - gyro_y) + 1;
    }
    gyro_y = gyro_y/GYRO_SCALE_MODIFIER_250DEG;
    parameters.gyro_y = gyro_y;
    
    gyro_z_H = i2cReadByteData(MPU6050, GYRO_ZOUT0); /* getting the H register 15:8 */
    gyro_z_L = i2cReadByteData(MPU6050, GYRO_ZOUT0+1); /* getting the H register 7:0 */
    gyro_z = (gyro_z_H << 8) + gyro_z_L;
    if (gyro_z >= 0x8000)
    {
      gyro_z = -(65535 - gyro_z) + 1;
    }
    gyro_z = gyro_z/GYRO_SCALE_MODIFIER_250DEG;
    gyro_z = gyro_z - 8.8; /*this is to correct an 8.6 offset detected on this axis */
    parameters.gyro_z = gyro_z;
    parameters_servo.PID_measurement[count_20] = gyro_z;
    count_20++;
    if (count_20>=30)
      {
	count_20 = 0;
      }
    usleep(100000);
    
    
    /*printf("Accelerometer X:  %f\n", acce_x);
    printf("Gyro X:  %f\n", gyro_x);
    printf("Accelerometer Y:  %f\n", acce_y);
    printf("Gyro Y:  %f\n", gyro_y);
    printf("Accelerometer Z:  %f\n", acce_z);
    printf("Gyro Z:  %f\n", gyro_z); 
    printf("%f\n", gyro_z); */
  }
   i2cClose(MPU6050);
   pthread_exit(NULL);
}


/* Next thread will manage the opencv stuff */

void *Camera(void *arg)
{
    clock_t before = 0;
    clock_t after = 0;
    Mat frame, framegray;
    Mat fgMaskMOG2; /* Foreground mask used by MOG2 method */ 
    vector<vector<Point> > contours; 
    vector<Point> approx;
    bool ok = false;
    int detection = 1;
    Rect2d mr; 
    Point centre;
    /* Parameters to the encoder for jpeg stream */
    std::vector<unsigned char> buffer;
    vector<int> params_stream(2);
    params_stream[0] = IMWRITE_JPEG_QUALITY;
    params_stream[1] = 80; /* JPEG quality (1...100) */
    
    Ptr<BackgroundSubtractor> pMOG2; /* MOG2 Background subtractor */
    pMOG2 = createBackgroundSubtractorMOG2(100,15,true); /* Create MOG2 Background Subtractor object */
    
    TrackerCSRT::Params params = TrackerCSRT::Params(); /* Creating parameters for the tracker so we are able to change threshold */
    /* Full list of parameters */
    params.use_channel_weights = true;
    params.use_segmentation = true;
    params.use_hog = true;
    params.use_color_names = true;
    params.use_gray = false; /* orginally true */
    params.use_rgb = true; /* originally false */
    params.window_function = "hann"; /* originally= hann, but others are possible: Window function: "hann", "cheb", "kaiser"*/
    params.kaiser_alpha = 3.75f;
    params.cheb_attenuation = 45;
    params.padding = 3.0f;
    params.template_size = 200;
    params.gsl_sigma = 1.0f;
    params.hog_orientations = 9;
    params.hog_clip = 0.2f;
    params.num_hog_channels_used = 18;
    params.filter_lr = 0.02f;
    params.weights_lr = 0.02f;
    params.admm_iterations = 6; // originally 4
    params.number_of_scales = 33;
    params.scale_sigma_factor = 0.250f;
    params.scale_model_max_area = 512.0f;
    params.scale_lr = 0.025f;
    params.scale_step = 1.020f;
    params.histogram_bins = 16;
    params.background_ratio = 2; /* Default value =2 */
    params.histogram_lr = 0.04f;
    params.psr_threshold = 0.07f; /* Default value= 0.035 CSRT Tracker parameter to make it more sensible to false positives */
    
    Ptr<Tracker> tracker; /* Create Pointer to tracker object */
    //tracker = TrackerCSRT::create(params); /* Create the tracker object */
    
    int contours_chosen = 0;

    /*--- INITIALIZE VIDEOCAPTURE */
    VideoCapture cap;
    cap.set(CAP_PROP_FRAME_WIDTH,640); /*set camera resolution */
    cap.set(CAP_PROP_FRAME_HEIGHT,480); /* set camera resolution */
    cap.set(CAP_PROP_BUFFERSIZE, 5); /* To store only the last 5 frames */
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID, apiID);
    // check if we succeeded
    if (!cap.isOpened()) 
      {
	cerr << "ERROR! Unable to open camera, exiting thread\n";
	pthread_exit(NULL);
      }
    
    while(interrupt)
    {
	detection = 1;
	parameters.seek = 0;
	before = clock(); /* allow time to settle the movement detector when start frame capture camera*/
	
	while (detection)
	{
	  parameters.seek = 0;
	  /* wait for a new frame from camera and store it into 'frame' */
	  cap.read(frame);
	  /* check if we succeeded */
	  if (frame.empty()) 
	  {
            printf("ERROR! blank frame grabbed\n");
            break;
	  }
	  
	  /* Put frame into buffer */
	  imencode(".jpg", frame, buffer, params_stream);
	  outbuf.swap(buffer);
	  
	  /* colour to gray conversion */
	  cvtColor(frame, framegray, COLOR_RGB2GRAY); 
	  GaussianBlur(framegray, framegray, Size(21, 21), 0, 0);
	  //GaussianBlur(framegray, framegray, Size(7, 7), 1.5, 1.5);
	  //medianBlur(framegray, framegray, 5);
	  
	  pMOG2->apply(framegray, fgMaskMOG2, 0.2); /* Update the MOG2 background model based 
	  on the current frame, 0=background model not updated, 1=background model is completely reinitialized from the 
	  last frame, negative=automatic, used=0.0035  */
	  
	  /* Adaptive Guassian Threshold is to detect sharp edges in the Image. For more information */
	  //adaptiveThreshold(framegray, framegray, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 7, 4);
	  
	  /* Eroding image*/
	  //erode(framegray, framegray, getStructuringElement(MORPH_RECT, Size(7, 7)));
	  //erode(fgMaskMOG2, fgMaskMOG2, getStructuringElement(MORPH_RECT, Size(5, 5)), Point(-1, -1),2,BORDER_DEFAULT);
	  
	  /* Dilate image*/
	  //dilate(framegray, framegray, getStructuringElement(MORPH_RECT, Size(5, 5)));
	  dilate(fgMaskMOG2, fgMaskMOG2, getStructuringElement(MORPH_RECT, Size(7, 7)), Point(-1, -1),2,BORDER_DEFAULT);
	  
	  
	  findContours(fgMaskMOG2, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); 
	  
	    /* test each contour for bigger area on screen) */
	  for( size_t i = 0; i < contours.size(); i++ ) 
	    {
	      contours_chosen = 0;
	      if (i>0)
	      {
		if (contourArea(contours[i]) > contourArea(contours[i-1]))
		{
		  contours_chosen = i;
		}
	      }
	    }
	  after = clock();

	  if ((contourArea(contours[contours_chosen]))>2000 && ((double)(after-before)/(double)CLOCKS_PER_SEC)>10)
	    {
	      //approxPolyDP(contours[contours_chosen], approx, arcLength(contours[contours_chosen], true)*0.02, true);
	     
	      mr = boundingRect(contours[contours_chosen]); //mr= boundingRect(approx);
	      if (mr.x>=0 && mr.y>=0 && mr.width>=0 && mr.height>=0) /* To check if the rectangle is out of screen (long negative),if not it will THROW EXCEPTION: 
	      error(-211) see: https://github.com/opencv/opencv/issues/7573 */
		{
		/* mr.width = mr.width;
		mr.height = mr.height;
		float scale = 1.0;
		centre.x = cvRound((mr.x + mr.width*0.5)*scale); 
		centre.y = cvRound((mr.y + mr.height*0.5)*scale); 
		printf("mr.x= %f \n",mr.x);
		printf("mr.y= %f \n",mr.y);
		printf("mr.width= %f \n",mr.width);
		printf("mr.height= %f \n",mr.height);*/
		detection = 0;
		tracker = TrackerCSRT::create(params);
		mr = mr & Rect2d(0, 0, 640, 480); /* To avoid exception  error: (-215:Assertion failed) */ 
		tracker->init(frame, mr); 
		parameters_servo.PID_start = 1;
		parameters.seek = 1;
		}
	  
	    }

	    //imshow("FG Mask MOG 2", fgMaskMOG2); /*Show the MOG2 foreground mask */
	    /* show live and wait */
	    //waitKey(1);
	}
	
	//destroyWindow("FG Mask MOG 2");
	while (parameters.seek)
	{
	  /* wait for a new frame from camera and store it into 'frame' */
	  cap.read(frame);
	  /* check if we succeeded */
	  if (frame.empty()) 
	  {
            printf("ERROR! blank frame grabbed\n");
            break;
	  }
	  //Mat roi = frame(mr);
	  //tracker->init(frame, mr);
	  ok = tracker->update(frame, mr);
	  //printf("OK: %d\n",ok);
	  if (ok)
	    {
	      rectangle(frame, mr, Scalar( 255, 0, 0 ), 2, 1 );
	      putText(frame, "TRACKING", Point(20, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0)); /* this one is the top-left corner */
	      float scale = 1.0;
	      centre.x = cvRound((mr.x + mr.width*0.5)*scale);
	      centre.y = cvRound((mr.y + mr.height*0.5)*scale);
	      parameters.camera_x = centre.x;
	    }
	  else
	    {
	    //putText(frame, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
	    detection = 1;
	    parameters.seek = 0;
	    parameters_servo.PID_start = 0;
	    }
	      
	  //putText(frame, "TopLeft", Point(0, 0), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0)); /* this one is the top-left corner */
	  //putText(frame, "BottomRight", Point(640, 480), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0)); /* this one is the bottom-right corner */
	  //putText(frame, "Centre", Point(320, 240), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0)); /* this one is the bottom-right corner */ 
	  
	  /* Put frame into buffer */
	  imencode(".jpg", frame, buffer, params_stream);
	  outbuf.swap(buffer);
	  
	  //imshow("Normal", frame); /*Track detected object */
	  /* show live and wait */
	  //waitKey(1);
	}
	  
	//destroyWindow("Normal");
    }
  //destroyAllWindows();
  pMOG2.release();
  tracker.release();
  printf("Tracker has been released\n");
  cap.release();
  printf("Camera has been released\n");
  pthread_exit(NULL);
}

/* Pigpio library initialization */

void Init_Pigpio(void)
{
  int Init = 0; 
  Init = gpioInitialise();
  if (Init < 0)
    {
      /* pigpio initialisation failed */
      printf("Pigpio initialisation failed. Finishing Program Error code:  %d\n", Init);
      interrupt = 0;
      //exit(Init);
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
  int oldmode;
  int newmode;
  int servos;
  servos = i2cOpen(1,0x40,0);
    if (servos >=  0)
    {
      /* PI connection to I2C slave 40 OK*/
      printf("Open I2C to slave 0x40 OK. Return code:  %d\n", servos);
      printf("PCA9685 number handler:  %d\n", servos);
    }
  else
    {
      /* No PI connection to I2C slave 40 */
      printf("Open I2C to slave 0x40 failed. Quitting Servos thread Error code:  %d\n", servos);
      interrupt = 0;
      //exit(servos);
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
	  FLB_pulse = 307 + wiggle_middle*FLB_direction*parameters_servo.PID_right;
	  FLM_pulse = 307 + wiggle_v*FLM_direction;
	  FLE_pulse = 307 + wiggle_v*FLE_direction;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*5/3 - wiggle_h))*HLB_direction*parameters_servo.PID_right;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*3/3 - wiggle_h))*FRB_direction*parameters_servo.PID_left;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*1/3 - wiggle_h))*HRB_direction*parameters_servo.PID_left;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 2:
	  FLB_pulse = 307 + (wiggle_middle + wiggle_h)*FLB_direction*parameters_servo.PID_right;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(4)/3 - wiggle_h))*HLB_direction*parameters_servo.PID_right;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(2)/3 - wiggle_h))*FRB_direction*parameters_servo.PID_left;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(0)/3 - wiggle_h))*HRB_direction*parameters_servo.PID_left;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 3:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(5)/3 - wiggle_h))*FLB_direction*parameters_servo.PID_right;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(3)/3 - wiggle_h))*HLB_direction*parameters_servo.PID_right;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(1)/3 - wiggle_h))*FRB_direction*parameters_servo.PID_left;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle)*HRB_direction*parameters_servo.PID_left;
	  HRM_pulse = 307 + wiggle_v*HRM_direction;
	  HRE_pulse = 307 + wiggle_v*HRE_direction;
	  break;
	  
	case 4:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(4)/3 - wiggle_h))*FLB_direction*parameters_servo.PID_right;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(2)/3 - wiggle_h))*HLB_direction*parameters_servo.PID_right;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(0)/3 - wiggle_h))*FRB_direction*parameters_servo.PID_left;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + wiggle_h)*HRB_direction*parameters_servo.PID_left;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 5:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(3)/3 - wiggle_h))*FLB_direction*parameters_servo.PID_right;
	  FLM_pulse = 307;
	  FLE_pulse = 307;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(1)/3 - wiggle_h))*HLB_direction*parameters_servo.PID_right;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + wiggle_middle*FRB_direction*parameters_servo.PID_left;
	  FRM_pulse = 307 + wiggle_v*FRM_direction;
	  FRE_pulse = 307 + wiggle_v*FRE_direction;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(5)/3 - wiggle_h))*HRB_direction*parameters_servo.PID_left;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 6:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(2)/3 - wiggle_h))*FLB_direction*parameters_servo.PID_right;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(0)/3 - wiggle_h))*HLB_direction*parameters_servo.PID_right;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + wiggle_h)*FRB_direction*parameters_servo.PID_left;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(4)/3 - wiggle_h))*HRB_direction*parameters_servo.PID_left;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 7:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(1)/3 - wiggle_h))*FLB_direction*parameters_servo.PID_right;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle)*HLB_direction*parameters_servo.PID_right;
	  HLM_pulse = 307 + wiggle_v*HLM_direction;
	  HLE_pulse = 307 + wiggle_v*HLE_direction;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(5)/3 - wiggle_h))*FRB_direction*parameters_servo.PID_left;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(3)/3 - wiggle_h))*HRB_direction*parameters_servo.PID_left;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 8:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(0)/3 - wiggle_h))*FLB_direction*parameters_servo.PID_right;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + wiggle_h)*HLB_direction*parameters_servo.PID_right;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(4)/3 - wiggle_h))*FRB_direction*parameters_servo.PID_left;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(2)/3 - wiggle_h))*HRB_direction*parameters_servo.PID_left;
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
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(5)/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(3)/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(1)/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 2:
	  FLB_pulse = 307 + (wiggle_middle + wiggle_h)*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(4)/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(2)/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(0)/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 3:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(5)/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(3)/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(1)/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle)*HRB_direction;
	  HRM_pulse = 307 + wiggle_v*HRM_direction;
	  HRE_pulse = 307 + wiggle_v*HRE_direction;
	  break;
	  
	case 4:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(4)/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(2)/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(0)/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + wiggle_h)*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 5:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(3)/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(1)/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + wiggle_middle*FRB_direction;
	  FRM_pulse = 307 + wiggle_v*FRM_direction;
	  FRE_pulse = 307 + wiggle_v*FRE_direction;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(5)/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 6:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(2)/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(0)/3 - wiggle_h))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + wiggle_h)*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(4)/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 7:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(1)/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle)*HLB_direction;
	  HLM_pulse = 307 + wiggle_v*HLM_direction;
	  HLE_pulse = 307 + wiggle_v*HLE_direction;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(5)/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(3)/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 8:
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(0)/3 - wiggle_h))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + wiggle_h)*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(4)/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(2)/3 - wiggle_h))*HRB_direction;
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
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(3)/3 - wiggle_h)*(-1))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(3)/3 - wiggle_h))*FRB_direction;
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
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(0)/3 - wiggle_h)*(-1))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(0)/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + wiggle_h)*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 3: 	
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(3)/3 - wiggle_h)*(-1))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307;
	  HLB_pulse = 307 + (-wiggle_middle)*HLB_direction;
	  HLM_pulse = 307 + wiggle_v*HLM_direction;
	  HLE_pulse = 307 + wiggle_v*HLE_direction;
	  FRB_pulse = 307 + wiggle_middle*FRB_direction;
	  FRM_pulse = 307 + wiggle_v*FRM_direction;
	  FRE_pulse = 307 + wiggle_v*FRE_direction;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(3)/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 4: 	
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(0)/3 - wiggle_h)*(-1))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + wiggle_h*(-1))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + wiggle_h)*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 +(-wiggle_middle + (wiggle_h*(0)/3 - wiggle_h))*HRB_direction;
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
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(3)/3 - wiggle_h)*(-1))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(3)/3 - wiggle_h))*FRB_direction;
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
	  HLB_pulse = 307 + (-wiggle_middle + (wiggle_h*(0)/3 - wiggle_h)*(-1))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + (wiggle_h*(0)/3 - wiggle_h))*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 + (-wiggle_middle + wiggle_h)*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	
	case 3: 	
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(3)/3 - wiggle_h)*(-1))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307;
	  HLB_pulse = 307 + (-wiggle_middle)*HLB_direction;
	  HLM_pulse = 307 + wiggle_v*HLM_direction;
	  HLE_pulse = 307 + wiggle_v*HLE_direction;
	  FRB_pulse = 307 + wiggle_middle*FRB_direction;
	  FRM_pulse = 307 + wiggle_v*FRM_direction;
	  FRE_pulse = 307 + wiggle_v*FRE_direction;
	  HRB_pulse = 307 + (-wiggle_middle + (wiggle_h*(3)/3 - wiggle_h))*HRB_direction;
	  HRM_pulse = 307;
	  HRE_pulse = 307;
	  break;
	  
	case 4: 	
	  FLB_pulse = 307 + (wiggle_middle + (wiggle_h*(0)/3 - wiggle_h)*(-1))*FLB_direction;
	  FLM_pulse = 307;
	  FLE_pulse = 307 ;
	  HLB_pulse = 307 + (-wiggle_middle + wiggle_h*(-1))*HLB_direction;
	  HLM_pulse = 307;
	  HLE_pulse = 307;
	  FRB_pulse = 307 + (wiggle_middle + wiggle_h)*FRB_direction;
	  FRM_pulse = 307;
	  FRE_pulse = 307;
	  HRB_pulse = 307 +(-wiggle_middle + (wiggle_h*(0)/3 - wiggle_h))*HRB_direction;
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
  const int FRONTLIGHT0 = 5; /*BCM pin 5, front LED*/
  const int FRONTLIGHT1 = 13; /*BCM pin 13, front LED*/
  gpioSetMode(FRONTLIGHT0, PI_OUTPUT); 
  gpioSetMode(FRONTLIGHT1, PI_OUTPUT); 
  
  while(interrupt)
  { 
    while(parameters.enable_walking)
    {
      Centre(servo_handler);
      gpioWrite(FRONTLIGHT0, 0);
      gpioWrite(FRONTLIGHT1, 0);
      
	while(!parameters.ultrimpct && parameters.seek && parameters.enable_walking)
	{
	  
	  Forward(servo_handler);
	}
	
	while(parameters.ultrimpct)
	{
	  gpioWrite(FRONTLIGHT0, 1);
	  gpioWrite(FRONTLIGHT1, 1);
	  Backward((int)arg);
	} 
    }
    sleep(1);
  }
  
  pthread_exit(NULL);
}

int main(int argc, char *argv[])
{
pthread_t callThd[6]; /*Number of threads to be used is defined here*/
pthread_attr_t attr;
void *status;
int i;
int pth_err;

/* Initializing the Pigpio here */

Init_Pigpio();

/* control signal() here IMPORTANT, if this is called before initializing the pigpio it will NOT work, pigpio initializes all flags */
signal(SIGINT, inthandler);
signal(SIGHUP, inthandler1);
signal(SIGABRT, inthandler1);
signal(SIGILL, inthandler1);
signal(SIGSEGV, inthandler1);
signal(SIGTERM, inthandler1);

/* Restart the PCA9685 here */
parameters_servo.servos = Restart_PCA9685();

/* Setting up frequency the PCA9685 here */
Init_PCA9685(parameters_servo.servos);

/* Initializing global parameters*/
parameters.global_position =1;
parameters.seek = 0;
parameters_servo.PID_start = 0;
parameters.enable_walking = 0;
	
/* Create threads to start seeing, will not use attributes on this occasion*/  
pthread_attr_init(&attr);
pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

pth_err = pthread_create(&callThd[0], &attr, Book_keeping, NULL); 
if (pth_err !=0)
  {
    printf("Thread 1 not created, exiting the program with error: %d\n", pth_err);
    exit(1);
  }

pth_err = pthread_create(&callThd[1], &attr, Distance, NULL); 
if (pth_err !=0)
  {
    printf("Thread 2 not created, exiting the program with error: %d\n", pth_err);
    exit(1);
  }
  
pth_err = pthread_create(&callThd[2], &attr, MPU6050_DATA, NULL); 
if (pth_err !=0)
  {
    printf("Thread 3 not created, exiting the program with error: %d\n", pth_err);
    exit(1);
  }

pth_err = pthread_create(&callThd[3], &attr, Camera, NULL); 
if (pth_err !=0)
  {
    printf("Thread 4 not created, exiting the program with error: %d\n", pth_err);
    exit(1);
  }

pth_err = pthread_create(&callThd[4], &attr, PID, NULL); 
if (pth_err !=0)
  {
    printf("Thread 5 not created, exiting the program with error: %d\n", pth_err);
    exit(1);
  }

pth_err = pthread_create(&callThd[5], &attr, Walking, (void *) parameters_servo.servos); 
if (pth_err !=0)
  {
    printf("Thread 6 not created, exiting the program with error: %d\n", pth_err);
    exit(1);
  }

/* Starting web server here */
struct mg_mgr mgr;
struct mg_timer t1;
mg_mgr_init(&mgr);
mg_http_listen(&mgr, "http://0.0.0.0:5000", cb, NULL); /* Accepting connections from any IP (0.0.0.0), TCP port 8888 cant be used as it is in use 
							by pigpio WTF */ 
mg_timer_init(&t1, 100, MG_TIMER_REPEAT, timer_callback, &mgr);

while(interrupt)
{
  mg_mgr_poll(&mgr, 50);
}
/* Releasing web server */ 
mg_timer_free(&t1);
mg_mgr_free(&mgr);
sleep(3);

for(i=0;i<3;i++) 
    {
      pthread_join(callThd[i], &status);
    }
for(i=4;i<6;i++) 
    {
      pthread_join(callThd[i], &status);
    }

exit(1);
}
