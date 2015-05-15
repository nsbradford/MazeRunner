/*==========================================================================================
  header.h  
  ==========================================================================================
*/

#ifndef HEADER_H
#define HEADER_H

//==========================================================================================

// enum used to represent state of range sensor, state of fire sensor
typedef enum {tooFar, tooClose, target, outOfRange,fire} distState;
typedef enum {ambiguous, xPos, xNeg, yPos, yNeg} compassState;

// ultrasonic(INPUTPIN, OUTPUTPIN)
ultrasonic sonicFront(26, 27);
ultrasonic sonicLeft(28, 29); 

// lcd display for debugging
LiquidCrystal lcd(40, 41, 42, 43, 44, 45);

// all motors
Servo right;      // drive train
Servo left;       // drive train
Servo firesweep;

// motor pins
const int pinRight = 11;
const int pinLeft = 10;
const int firePin = 9;

// sensor pins
const int pinFire = 0;
const int pinPhotoLeft = 7;

// gyro
L3G gyro;
LSM303 compass;
int master_xVal = 0;
int master_yVal = 0;
compassState myCompassState = yPos;

// important constants
const int ultrasonicMaxRange = 108;
const double distanceError = .2;
const unsigned int photo_threshold = 950;

// drive speeds
const int motorStop = 90;
const int leftForwards = 180;
const int leftBack = 0;
const int rightForwards = 0;
const int rightBack = 180;

const int leftTurn = leftForwards - 60;
const int rightTurn = rightForwards + 60;

// globals
double wallFollow_dist = 5;
double dist_dangerFront = 80; // never get closer to a wall than this
distState state_sonicFront;
distState state_sonicLeft;
bool state_cliff = false;
bool state_fire = false;
bool flag_jump = false;
bool flag_isFireExtinguished = false;
bool fire_found = false;

// fire code
int pos=50; // servo position
const int sensorMin = 0;     // sensor minimum
const int sensorMax = 1024;  // sensor maximum
unsigned int fireCount = 0;

// LCD display
char buf1[12];
char buf2[4];
char currentState[12] = "STATE1"; // hold current state
char currentState2[12] = "state2"; // hold current state2

//==========================================================================================
//================================= ENCODER/GYRO ===========================================
// @author Batu Sipka / Alex Bittle
//==========================================================================================

int distance = 0;                    //distance variables to store data
int distance1 = 0;
int distance2 = 0;
int distance3 = 0;

int t = 0;                //flag for encoder
/* gyro headers */

// LSM303 accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
//Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X))
 //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y))
//Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) 

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -421
#define M_Y_MIN -639
#define M_Z_MIN -238
#define M_X_MAX 424
#define M_Y_MAX 295
#define M_Z_MAX 472

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13 

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer_initial;
long timer24=0; //Second timer used to print values 
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float currentYaw;
float referenceYawLeft;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float notFloat = 0 ;
float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

//flags for timer and setting reference
boolean timerFlag = false; //to be set true after a certain amount of time
boolean referenceFlag = false; //to be set when the reference is set, to be set false after tbe turn is completed

//==========================================================================================

#endif
