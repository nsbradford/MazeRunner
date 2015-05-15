/*==========================================================================================
  TEAM 8: Nicholas S. Bradford, Yesugey Batu Sipka, Alex Bittle, Michael Chan
  RBE 2002 Final Project
  ==========================================================================================
  
  Nicholas Bradford:         Framework, control system, and wall-following.
  Michael Chan:              Fire detection and approach.
  Batu Sipka/Alex Bittle:    Encoder / gyro.
  
  FinalProject.ino
    setup
    loop
    big functions
  encoder.ino         (Batu Sipka / Alex Bittle - not working)
    gyro
      compass
      position (double integral)
    encoders
      interrupts
    tests
  fire_code.ino       
    search for fire
    navigate to fire
  header.h
    enums
    constants
    globals
  helpers.ino
    read sensors
    LCD printouts
    basic motor motion
    debugging code  
  left_corners.ino
    cliffs
    detect left corners
    turn left
    
  ==========================================================================================
*/

#include <Servo.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>
#include <ultrasonic.h>
#include "header.h"

/*==========================================================================================
  @author Nicholas S. Bradford
  Setup all (and gyro), find wall to align with for wall-following 
*/
void setup()
{
  //Serial.begin(9600); // will use pin 0?
  lcd.begin(16, 2); // 16 columns, 2 rows
  right.attach(pinRight);
  left.attach(pinLeft);
  pinMode(pinPhotoLeft, INPUT);
  pinMode(23,OUTPUT);
  pinMode(22,OUTPUT);
  digitalWrite(23,LOW);
  digitalWrite(22,LOW);
  firesweep.attach(firePin);
  attachInterrupt(0,encLISR,RISING);                    //attach interrupts
  attachInterrupt(1,encRISR,RISING);

  /* DEBUG */
  //timerPrint("BEGIN", "PROGRAM");
  //calibrate_sonic_and_light();
  //calibrate_state();
  //calibrate_motors();
  
  gyro_setup();
  findWall(); /* align left side of robot with wall, set wallFollow_dist */
}

/*==========================================================================================
  @author Nicholas S. Bradford
  Follow the wall along the right side. Find the flame and put it out. Return to start.
*/
void loop()
{
  readAllSensors(); /* Read sensors, set states */
  //if (flag_isFireExtinguished){returnToStart();}
  
  lcd.setCursor (10,0);
  lcd.print(distance1);
  lcd.setCursor (10,1);
  lcd.print(distance2);
  lcd.setCursor (13,0);
  lcd.print(distance3);  
  /* check for fire */
  if (state_fire)
  {
    right.write(90);
    left.write(90);
    extinguishFire();
  }
  else
  {
    /* wall-follow in search of fire */
    switch (state_sonicFront)
    {
      case outOfRange: // fall through
      fire_found = false;
      case tooFar:
        followWall();
        fire_found = false;
        break;
      case tooClose: // fall through
      fire_found = false;
      case target:
        followWall_corner();
        fire_found = false;
        break;
    }
  }
  while (fire_found);
  {     
    timerPrint("Fire Found", "LOOP");
      //firepoll();
     // findfire();
     /*
    if (state_sonicLeft==tooClose){
      right.write(60);
      left.write(90);
    }
*/
    if(state_sonicFront==fire)
    {
      right.write(90);
      left.write(90);
      firepoll();
      findfire();
      extinguishFire();
      /*
      firepoll();
      findfire();
      extinguishFire();
      */  
    }
  } // end fire_found loop
  
} // end loop()


/*==========================================================================================
  @author Nicholas S. Bradford
  Find the wall and align for left-side wall-following.
  Gets robot perpendicular to wall, at least 10 inches away on each side.
*/
void findWall()
{
  enc_main(); // stores distance data in 1st variable initially
  
  /* rotate left until you read correctly */
  timerPrint("FIND", "rot Right 1");
  readAllSensors();
  while (isOutOfRange(sonicFront))
  {
    readAllSensors();
    if (state_sonicLeft != outOfRange)
    {
      t =1 ; //store distance data in 2nd variable
      halt();
      return;
    }
    else
      rotateRight();
  }
  halt();
  
  dist_dangerFront = 5;
  
  /* approach wall to 10(?) inches */
  timerPrint(currentState, "forwards");
  readAllSensors();
  while (state_sonicFront == tooFar || state_sonicFront == outOfRange)
  {
    readAllSensors();
    forwards();
  }

  /* rotate until left side is aligned with wall */
  timerPrint(currentState, "rot Right 2");
  rotateRight();
  myDelay(500);
  readAllSensors();
  while (state_sonicLeft == outOfRange)
  {
    readAllSensors();
    rotateRight();
  }
  halt();

  wallFollow_dist = sonicLeft.distance(); /* set wall-follow distance for left side */
}


