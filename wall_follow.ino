/*==========================================================================================
  wall_follow.ino  
  ==========================================================================================
*/

/*==========================================================================================
  @author Nicholas S. Bradford
  Master controller for Wall-following.
*/
void followWall()
{
  enc_main(); // stores data in second variable
  timerPrint("FOLLOW", "");
  readAllSensors();
  switch (state_sonicLeft)
  {
    case tooFar:
      turnLeft();
      break;
    case tooClose:
      turnRight();
      break;
    case target:
      forwards();
      break;
    case outOfRange: /* orient to become perpendicular again */
      followWall_reorient();
      break;
  }
}

/*==========================================================================================
  @author Nicholas S. Bradford
  Begin in the corner (left side), turn right to face out of the corner
*/
void followWall_corner()
{
  timerPrint("CORNER", "orient");
  rotateRight();
  t=2;                      //stores distance data in 3rd variable
  enc_main();
  myDelay(750);

  readAllSensors();
  while (state_sonicLeft == outOfRange)
  {
    rotateRight();
    readAllSensors();
  }
  halt();
  
  wallFollow_dist = sonicLeft.distance(); /* set wall-follow distance for left side */
}

/*==========================================================================================
  @author Nicholas S. Bradford
  If left sensor began reading out of range, reorient with wall. 
  If necessary, may need to make 90 degree turn left, and possibly another 90 degree turn.
*/
void followWall_reorient()
{
  /* check for a radical jump in ultrasonic sensor */
  
  timerPrint("Lost wall", "ORIENT");
  unsigned long _delay1 = 100; // time to nudge, rotate right
  unsigned long _delay2 = 400; // time to nudge, rotate left
  unsigned long _delay3 = 200; // get back to original orientation

  /* check while rotating right */
  unsigned long _startTime = millis();
  rotateRight();
  while(_startTime + _delay1 > millis())
  {
    timerPrint("Lost wall", "TURN RIGHT");
    readAllSensors();
    if (state_sonicLeft != outOfRange)
    {
      halt();
      wallFollow_dist = sonicLeft.distance(); //set wall-follow distance for left side
      return; // it's OK! re-found the wall
    }
  }
  halt();
 
  /* check while rotating left */
  _startTime = millis();
  rotateLeft();
  while(_startTime + _delay2 > millis())
  {
    timerPrint("Lost wall", "TURN LEFT");
    readAllSensors();
    if (state_sonicLeft != outOfRange)
    {
      halt();
      wallFollow_dist = sonicLeft.distance(); /* set wall-follow distance for left side */
      return; // it's OK! re-found the wall
    }
  }
  halt();

  /* this should get you straight again */
  _startTime = millis();
  rotateRight();
  while(_startTime + _delay3 > millis())
  {
    timerPrint("Lost wall", "TURN RIGHT");
    readAllSensors();
    if (state_sonicLeft != outOfRange)
    {
      halt();
      wallFollow_dist = sonicLeft.distance(); /* set wall-follow distance for left side */
      return; // it's OK! re-found the wall
    }
  }
  halt();
  
  /* If you haven't been realigned, it means you saw*/
  followWall_LEFTCORNER();
}

