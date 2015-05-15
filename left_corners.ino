/*==========================================================================================
  left_corners.ino
  @author Nicholas S. Bradford
  ==========================================================================================
*/

/*==========================================================================================
  @author Nicholas S. Bradford
  Use modified delays to turn 90 degrees around a corner.
*/
void followWall_LEFTCORNER()
{
  timerPrint("Lost wall", "CORNER!");
  unsigned long _delayForwards = 1100; // go forwards
  
  // go forwards enough to avoid hitting the wall when rotating
  forwards();
  myDelay(_delayForwards);
  halt();

  turnLeft90();

  /* Make sure that you're oriented properly.
    If you're not, this will eventually call followWall_LEFTCORNER again 
    (this is 2-level recursive behavior) */
  if (state_sonicLeft != outOfRange)
  {
    wallFollow_dist = sonicLeft.distance(); /* set wall-follow distance for left side */
  }
  /* RETURN TO MAIN LOOP() */
}

/*==========================================================================================
  @author Nicholas S. Bradford
  Make a 90 degree turn left
*/
void turnLeft90()
{
  timerPrint("turn", "LEFT 90");
  unsigned long _delay90 = 2700;
  unsigned long _delayForwards = 1700; // go forwards
  
  left.write(motorStop);
  right.write(rightForwards);

  unsigned long _startTime = millis();
  while(_startTime + _delay90 > millis())
  {
    readAllSensors();
    if (state_cliff)
    {
      turnRight90();
      return;
    }
    else if (state_sonicLeft != outOfRange)
    {
      return; // it's OK! re-found the wall
    } 
  }
  halt();
  /* May be returning from a cliff */
  
  /* Go forwards.
    Make sure you're not running into a cliff */
  _startTime = millis();
  forwards();
  while(_startTime + _delayForwards > millis())
  {
    readAllSensors();
    if (state_cliff)
    {
      turnRight90();
      return;
    }
  }
  halt();
  
  /* RETURN TO followWall_LEFTCORNER() -> main loop */
}

/*==========================================================================================
  @author Nicholas S. Bradford
  If you hit a cliff, Backup and Make a 90 degree turn right
*/
void turnRight90()
{
  timerPrint("turn", "RIGHT 90");
  unsigned long _delayBackwards = 1100;
  unsigned long _delay90 = 2600;
  unsigned long _delayForwards = 5000;
  
  // move backwards to get off of cliff
  backwards();
  myDelay(_delayBackwards);
  halt();
  
  // rotate right
  rotateRight();
  myDelay(_delay90);
  halt();
  
  unsigned long _startTime = millis();
  forwards();
  while(_startTime + _delayForwards > millis())
  {
    readAllSensors();
    if (state_sonicLeft != outOfRange)
    {
      return; 
    }
  }
  halt();
  
  findWall_AFTERCLIFF();
  
  /* RETURN TO followWall_LEFTCORNER() -> main loop */
}

/*==========================================================================================
  @author Nicholas S. Bradford
  Align with wall after the messy cliff code
*/
void findWall_AFTERCLIFF()
{
  dist_dangerFront = 80;
  
  /* rotate left until you read correctly */
  timerPrint("FIND 2", "rot Left 1");
  rotateLeft();
  myDelay(500);
  readAllSensors();
  while (state_sonicFront == outOfRange)
  {
    readAllSensors();
    rotateLeft();
  }
  myDelay(400);
  halt();
  
  dist_dangerFront = 10;
  
  /* approach wall to 5(?) inches */
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

  dist_dangerFront = 5;
  wallFollow_dist = sonicLeft.distance(); /* set wall-follow distance for left side */
}


