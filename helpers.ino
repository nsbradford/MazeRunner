/*==========================================================================================
  helpers.ino
  @author Nicholas S. Bradford

  Contains basic helper functions; requires very little editing.
  ==========================================================================================
*/

/*==========================================================================================
  @author Nicholas S. Bradford      
  Delay safely without causing problems with interrupts.
*/
void myDelay(unsigned long delayTime)
{
  unsigned long _startTime = millis();
  while (_startTime + delayTime > millis())
  {
    readAllSensors(); /* waste time */
  }
}

/*==========================================================================================
  @author Nicholas S. Bradford      
  Read and set global sensor states. Print to LCD display.
*/
void readAllSensors()
{
  state_sonicFront = ultrasonicState(sonicFront, dist_dangerFront);
  state_sonicLeft = ultrasonicState(sonicLeft, wallFollow_dist);
  state_cliff = readPhotoSensor();
  firepoll();
  findfire(); 
  //state_fire = readFireSensor();
   
  timerPrint(currentState, currentState2);
}

/*==========================================================================================
  @author Nicholas S. Bradford, Michael Chan     
  Returns if the fire is closeFire, farFire, or noFire
*/
bool readFireSensor()
{
  bool answer;
  int sensorReading = analogRead(A0); /* read the sensor on analog A0 */
  
  /* Michael Chan's work: */
  
  return false;
}

/*==========================================================================================
  @author Nicholas S. Bradford, Michael Chan     
  Returns true if a cliff was found
*/
bool readPhotoSensor()
{
  // found a cliff
  if (analogRead(pinPhotoLeft) > photo_threshold)
    return true;
  // no cliff
  else
    return false;
}

/*==========================================================================================
  @author Nicholas S. Bradford      DONE
  Tells if the sensor is tooFar, tooClose, or in target of its goal
*/
distState ultrasonicState(ultrasonic sensor, double goal/* = ultrasonicMaxRange*/)
{
  double _dist = sensor.distance();
  distState answer;
  
  if ((_dist > ultrasonicMaxRange) || (_dist > goal + 10))
  {
    answer = outOfRange;
  }
  else if (  (_dist >= goal - distanceError) &&
        (_dist <= goal + distanceError))
  {
    answer = target;
  }
  else if (_dist > goal)
  {
    answer = tooFar;
  }
  else if (_dist < goal)
  {
    answer = tooClose;
  }
  else if (_dist<=goal-10)
  {
    answer= fire;
  }
  else
  {
    //error
  }
  
  return answer;
}

/*==========================================================================================
  @author Nicholas S. Bradford      DONE
  Tells if sensor is out of range (detects more than 108 inches away)
*/
bool isOutOfRange(ultrasonic sensor)
{
  if (sensor.distance() > ultrasonicMaxRange)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*==========================================================================================
  @author Nicholas S. Bradford      DONE
  Stop driving. No delay.  
*/
void halt()
{
  left.write(motorStop);
  right.write(motorStop);
}

/*==========================================================================================
  @author Nicholas S. Bradford      DONE
  Drive forwards. No delay
*/
void forwards()
{
  left.write(leftForwards);
  right.write(rightForwards);
}

/*==========================================================================================
  @author Nicholas S. Bradford      DONE
  Drive backwards. No delay
*/
void backwards()
{
  left.write(leftBack);
  right.write(rightBack);
}

/*==========================================================================================
  @author Nicholas S. Bradford      DONE
  Rotate right. No delay
*/
void rotateRight()
{
  left.write(leftForwards);
  right.write(rightBack);
}

/*==========================================================================================
  @author Nicholas S. Bradford      DONE
  Rotate left. No delay
*/
void rotateLeft()
{
  left.write(leftBack);
  right.write(rightForwards);
}

/*==========================================================================================
  @author Nicholas S. Bradford      DONE
  Turn right. No delay
*/
void turnRight()
{
  left.write(leftForwards);
  right.write(rightTurn);
}

/*==========================================================================================
  @author Nicholas S. Bradford      DONE
  Turn left. No delay
*/
void turnLeft()
{
  left.write(leftTurn);
  right.write(rightForwards);
}

/*==========================================================================================
  @author Nicholas S. Bradford		DONE
  Display light sensor readings
*/
void calibrate_sonic_and_light()
{
  double d1, d2;
  while (true)
  {
    /*
    d1 = sonicFront.distance();
    d2 = sonicLeft.distance();
    sprintf(buf1, "F:%5f L:%5f", d1, d2);    
    //Serial.println(buf1);
    //lcd.setCursor(0, 0);
    //lcd.print(buf1);
    Serial.print("F: ");
    Serial.print(d1);
    Serial.print("   L: ");
    Serial.println(d2);
    delay(10);
    */
    sprintf(buf2, "L: %5u", analogRead(pinPhotoLeft)); 
    Serial.println(analogRead(pinPhotoLeft));   
    lcd.setCursor(0, 1);
    lcd.print(buf2);
  }
}

/*==========================================================================================
  @author Nicholas S. Bradford      DONE
  Calibrate state display
*/
void calibrate_state()
{
  while (true)
  {
    readAllSensors();
    timerPrint(currentState, currentState2);
  }
}

/*==========================================================================================
  @author Nicholas S. Bradford    	DONE 
  Calibrate motors
*/
void calibrate_motors()
{
  timerPrint("MOTORS", "forwards");
  forwards();
  delay(3000);
  timerPrint(currentState, "backwards");
  backwards();
  delay(3000);
  timerPrint(currentState, "rotRight");
  rotateRight();
  delay(3000);
  timerPrint(currentState, "rotLeft");
  rotateLeft();
  delay(3000);
  exit(0);
}

/*==========================================================================================
  @author Nicholas S. Bradford      DONE
  Print to LCD: timerPrint(currentState, currentState2)
*/
void timerPrint(char newState[], char newState2[])
{  
  sprintf(buf1, "%11s", newState);   
  lcd.setCursor(0, 0);
  lcd.print(buf1);
  
  sprintf(buf1, "%11s", newState2);
  lcd.setCursor(0, 1);
  lcd.print(buf1);

  switch(state_sonicFront)
  {
    case tooFar:
      sprintf(buf2, "Far");
      break;
    case tooClose:
      sprintf(buf2, "Clo");
      break;
    case target:
      sprintf(buf2, "Tar");
      break;
    case outOfRange:
      sprintf(buf2, "OoR");
      break;
  }
  lcd.setCursor(12, 0);
  lcd.print(buf2);

  switch(state_sonicLeft)
  {
    case tooFar:
      sprintf(buf2, "Far");
      break;
    case tooClose:
      sprintf(buf2, "Clo");
      break;
    case target:
      sprintf(buf2, "Tar");
      break;
    case outOfRange:
      sprintf(buf2, "OoR");
      break;
  }
  lcd.setCursor(12, 1);
  lcd.print(buf2);    
  
  sprintf(currentState, "%11s", newState);
  sprintf(currentState2, "%11s", newState2);
}

