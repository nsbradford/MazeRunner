/*==========================================================================================
 * fire_code.ino
 *
 * Move sensor tower, detect fire, move towards fire
 * @author Michael Chan, Nicholas S. Bradford
 *
  ==========================================================================================
*/

/*==========================================================================================
  @author Michael Chan and Nicholas S. Bradford
  Extinguish the fire.
  Motor code seems to stop working here, so have to manually write values to motors.
*/
void extinguishFire()
{
  fireCount++;
  if (fireCount > 100)
  {
    fire_found = false;
    timerPrint("FIRE", "OUT");
    
    right.write(rightBack);
    left.write(leftBack);
    myDelay(1000);
    
    right.write(rightBack);
    left.write(leftForwards);
    myDelay(1000);
    
    /* TRY TO NAVIGATE BACK TO STARTING POSITION */
    findWall();
    
    /* PRINT X AND Y VALUES (not working; integration buggy) */
    itoa(master_xVal, currentState, 10);
    itoa(master_yVal, currentState2, 10);
    timerPrint(currentState, currentState2);
    exit(0);
  }
  
  int myFire;
  right.write(90);
  left.write(90);
  delay(15);
  //fire_found = true;
    
  /* as long as you're detecting a fire */
  while (state_fire)
  {
    fireCount = 0;
    if (fire_found)
      timerPrint("FOUND", "FIRE");
    if (pos<120)
    {
      right.write(90);
      left.write(90);
      delay(1000);
      
      right.write(60);
      left.write(90);
      delay(1000);
      
      firepoll();
      findfire();
      fire_found = true;
      // there's a problem with readallsensors when used in this funcion
      readAllSensors();
    }
  
    else if (pos>140)
    {
      right.write(90);
      left.write(90);
      delay(1000);
      
      right.write(90);
      left.write(120);
      delay(1000);
      
      firepoll();
      findfire();
      fire_found = true;
      readAllSensors;
    }  
    else // pos is between 110 and 150; aligned in the middle
    {
      right.write(90);
      left.write(90);
      delay(1000);
      
      right.write(0);
      left.write(180);
      delay(500);
      
      firepoll();
      findfire();
      
      right.write(90);
      left.write(90);
      digitalWrite(22,HIGH);
      fire_found = true;
      readAllSensors;
    }
  } // while loop
  
  /* stop motors */
  right.write(90);
  left.write(90);
  firepoll();
  findfire();
  extinguishFire();
}

/*==========================================================================================
  @author Michael Chan,  Alex Bittle 
  Check to see if fire has been found using polling.
 */
void firepoll()
{
  /* Found the fire*/
  if (analogRead(pinFire) <= 50)
  {
    halt();
    state_fire = true;
    digitalWrite(23,HIGH);
    
  }
  /* no fire */
  else
  {
    state_fire = false;
  }
}

/*==========================================================================================
  @author Michael Chan,  Alex Bittle 
  Move servo to search for fire
 */

void findfire()
{   
   if(state_fire == false)
   {     
      if(pos <= 210)
      {
        firesweep.write(pos);
        pos = pos + 5;
      }
      else //pos = 210
      {
        pos = 50; 
        //firesweeping = false;
      }
  }
}

