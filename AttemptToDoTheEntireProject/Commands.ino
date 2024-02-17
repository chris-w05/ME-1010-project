//command to bring launcher to limit switch and sets the current position equal to zero. Also brings trough up to intake height
void zeroX(){
  Serial.println("Zeroing");
  //move left until the left limit switch is contacted
  while (digitalRead(leftSwitchPin) != 1){
    digitalWrite(motorDirPin, HIGH);
    analogWrite( motorPowerPin, 255);

  }
  //indicated the robot is as far left as measureable
  nEncoderSteps = 0;

  //may want to comment this out because it is not necessary for zeroing the linear slide. 
  /*
  while( angleServo.read() != loadAngle ){
    angleServo.write(loadAngle);
  }
  */
  Serial.println("Zeroed");
}

void detectAndSetState(){

  /*psuedocode:
  -> if it has a ball it is on its way to shoot
    -> motor moves int LOW direction (positive X)
    -> servo moves to angle to shoot at
    -> xGoal becomes shootAtX

  -> if it does not have a ball:
    -> if it has completed all shots
      -> motor set to brake
      -> servo can be at any angle
      -> xGoal is irrelevant - not moving

    -> else if the left switch is pressed (at 0)
      -> it is ready to load
      -> motor power set to brake
      -> target X is irrlevant; motor is not moving
      -> open servo or whatever control to drop ball into bot

    -> if it has not completed all shots and it is not in loading position
      -> motor set to LOW direction (negative X)
      -> servo set to the angle to load at

  */
  
  if( hasBall ){ //currentState value of 1
    currentState = TOSHOT;
  }
  else if(shotNum >= numShots){ //currentState value of 4
    currentState = DONE;
  }
  else if( leftSwitchVal == 1){ //currentState value of 3
    currentState = LOADING;
  }
  else{ //currentState value of 3
    currentState = FROMSHOT;
  }
}

// see psuedo code detectAndSet state
void toShot(){
  motorDirection = LOW;
  xGoal = shootAtX;
  digitalWrite( motorDirPin, motorDirection);
  analogWrite( motorPowerPin, xControl());
  angleServo.write(thetaServo(shotAngle));
}

void fromShot(){
  motorDirection = HIGH;
  xGoal = 0;
  digitalWrite( motorDirPin, motorDirection);
  analogWrite( motorPowerPin, xControl());
  angleServo.write(thetaServo(loadAngle));
}

void loading(){
  //GET RID OF THIS WHEN THERE IS A SENSOR TO DETECT THE BALL
  hasBall = true;
  analogWrite( motorPowerPin, brake());
  angleServo.write(thetaServo(loadAngle));
  hasBall = true;
}

void done(){
  analogWrite( motorPowerPin, brake());
}

void setShootAtX(){
  //determine theta and x to shoot from
  //shotAngle = shallowAngleFromRange(d1, d2, shotVel, targetY);
  float shotX = shotList[shotNum].x;
  if( movingMode == WHILEMOVING){
    shootAtX = shotX - launchXOffset(dX, d1, d2, shotVel, shotAngle);
  }
  else{
    shootAtX = shotX;
  }
}

void shoot(){
  Serial.println("Shooting");
  if(movingMode = STATIONARY){
    //ensures the shot has no horizontal velocity
    analogWrite(motorPowerPin, brake());
    delay(100);
  }
  //leave this commented out until state machine is stable
  //digitalWrite(solenoidPin, HIGH);
  shotTime = millis(); //DO NOT DELETE THIS - at risk of destroying solenoid
  hasBall = false;
  shotNum ++; 
  shootAtX = shotList[shotNum].x; 
}

void printStates(){
  if( millis() - lastPrintTime > interval){
    Serial.print("xPos: ");
    Serial.print(xPosition);
    Serial.print("\tState: ");
    Serial.print(currentState);
    Serial.print("\tshotNum: ");
    Serial.print(shotNum);
    Serial.print("\tshootAtX: ");
    Serial.print(shootAtX);
    Serial.print("\txGoal: ");
    Serial.println(xGoal);
    lastPrintTime = millis();
  }
}
