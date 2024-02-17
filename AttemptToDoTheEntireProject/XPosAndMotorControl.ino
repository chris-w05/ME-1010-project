//PID control for linear slide motor.
//Probably not necessary but would make the robot functional with a more powerful motor
float xControl(){ //PID control for X motor
  if( (rightSwitchVal != 0 || leftSwitchVal != 0) ) 
  return 255;
  //return constrain( abs(kP*(xGoal-xPosition) + kD*dX + kI*accumulatedX), 0, 255 );
}

//tempted to use PID here but with the size of the gearbox its not really necessary
float brake(){
  return 0.0;
}

void updatePosition(){
  //update position estimate
  float timeElapsed = millis() - lastCycleTime;
  lastXPos = xPosition;
  accumulatedX = xGoal - xPosition;
  xPosition = xPosFromEncoder();
  dX = (xPosition - lastXPos)/timeElapsed;
}

float xPosFromEncoder(){
  if( !stepCounted ){
    //detect if the encoder is more or less in the center of a black or white square (high or low value)
    //add a step if moving in the positve direction
    if( ((irVal > irHighThresh) || (irVal < irLowThresh)) && (motorDirection == LOW) ){
      nEncoderSteps ++;
      stepCounted = true;
    }
    //remove a step if moving in the negative direction 
    if( ((irVal > irHighThresh) || (irVal < irLowThresh)) && (motorDirection == HIGH) ){
      nEncoderSteps --;
      stepCounted = true;
    }
  }
  else if( (irVal <= irHighThresh) && (irVal >= irLowThresh) ){
    stepCounted = false;
  }
  //adds .07 since the switch and IR are not at the same x value
  return (nEncoderSteps * encoderResolution) - encoderOffset;
}