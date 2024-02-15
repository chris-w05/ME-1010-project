//PID control for linear slide motor.
//Probably not necessary but would make the robot functional with a more powerful motor
float xControl(){ //PID control for X motor 
  return 255;
  //return constrain( abs(kP*(targetX-xPosition) + kD*dX*CYCLETIME + kI*accumulatedX), 0, 255 );
}

//tempted to use PID here but with the size of the gearbox its not really necessary
float brake(){
  return 0.0;
}

float xPosFromEncoder(){
  if( !stepCounted ){
    //detect if the encoder is more or less in the center of a black or white square (high or low value)
    //add a step if moving in the positve direction
    if( ((analogRead(irPin) > irHighThresh) || (analogRead(irPin) < irLowThresh)) && (motorDirection = LOW) ){
      nEncoderSteps ++;
      stepCounted = true;
      Serial.println("1");
    }
    //remove a step if moving in the negative direction 
    if( ((analogRead(irPin) > irHighThresh) || (analogRead(irPin) < irLowThresh)) && (motorDirection = HIGH) ){
      nEncoderSteps --;
      stepCounted = true;
      Serial.println("1");
    }
  }
  else if( (analogRead(irPin) <= irHighThresh) && (analogRead(irPin) >= irLowThresh) ){
    stepCounted = false;
  }
  return nEncoderSteps * encoderResolution;
}