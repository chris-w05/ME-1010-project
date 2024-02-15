//commands to bring launcher to limit switch and sets the current position equal to zero. Also brings trough up to intake height
void zeroX(){
  while (digitalRead(leftSwitchPin) != 1){
    digitalWrite(motorDirPin, 1);
    analogWrite( motorPowerPin, 255);
  }
  xPosition = 0;
  nEncoderSteps = 0;

  //may want to comment this out because it is not necessary for zeroing the linear slide. 
  while( angleServo.read() != loadAngle ){
    angleServo.write(loadAngle);
  }
  Serial.println("Zeroed");
}