void readSensors(){
  leftSwitchVal = digitalRead(leftSwitchPin);
  rightSwitchVal = digitalRead(rightSwitchPin);
  irVal = analogRead(irPin);
  buttonVal = readButtons();
  angleServoPos = angleServo.read();
  //hasBall = 
}


int cibn = 0;
int buttonNumber = 0;

int readButtons(){
  // Keep track of the previous button number
  int prevButtonNumber = buttonNumber;
  // Read the button value
  int buttonValue = analogRead(buttonsPin);
  // Determine the button number that corresponds to the button value
  if( buttonValue < 5){
    buttonNumber = 1;
  }
  else if (buttonValue > 115 && buttonValue < 175 ){
    buttonNumber = 2;
  }
  else if (buttonValue > 305 && buttonValue < 365){
    buttonNumber = 3;
  }
  else if (buttonValue > 480 && buttonValue < 540){
    buttonNumber = 4;
  }
  else if (buttonValue > 715 && buttonValue < 775){
    buttonNumber = 5;
  }
  else if (buttonValue > 1015 ){
    buttonNumber = 0;
  }
  else{
    buttonNumber = -1;
  }

  int buttonPressed = 0;
  // Determine if the button number is an actual button press or a bounce
  if( buttonNumber == prevButtonNumber){
    cibn ++;
    if(cibn > 100){
       buttonPressed = buttonNumber;
    }
  }
  else{
    buttonPressed = 0;
    cibn = 0;
  }
  return buttonPressed;
}

