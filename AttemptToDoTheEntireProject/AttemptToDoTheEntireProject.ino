//Trying a full solution
#include <Servo.h>

Servo angleServo;
Servo ballDrop;

// pins 
const int angleServoPin = 9;
const int ballDropPin = 8;
const int solenoidPin = 6;
const int motorDirPin =  4;
const int motorPowerPin = 5;
const int irPin = A5;
const int buttonsPin = A7;
const int leftSwitchPin = 12;
const int rightSwitchPin = 11;
const int ballSensorPin = 10; //Needs to be changed


// constants/variables
struct Coordinate{
  float x;
  float y;
};

struct lengthSet{
  float l1;
  float l2;
  float l3;
  float l4;
};

struct servoParams{
  float alpha = 0;
  float beta = 0;
  float thetaL0 = 0;
};

//states
enum state{
  LOADING,
  TOSHOT,
  FROMSHOT,
};

enum shotType{
  FIXEDSHOTVEL,
  FIXEDSHOTANGLE,
};

enum mode{
  WHILEMOVING,
  STATIONARY,
};

//constants
const lengthSet servoBars = {0.0866, 0.025, 0.1060, 0.0750};
const lengthSet launchBars = {0.0866, 0.0750, 0.1060, 0.025};
const servoParams sParams;
const float shotVel = 3.2; // m/s
const float g = 9.81; // m/s^2
const float d1 = 0.0876;
const float d2 = .1190;
const float shotTolerance = .01; //m,  should be between 1/2 and 1 times the encoder resolution
//motor PID:
const float kP = 30;
const float kI = 0.0;
const float kD = 0.3;


const float loadAngle = 120; //deg, check this
const float CYCLETIME = 0.0000625; //time per each iteration of the code
const float encoderResolution = 0.01; // m - need to measure
const float irHighThresh = 590;
const float irLowThresh = 60;

Coordinate shotList[] = {
        {0.01, 1.2},  
        {0.02, .5},  
        {0.02, .3}   
    };


//variables
shotType shotMode = FIXEDSHOTVEL;
mode movingMode = WHILEMOVING;
state currentState;
int shotNum = 0;
float targetX = 0;
float targetY = 0;

bool hasBall;
int motorDirection;
int nEncoderSteps = 0;
bool stepCounted = false;
float lastXPos = 0;
float xPosition = 0;
float accumulatedX = 0;
float dX = 0;
float shootAtX;
float shotAngle;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("findingAngle");
  targetX = shotList[0].x;
  targetY = shotList[0].y;

  shotAngle = shallowAngleFromRange(d1, d2, shotVel, targetX);

  Serial.print("Target: ");
  Serial.print(targetX);
  Serial.print(", ");
  Serial.println(targetY);
  Serial.print("Launch angle: ");
  Serial.println(shotAngle);


  shotAngle = steepAngleFromRange(d1, d2, shotVel, targetX);
  //shotAngle = 110;

  Serial.print("Target: ");
  Serial.print(targetX);
  Serial.print(", ");
  Serial.println(targetY);
  Serial.print("Launch angle: ");
  Serial.print(shotAngle);

  currentState = LOADING;
  hasBall = false;

  pinMode(solenoidPin, OUTPUT);
  pinMode(leftSwitchPin, INPUT_PULLUP);
  pinMode(rightSwitchPin, INPUT_PULLUP);

  angleServo.attach(angleServoPin);

  //Move motors to starting configuration
  Serial.println("Seroingh");
  zeroX();

}

void loop() {
  // put your main code here, to run repeatedly:
  xPosition = xPosFromEncoder();

  if( hasBall ){
    currentState = TOSHOT;
    motorDirection = LOW;
  }
  else if( xPosition >= encoderResolution + 0.001){
    currentState = FROMSHOT;
    motorDirection = HIGH;
    targetX = 0;
  }
  else{
    currentState = LOADING;
    //GET RID OF THIS WHEN THERE IS A SENSOR TO DETECT THE BALL
    hasBall = true;
  }

  //determine theta and x to shoot from
  //shotAngle = shallowAngleFromRange(d1, d2, shotVel, targetY);
  if( movingMode = WHILEMOVING){
    shootAtX = launchXOffset(dX, d1, d2, shotVel, shotAngle);
  }
  else{
    shootAtX = targetX;
  }


  //what to do depending on mode
  switch( currentState){
    LOADING:
    angleServo.write(loadAngle);
    hasBall = true;


    TOSHOT:
    digitalWrite( motorDirPin, motorDirection);
    analogWrite( motorPowerPin, xControl());
    angleServo.write(thetaServo(shotAngle));


    FROMSHOT:
    digitalWrite( motorDirPin, motorDirection);
    analogWrite( motorPowerPin, xControl());
    angleServo.write(thetaServo(loadAngle));
  }

  //shooting
  if( (xPosition > shootAtX - shotTolerance) && 
  (xPosition < shootAtX + shotTolerance)){
    if(movingMode = STATIONARY){
      //ensures the shot has no horizontal velocity
      analogWrite(motorPowerPin, brake());
      delay(100);
    }
    digitalWrite(solenoidPin, HIGH);
    delay(30);
    digitalWrite(solenoidPin, LOW);
    hasBall = false;
    shotNum ++;
  }
  else{
    digitalWrite(solenoidPin, LOW);
  }

  //update position estimate
  dX = xPosition - lastXPos;
  lastXPos = xPosition;
  
  accumulatedX = targetX - xPosition;
}

