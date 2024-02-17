//Trying a full solution
#include <Servo.h>

Servo angleServo;
Servo ballDrop;

// pins
const int angleServoPin = 9;
const int ballDropPin = 8;
const int solenoidPin = 6;
const int motorDirPin = 4;
const int motorPowerPin = 5;
// sensor pins
const int irPin = A5;
const int buttonsPin = A7;
const int leftSwitchPin = 12;
const int rightSwitchPin = 11;
const int ballSensorPin = 10;  //Needs to be changed


//Structures -----------------------------------------------------------------------------------------------------
struct Coordinate {
  float x;
  float y;
};

struct lengthSet {
  float l1;
  float l2;
  float l3;
  float l4;
};

struct servoParams {
  float alpha = 0;
  float beta = 0;
  float thetaL0 = 0;
};

//enums       -----------------------------------------------------------------------------------------------------
enum state {
  LOADING,
  TOSHOT,
  FROMSHOT,
  DONE,
};

enum mode {
  WHILEMOVING,
  STATIONARY,
};

//constants   -----------------------------------------------------------------------------------------------------
//four bar control
const lengthSet servoBars = { 0.0866, 0.025, 0.1060, 0.0750 };
const lengthSet launchBars = { 0.0866, 0.0750, 0.1060, 0.025 };
const servoParams sParams;
const float loadAngle = 120;           //deg, check this

//trajectory constants
const float shotVel = 3.2;  // m/s
const float g = 9.81;       // m/s^2
const float d1 = 0.0876;
const float d2 = .1190;

//
const float shotTolerance = .02;  //m,  should be between 1/2 and 1 times the encoder resolution

//motor PID (completely over the top and probably wont be used):
const float kP = 30;
const float kI = 0.0;
const float kD = 0.3;

//encoder constants
const float encoderOffset = 0.06;
const float encoderResolution = 0.01;  // m - need to measure
const float irHighThresh = 660;
const float irLowThresh = 55;

//variables   -----------------------------------------------------------------------------------------------------
//shot control
Coordinate shotList[] = {
  { 0.1, 1.2 },
  { 0.2, .5 },
  { 0.3, .3 }
};
const int numShots = 3;
int shotNum = 0;
float shotAngle;

//state control
mode movingMode = STATIONARY;
state currentState;
bool hasBall;

//motor/positional control
int motorDirection;
int nEncoderSteps = 0;
bool stepCounted = false;
float lastXPos = 0;
float xGoal = 0;
float xPosition = 0;
float accumulatedX = 0;
float dX = 0;
float shootAtX;

//time stuff
unsigned long lastPrintTime = 0;
unsigned long lastCycleTime = 0;
unsigned long shotTime = 0;
int interval = 500;

//sensor values
bool leftSwitchVal;
bool rightSwitchVal;
int irVal;
int buttonVal;
float angleServoPos;


//setup -----------------------------------------------------------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  xGoal = shotList[0].x;

  shotAngle = 45;
  shotNum = 0;

  currentState = LOADING;
  hasBall = true;

  pinMode(solenoidPin, OUTPUT);
  pinMode(leftSwitchPin, INPUT_PULLUP);
  pinMode(rightSwitchPin, INPUT_PULLUP);

  angleServo.attach(angleServoPin);

  zeroX();
}

//loop  -----------------------------------------------------------------------------------------------------
void loop() {
  /*
  psuedocode:
  -> read sensors to find data for current iteration of loop
  -> update the position estimate
  -> update the state using the above two
  -> (optional) print state
  -> run command associated with current state
  -> if x position (updated from updateposition) is > than than the target X and the mode is in TOSHOT
    -> fire cannon
    -> after 30 ms retract servo as not to burn it out
  */
  readSensors(); //this is the only place the sensor values should be updated from
  updatePosition(); //if this is not called every loop position data will be innacurate
  detectAndSetState();
  printStates();

  //what to do depending on mode
  switch (currentState) {
    case LOADING:
      loading();
      break;

    case TOSHOT:
      toShot();
      break;

    case FROMSHOT:
      fromShot();
      break;

    case DONE:
      done();
      break;
  }

  //shooting
  if((xPosition > shotList[shotNum].x) && (currentState == TOSHOT )){
    //take shot 
    shoot();
    //find where next shot needs to be taken
    setShootAtX();
  }
  else {
    if( millis() - shotTime > 30 ){
      digitalWrite(solenoidPin, LOW);
    }
  }

}
