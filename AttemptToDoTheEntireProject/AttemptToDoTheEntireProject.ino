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
  else if( xPosition >= ecoderResolution + 0.001){
    currentState = FROMSHOT;
    motorDirection = HIGH;
    targetX = 0;
  }
  else{
    currentState = LOADING;
    //GET RID OF THIS WHEN THERE IS A SENSOR TO DETECT THE BALL
    hasBall = true
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
    angleServo.write(thetaServo.write(shotAngle));


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

// --------------------------------------------------------------------------

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

//finds the current step of the encoder. resolution is limited to the size of each square on the encoder strip
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

//input angle in degrees
float thetaFour( float inputAngle){
  //Find l1-4 from lengths
  float inputRad = degToRad(inputAngle);
  //find K1 - 3
  float K1 = servoBars.l1/servoBars.l2;
  float K2 = servoBars.l1/servoBars.l4;
  float K3 = (servoBars.l1 * servoBars.l1 + servoBars.l2*servoBars.l2 - servoBars.l3
  *servoBars.l3 + servoBars.l4*servoBars.l4)/(2*servoBars.l2*servoBars.l4);

  //Compute A, B, C coeffecients
  float A = cos(inputRad) - K1 - K2*cos(inputRad) + K3;
  float B = -2 * sin(inputRad);
  float C = K1 - (K2 + 1)*cos(inputRad) + K3;
  //compute theta4 using quadratic function 
  float theta4 = 2 * atan(quadratic(A, B, C));
  return radToDeg(theta4);
}

float inverseThetaFour( float inputAngle){
  //Find l1-4 from lengths
  float inputRad = degToRad(inputAngle);
  //find K1 - 3
  float K1 = launchBars.l1/launchBars.l2;
  float K2 = launchBars.l1/launchBars.l4;
  float K3 = (launchBars.l1 * launchBars.l1 + launchBars.l2*launchBars.l2 - launchBars.l3
  *launchBars.l3 + launchBars.l4*launchBars.l4)/(2*launchBars.l2*launchBars.l4);

  //Compute A, B, C coeffecients
  float A = cos(inputRad) - K1 - K2*cos(inputRad) + K3;
  float B = -2 * sin(inputRad);
  float C = K1 - (K2 + 1)*cos(inputRad) + K3;
  //compute theta4 using quadratic function 
  float theta4 = 2 * atan(quadratic(A, B, C));
  return radToDeg(theta4);
}

float thetaServo(float thetaL){
  float theta2 = thetaL + sParams.thetaL0;
  float theta4 = inverseThetaFour(theta2);
  float thetaS = (theta4 + sParams.alpha)/(1 - sParams.beta);
  return constrain(thetaS, 0, 180);
}

//input angle in degrees
float thetaLaunch(float thetaS ){
  float thetaSC = sParams.alpha + sParams.beta*thetaS;
  float theta2 = 180 - thetaS + thetaSC;
  float theta4 = thetaFour( theta2 );
  float thetaL = 180 - theta4 - sParams.thetaL0;
  return thetaL;
}

//quadratic formula 
float quadratic(float a, float b, float c){
  //float pos = (-b + sqrt(b*b - 4*a*c))/(2*a);
  float neg = (-b - sqrt(b*b - 4*a*c))/(2*a);
  //Serial.print("root: ");
  //Serial.println(neg);
  return neg;
}

//finds the offset on the linear slide to launch while moving 
float launchXOffset( float vx0, float d1, float d2, float v0, float angle ){ // angle in deg
  angle = degToRad(angle);
  Coordinate initCords =  initialCoordinates( d1, d2, angle);
  float a = g*.5;
  float b = v0 * sin(angle);
  float c = initCords.y;
  return vx0 * quadratic( a, b, c);
}

//finds the angle that maximizes a range for a given velocity
float optimizeRange(float d1, float d2, float v0){ //returns angle in deg
  const float stepSize = 0.05;
  float lastDistance = 0;
  float launchAngle = 0;
  float distance = 0;
  launchAngle += stepSize;
  //set variable distance in the condition of the while statement to minimize number of times LandingDistance is called.
  //otherwise the program will call LandingDistance for values of launchAngle that are already known.
  while( lastDistance < (distance = landingDistance(d1, d2, v0, launchAngle))){
    lastDistance = distance;
    launchAngle += stepSize;
  }
  //Serial.println(launchAngle);
  return launchAngle;
}

//finds the best angle to launch to hit a target at a specified distance
float shallowAngleFromRange(float d1, float d2, float v0, float targetDistance){ //returns angle in deg
  const float stepSize = 0.01;
  float launchAngle = 0;
  float lastDistance = 0;
  float distance = .001;
  while( (distance < targetDistance) && (launchAngle < 90) ){
    launchAngle += stepSize;
    distance = landingDistance(d1, d2, v0, launchAngle);
    //Serial.print(launchAngle);
    //Serial.print("\t");
    //Serial.print(distance);
    //Serial.println();
  }
  return launchAngle;
}

float steepAngleFromRange(float d1, float d2, float v0, float targetDistance){ //returns angle in deg
  const float stepSize = 0.05;
  float launchAngle = 90;
  float lastDistance = 0;
  float distance = .001;
  while( (distance < targetDistance) && (launchAngle > -90) ){
    launchAngle -= stepSize;
    distance = landingDistance(d1, d2, v0, launchAngle);
    //Serial.print(launchAngle);
    //Serial.print("\t");
    //Serial.print(distance);
    //Serial.println();
  }
  return launchAngle;
}

//finds the distance the ball will land given the luanch angle and velocity
float landingDistance(float d1, float d2, float v0, float angle){// angle in degrees
  angle = degToRad(angle);
  Coordinate initCords = initialCoordinates( d1, d2, angle );
  //Serial.println("Quadratic:");
  float a = -g*.5;
  float b = v0 * sin(angle);
  float c = initCords.y;
  //Serial.print("a: ");
  //Serial.print(a);
  //Serial.print(" b: ");
  //Serial.print(b);
  //Serial.print(" c: ");
  //Serial.println(c);
  return initCords.x + cos(angle) * v0 * quadratic(a, b , c);
}

//converts an angle in degrees to raidans
float degToRad( float deg ){
  return deg*M_PI/180.0;
}

float radToDeg( float rad){
  return rad * 180/M_PI;
}

//returns an array of the initial coordinates given a launch angle
Coordinate initialCoordinates(float d1, float d2, float angle){ // returns [x0,y0], angle in radians
  Coordinate coordinates ;
  coordinates.x = d2 * cos(angle);
  coordinates.y = d1 + d2 * sin(angle);
  //Serial.print("initialCoordinates\nFinding coords\nAngle:");
  //Serial.println(coordinates.x);
  return coordinates;
}
