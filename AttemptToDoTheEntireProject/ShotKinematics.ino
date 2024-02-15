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

//returns an array of the initial coordinates given a launch angle
Coordinate initialCoordinates(float d1, float d2, float angle){ // returns [x0,y0], angle in radians
  Coordinate coordinates ;
  coordinates.x = d2 * cos(angle);
  coordinates.y = d1 + d2 * sin(angle);
  //Serial.print("initialCoordinates\nFinding coords\nAngle:");
  //Serial.println(coordinates.x);
  return coordinates;
}