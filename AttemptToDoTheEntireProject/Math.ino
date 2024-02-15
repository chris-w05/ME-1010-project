//quadratic formula 
float quadratic(float a, float b, float c){
  //float pos = (-b + sqrt(b*b - 4*a*c))/(2*a);
  float neg = (-b - sqrt(b*b - 4*a*c))/(2*a);
  //Serial.print("root: ");
  //Serial.println(neg);
  return neg;
}

//converts an angle in degrees to raidans
float degToRad( float deg ){
  return deg*M_PI/180.0;
}

float radToDeg( float rad){
  return rad * 180/M_PI;
}