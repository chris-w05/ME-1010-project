
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
