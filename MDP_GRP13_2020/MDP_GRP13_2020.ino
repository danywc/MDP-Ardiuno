void setup() {
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  setupMotorEncoder();
  setupPID();
}

void loop() {
  //md.setSpeeds(300,300); // // L//R//default right wheel is faster

  /*moveForward(5);
    turnLeft(90);
    moveForward(5);*/
  //moveBack()
  //moveForward(9);
  //rotateRight(7,1);
  //rotateLeft(7,1);


  //turnLeft(90);
  //turnRight(90);
  //turnRightDeg(45);
  //turnLeftDeg(45);

  //AvoidFrontObstacleDiagonalMovement(15);

  //AvoidFrontObstacle(15);


  /*int i = 0;
    while(i <10){
    moveForward(1);
    i++;
    delay(500);
    }*/
  /*moveForward(5);
    calibrateRightwall();
    caliFlat();
    turnLeft(90);
    moveForward(5);*/

  //turnRight(90);
  //delay(4000);
  //initializeMotor_End();

  //getMedianA0();
  //getMedianA2();




  moveForward(5);
  calibrateRightwall();
  moveForward(5);
  calibrateRightwall();
  moveForward(6);
  caliFlat();
  turnLeft(90);
  delay(4000);
  calibrateRightwall();
  moveForward(5);
  calibrateRightwall();
  moveForward(5);
  calibrateRightwall();
  moveForward(1);
  caliFlat();
  turnLeft(90);
  delay(4000);

  calibrateRightwall();
  moveForward(5);
  calibrateRightwall();
  moveForward(5);
  calibrateRightwall();
  moveForward(6);
  caliFlat();
  turnLeft(90);
  delay(4000);
  calibrateRightwall();
  moveForward(5);
  calibrateRightwall();
  moveForward(5);
  calibrateRightwall();
  moveForward(1);
  caliFlat();




  /*
    calibrateRightwall();
    moveForward(5);*/

  //turnRight(1000);
  //getMedianA1();
  //getMedianA3();
  //getMedianA4();
  //getMedianA5();

  //calibrateFrontObstacle();
  //turnLeft(90);
  //turnRight(90);

  delay(4000);

}
