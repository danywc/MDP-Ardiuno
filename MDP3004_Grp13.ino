#include "DualVNH5019MotorShield.h"
#include "PID_v1.h"
#include "EnableInterrupt.h"
#include "RunningMedian.h"
#include <SharpIR.h>
#include <RunningMedian.h>


#define model 1080
int analogPin1=A0;
int analogPin2=A1;
int analogPin3=A2;
int analogPin4=A3;
int analogPin5=A4;
int analogPin6=A5;
int val=0;
const int NUM_SAMPLE_MEDIAN=20;
double ir1=0;


  const int RIGHT_PULSE = 3;     //Pin for right encoder
  const int LEFT_PULSE = 11;     //Pin for left encoder
  DualVNH5019MotorShield md;
  const double turnSpeed = 300;
  const double forSpeed = 300;
  float offset1 = 0;                     // Offsets for the front 3 sensors for calibration
  float offset2 = 0;
  float offset3 = 0;
  double desiredDistanceSensor = 13.5; 
  double desiredDistanceSensorforRightwallCali = 14.5; 
  
  
  double tick_R = 0;
  double tick_L = 0;
  double speed_O = 0;
  double previous_tick_R = 0;
  double previous_error = 0;
  const double kp =  4, ki = 5, kd = 0.00;
  const double kp2 = 19, ki2 = 5 , kd2 = 0; // PID for above 10 cm 
  PID myPID(&tick_R, &speed_O, &tick_L, kp, ki, kd, REVERSE);
  PID myPID2(&tick_L, &speed_O, &tick_R, kp2, ki2, kd2, DIRECT);
  int TURN_TICKS_L = 970 ;   // 970 is for checklist // 930 for turning left 90 degree
  int TURN_TICKS_R = 930;  //   
  const int sample = 19;

  const int LEFTTICK[18] = {200, 200, 100, 0, 400, 450, 500, 55, 500, 65, 700, 75, 600, 85, 925,0, 730, 500};
  int Ticks[11] = {570, 1175, 1765, 2380, 2950, 3565, 4155, 4735, 5340, 0,6}; //at speed 300 //calibrate again once added power bank and RPI

  
  float RPM1;
 float RPM2;



 ///////////////////////////////////////////////////////////////////////////////SENSORS/////////////////////////////////////////////////////////////////

  double getMedianA0(){
 double getMedianValue;

    RunningMedian med= RunningMedian(sample);
    for(int n=0;n<sample;n++){
      med.add(getShortRangeA0());
      }
      getMedianValue=med.getMedian();
        Serial.println("Median value of A0");
      Serial.println((0.95*getMedianValue));
      return (0.95*getMedianValue);
      
  }
  
  double getMedianA1(){
 double getMedianValue;

    RunningMedian med= RunningMedian(sample);
    for(int n=0;n<sample;n++){
      med.add(getShortRangeA1());
      }
      getMedianValue=med.getMedian();
        Serial.println("Median value of A1");
      Serial.println(0.95*getMedianValue);
      return (0.95*getMedianValue);
  }

double getMedianA2(){
 double getMedianValue;

    RunningMedian med= RunningMedian(sample);
    for(int n=0;n<sample;n++){
      med.add(getShortRangeA2());
      }
      getMedianValue=med.getMedian();
        Serial.println("Median value of A2");
      Serial.println(0.89*getMedianValue);
      return (0.89*getMedianValue);
  }
  
 double getMedianA3(){
 double getMedianValue;

    RunningMedian med= RunningMedian(sample);
    for(int n=0;n<sample;n++){
      med.add(getShortRangeA3());
      }
      getMedianValue=med.getMedian();
        Serial.println("Median value of A3");
      Serial.println(0.9*getMedianValue);
      return (0.9*getMedianValue);
  }
  
  
  
 double getMedianA4(){
 double getMedianValue;

    RunningMedian med= RunningMedian(sample);
    for(int n=0;n<sample;n++){
      med.add(getShortRangeA4());
      }
      getMedianValue=med.getMedian();
        Serial.println("Median value of A4");
      Serial.println(0.95*getMedianValue);
      return (0.95*getMedianValue);
  }

  double getMedianA5(){
 double getMedianValue;

    RunningMedian med= RunningMedian(sample);
    for(int n=0;n<sample;n++){
      med.add(getLongRangeNew2());
      }
      getMedianValue=med.getMedian();
        Serial.println("Median value of A5");
      Serial.println(0.94*getMedianValue);
      return (0.94*getMedianValue);
  }

  double getShortRangeA0(){
      return abs( (1/(0.00018401809006023533*(analogRead(analogPin1))+(-0.003279242753008803))) - 1  );
      }
      double getShortRangeA1(){
      return abs( (1/(0.00018401809006023533*(analogRead(analogPin2))+(-0.003279242753008803))) - 1  );
      }
        double getShortRangeA2(){
      return abs( (1/(0.00017919115658212846*(analogRead(analogPin3))+(-0.0004533890357601344))) - 1  );
      }
        double getShortRangeA3(){
      return abs( (1/(0.00019745944011122094*(analogRead(analogPin4))+(-0.008001111286848248))) - 0.5  );
      }
        double getShortRangeA4(){
      return abs( (1/(0.00017924334790086593*(analogRead(analogPin5))+(-0.002329940347835141))) - 1  );
      }

      double getLongRange(){
        return abs((1/(0.000055660449724735444*(analogRead(analogPin6))+(-0.0005342773671003311))) - 10  );
        }
        double getLongRangeNew(){
        return abs((1/(0.000058502676165925303*(analogRead(analogPin6))+(-0.0011225182818268035))) - 10  );
        }
        double getLongRangeNew2(){
        return abs((1/(0.00003839571932781073*(analogRead(analogPin6))+(0.003883936543500112))) - 21  );
        }

 ///////////////////////////////////////////////////////////////////////////////SENSORS/////////////////////////////////////////////////////////////////



  ///////////////////////////////////////////////////////////////////////////////MOTORS/////////////////////////////////////////////////////////////////


void setupMotorEncoder() {
  md.init();
  pinMode(LEFT_PULSE, INPUT);
  pinMode(RIGHT_PULSE, INPUT);
  enableInterrupt(LEFT_PULSE, leftEncoder, CHANGE);
  enableInterrupt(RIGHT_PULSE, rightEncoder, CHANGE);
}

void leftEncoder() {
  tick_L++;
}

void rightEncoder() {
  tick_R++;
}


void setupPID() {
 
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-400, 400);
  myPID.SetSampleTime(5);
  
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-400, 400);
  myPID2.SetSampleTime(5);
}

void initializeTick(){
  tick_R = 0;
  tick_L = 0;
  speed_O = 0;
  previous_tick_R = 0;
}

void initializeMotor_Start() {
  md.setSpeeds(0, 0);
  md.setBrakes(0, 0);
}

void initializeMotor_End() {
  md.setSpeeds(0, 0);
  md.setBrakes(400, 400); //L//R
  delay(50);
}

void moveForward(int distance) {
  initializeTick();
  initializeMotor_Start();
  int counter = 0;
  double currentSpeed = forSpeed;
  int distancee = Ticks[distance - 1];
  RPM1 = 0;
  RPM2 = 0;
  while (tick_R<= distancee || tick_L <=distancee) {

      
     if (myPID2.Compute())
          md.setSpeeds(currentSpeed + 2*speed_O , currentSpeed);
      double pulse1 = pulseIn(RIGHT_PULSE,HIGH);
      double pulse2 = pulseIn(LEFT_PULSE,HIGH);
      RPM1 = RPM1 + ((1/(pulse1*2))*(pow(10,6))*60)/562.25;
      RPM2 = RPM2 + ((1/(pulse2*2))*(pow(10,6))*60)/562.25;

      //Serial.print((float)RPM1);
      //Serial.print("    ");
      //Serial.println((float)RPM2); 
      //Serial.println((double)speed_O); 

     
     counter++;
     
    }
    initializeMotor_End();

      float avgOfRPM1 = 0;
      float avgOfRPM2 = 0;
      avgOfRPM1 = RPM1/counter;
      avgOfRPM2 = RPM2/counter;
      
      Serial.print((float)avgOfRPM1);
      Serial.print("    ");
      Serial.println((float)avgOfRPM2);
 
    initializeMotor_End();
 
}


//need calibrate Turnright
void turnRight(int angle) {
  int i=0;
  RPM1 = 0;
  RPM2 = 0;
  int counter = 0;
  double currentSpeed = turnSpeed;
   if (angle >= 90){
    for (i=0; i<angle; i=i+90){
      initializeMotor_Start();
      initializeTick();
      while (tick_R < (TURN_TICKS_R) || tick_L < (TURN_TICKS_R)) {
        if (myPID.Compute())
          md.setSpeeds((currentSpeed + speed_O), -(currentSpeed - speed_O));
          double pulse1 = pulseIn(RIGHT_PULSE,HIGH);
          double pulse2 = pulseIn(LEFT_PULSE,HIGH);
          RPM1 = RPM1 + ((1/(pulse1*2))*(pow(10,6))*60)/562.25;
          RPM2 = RPM2 + ((1/(pulse2*2))*(pow(10,6))*60)/562.25;
          counter ++;

          
      }
      
      float avgOfRPM1 = 0;
      float avgOfRPM2 = 0;
      avgOfRPM1 = RPM1/counter;
      avgOfRPM2 = RPM2/counter;
      
      Serial.print((float)avgOfRPM1);
      Serial.print("    ");
      Serial.println((float)avgOfRPM2); 
    }
    initializeMotor_End();
  }
  i= i - 90;
   
  //delay(50);
}

//turn left used for both turning left and checklist //calibrated without RPI and powerbank
void turnLeft(int angle) {
  int i=0;
  RPM1 = 0;
  RPM2 = 0;
  int counter = 0;
  double currentSpeed = turnSpeed;
  if (angle >= 90) {
    for (i = 90; i<=angle; i+=90) {
      initializeTick();
      initializeMotor_Start();
      while (tick_R < TURN_TICKS_L || tick_L < TURN_TICKS_L) {
        if (myPID2.Compute())
          md.setSpeeds(-(currentSpeed + speed_O), currentSpeed - speed_O);
          double pulse1 = pulseIn(RIGHT_PULSE,HIGH);
          double pulse2 = pulseIn(LEFT_PULSE,HIGH);
          RPM1 = RPM1 + ((1/(pulse1*2))*(pow(10,6))*60)/562.25;
          RPM2 = RPM2 + ((1/(pulse2*2))*(pow(10,6))*60)/562.25;
          counter ++;
      }
      
      
      float avgOfRPM1 = 0;
      float avgOfRPM2 = 0;
      avgOfRPM1 = RPM1/counter;
      avgOfRPM2 = RPM2/counter;
      
      Serial.print((float)avgOfRPM1);
      Serial.print("    ");
      Serial.println((float)avgOfRPM2);
      //delay(50); 
    }
    initializeMotor_End();
    i = i-90;
   
  }
  if (angle - i > 0) {
    Serial.println(angle-i);
    turnLeftDeg(angle-i);
  }
  initializeMotor_End();
  delay(50);
}


//used for checklist
void turnLeftDeg(int angle) {
  int index = (angle-20)/5; //angle will be less than 90 degree
  int tick;
  int counter = 0;
  
  RPM1 = 0;
  RPM2 = 0;
  if (index <= 14)
    tick = LEFTTICK[index];
    Serial.println(tick);
    
  initializeMotor_Start();
  double currentSpeed = turnSpeed;
  initializeTick();
  while (tick_R < tick || tick_L < tick) {
    if (myPID.Compute())
      md.setSpeeds(-(currentSpeed + speed_O), currentSpeed - speed_O);
          double pulse1 = pulseIn(RIGHT_PULSE,HIGH);
          double pulse2 = pulseIn(LEFT_PULSE,HIGH);
          RPM1 = RPM1 + ((1/(pulse1*2))*(pow(10,6))*60)/562.25;
          RPM2 = RPM2 + ((1/(pulse2*2))*(pow(10,6))*60)/562.25;
          counter ++;
  }
  initializeMotor_End();
  float avgOfRPM1 = 0;
      float avgOfRPM2 = 0;
      avgOfRPM1 = RPM1/counter;
      avgOfRPM2 = RPM2/counter;
      
      Serial.print((float)avgOfRPM1);
      Serial.print("    ");
      Serial.println((float)avgOfRPM2); 
}

void turnRightDeg(int angle){

  int tick = 460;
  initializeMotor_Start();
  double currentSpeed = turnSpeed;
  initializeTick();
  while (tick_R < tick || tick_L < tick) {
    if (myPID.Compute())
      md.setSpeeds(currentSpeed + speed_O, -(currentSpeed - speed_O));
          double pulse1 = pulseIn(RIGHT_PULSE,HIGH);
          double pulse2 = pulseIn(LEFT_PULSE,HIGH);
          RPM1 = RPM1 + ((1/(pulse1*2))*(pow(10,6))*60)/562.25;
          RPM2 = RPM2 + ((1/(pulse2*2))*(pow(10,6))*60)/562.25;
          
  }
  initializeMotor_End();
  
}

void moveBack(){
  initializeTick();
  initializeMotor_Start();
  int distance = 7;
  double currentSpeed = 0;
  currentSpeed = 400;   
  while (tick_R <= distance || tick_L <= distance) {
        if(myPID2.Compute())
           md.setSpeeds(-(currentSpeed + 2*speed_O), -(currentSpeed));
           
  }
  initializeMotor_End();
}




//calibrating if the front 3 blocks are wall
void caliFlat(){
  //offset3 =0.1;
  
  moveBack();
  
  while(getMedianA3() > 14 || getMedianA4() > 13.85) //too back, need forward
    moveForward(11); //distance = 11 means tick = 6
  while((getMedianA3())-offset1 < 13.75 || (getMedianA4())-offset3 < 13.25)   //bot is too front, need move back
    moveBack(); 
  delay(50);
  int count = 0;
  double ir1Distance = getMedianA1();
  double ir2Distance = getMedianA4();
  double ir3Distance = getMedianA3();
  double diffLeft = (ir2Distance-offset1) - desiredDistanceSensor;
  double diffRight = (ir3Distance-offset3) - desiredDistanceSensor;
  
  while(ir2Distance-offset2 > desiredDistanceSensor || ir3Distance-offset3 > desiredDistanceSensor){ //may need the ir number

    //in andy case; ir1 is the left front and ir3 is the front right IR sensor
    if (ir3Distance-offset3 > ir2Distance-offset2){ //this mean, the bot is tilted at the right, need rotate left to correct
      rotateLeft(abs(diffLeft*5), 1);
    }
    else if (ir2Distance-offset2 > ir3Distance-offset3) { //this mean the bot is tilted to the left, need rotate right to correct
      rotateRight(abs(diffRight*5), 1);
    }
    count++;
    if (count >= 17)
      break;
    
     ir2Distance = getMedianA4();
     ir3Distance = getMedianA3();
  }
  delay(50);
  //checkCali();
}

void caliRight2(){
  while(getMedianA1() > 12.25){
      moveForward(11);
    }
  while(getMedianA3()-offset3 < 13.9 || getMedianA1()-offset1 < 12.25)        // Move back to desire distance
    moveBack(); 
  delay(50);
  int count = 0;
  double diffLeft = (getMedianA3()-offset2) - desiredDistanceSensor;
  double diffRight = (getMedianA1()-offset1) - desiredDistanceSensor;
  
  while(getMedianA1()-offset1 > desiredDistanceSensor ||  getMedianA3()-offset3 > desiredDistanceSensor){
    if (getMedianA1()-offset1 < getMedianA3()-offset3){
      rotateLeft(abs(diffLeft*5), 1);
    }
    else if ( getMedianA3()-offset3 < getMedianA1()-offset1) {
      rotateRight(abs(diffRight*5), 1);
    }
    count++;
    if (count >= 16)
      break;
  }
}

void caliLeft2(){
  offset1 = 1;
  while(getMedianA1() > 12.25)
     moveForward(11);
  while(getMedianA4()-offset2 < 13.25 || getMedianA1() < 12.52)        // Move back to desire distance
    moveBack(); 
  delay(50);
  int count = 0;
  double diffLeft = (getMedianA4()-offset2) - desiredDistanceSensor;
  double diffRight = (getMedianA1()-offset1) - desiredDistanceSensor;
  
  while(getMedianA4()-offset2 > desiredDistanceSensor ||  getMedianA1()-offset1 > desiredDistanceSensor){
    //Serial.print("Calibrating...");
    //Serial.println(count+1);
    if (getMedianA1()-offset1 > getMedianA4()-offset2){
      rotateLeft(abs(diffLeft*5), 1);
    }
    else if (getMedianA4()-offset2 > getMedianA1()-offset1) {
      rotateRight(abs(diffRight*5), 1);
    }
    count++;
    if (count >= 16)
      break;
  }   
}


//used for calibration 
void rotateRight(int distance, int direct) {
  initializeTick();
  initializeMotor_Start();
  double currentSpeed = 400;

  while (tick_L < distance) {
    if (myPID2.Compute())
      md.setSpeeds(direct*(currentSpeed - 2*speed_O), 0);
 
  }
  
  initializeMotor_End();
}


//used for calibration 
void rotateLeft(int distance, int direct) {
  initializeTick();
  initializeMotor_Start();

  double currentSpeed = 400;
  while (tick_R < distance) {
    if (myPID2.Compute())
      md.setSpeeds(0, direct*(currentSpeed - 2*speed_O));
  }
  initializeMotor_End();
}

//for checklist: straightline motion
void calibrateRightwall(){
    int count = 0;
    double ir4Distance = getMedianA0();
    double ir5Distance = getMedianA2();
    double diffRight = (ir4Distance) - desiredDistanceSensorforRightwallCali;
    double diffLeft = (ir5Distance) - desiredDistanceSensorforRightwallCali;
    double diff = ir4Distance - ir5Distance;
  
  while( ir4Distance > desiredDistanceSensorforRightwallCali ||  ir5Distance > desiredDistanceSensorforRightwallCali ){ 
  
    
    if (ir5Distance > ir4Distance){ //this mean, the bot is tilted at the right, need rotate left to correct
      /*if(diff>2){
        for(int i = 0;i<7;i++){
          rotateLeft(abs(diff*5), 1);
        }
      }*/
      //else{
      rotateLeft(abs(diffLeft*5), 1);
      //}
    }
    else if (ir4Distance > ir5Distance) { //this mean the bot is tilted to the left, need rotate right to correct
      
      /*if(diff>2){ //overcompenstating
        for(int i = 0;i<7;i++){
          rotateRight(abs(diff*5), 1);
        }
      }
      else{*/
      rotateRight(abs(diffRight*5), 1);
      //}
    }
    count++;
    if (count >= 17){
      break;}
    ir5Distance = getMedianA2();
    ir4Distance = getMedianA0();
    diff = ir4Distance - ir5Distance;
  }
  delay(50);



  
}

//for checklist: 1 obstacle in straightline motion of 150cm
void AvoidFrontObstacle(int distance){
    //check if obstacle 10cm away
    double ir1Distance = 0; //front center sensor //getMedianA1()
    double ir2Distance = 0; //front left sensor //getMedianA4()
    double ir3Distance = 0; //front right sensor //getMedianA3()
    int count = 0;
  

  while(count < distance){
       ir1Distance = getMedianA1();
       Serial.println(ir1Distance);
       ir3Distance = getMedianA3();
       Serial.println(ir3Distance);
       ir2Distance = getMedianA4();
       Serial.println(ir2Distance);
      //check if got any obstacle 10 cm ahead
      if(   ((ir1Distance < 15) && (ir1Distance > 0)) || ((ir2Distance < 15) && (ir2Distance > 0)) || ((ir3Distance < 15) && (ir3Distance > 0))    ){
        //obstacle in 10cm

        //now check if obstacle on the right or left
          if(    ((ir2Distance < 15) && (ir2Distance > 0))   ){ //obstacle on the left
              turnRight(90);
              delay(1000);
              moveForward(2);
              delay(1000);
              turnLeft(90);
              delay(1000);
              moveForward(5);
              delay(1000);
              turnLeft(90);
              delay(1000);
              moveForward(2);
              delay(1000);
              turnRight(90);
              break;
              
              
          
          }
  
          else if(   ((ir3Distance < 15) && (ir3Distance > 0)) || ((ir1Distance < 15) && (ir1Distance > 0))   ) { //obstacle on the right 
  
            //take evasive action 
            turnLeft(90);
             delay(1000);
            moveForward(2);
             delay(1000);
            turnRight(90);
             delay(1000);
            moveForward(5);
             delay(1000);
            turnRight(90);
             delay(1000);
            moveForward(2);
             delay(1000);
            turnLeft(90);
            break;
          }
  
          count = count +5;

        
      }
      else{
        moveForward(1);//moveforward by 10cm
        count++; 
      }

      
    
  }

  
  
}
void AvoidFrontObstacleDiagonalMovement(int distance){
    //check if obstacle 10cm away
    double ir1Distance = 0; //front center sensor //getMedianA1()
    double ir2Distance = 0; //front left sensor //getMedianA4()
    double ir3Distance = 0; //front right sensor //getMedianA3()
    int count = 0;
  

  while(count < distance){
       ir1Distance = getMedianA1();
       Serial.println(ir1Distance);
       ir3Distance = getMedianA3();
       Serial.println(ir3Distance);
       ir2Distance = getMedianA4();
       Serial.println(ir2Distance);
      //check if got any obstacle 10 cm ahead
      if(   ((ir1Distance < 15) && (ir1Distance > 0)) || ((ir2Distance < 15) && (ir2Distance > 0)) || ((ir3Distance < 15) && (ir3Distance > 0))    ){
        //obstacle in 10cm

        //now check if obstacle on the right or left
          if(    ((ir2Distance < 15) && (ir2Distance > 0)) ){ //obstacle on the left
              turnRightDeg(45);
              delay(1000);
              moveForward(4);
              delay(1000);
              turnLeftDeg(90);
              delay(1000);
              moveForward(4);
              delay(1000);
              
              turnRightDeg(45);
              
              break;
              
          
          }
  
          else if(   ((ir3Distance < 15) && (ir3Distance > 0))  || ((ir1Distance < 15) && (ir1Distance > 0))     ) { //obstacle on the right 
  
            //take evasive action 
             turnLeftDeg(45);
             delay(1000);
              
              moveForward(4);
              delay(1000);
              
              turnRight(90);
              delay(1000);
     
              moveForward(4);
              delay(1000);
              
              turnLeftDeg(45);
              break;
                      }
  
          count = count +5;

        
      }
      else{
        moveForward(1);//moveforward by 10cm
        count++; 
      }

      
    
  }

  
  
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  setupMotorEncoder();
  setupPID();
  
  
}

void loop()
{
 
   //md.setSpeeds(300,300); // // L//R//default right wheel is faster
  
  /*moveForward(5);
  turnLeft(90);
  moveForward(5);*/
  //moveBack()
  //moveForward(9);
  //rotateRight(7,1);
  //rotateLeft(7,1);
 
  
  //turnLeft(1000);
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
  
 /*p moveForward(5);
  calibrateRightwall();
  moveForward(5);
  calibrateRightwall();
  moveForward(3);*/
  
  
  /*
  calibrateRightwall();
  moveForward(5);*/
   
  //turnRight(1000);
  //getMedianA1();
  //getMedianA3();
  //getMedianA4();
  getMedianA5();
  
  //calibrateFrontObstacle();
  //turnLeft(90);
  //turnRight(90);
  
  delay(4000);


  

  
 
  
}
 
 
