#include "DualVNH5019MotorShield.h"
#include "PID_v1.h"
#include "EnableInterrupt.h"

const int RIGHT_PULSE = 3;     //Pin for right encoder
  const int LEFT_PULSE = 11;     //Pin for left encoder
  DualVNH5019MotorShield md;
  const double turnSpeed = 300;
  const double forSpeed = 300;
  float offset1 = 0;                     // Offsets for the front 3 sensors for calibration
  float offset2 = 0;
  float offset3 = 0;
  double desiredDistanceSensor = 8.8 ; 
  double desiredDistanceSensorCaliLeft2 = 8;
  double desiredDistanceSensorCaliRight2 = 8.5;
  double desiredDistanceSensorforRightwallCali = 8.44; 
  
  
  double tick_R = 0;
  double tick_L = 0;
  double speed_O = 0;
  double previous_tick_R = 0;
  double previous_error = 0;
  const double kp =  4, ki = 5, kd = 0.00;
  const double kp2 = 19, ki2 = 5 , kd2 = 0; // PID for above 10 cm 
  PID myPID(&tick_R, &speed_O, &tick_L, kp, ki, kd, REVERSE);
  PID myPID2(&tick_L, &speed_O, &tick_R, kp2, ki2, kd2, DIRECT);
  int TURN_TICKS_L = 820 ;   // 970 is for checklist // 820 for turning left 90 degree @6.31 volt
  int TURN_TICKS_R = 815;  //   
  const int sample = 19;

  const int LEFTTICK[18] = {200, 200, 100, 0, 400, 450, 500, 55, 500, 65, 700, 75, 600, 85, 925,0, 730, 500};
  int Ticks[11] = {570, 1175, 1765, 2380, 2950, 3565, 4155, 4735, 5340, 0,6}; //at speed 300 //calibrate again once added power bank and RPI

  
  float RPM1;
 float RPM2;



void setupMotorEncoder() 
{
  md.init();
  pinMode(LEFT_PULSE, INPUT);
  pinMode(RIGHT_PULSE, INPUT);
  enableInterrupt(LEFT_PULSE, leftEncoder, CHANGE);
  enableInterrupt(RIGHT_PULSE, rightEncoder, CHANGE);
}

void leftEncoder() 
{
  tick_L++;
}

void rightEncoder() 
{
  tick_R++;
}


void setupPID() 
{
 
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-400, 400);
  myPID.SetSampleTime(5);
  
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-400, 400);
  myPID2.SetSampleTime(5);
}

void initializeTick()
{
  tick_R = 0;
  tick_L = 0;
  speed_O = 0;
  previous_tick_R = 0;
}

void initializeMotor_Start() 
{
  md.setSpeeds(0, 0);
  md.setBrakes(0, 0);
}

void initializeMotor_End() 
{
  md.setSpeeds(0, 0);
  md.setBrakes(400, 400); //L//R
  delay(50);
}



void moveForward(int distance)
{
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
void turnRight(int angle) 
{
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
void turnLeft(int angle) 
{
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
void turnLeftDeg(int angle) 
{
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

void turnRightDeg(int angle)
{

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

void moveBack()
{
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


void calibrate()
{
  if (  (ir2GetGrid(getMedianA4()) == 0) && (ir1GetGrid(getMedianA1()) == 0) && (ir3GetGrid(getMedianA3()) == 0) )
  {
    Serial.println("Flat wall calibrating...");
     caliFlat();
  }

  else if(  (ir1GetGrid(getMedianA1()) == 0) && (ir3GetGrid(getMedianA3()) == 0) )
  {
    Serial.println("caliRight2");
    caliRight2();
  }

   else if(  (ir1GetGrid(getMedianA1()) == 0) && (ir2GetGrid(getMedianA4()) == 0) )
  {
    Serial.println("caliLeft2...");
    caliLeft2();
  }
  

  else if(  (ir4GetGrid(getMedianA0()) == 0) && (ir5GetGrid(getMedianA2()) == 0)  )
  {
    Serial.println("Right wall calibrating...");
    calibrateRightwall();
  }
  
}

//calibrating if the front 3 blocks are wall
void caliFlat()
{
  //offset3 =0.2;
  //offset2 = 1;
  
  moveBack();
  
  while(getMedianA3() > 9 || getMedianA4() > 8.9) //too back, need forward
    moveForward(11); //distance = 11 means tick = 6
  while((getMedianA3())-offset1 < 9|| (getMedianA4())-offset3 < 8.9)   //bot is too front, need move back
    moveBack(); 
  delay(50);
  int count = 0;
  double ir1Distance = getMedianA1();
  double ir2Distance = getMedianA4();
  double ir3Distance = getMedianA3();
  double diffLeft = (ir3Distance-offset1) - desiredDistanceSensor;
  double diffRight = (ir2Distance+offset3) - desiredDistanceSensor; 
  
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
  
}

void caliRight2()
{
  offset3 = 1;
  
  while( (getMedianA3() > 9.3 )  ){
      moveForward(11);
    }
  while(getMedianA3() < 9.3 || getMedianA1()-offset1 < 7.84)        // Move back to desire distance
    moveBack(); 
  delay(50);
  double ir1Distance = getMedianA1();
  double ir3Distance = getMedianA3();
  int count = 0;
  double diffRight = (ir1Distance+offset1) - desiredDistanceSensorCaliRight2;
  double diffLeft = (ir3Distance-offset3) - desiredDistanceSensorCaliRight2;
  
  while(ir1Distance-offset1 > desiredDistanceSensor ||  ir3Distance-offset3 > desiredDistanceSensor){
    
    if (ir1Distance-offset1 < ir3Distance-offset3){ //robot is tilted right
      rotateLeft(abs(diffLeft*5), 1);
    }
    else if ( ir3Distance-offset3 < ir1Distance-offset1) { //robot is tilted left 
      rotateRight(abs(diffRight*5), 1);
    }
    count++;
    if (count >= 17)
      break;


       ir1Distance = getMedianA1();
     ir3Distance = getMedianA3();
      
  }
}

void caliLeft2()
{
  //offset1 = 1;
  offset2 = 1;

  
  while(  (getMedianA4() > 8.9) )
     moveForward(11);
  while(  getMedianA1() < 7.84  || getMedianA4() < 8.9 )        // Move back to desire distance
    moveBack(); 
  delay(50);
  double ir1Distance = getMedianA1();
  double ir2Distance = getMedianA4();
  int count = 0;
  double diffLeft = (ir1Distance-offset1) - desiredDistanceSensorCaliLeft2;
  double diffRight = (ir2Distance-offset2) - desiredDistanceSensorCaliLeft2;
  
  while(ir2Distance-offset2 > desiredDistanceSensor ||  ir1Distance-offset1 > desiredDistanceSensor){
    //Serial.print("Calibrating...");
    //Serial.println(count+1);
    if (ir1Distance-offset1 > ir2Distance-offset2){ //robot is tilted right
      rotateLeft(abs(diffLeft*5), 1);
    }
    else if (ir2Distance-offset2 > ir1Distance-offset1) { //robot is tilted left
      rotateRight(abs(diffRight*5), 1);
    }
    count++;
    if (count >= 17)
      break;
    ir1Distance = getMedianA1();
    ir2Distance = getMedianA4();
  }   
}


//used for calibration 
void rotateRight(int distance, int direct) 
{
  initializeTick();
  initializeMotor_Start();
  double currentSpeed = 400;

  while (tick_L < distance) {
    if (myPID.Compute())
      md.setSpeeds(direct*(currentSpeed - 2*speed_O), -50);
  }
  
  initializeMotor_End();
}


//used for calibration 
void rotateLeft(int distance, int direct) 
{
  initializeTick();
  initializeMotor_Start();

  double currentSpeed = 400;
  while (tick_R < distance) {
    if (myPID.Compute())
      md.setSpeeds(-50, direct*(currentSpeed + 2*speed_O));
  }
  initializeMotor_End();
}

//for checklist: straightline motion
void calibrateRightwall()
{
    float offset0 = 0.5;
    int count = 0;
    double ir4Distance = getMedianA0()-offset0;
    double ir5Distance = getMedianA2();
    double diffRight = (ir4Distance) - desiredDistanceSensorforRightwallCali;
    double diffLeft = (ir5Distance) - desiredDistanceSensorforRightwallCali;
    double   diffR = (ir4Distance) - desiredDistanceSensorforRightwallCali;
    double   diffL = (ir5Distance) - desiredDistanceSensorforRightwallCali;
  
  if(ir5Distance> ir4Distance)//this mean, the bot is tilted at the right, need rotate left to correct
  {
    while(abs(diffR) > 0.5 ||  abs(diffL) > 0.5 ){ 
  
    
      if (ir5Distance > ir4Distance){ //this mean, the bot is tilted at the right, need rotate left to correct
      
      rotateLeft(abs(diffLeft*5), 1);
     
      
      }
       count++;
      if (count >= 17){
      break;}
      ir5Distance = getMedianA2();
      ir4Distance = getMedianA0()-offset0;
       diffR = (ir4Distance) - desiredDistanceSensorforRightwallCali;
       diffL = (ir5Distance) - desiredDistanceSensorforRightwallCali;
    }
    if(ir4Distance < desiredDistanceSensorforRightwallCali ||   ir5Distance < desiredDistanceSensorforRightwallCali) 
    //bot is straight but too close to wall, need compensate
    {
      diffLeft = ir4Distance - desiredDistanceSensorforRightwallCali;
      //may need to a loop
      rotateLeft(abs(diffLeft*2), 1);
    }
    
  }
  
   else if (ir4Distance > ir5Distance) { //this mean the bot is tilted to the left, need rotate right to correct
      
      while(abs(diffR) > 0.5 ||  abs(diffL) > 0.5 )
      {
      if (ir5Distance < ir4Distance)
        {
      rotateRight(abs(diffRight*5), 1);

        }
        ir5Distance = getMedianA2();
      ir4Distance = getMedianA0()-offset0;
       diffR = (ir4Distance) - desiredDistanceSensorforRightwallCali;
       diffL = (ir5Distance) - desiredDistanceSensorforRightwallCali;
        count++;
        if (count >= 17){
        break;}
      }

      if(ir4Distance > desiredDistanceSensorforRightwallCali ||   ir5Distance > desiredDistanceSensorforRightwallCali) 
    //bot is straight but too far from right wall, need compensate
    {
      diffRight = ir4Distance - desiredDistanceSensorforRightwallCali;
      //may need to a loop
      rotateRight(abs(diffRight*2), 1);
    }

    }
    
   

  delay(50);
}
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
      if(   ir2GetGrid(getMedianA4()) == 0 || ir1GetGrid(getMedianA1()) == 0 || (ir3GetGrid(getMedianA3()) == 0)   ){
        //obstacle in 0 grid away

        //now check if obstacle on the right or left or center 
          //if(    ir2GetGrid(getMedianA4()) == 0   ){ //obstacle on the left
              turnLeft(90);
              delay(5000);
              moveForward(3);
              delay(5000);
              turnRight(90);
              delay(5000);
              moveForward(4);
              delay(5000);
              turnRight(90);
              delay(5000);
              moveForward(3);
              calibrate();
              
              turnLeft(90);
              delay(5000);
              calibrate();
              break;
              
          
         // }
  
          /*else if(   ((ir3Distance < 15) && (ir3Distance > 0)) || ((ir1Distance < 15) && (ir1Distance > 0))   ) { //obstacle on the right 
  
            //take evasive action 
            turnLeft(90);
             delay(5000);
            moveForward(2);
             delay(1000);
            turnRight(90);
             delay(5000);
            moveForward(5);
             delay(1000);
            turnRight(90);
             delay(5000);
            moveForward(2);
            caliFlat();
          
            turnLeft(90);
            delay(5000);
            calibrateRightwall();
            
          }
  
          count = count +5;

        
      }
      else{
        moveForward(1);//moveforward by 10cm
        count++; 
      }
*/
      
    
  }

  
  
}
}
