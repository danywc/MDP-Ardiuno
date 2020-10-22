#include "RunningMedian.h"
#include <SharpIR.h>
#include <RunningMedian.h>

#define model 1080
int analogPin1 = A0;
int analogPin2 = A1;
int analogPin3 = A2;
int analogPin4 = A3;
int analogPin5 = A4;
int analogPin6 = A5;
int val = 0;
const int NUM_SAMPLES_MEDIAN = 20;



double getMedianA0() 
  { //right front SR sensor
  double getMedianValue;

  RunningMedian med = RunningMedian(NUM_SAMPLES_MEDIAN);
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    med.add(getShortRangeA0());
  }
  getMedianValue = med.getMedian();
  Serial.println("Median value of A0");
  Serial.println((0.95 * getMedianValue));
  return (0.95 * getMedianValue);

}

double getMedianA1() 
{ //front center SR sensor

  double getMedianValue;

  RunningMedian med = RunningMedian(NUM_SAMPLES_MEDIAN);
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    med.add(getShortRangeA1());
  }
  getMedianValue = med.getMedian();
  Serial.println("Median value of A1");
  Serial.println(0.95 * getMedianValue);
  return (0.95 * getMedianValue);
}

double getMedianA2() 
{ //right back SR sensor
  double getMedianValue;

  RunningMedian med = RunningMedian(NUM_SAMPLES_MEDIAN);
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    med.add(getShortRangeA2());
  }
  getMedianValue = med.getMedian();
  Serial.println("Median value of A2");
  Serial.println(0.89 * getMedianValue);
  return (0.89 * getMedianValue);
}

double getMedianA3() 
{ //front right SR sensor
  double getMedianValue;

  RunningMedian med = RunningMedian(NUM_SAMPLES_MEDIAN);
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    med.add(getShortRangeA3());
  }
  getMedianValue = med.getMedian();
  Serial.println("Median value of A3");
  Serial.println(0.9 * getMedianValue);
  return (0.9 * getMedianValue);
}



double getMedianA4() 
{ //front left SR sensor

  double getMedianValue;

  RunningMedian med = RunningMedian(NUM_SAMPLES_MEDIAN);
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    med.add(getShortRangeA4());
  }
  getMedianValue = med.getMedian();
  Serial.println("Median value of A4");
  Serial.println(0.95 * getMedianValue);
  return (0.95 * getMedianValue);
}

double getMedianA5() 
{ //left LR sensor

  double getMedianValue;

  RunningMedian med = RunningMedian(NUM_SAMPLES_MEDIAN);
  for (int n = 0; n < NUM_SAMPLES_MEDIAN; n++) {
    med.add(getLongRangeNew2());
  }
  getMedianValue = med.getMedian();
  Serial.println("Median value of A5");
  Serial.println(0.94 * getMedianValue);
  return (0.94 * getMedianValue);
}

double getShortRangeA0() 
{
  return abs( (1 / (0.00018401809006023533 * (analogRead(analogPin1)) + (-0.003279242753008803))) - 1  );
}
double getShortRangeA1() 
{
  return abs( (1 / (0.00018401809006023533 * (analogRead(analogPin2)) + (-0.003279242753008803))) - 1  );
}
double getShortRangeA2() 
{
  return abs( (1 / (0.00017919115658212846 * (analogRead(analogPin3)) + (-0.0004533890357601344))) - 1  );
}
double getShortRangeA3() 
{
  return abs( (1 / (0.00019745944011122094 * (analogRead(analogPin4)) + (-0.008001111286848248))) - 0.5  );
}
double getShortRangeA4() 
{
  return abs( (1 / (0.00017924334790086593 * (analogRead(analogPin5)) + (-0.002329940347835141))) - 1  );
}

double getLongRange() 
{
  return abs((1 / (0.000055660449724735444 * (analogRead(analogPin6)) + (-0.0005342773671003311))) - 10  );
}
double getLongRangeNew() 
{
  return abs((1 / (0.000058502676165925303 * (analogRead(analogPin6)) + (-0.0011225182818268035))) - 10  );
}
double getLongRangeNew2() 
{
  return abs((1 / (0.00003839571932781073 * (analogRead(analogPin6)) + (0.003883936543500112))) - 21  );
}




int ir1GetGrid(double range) 
{ //used to get grid for A1 (front center)
  //return 1 if obstacle 1 grid away, return 2 if obstacle 2 grid away, return 0 if obstacle infront, return -1 if no obstacle within 2 grid

  if (range >= 7 && range <= 12.25)
  { Serial.println("0");
    return 0;
  }
  if (range > 12.25 && range <= 23.5)
  { Serial.println("1");
    return 1;
  }
  else   if (range > 23.5 && range <=36)
  { Serial.println("2");
    return 2;
  }
  else {
    Serial.println("-1");
    return -1;

  }
}

int ir2GetGrid(double range) 
{ //used to get grid for A4 (front left)
  if ( range >= 7 && range <= 14.5)
  { Serial.println("0");
  return 0;
  }  
 if ( range >= 14.5 && range <= 25)
  { Serial.println("1");
    return 1;
  }
  else if ( range > 25 && range <= 34.5) {
    Serial.println("2");
    return 2;

  }
  else {
    Serial.println("-1");
    return -1;

  }
}

int ir3GetGrid(double range) 
{ //used to get grid for A3 (front right)
   if ( range >= 7 && range <= 15)
  { Serial.println("0");
  return 0;
  }  if ( range >= 15 && range <=25) {
    Serial.println("1");
    return 1;

  }
  else if ( range > 25 && range <= 32)
  { Serial.println("2");
    return 2;

  }
  else {
    Serial.println("-1");
    return -1;

  }
}

int ir4GetGrid(double range) 
{ //used to get grid for A0 (right front)
  if (range >= 6.5 && range <= 12.5) {
    Serial.println("0");
    return 0;
  
  }
  else if (range > 12.5 && range <= 23.5 )
  { Serial.println("1");
    return 1;

  }
  else if (range > 23.5 && range <= 34.8 )
  { Serial.println("2");
    return 2;

  }
  else {
    Serial.println("-1");
    return -1;

  }
}

int ir5GetGrid(double range) 
{ //used to get grid for A2 (right back)
  if (range >= 6.5 && range <=13)
  { Serial.println("0");
    return 0;

  }
  if (range > 13 && range <= 24)
  { Serial.println("1");
    return 1;

  }
  else if (range > 24  && range <= 32.1)
  { Serial.println("2");
    return 2;

  }
  else {
    Serial.println("-1");
    return -1;

  }
}

int ir6GetGrid(double range) 
{
  if (range >= 18 && range <= 24  )
  { Serial.println("0");
    return 0;

  }


   else if (range >= 25 && range <= 33  )
  { Serial.println("1");
    return 1;

  }

   else if (range >= 34 && range <=  45 )
  { Serial.println("2");
    return 2;

  }

  else if (range >= 46 && range <=  55 )
  { Serial.println("3");
    return 3;

  }

  else if (range >= 56 && range <=  65 )
  { Serial.println("4");
    return 4;

  }

else {
    Serial.println("-1");
    return -1;
  }


/*  

  //original placement of LR sensor
  if (range >= 27 && range <= 35 )
  { Serial.println("2");
    return 2;

  }
  else if (range > 35 && range <= 46)
  { Serial.println("3");
    return 3;

  }
  else if (range > 46 && range <= 56)
  { Serial.println("4");
    return 4;

  }
  else if (range > 56 && range <= 65)
  { Serial.println("5");
    return 5;

  }
  else if (range > 65 && range <= 76)
  { Serial.println("6");
    return 6;

  }
  else if (range > 76 && range <= 85)
  { Serial.println("7");
    return 7;

  }
  else {
    Serial.println("-1");
    return -1;
  }

  */

}



void readAllSensors() 
{
  Serial.print("A0 uncalibrated");
  Serial.println(analogRead(A0));
  Serial.print("A1 uncalibrated");
  Serial.println(analogRead(A1));
  Serial.print("A2 uncalibrated");
  Serial.println(analogRead(A2));
  Serial.print("A3 uncalibrated");
  Serial.println(analogRead(A3));
  Serial.print("A4 uncalibrated");
  Serial.println(analogRead(A4));
  Serial.print("A5 uncalibrated");
  Serial.println(analogRead(A5));


  Serial.println("/////Get distance in cm/////");
  Serial.println(getMedianA0());
  Serial.println(getMedianA1());
  Serial.println(getMedianA2());
  Serial.println(getMedianA3());
  Serial.println(getMedianA4());
  Serial.println(getMedianA5());

}

void sendGridData() 
{
  // "SDATA:SRFLeft:SRFCentre:SRFRight:SideFront:SideBack:LR" FORMAT
  delay(2);
  // ir1 = front center, ir2 = front left, ir 3 = front right, ir4 = right front, ir 5 = right back, ir6 = long range left
  String s = "XSDATA:" + String(ir2GetGrid(getMedianA4())) + ":" + String(ir1GetGrid(getMedianA1())) + ":" +
             String(ir3GetGrid(getMedianA3())) + ":" + String(ir4GetGrid(getMedianA0())) + ":" +
             String(ir5GetGrid(getMedianA2())) + ":" + String(ir6GetGrid(getMedianA5()));
  Serial.println(s);

}
