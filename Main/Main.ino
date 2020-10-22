
char current;

void setup() 
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  setupMotorEncoder();
  setupPID();
  delay(1000);
}

void loop() 
{


if(Serial.available()>0){
String data= Serial.readStringUntil('\n');
Serial.println("Ardiuno receieved  " + data);
int dataLength=data.length();
char dataBuffer[dataLength];
data.toCharArray(dataBuffer, dataLength);
Serial.println("length of data received");
Serial.println(dataLength);
int i;

//int gridValue=1;
int gridValue;
if(dataLength>1){
  for(i=0;i<dataLength;i++){
      current=dataBuffer[i];

      switch(current){
      case 'A': 
              Serial.print("command = ");
              Serial.println(int(dataBuffer[i+1]));
              gridValue = int(dataBuffer[i+1]) - 48;
              i++;
              Serial.print("gridValue = ");
              Serial.println(gridValue); 
              moveForward((int)gridValue);
              
                               
              break;
      case 'L': turnLeft(90);
                
               
              break;
      case 'R': turnRight(90);
               
              break;
              
      case 'C': calibrate();
              sendGridData();
              break; 
      case 'S': sendGridData();
             break;         
       
          
      }
    }
    
  }
   
}

//Serial.println(analogRead(analogPin2));
//delay(1000);
//moveForward(9); //drain battery
//  md.setSpeeds(400,400);// // L//R//default right wheel is faster

//sendGridData();

//moveForward(1);
//turnLeft(90);
//turnRight(90);

//AvoidFrontObstacle(10);


/*getMedianA0();
getMedianA2();
  getMedianA1();
  getMedianA3();
  getMedianA4();*/
  //getMedianA5(); 
  /*ir2GetGrid(getMedianA4());
  ir1GetGrid(getMedianA1());
  ir3GetGrid(getMedianA3());  */
  //ir4GetGrid(getMedianA0());
  //ir5GetGrid(getMedianA2());
  //ir6GetGrid(getMedianA5());

//calibrate();
//turnLeft(90);
//turnRight(90);
//caliLeft2();
//caliRIght2();
//delay(5000);
  

 

}
