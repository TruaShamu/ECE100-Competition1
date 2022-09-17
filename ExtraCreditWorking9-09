/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/
 * Osoyoo Wifi Arduino Robot Car project
 * OSOYOO V2.1 Robot Car Lesson 2: Line Tracking Auto Driving
 * tutorial url: https://osoyoo.com/?p=32249
 */
/*Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.*/


//Define L298N Dual H-Bridge Motor Controller Pins


#include <IRremote.h>  
#define IR_PIN    10 //IR receiver Signal pin connect to Arduino pin D10
 IRrecv IR(IR_PIN);  //   IRrecv object  IR get code from IR remoter
 decode_results IRresults;  
/*#define speedPinR 9    //  RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1  12    //Right Motor direction pin 1 to MODEL-X IN1
#define RightDirectPin2  11    //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 5    // Left PWM pin connect MODEL-X ENB //modify it!!!
#define LeftDirectPin1  7    //Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2  8   //Left Motor direction pin 1 to MODEL-X IN4 */

 #define IR_ADVANCE       0x00FF18E7       //code from IR controller "▲" button
 #define IR_BACK          0x00FF4AB5       //code from IR controller "▼" button
 #define IR_RIGHT         0x00FF5AA5       //code from IR controller ">" button
 #define IR_LEFT          0x00FF10EF       //code from IR controller "<" button
 #define IR_STOP          0x00FF38C7       //code from IR controller "OK" button
 #define IR_turnsmallleft 0x00FFB04F       //code from IR controller "#" button

enum DN
{
  GO_ADVANCE, //go forward
  GO_LEFT, //left turn
  GO_RIGHT,//right turn
  GO_BACK,//backward
  STOP_STOP,
  DEF
}Drive_Num=DEF;

bool stopFlag = true;//set stop flag
bool JogFlag = false;
uint16_t JogTimeCnt = 0;
uint32_t JogTime=0;
uint8_t motor_update_flag = 0;





//original


/*From left to right, connect to D3,A1-A3 ,D10*/
#define LFSensor_0 A0  //OLD D3
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 A4  //OLD D10

#define FAST_SPEED 220
#define MID_SPEED 190
#define SLOW_SPEED  170     //back speed
#define speedPinR 9    //  RIGHT PWM pin connect MODEL-X ENA
#define RightMotorDirPin1  12    //Right Motor direction pin 1 to MODEL-X IN1
#define RightMotorDirPin2  11    //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 5    // Left PWM pin connect MODEL-X ENB // nEW PIN SHOULD BE 1
#define LeftMotorDirPin1  7    //Left Motor direction pin 1 to MODEL-X IN3
#define LeftMotorDirPin2  8   //Left Motor direction pin 1 to MODEL-X IN4


/*motor control*/
void go_Advance(void)  //Forward
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
}
void go_Left(int t=0)  //Turn left
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}
void go_Right(int t=0)  //Turn right
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}
void go_Back(int t=0)  //Reverse
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}
void stop_Stop()    //Stop
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,LOW);
}
/*set motor speed */
void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L);
  analogWrite(speedPinR,speed_R);  
}


//NEW

void do_IR_Tick()
{
  if(IR.decode(&IRresults))
  {
   
    if(IRresults.value==IR_ADVANCE)
    {
      Serial.println("Advance");
      Drive_Num=GO_ADVANCE;
    }
    else if(IRresults.value==IR_RIGHT)
    {
       Drive_Num=GO_RIGHT;
       Serial.println("Go right");
    }
    else if(IRresults.value==IR_LEFT)
   
    {
      Serial.println("Go left");
       Drive_Num=GO_LEFT;
    }
    else if(IRresults.value==IR_BACK)
    {
      Serial.println("Go back");
        Drive_Num=GO_BACK;
    }
    else if(IRresults.value==IR_STOP)
    {
      Serial.println("Stop");
        Drive_Num=STOP_STOP;
    }
    IRresults.value = 0;
    IR.resume();
  }
}

/**************car control**************/
void do_Drive_Tick()
{
    switch (Drive_Num)
    {
      case GO_ADVANCE:go_Advance();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_ADVANCE code is detected, then go advance
      case GO_LEFT: go_Left();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_LEFT code is detected, then turn left
      case GO_RIGHT:  go_Right();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_RIGHT code is detected, then turn right
      case GO_BACK: go_Back();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_BACK code is detected, then backward
      case STOP_STOP: stop_Stop();JogTime = 0;break;//stop
      default:break;
    }
    Drive_Num=DEF;
   //keep current moving mode for  200 millis seconds
    if(millis()-JogTime>=200)
    {
      JogTime=millis();
      if(JogFlag == true)
      {
        stopFlag = false;
        if(JogTimeCnt <= 0)
        {
          JogFlag = false; stopFlag = true;
        }
        JogTimeCnt--;
      }
      if(stopFlag == true)
      {
        JogTimeCnt=0;
        stop_Stop();
      }
    }
}

//NEW END

void setup()
{

  //new

  pinMode(IR_PIN, INPUT);
  digitalWrite(IR_PIN, HIGH);  
  IR.enableIRIn();  

  //new end
   
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);  
 
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);

  Serial.begin(9600);   // initialize serial for debugging

}

boolean flag=false;
void loop()
{
 do_IR_Tick();
  do_Drive_Tick();
 auto_tracking();
} //end of loop
 
char sensor[5];
 /*read sensor value string, 1 stands for black, 0 starnds for white, i.e 10000 means the first sensor(from left) detect black line, other 4 sensors detected white ground */
String read_sensor_values()
{   int sensorvalue=32;
  sensor[0]= digitalRead(LFSensor_0);
 
  sensor[1]=digitalRead(LFSensor_1);
 
  sensor[2]=digitalRead(LFSensor_2);
 
  sensor[3]=digitalRead(LFSensor_3);
 
  sensor[4]=digitalRead(LFSensor_4);
  sensorvalue +=sensor[0]*16+sensor[1]*8+sensor[2]*4+sensor[3]*2+sensor[4];
 
  String senstr= String(sensorvalue,BIN);
  senstr=senstr.substring(1,6);


  return senstr;
}

void auto_tracking(){
 String sensorval= read_sensor_values();
  Serial.println(sensorval);
  if (   sensorval=="10000" )
  {
    //The black line is in the left of the car, need  left turn
    go_Left();  //Turn left
    set_Motorspeed(FAST_SPEED,FAST_SPEED);
  }
  if (sensorval=="10100"  || sensorval=="01000" || sensorval=="01100" || sensorval=="11100"  || sensorval=="10010" || sensorval=="11010")
  {
    go_Advance();  //Turn slight left
    set_Motorspeed(0,FAST_SPEED);
  }
  if (    sensorval=="00001"  ){ //The black line is  on the right of the car, need  right turn
    go_Right();  //Turn right
    set_Motorspeed(FAST_SPEED,FAST_SPEED);
  }
  if (sensorval=="00011" || sensorval=="00010"  || sensorval=="00101" || sensorval=="00110" || sensorval=="00111" || sensorval=="01101" || sensorval=="01111"   || sensorval=="01011"  || sensorval=="01001")
  {
    go_Advance();  //Turn slight right
    set_Motorspeed( FAST_SPEED,0);
  }
 
  if (sensorval=="11111"){
    stop_Stop();   //The car front touch stop line, need stop
    set_Motorspeed(0,0);
  }
   
}
