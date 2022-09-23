//works
//Define L298N Dual H-Bridge Motor Controller Pins
/*From left to right, connect to D3,A1-A3 ,D10*/
#define LFSensor_0 A0  //OLD D3
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 A4  //OLD D10

#define speedPinR 9    //  RIGHT PWM pin connect MODEL-X ENA
#define RightMotorDirPin1  12    //Right Motor direction pin 1 to MODEL-X IN1
#define RightMotorDirPin2  11    //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 5    // Left PWM pin connect MODEL-X ENB // nEW PIN SHOULD BE 1
#define LeftMotorDirPin1  7    //Left Motor direction pin 1 to MODEL-X IN3
#define LeftMotorDirPin2  8   //Left Motor direction pin 1 to MODEL-X IN4

#define FAST_SPEED 185
#define MID_SPEED 160
#define SLOW_SPEED  140
#define ROTATE_LEFT_TIME 500 // Time for it to rotate 90 degrees LEFT
#define ROTATE_RIGHT_TIME 1250 // Time for it to rotate 90+135 degrees RIGHT
#define ROTATE_ITER_TIME 25 // Each rotation duration

static String lastTurn = "NULL";

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


void setup()
{

	pinMode(RightMotorDirPin1, OUTPUT);
	pinMode(RightMotorDirPin2, OUTPUT);
	pinMode(speedPinL, OUTPUT);

	pinMode(LeftMotorDirPin1, OUTPUT);
	pinMode(LeftMotorDirPin2, OUTPUT);
	pinMode(speedPinR, OUTPUT);
	stop_Stop();//stop move

	Serial.begin(9600);   // initialize serial for debugging

}

void loop()
{
	auto_tracking();
} //end of loop

char sensor[5];
// Read the sensor values. 1 is white, 0 is black. (VERY IMPORTANT)
String read_sensor_values()
{   int sensorvalue=32;
	sensor[0] = digitalRead(LFSensor_0);

	sensor[1] = digitalRead(LFSensor_1);

	sensor[2] = digitalRead(LFSensor_2);

	sensor[3] = digitalRead(LFSensor_3);

	sensor[4] = digitalRead(LFSensor_4);
	sensorvalue += sensor[0]*16 + sensor[1]*8 + sensor[2]*4 + sensor[3]*2 + sensor[4];

	String senstr = String(sensorvalue,BIN);
	senstr = senstr.substring(1,6);


	return senstr;
}

void auto_tracking(){
	String sensorval = read_sensor_values();
	Serial.println("Sensor Values: "+ sensorval);

	if (sensorval == "00000") {
		Serial.println("Robot is off-track.");
		// Make it go slightly backwards.
		go_Back(50);
		set_Motorspeed(SLOW_SPEED,SLOW_SPEED);
		int i = 0;
		while (i < ROTATE_LEFT_TIME) {
			if (lastTurn == "LEFT") {
				go_Left(ROTATE_ITER_TIME);
				set_Motorspeed(MID_SPEED,MID_SPEED);
			} else {
				go_Right(ROTATE_ITER_TIME);
				set_Motorspeed(MID_SPEED,MID_SPEED);
			}
			i += ROTATE_ITER_TIME;
			if (read_sensor_values() != "00000") {
				break;
			}
		}

		i = 0;
		while (i < ROTATE_RIGHT_TIME) {
			if (lastTurn == "LEFT") {
				go_Right(ROTATE_ITER_TIME);
				set_Motorspeed(MID_SPEED,MID_SPEED);
			} else {
				go_Left(ROTATE_ITER_TIME);
				set_Motorspeed(MID_SPEED,MID_SPEED);
			}
			i+= ROTATE_ITER_TIME;
			if (read_sensor_values() != "00000") {
				break;
			}

		}

	}

	/*
	 * Rotate right cases:
	 * 00001
	 * 00011
	 * 00111
	 * 01111
	 */
	if (sensorval=="00001" || sensorval == "00011" || sensorval=="00111" || sensorval == "01111" ) {
		// Save the turn, rotate right
		lastTurn = "RIGHT";
		go_Right();
		Serial.println("Rotate Right.");
		/*
		 * Changed speed to mid.
		 * Slow means it might not be rotating fast enough, fast means it might miss sensor values
		 */
		set_Motorspeed(MID_SPEED, MID_SPEED);
	}

	/*
	 * Rotate left cases:
	 * 10000
	 * 11000
	 * 11100
	 * 11110
	 */
	if (sensorval=="10000" || sensorval=="11000" || sensorval=="11100" || sensorval=="11110") {
		// Save the turn, rotate left
		lastTurn = "LEFT";
		go_Left();
		Serial.println("Rotate left");
		set_Motorspeed(MID_SPEED, MID_SPEED);
		/*
		 * Changed speed to mid.
		 * Slow means it might not be rotating fast enough, fast means it might miss sensor values
		 */
	}

	if (sensorval == "00100") {
		go_Advance();
		set_Motorspeed(MID_SPEED,MID_SPEED);
	}
	if (sensorval=="00010"  || sensorval=="00101" || sensorval=="00110"  || sensorval=="01101" || sensorval=="01011"  || sensorval=="01001") {
		lastTurn = "RIGHT";
		go_Advance();  //Turn slight right
		set_Motorspeed( SLOW_SPEED,0);
	}
	
	if (sensorval == "10100"  || sensorval=="01000" || sensorval=="01100"|| sensorval == "10010" || sensorval == "11010") {
		go_Advance();  //Turn slight left
		lastTurn = "LEFT";
		set_Motorspeed(0,SLOW_SPEED);
	}

}
