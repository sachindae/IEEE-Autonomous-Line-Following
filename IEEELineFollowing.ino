#include <Servo.h>

//Output pins for motor driver circuit
#define lowPin  5
#define highPin  6

//Pins for LineScan Camera
#define SI  3
#define CLK  4

//Pin for Servo
#define servoPin 9

//Constants
#define servoMiddle 90
#define maxTurnAngle 22
#define rightTurnScale 1.5

//Servo object
Servo servo1;

//Average spds of past 0.3 seconds
int prevSpds[3] = {0,0,0};
int prevAngles[5] = {0,0,0,0,0};
int startTime;

//Number of lines detected
int lines = 0;

//Tape out of sight, -1 means it is on left, 0 means in sight, 1 means it is on right
int range = 0;

//Median filtered array
int medianValues[128];
//Gradient filtered array
int gradientValues[128];

//Gradient fuction
int gradient_filter(int[]);
//Median function
int* median_filter(int[]);

//variable in function median_filter to record the side, 1 represent it's on left side, 0 means it's on the right side
int side;

//Array with pixels of linescan camera
int linescanValues[128];
int numOfLines = 0;

//Camera one-shot function
void capture_Linescan();

//Drive straight constant speed function
void constantSpdDrive();

//Accelerate to max speed function
void accelerateMaxSpd();

//Brake function
void brake();

//PID function
int turn( int center );

//PID variables
long currentTime, lastTIme;
double dt, dError,iError,output,turnAngle;
int Error, prevError = 0;
float kP, kI, kD;
int lastTime = 0;

//Returns the center after filtering
int findCenter();

//Constants
int MAX_PULSE_SPEED = 200; //max motor speed (0-255)
int LINE_SCAN_PIXELS = 128;
int SATURATION_TIME = 20000; //time in microseconds, use higher in dark rooms

//Setup code, initializes pins
void setup() {

  //Configures output pins
  pinMode( lowPin, OUTPUT );
  pinMode( highPin, OUTPUT );
  pinMode( SI, OUTPUT );
  pinMode( CLK, OUTPUT );

  //Configures input pins
  pinMode( A0, INPUT );

  //Configures servo, sets it in middle (0 - 180 degree range)
  servo1.attach( servoPin );
  servo1.write( 90 );

  //Initialize lastTime
  lastTime = millis();
  
  Serial.begin(9600);
  startTime = millis();
}

//Main code, runs repeatedly
void loop() {
  
  //Update Linescan values
  capture_linescan();

  //Gets center of line and turns servo
  int center = findCenter();
  int angle = turn( center );

  //Set speed of car based on angle

  int spd = 100;

  // Turn servo
  if ( angle < servoMiddle ){
    servo1.write( servoMiddle - abs( angle - servoMiddle )*rightTurnScale  );
  } else {
    servo1.write( angle );
  }
  
  delay(100);
}

int turn(int center){
  
  currentTime = millis();
  kP=0.6;
  kI=0;
  kD=1.3;
  dt=double(currentTime-lastTime);

  Error=center-63;

  if ( abs(Error) <= 2 ){
    return 90;
  }
  
  dError=(Error-prevError)/dt;
  iError=(Error+prevError)*dt;
  output=kP*Error+kD*dError+kI*iError;

  turnAngle = output; // Ranges from positive to negative
  prevError = Error;
  lastTime = currentTime;

  if ( turnAngle > 0 ) {
    return max(-turnAngle + servoMiddle, servoMiddle - maxTurnAngle);
  } else {
    return min(-turnAngle + servoMiddle, servoMiddle + maxTurnAngle);
  }

}

//Returns center of line
int findCenter(){
  //Apply the filters
  median_filter(linescanValues);
  return gradient_filter(medianValues);
}

void capture_linescan(){
  digitalWrite(CLK, LOW);
  digitalWrite(SI, HIGH);
  digitalWrite(CLK, HIGH);
  digitalWrite(SI, LOW);
  digitalWrite(CLK, LOW);

  //Clock out garbage
  for ( int i = 0; i < LINE_SCAN_PIXELS; i++ ) {
    digitalWrite(CLK, HIGH);
    digitalWrite(CLK, LOW);
  }

  delayMicroseconds( SATURATION_TIME ); //longer delay means higher values

  digitalWrite(SI, HIGH);
  digitalWrite(CLK, HIGH);
  digitalWrite(SI, LOW);
  digitalWrite(CLK, LOW);

  //Clock out scan
  for ( int i = 0; i < LINE_SCAN_PIXELS; i++ ) {
    delayMicroseconds(20);
    linescanValues[i] = analogRead(A0);
    digitalWrite(CLK, HIGH);
    digitalWrite(CLK, LOW);
  }

}

void constantSpdDrive( int spd ){
  //Writes analog values (PWM wave) to pins (value ranges from 0 to 255)
  analogWrite( lowPin, 0 ); 
  analogWrite( highPin, spd );
}

//Method not tested yet
void brake( int brakeAmount ){
  //Brake Test 
  analogWrite( highPin, 0 );
  analogWrite( lowPin, brakeAmount );    
}

//Accerleates to max speed
void accelerateMaxSpd(){
  analogWrite( lowPin, 0 );
  
  //The last PWM value that was written to the high pin
  int lastPWMValue = 25;

  //Acceleration To Max Speed Test
  for ( int i = 0; i < MAX_PULSE_SPEED; i++ ) {
    analogWrite( highPin, i ); 
    lastPWMValue = i;
    delay(100);
  }
}


int* median_filter(int linescan[])
{
   //Calculates median value based on neighboring values for each linescan value
  for ( int i = 0; i < 128; i++ ) {
    //Creates array of neighboring values
    int neighbors[3];
    neighbors[1] = linescan[i]; //this value is always the ith value
    if ( i == 0) {
      neighbors[0] = linescan[i];
      neighbors[2] = linescan[i+1];
    } else if ( i == 127 ) {
      neighbors[0] = linescan[i];
      neighbors[2] = linescan[i-1];
    } else {
      neighbors[0] = linescan[i-1];
      neighbors[2] = linescan[i+1];
    }

    int median;

    //Determines median of three numbers
    if ( neighbors[0] >= neighbors[1] ) {
      if ( neighbors[0] >= neighbors[2]) { //max value of list calculated
        median = max( neighbors[1], neighbors[2] );
      } else {
        median = neighbors[0];
      }
    } else {
      if ( neighbors[0] >= neighbors[2] ) { 
        median = neighbors[0];
      } else { //min value of list calculated
        median = min( neighbors[1], neighbors[2] );
      }
    }
    
    medianValues[i] = median;
  }
  
  return medianValues;
}

//Returns center of line from 0-127
int gradient_filter(int median[])
{
  numOfLines = 0;
  
  //Calculate gradient values
  for ( int i = 0; i < 128; i++ ) {
    if ( i == 127 ) {
      gradientValues[i] = 0;
    } else {
      gradientValues[i] = median[i + 1] - median[i]; 
    }
  }

  //Finds line
  
  //Variables to store highest gradients
  int maxVal = 0;
  int maxIndex = 0;
  int minVal = 0;
  int minIndex = 0;

  //Find the two highest gradients
  for ( int i = 0; i < 128; i++ ) {
    if ( gradientValues[i] > maxVal ) {
      maxVal = gradientValues[i];
      maxIndex = i;
    }
    if ( gradientValues[i] < minVal ) {
      minVal = gradientValues[i];
      minIndex = i;
    }
    if ( gradientValues[i] > 7 ) {
      numOfLines++;
    }

  }
  
  //Sets the left and right index of line
  int left = min(maxIndex, minIndex);
  int right = max(maxIndex, minIndex);
  int middle = (left + right) / 2;

  return middle;

  
}
