#include <PID_v1_bc.h> // Include the PID library
#include <SoftwareSerial.h>

#include <ros.h>
#include <std_msgs/Int32.h>

#define WHITE_THRESHOLD_LOW 500
#define WHITE_THRESHOLD_HIGH 1200
#define BLACK_THRESHOLD_LOW 0
#define BLACK_THRESHOLD_HIGH 200

// Line follower sensor pins
#define SENSOR_PIN_1 A0 
#define SENSOR_PIN_2 A1
#define SENSOR_PIN_3 A2
#define SENSOR_PIN_4 A3
#define SENSOR_PIN_5 A4
 
#define ENCA 20 // YELLOW
#define ENCB 21 // Green
#define IN1 9 //right motor1
#define IN2 10
#define PWM_R 11
#define TRIGGER_PIN 7
#define ECHO_PIN 8
#define MAX_DISTANCE 200

#define ENCC 2
#define ENCD 3
#define IN3 5
#define IN4 4
#define PWM_L 6

// Define PID parameters
double Kp = 17.0;
double Ki = 2;
double Kd = 0.01;
double error;
double outputLeft ;
double outputRight;
double leftSpeed;
double rightSpeed;
double setpoint = (WHITE_THRESHOLD_HIGH + BLACK_THRESHOLD_LOW) / 2;
int baseSpeed=160 ;
// Initialize PID controller objects for left and right motors
PID pidLeft(&error, &outputLeft, &setpoint, Kp, Ki, Kd, DIRECT);
PID pidRight(&error, &outputRight, &setpoint, Kp, Ki, Kd, DIRECT);

////////////////////////////////////////////////// GLOBAL VARIABLES ////////////////////////////////////////////
long duration, distance;
int ImRotating = 0;
int sensor1, sensor2, sensor3, sensor4, sensor5;
bool isSensor1Black, isSensor2Black, isSensor3Black, isSensor4Black, isSensor5Black;

int RoomNumber = 1;
int RobotPosition = 0;
int MyRobotSpeed;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Bluetooth Setup
SoftwareSerial BTSerial(18, 19);

// ROSSerial flags 
bool IArrivedHome = true; // ready to receive

// Define the ROS node handle
ros::NodeHandle nh;
// Define the callback function to handle incoming messages on the topic
void messageCallback(const std_msgs::Int32& msg) 
{
  RoomNumber = msg.data;
  Serial.println("in callback");
  if (RoomNumber == 1 || RoomNumber == 2)
    IArrivedHome = false;  
}

// Define the subscriber
ros::Subscriber<std_msgs::Int32> sub("QT_arduino", &messageCallback);

void setup() {
  Serial.begin(57600);
  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);
  pinMode(PWM_R,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  pinMode(ENCC,INPUT_PULLUP);
  pinMode(ENCD,INPUT_PULLUP);
  pinMode(PWM_L,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  pidLeft.SetOutputLimits(0, 255); // Adjust these limits as needed
  pidRight.SetOutputLimits(0, 255);
  pidLeft.SetMode(AUTOMATIC);
  pidRight.SetMode(AUTOMATIC);

  //ultrasonic
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT); 

  //Bluetooth Setup
  BTSerial.begin(9600);

  nh.initNode();
  // Subscribe to the topic67w
  nh.subscribe(sub);
}

void loop() 
{
  while (IArrivedHome)
  {
    nh.spinOnce();
    //Serial.println("Waiting For Message");
    delay(10);
  }

  // Read sensor values
  SensorsValues();

  // Check if all sensors read black
  if (isSensor1Black && isSensor2Black && isSensor3Black && isSensor4Black && isSensor5Black) 
  {
    RobotPosition++;
    noInterrupts();
    analogWrite(PWM_R,120); // Adjust speed as needed
    analogWrite(PWM_L,120); // Adjust speed as needed
    // MyRobotSpeed = 3;
    interrupts();
    if ((RobotPosition == 4) || (RobotPosition == 2))
    {
      TurnLeft();
      if (RobotPosition == 4)
      {
        RobotPosition = 0;
        RoomNumber = 0;
        IArrivedHome = true;
       // Serial.println("Going Home");
      }
      else if (RobotPosition == 2)
        BTSerial.println("z");
    }
    else 
    {
      if (((RobotPosition == 1) && (RoomNumber == 1)) || ((RobotPosition == 3) && (RoomNumber == 2)))
      {
      Serial.println("I'm Going Left");
      delay(1500);
      TurnLeft();
      delay(500);
      }
      else if (((RobotPosition == 3) && (RoomNumber == 1)) || ((RobotPosition == 1) && (RoomNumber == 2)))
      {
        Serial.println("I'm Going Left");
        delay(1500);
        TurnRight();
        delay(500);
      }
    }
    
    ImRotating =1;
    //delay(500);
  }
  int Counter = 0;
  int IArrived = 0;

  while(ImRotating == 1) 
  {
     if(((analogRead(SENSOR_PIN_3) >= BLACK_THRESHOLD_LOW) && (analogRead(SENSOR_PIN_3) <= BLACK_THRESHOLD_HIGH)) && ((analogRead(SENSOR_PIN_2) >= WHITE_THRESHOLD_LOW) && (analogRead(SENSOR_PIN_2) <= WHITE_THRESHOLD_HIGH)) && ((analogRead(SENSOR_PIN_4) >= WHITE_THRESHOLD_LOW) && (analogRead(SENSOR_PIN_4) <= WHITE_THRESHOLD_HIGH)))
     {
        Counter++;
     } 
      
    if (Counter == 1)
    {
      ImRotating = 0;
      stopMotors();
      delay(2000);
    }

  }
  if (ObjectDetected() > 30 && !IArrivedHome)
  {
       if ((isSensor1Black || isSensor2Black))
      {
        leftSlightly();
        
      } 
      else if ((isSensor4Black || isSensor5Black))
      {
        rightSlightly();
        
      } 
      else if (isSensor3Black) 
      {
       error = sensor2 - sensor4; 
      outputLeft = pidLeft.Compute();
      outputRight = pidRight.Compute();
      leftSpeed = baseSpeed + (outputLeft*3);
      rightSpeed = baseSpeed + (outputRight*3);
      setMotorSpeed(leftSpeed, rightSpeed);
      //Serial.println("yooh");       
      } 
      else 
      { 
        error = sensor2 - sensor4;
      }
  }
  else stopMotors();

  }

void stopMotors() {
  analogWrite(PWM_R, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(PWM_L, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forward() {
  analogWrite(PWM_R, 120); // Adjust speed as needed
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(PWM_L, 120); // Adjust speed as needed
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}


void leftSlightly() {
  analogWrite(PWM_R, 130); // Adjust speed as needed
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(PWM_L, 100); // Adjust speed as needed
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void rightSlightly() {
  analogWrite(PWM_R, 100); // Adjust speed as needed
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(PWM_L, 130); // Adjust speed as needed
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Set right motor speed and direction
  digitalWrite(IN1, (rightSpeed >= 0) ? HIGH : LOW);
  digitalWrite(IN2, (rightSpeed >= 0) ? LOW : HIGH);
  analogWrite(PWM_R, abs(rightSpeed)); // Ensure positive value for PWM

  // Set left motor speed and direction
  digitalWrite(IN3, (leftSpeed >= 0) ? HIGH : LOW);
  digitalWrite(IN4, (leftSpeed >= 0) ? LOW : HIGH);
  analogWrite(PWM_L, abs(leftSpeed)); // Ensure positive value for PWM
}

void SensorsValues()
{
  sensor1 = analogRead(SENSOR_PIN_1);
  sensor2 = analogRead(SENSOR_PIN_2);
  sensor3 = analogRead(SENSOR_PIN_3);
  sensor4 = analogRead(SENSOR_PIN_4);
  sensor5 = analogRead(SENSOR_PIN_5);
  error = sensor2 - sensor4;
  // Determine states of sensors
  isSensor1Black = (sensor1 >= BLACK_THRESHOLD_LOW && sensor1 <= BLACK_THRESHOLD_HIGH);
  isSensor2Black = (sensor2 >= BLACK_THRESHOLD_LOW && sensor2 <= BLACK_THRESHOLD_HIGH);
  isSensor3Black = (sensor3 >= BLACK_THRESHOLD_LOW && sensor3 <= BLACK_THRESHOLD_HIGH);
  isSensor4Black = (sensor4 >= BLACK_THRESHOLD_LOW && sensor4 <= BLACK_THRESHOLD_HIGH);
  isSensor5Black = (sensor5 >= BLACK_THRESHOLD_LOW && sensor5 <= BLACK_THRESHOLD_HIGH);
}

void TurnLeft()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void TurnRight()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

long ObjectDetected() {

    delay(5);
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(TRIGGER_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    distance = duration * 0.034 / 2;
    //Serial.println(distance);
    // Check if an obstacle is too close
      // Stop the motors
      return distance;
}