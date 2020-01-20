// ros lib
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

//ros vars
ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

// Arduino pins
const int EN1 = 24;
const int EN2 = 25;
const int REV1 = 7;
const int REV2 = 3;
const int FWD1 = 6; 
const int FWD2 = 2;
const int SR_TRIGGER = 23;
const int SR_ECHO = 22;
const int Y_LED = 13;

// defines variables
long duration;
int distance;
boolean runForward = false;

// define speed vars
int currSpeed = 125;
int highSpeed = 250;
int midSpeed = 125;
int lowSpeed = 50;

//clear all pins
void clearPins() {
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  delay(1);
  digitalWrite(REV1, LOW);
  digitalWrite(REV2, LOW);
  digitalWrite(FWD1, LOW);
  digitalWrite(FWD2, LOW);
  delay(1);
}

//define callbacks and subscriber
void forward( const std_msgs::Empty& empty_msg){
  clearPins();
  runForward = true;
  
  Serial.print("run forward!");
}
ros::Subscriber<std_msgs::Empty> subForward("run_forward", &forward );

void backward( const std_msgs::Empty& empty_msg){
  clearPins();
  runForward = false;
  
  Serial.print("run backward!");
  
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  delay(1);
  analogWrite(REV1, midSpeed);
  analogWrite(REV2, midSpeed);
  delay(1);
}
ros::Subscriber<std_msgs::Empty> subBackward("run_backward", &backward );

void right( const std_msgs::Empty& empty_msg){
  clearPins();
  runForward = false;

  Serial.print("turn right!");
  
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  delay(1);
  analogWrite(FWD1, midSpeed);
  analogWrite(FWD2, lowSpeed);
  
  //do slight turn; wait 0.5 sec then go forward
  delay(500);
  clearPins();
}
ros::Subscriber<std_msgs::Empty> subRight("turn_right", &right );

void left( const std_msgs::Empty& empty_msg){
  clearPins();
  runForward = false;
  
  Serial.print("turn left!");
  
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  delay(1);
  analogWrite(FWD1, lowSpeed);
  analogWrite(FWD2, midSpeed);
  
  //do slight turn; wait 0.5 sec then go forward
  delay(500);
  clearPins();
}
ros::Subscriber<std_msgs::Empty> subLeft("turn_left", &left );

void stop( const std_msgs::Empty& empty_msg){
  clearPins();
  runForward = false;
}
ros::Subscriber<std_msgs::Empty> subStop("stop", &stop );

//Change speed callbacks and Subscribers
void highSpeedCall( const std_msgs::Empty& empty_msg){
  currSpeed = highSpeed;
}
ros::Subscriber<std_msgs::Empty> subHighSpeed("highSpeed", &highSpeedCall );

void lowSpeedCall( const std_msgs::Empty& empty_msg){
  currSpeed = lowSpeed;
}
ros::Subscriber<std_msgs::Empty> subLowSpeed("lowSpeed", &lowSpeedCall );


//listener to twist and its callback
 void processTwist(const geometry_msgs::Twist& msg) {
   clearPins();
 }
 ros::Subscriber<geometry_msgs::Twist> subTwist("cmd_vel", &processTwist );


void setup() {
 //initialize the pins
 pinMode(EN1, OUTPUT);
 pinMode(EN2, OUTPUT);
 pinMode(REV1, OUTPUT);
 pinMode(REV2, OUTPUT);
 pinMode(FWD1, OUTPUT); 
 pinMode(FWD2, OUTPUT);
 pinMode(SR_TRIGGER, OUTPUT); // Sets the trigPin as an Output
 pinMode(SR_ECHO, INPUT); // Sets the echoPin as an Input
 pinMode(Y_LED, OUTPUT);
 
 //ros initialization
 nh.initNode();
 nh.advertise(chatter);
 nh.subscribe(subForward);
 nh.subscribe(subBackward);
 nh.subscribe(subLeft);
 nh.subscribe(subRight);
 nh.subscribe(subStop);
 nh.subscribe(subHighSpeed);
 nh.subscribe(subLowSpeed);
 
 nh.subscribe(subTwist);
}


void loop() {

// ----------------- distance sensore -----------------  
  // Clears the trigPin
  digitalWrite(SR_TRIGGER, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(SR_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(SR_TRIGGER, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(SR_ECHO, HIGH);
  
  // Calculating the distance
  distance = duration*0.034/2;
  
  //run forward and stop when you encounter a blocking object
  if(distance > 20 && runForward) {
      Serial.print("Distance: ");
      Serial.println(distance);
    
      digitalWrite(EN1, HIGH);
      digitalWrite(EN2, HIGH);
      delay(1);
      analogWrite(FWD1, currSpeed);
      analogWrite(FWD2, currSpeed);
      delay(1);
  } else {
      digitalWrite(EN1, HIGH);
      digitalWrite(EN2, HIGH);
      delay(1);
      digitalWrite(FWD1, LOW);
      digitalWrite(FWD2, LOW);
      delay(1);
  }
  
// ROS handler spin --------------------
  nh.spinOnce();
  delay(1);
}
