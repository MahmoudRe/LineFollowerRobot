// ros lib
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

//ros vars
ros::NodeHandle  nh;
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

//clear all pins
void clearPins() {
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  delay(100);
  digitalWrite(REV1, LOW);
  digitalWrite(REV2, LOW);
  digitalWrite(FWD1, LOW);
  digitalWrite(FWD2, LOW);
  delay(100);
}

//define callbacks and subscriber
void forward( const std_msgs::Empty& toggle_msg){
  clearPins();
  runForward = true;
  
  Serial.print("run forward!");
}
ros::Subscriber<std_msgs::Empty> sub("run_forward", &forward );

void forward( const std_msgs::Empty& toggle_msg){
  clearPins();
  runForward = false;
  
  Serial.print("run backward!");
  
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  delay(100);
  digitalWrite(REV1, HIGH);
  digitalWrite(REV2, HIGH);
  delay(100);
}
ros::Subscriber<std_msgs::Empty> sub2("run_backward", &backward );

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
 nh.subscribe(sub);
 nh.subscribe(sub2);
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
      delay(100);
      digitalWrite(FWD1, HIGH);
      digitalWrite(FWD2, HIGH);
      delay(100);
  } else {
      digitalWrite(EN1, HIGH);
      digitalWrite(EN2, HIGH);
      delay(100);
      digitalWrite(FWD1, LOW);
      digitalWrite(FWD2, LOW);
      delay(100);
  }
  
// ---------------
  nh.spinOnce();
  delay(1);
}
