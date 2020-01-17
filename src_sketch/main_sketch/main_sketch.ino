// ros lib
#include <ros.h>
#include <std_msgs/String.h>

//ros vars
ros::NodeHandle  nh;
std_msgs::String str_msg;
std_msgs::Int8 int_msg;
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
}

void loop() {
  
// -------------- Clear / Stop all motors -------------
//  digitalWrite(EN1, HIGH);
//  digitalWrite(EN2, HIGH);
//  delay(500);
//  digitalWrite(REV1, LOW);
//  digitalWrite(REV2, LOW);
//  digitalWrite(FWD1, LOW);
//  digitalWrite(FWD2, LOW);
//  delay(500);


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
  
  //stop when you encounter a blocking object
  if(distance > 20) {
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
}
