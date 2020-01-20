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
int controlDelay = 200;
int FW1_Speed = 125;
int FW2_Speed = 125;

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
void forward(const float fw1_speed, const float fw2_speed){
  clearPins();
  runForward = true;
  FW1_Speed = fw1_speed;
  FW2_Speed = fw2_speed;
}

void backward(const float rev1_speed, const float rev2_speed){
  clearPins();
  runForward = false;
  
  analogWrite(REV1, rev1_speed);
  analogWrite(REV2, rev2_speed);
  
  delay(controlDelay);
  clearPins();
}

void turn(const float fw1_speed, const float fw2_speed){
  clearPins();
  runForward = false;
  
  analogWrite(FWD1, fw1_speed);
  analogWrite(FWD2, fw2_speed);
  
  delay(controlDelay);
  clearPins();
}

void stop(){
  clearPins();
  runForward = false;
}


//listener to twist and its callback
 void controlTwist(const geometry_msgs::Twist& msg) {
   
   if(msg.linear.x == 0 && msg.angular.z == 0) {     //stop
     stop();
   }  
   else if(msg.linear.x > 0 && msg.angular.z == 0) { //go forward
     forward(100 * msg.linear.x, 100 * msg.linear.x);
   }
   else if(msg.linear.x > 0 && msg.angular.z > 0) {  //turn left-up
     forward(100 * msg.linear.x, 100);
   }
   else if(msg.linear.x > 0 && msg.angular.z < 0) {  //turn right-up
     forward(100, 100 * msg.linear.x);
   } 
   else if (msg.linear.x == 0 && msg.angular.z > 0) { //turn left
     forward(200, 0);
   }
   else if (msg.linear.x == 0 && msg.angular.z < 0) { //turn right
     forward(0, 200);
   }
   else if (msg.linear.x < 0 && msg.angular.z == 0) {  //go backward
     backward(50 * abs(msg.linear.x), 50 * abs(msg.linear.x));
   }
   else if (msg.linear.x < 0 && msg.angular.z > 0) {  //turn right-back
     backward(50, 50 * abs(msg.linear.x));
   }
   else if (msg.linear.x < 0 && msg.angular.z < 0) {  //turn left-back
     backward(50 * abs(msg.linear.x), 50);
   }
 }
 ros::Subscriber<geometry_msgs::Twist> subTwist("cmd_vel", &controlTwist );


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
 
 //set the enable pins
 digitalWrite(EN1, HIGH);
 digitalWrite(EN2, HIGH);
 
 //ros initialization
 nh.initNode();
 nh.advertise(chatter);
 
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
      analogWrite(FWD1, FW1_Speed);
      analogWrite(FWD2, FW2_Speed);
      
      //stop after a quick delay, so doesn't run unless the forward key is pressed
      delay(controlDelay);
      runForward = false;
      clearPins();
  } 
  
// ROS handler spin --------------------
  nh.spinOnce();
  delay(1);
}
