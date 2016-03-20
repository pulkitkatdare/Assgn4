#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "BMSerial.h"
#include "RoboClaw.h"

//Roboclaw Address
#define address1 0x80
#define address2 0x81

//Velocity PID coefficients
#define Kp 1.0
#define Ki 0.5
#define Kd 0.25
#define qpps 44000
int targetAngle=0;
int incomingValue=0;
int motorspeed=0;
int motorleftdir = 0;
int motorrightdir = 0;
int select = 2; // 1, 2, 3 left right , both
float k1,k2,k3,k4;
float radius
//Definte terminal for display. Use hardware serial pins 0 and 1
//BMSerial terminal(0,1);

//Setup communcaitions with roboclaw. Use pins 10 and 11 with 10ms timeout
RoboClaw roboclaw1(15,14,10000);
RoboClaw roboclaw2(15,14,10000);
ros::NodeHandle  nh;
int Rm = 2;
int Lm = 3; 
void bothMotorForward(){
  motorleftdir = 1;
  motorrightdir = -1;
  Serial.println("Moving Forward");
}
void bothMotorBackward(){
  motorleftdir = -1;
  motorrightdir = 1;
  Serial.println("Moving Backward");
}
void stopBothMotor(){
  motorrightdir = 0;
  motorleftdir = 0;
  Serial.println("Motors Stopped");
}
void actuatorLoop(){
  Serial.println("True");
  Serial.println(motorspeed);
  if(motorleftdir == 1){
    if (motorspeed > 0){motorspeed = motorspeed;}
    if(motorspeed<0){motorspeed = -motorspeed;}
    analogWrite(Lm, motorspeed);
    digitalWrite(22, LOW); 
    digitalWrite(23, HIGH);
  }
  if(motorleftdir == -1){
    
    if (motorspeed > 0){motorspeed = motorspeed;}
    if(motorspeed<0){motorspeed = -motorspeed;}
    analogWrite(Lm, motorspeed);
    digitalWrite(22, HIGH); 
    digitalWrite(23, LOW);
  }
  if(motorleftdir == 0){
    analogWrite(Lm, 0);
  }
  
  
  if(motorrightdir == 1){
    if (motorspeed > 0){motorspeed = motorspeed;}
    if(motorspeed<0){motorspeed = -motorspeed;}
    analogWrite(Rm, motorspeed);
    digitalWrite(24, LOW); 
    digitalWrite(25, HIGH);
  }
  if(motorrightdir == -1){
    if (motorspeed > 0){motorspeed = motorspeed;}
    if(motorspeed<0){motorspeed = -motorspeed;}
    analogWrite(Rm, motorspeed);
    digitalWrite(24, HIGH); 
    digitalWrite(25, LOW);
  }
  if(motorrightdir == 0){
    analogWrite(Rm, 0);
  }
  gotoAngle(targetAngle);

}
void gotoAngle(int target){
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;

  struct AngleMap {
   int minA = -90;
   int maxA = 90;

   int minEnc = -450;
   int maxEnc = 450;
   
  }angleMap; 
  
  if(select == 2){
    int enc1pos = roboclaw1.ReadEncM1(address1, &status1, &valid1);
        int theta=targetAngle;//type in the value for the theta...it could be between -90 and +90
  int ne=5.34*theta;
  float c=ne-enc1pos;
  float k=0.13;
  float vel=k*c;
  if(enc1pos<ne-50)
  { c=(ne-50)-enc1pos;
    vel=k*c;
    vel = vel +10;
    roboclaw1.BackwardM1(address1,vel);
    delay(100);
   terminal.println(enc1pos); 
  }
 else if(enc1pos>ne+50)                   // +5 encoder reading is the tolerance
{
  c=(ne+50)-enc1pos;
    vel=k*c;
  vel = vel - 10;
  roboclaw1.ForwardM1(address1,-vel);
  delay(100);
  terminal.println(enc1pos);
} 
else
{
  roboclaw1.ForwardM1(address1,0);
  delay(100);
  terminal.println(enc1pos);
}
    
  }
  
  if(select == 2){
    int enc2pos = roboclaw1.ReadEncM2(address1, &status2, &valid2);
        int theta=-targetAngle;//type in the value for the theta...it could be between -90 and +90
  int ne=5.34*theta;
  float c=ne-enc2pos;
  float k=0.13;
  float vel=k*c;
  if(enc2pos<ne-50)
  { c=(ne-50)-enc2pos;
    vel=k*c;
    vel = vel + 10;
    roboclaw1.ForwardM2(address1,vel);
    delay(100);
   terminal.println(enc2pos); 
  }
 else if(enc2pos>ne+50)                   // +5 encoder reading is the tolerance
{c=(ne+50)-enc2pos;
    vel=k*c;
  vel = vel - 10;
  roboclaw1.BackwardM2(address1,-vel);
  delay(100);
  terminal.println(enc2pos);
} 
else
{
  roboclaw1.ForwardM2(address1,0);
  delay(100);
  terminal.println(enc2pos);
}
  }
  
if(select == 2){
    int enc1pos = roboclaw2.ReadEncM1(address2, &status1, &valid1);
        int theta=+targetAngle;//type in the value for the theta...it could be between -90 and +90
  int ne=5.34*theta;
  float c=ne-enc1pos;
  float k=0.13;
  float vel=k*c;
  if(enc1pos<ne-100)
  {c=(ne-100)-enc1pos;
  vel=k*c;
    vel = vel+10;
    roboclaw2.BackwardM1(address2,vel);
    delay(100);
  terminal.println(enc1pos); 
  }
 else if(enc1pos>ne+100)                   // +5 encoder reading is the tolerance
{ c=(ne+100)-enc1pos;
vel=k*c;
  vel = vel-10;
  roboclaw2.ForwardM1(address2,-vel);
  delay(100);
  terminal.println(enc1pos);
} 
else
{
  roboclaw2.BackwardM1(address2,0);
  delay(100);
  terminal.println(enc1pos);
}
    
  }
if(select == 2){
    int enc2pos = roboclaw2.ReadEncM2(address2, &status2, &valid2);
        int theta=-targetAngle;//type in the value for the theta...it could be between -90 and +90
  int ne=5.34*theta;
  float c=ne-enc2pos;
  float k=0.11;
  float vel=k*c;
  if(enc2pos<ne-100)
  {c=(ne-100)-enc2pos;
  vel=k*c;
    vel = vel+10;
    roboclaw2.BackwardM2(address2,vel);
    delay(100);
   terminal.println(enc2pos); 
  }
 else if(enc2pos>ne+100)                   // +5 encoder reading is the tolerance
{c=(ne+100)-enc2pos;
  vel=k*c;
  vel = vel-10 ;
  roboclaw2.ForwardM2(address2,-vel);
  delay(100);
  terminal.println(enc2pos);
} 
else
{
  roboclaw2.ForwardM2(address2,0);
  delay(100);
  terminal.println(enc2pos);
}
    
  }

}

void messageCb( const std_msgs::Float32MultiArray& nav_msg){
  int motorspeed = (nav_msg.data[0]) ;
  if (motorspeed > 0){
  bothMotorForward();
} 

else if (motorspeed < 0){

   bothMotorBackward();
}



else {
    stopBothMotor(); 
}
targetAngle = nav_msg.data[7];
actuatorLoop();
}
ros::Subscriber<std_msgs::Float32MultiArray> sub("/rover/ros_mob_data", &messageCb );
void setup(){
  pinMode(2, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  
  pinMode(3, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  Serial.begin(9600);
  //terminal.begin(9600);
  //roboclaw1.begin(9600);
  //roboclaw2.begin(9600);
  nh.initNode();
  nh.subscribe(sub);
  nh.getParam("/navigation/rover_dimension/radius",radius);
  nh.getParam("/navigagion/control_parameters/k1",k1);
  nh.getParam("/navigation/control_parameters/k2",k2);
  nh.getParam("/navigation/control_parameters/k3",k3);
  nh.getParam("/navigation/control_parameters/k4",k4);
}


void loop()
{  
  nh.spinOnce();
  delay(1);
}
