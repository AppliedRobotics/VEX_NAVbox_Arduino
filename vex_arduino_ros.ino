
#include <Wire.h>
#include "I2CEncoder.h"
#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
// Create an I2CEncoder for measuring the speed
#define rightMPin A0
#define leftMPin A1
#define WheelRadius 0.05
#define baseLength 0.25
I2CEncoder leftEncoder;
I2CEncoder rightEncoder;
Servo rightMotor;
Servo leftMotor;
int time_millis = 0;
float current_velocity[2] = {0.0, 0.0};
float odometry[3] = {0.0, 0.0, 0.0};

void cmd_cb(const geometry_msgs::Twist& vel){
  float vX = vel.linear.x;
  float wZ =vel.angular.z;
  int v_r = (int)-1*(2*vX - baseLength*wZ)/(2*WheelRadius);
  int v_l = (int)(2*vX + baseLength*wZ)/(2*WheelRadius); 
  rightMotorControl(v_r);
  leftMotorControl(v_l);
}


typedef ros::NodeHandle_<ArduinoHardware, 5, 5, 384, 384, ros::FlashReadOutBuffer_> NodeHandleCompact;
NodeHandleCompact nh;  //Создаем объект класса NodeHandleCompact
sensor_msgs::JointState joint_state;
nav_msgs::Odometry odom;
ros::Subscriber<geometry_msgs::Twist> sub_cmd("cmd_vel", cmd_cb);
ros::Publisher joint_state_pub("joint_state", &joint_state);
ros::Publisher odom_pub("odom", &odom);

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  //INIT MOTORS
  pinMode(rightMPin, OUTPUT);
  rightMotor.attach(rightMPin);
  pinMode(leftMPin, OUTPUT);
  leftMotor.attach(leftMPin);
  
  // Initialize the encoders for a 269 motors that are moving 2/3 of a
  // foot per motor output shaft rotation.
  leftEncoder.init((2.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  rightEncoder.init((2.0/3.0)*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  rightEncoder.setReversed(true);

  nh.getHardware()->setBaud(115200);//задаем частоту общения с росом 
  nh.initNode(); //инициализируем ноду
  nh.advertise(joint_state_pub);//инициализируем паблишеры и субскрайберы
  nh.advertise(odom_pub);
  nh.subscribe(sub_cmd);
}

void loop() {
  // Calculate the average speed of the robot in feet per second and print it.
  int dt = millis() - time_millis;
  if(dt > 15)
  {
    time_millis = millis();
    current_velocity[0] = leftEncoder.getSpeed()*0.10471975942; 
    current_velocity[1]  = rightEncoder.getSpeed()*0.10471975942; 
    nh.spinOnce();
    joint_state.header.stamp = nh.now();
    joint_state.velocity = current_velocity;
    joint_state_pub.publish(&joint_state); 

    float vX = (WheelRadius)*(current_velocity[0]+current_velocity[1])/2;
    float wZ = (WheelRadius)*(current_velocity[0]+current_velocity[1])/baseLength;
    odometry[2] += (dt/1000)*wZ;
    odometry[0] += (dt/1000)*(cos(odometry[2])*vX);
    odometry[1] += (dt/1000)*(sin(odometry[2])*vX);
    odom.pose.pose.position.x = odometry[0];
    odom.pose.pose.position.y = odometry[1];
    odom.pose.pose.orientation.z = sin(odometry[2]/2);
    odom.pose.pose.orientation.w = cos(odometry[2]/2);
    odom.twist.twist.linear.x = vX;
    odom.twist.twist.angular.z = wZ;
    odom_pub.publish(&odom);
  }
}

void rightMotorControl(int value){

rightMotor.write(map(value,-100,100,1000,2000));

}
void leftMotorControl(int value){

leftMotor.write(map(value,-100,100,1000,2000));

}
