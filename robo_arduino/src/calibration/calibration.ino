/************************************************************
 * Inmoov ROS calibration program
 * Casimir Sowinski 2016 
 */

// Check Arduino version
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

// Includes
#include <ros.h>
#include <string.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <PololuMaestro.h>

// Prototypes 
void teleop_cb(const geometry_msgs::Twist&);
float rad2Polo(float);
float deg2Polo(float);
float deg2rad(float);
int D2P_elbow(float);
int D2P_arm_rot(float);
int D2P_arm_fwd(float);
int D2P_arm_out(float);

// Init ROS vars
std_msgs::Float64 ang_0;
std_msgs::Float64 ang_0_old;
std_msgs::Float64 ang_1;
std_msgs::Float64 ang_1_old;
std_msgs::Float64 ang_2;
std_msgs::Float64 ang_2_old;
std_msgs::Float64 ang_out;
std_msgs::Float64 ang_out_old;

int testAng = 2400;
int up = 1;

// Init other vars
#define maestroSerial Serial1
MiniMaestro maestro(Serial1);

// Nodes, pubs, and subs
ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> sub_teleop("turtle1/cmd_vel", teleop_cb);

void setup() {
  // Assign vars  
  ang_0.data      = 0;
  ang_0_old.data  = 0;
  ang_1.data      = 0;
  ang_1_old.data  = 0;
  ang_2.data      = 0;  
  ang_2_old.data  = 0;    

  // Setup servos
  maestroSerial.begin(9600);  
  for(int i = 0; i < 2; i++){
    maestro.setSpeed(i, 10);
    maestro.setAcceleration(i, 50);
    maestro.setTarget(i, D2P_elbow(0));
  }

  // Setup ROS node
  nh.initNode();
  
  // Handle pubs and subs
  nh.subscribe(sub_teleop);  

}

void loop() {

  // Cycle between 600 and 2400
  if(up){
    if(testAng < 8000){
      testAng += 2;
    } else {
      testAng = 8000;
      up = 0;
    }    
  } else {
    if(testAng > 2400){
      testAng -= 2;
    } else {
      testAng = 2400;
      up = 1;
    }
  }

  /*
  // Print angle
  char msg [30];
  int len;
  len = sprintf(msg, "cmd: %d", testAng);  
  nh.loginfo(msg);

  // Set angle
  maestro.setTarget(0, testAng);
  */
  
  nh.spinOnce();
  delay(1);

}


void teleop_cb(const geometry_msgs::Twist& cmd){     

  //std_msgs::Float64 ang_0_del;
  //std_msgs::Float64 ang_1_del;


  const int lim_0_l = 0;
  const int lim_0_h = 58;
  const int lim_1_l = -90;
  const int lim_1_h = 50;
  const int lim_2_l = -50;
  const int lim_2_h = 90;
  const int lim_out_l = 0;
  const int lim_out_h = 80;
   
  //ang_0_del.data = cmd.angular.z / 2;
  //ang_1_del.data = cmd.linear.x / 2;  
  
  ang_0.data += cmd.linear.x / 2;  
  ang_1.data -= cmd.angular.z / 2; 
  ang_2.data += cmd.linear.x / 2;  
  ang_out.data += cmd.linear.x / 2; 


  // Limit checking
  if (ang_0.data > lim_0_h) 
  { ang_0.data = lim_0_h; }
  if (ang_0.data < lim_0_l) 
  { ang_0.data = lim_0_l; } 
  if (ang_1.data > lim_1_h) 
  { ang_1.data = lim_1_h; }
  if (ang_1.data < lim_1_l) 
  { ang_1.data = lim_1_l; }
  if (ang_2.data > lim_2_h) 
  { ang_2.data = lim_2_h; }
  if (ang_2.data < lim_2_l) 
  { ang_2.data = lim_2_l; }

  if (ang_out.data > lim_out_h) 
  { ang_out.data = lim_out_h; }
  if (ang_out.data < lim_out_l) 
  { ang_out.data = lim_out_l; }

  
  // Send commands to servos 
 // maestro.setTarget(0, D2P_elbow(ang_0.data));
  maestro.setTarget(1, D2P_arm_rot(ang_1.data));
  //maestro.setTarget(0, D2P_arm_fwd(ang_2.data));  
  maestro.setTarget(0, D2P_arm_out(ang_out.data));
  
  
  char msg [50];
  int len;      
  //len = sprintf(msg, "elbow:[d: %d, p: %d] rot:[d: %d, p: %d]", (int) ang_0.data, D2P_elbow(ang_0.data), (int) ang_1.data, D2P_arm_rot(ang_1.data));  
  //len = sprintf(msg, "fwd:[d: %d, p: %d] rot:[d: %d, p: %d]", (int) ang_2.data, D2P_arm_fwd(ang_2.data), (int) ang_1.data, D2P_arm_rot(ang_1.data));  
  len = sprintf(msg, "out:[d: %d, p: %d] rot:[d: %d, p: %d]", (int) ang_out.data, D2P_arm_out(ang_out.data), (int) ang_1.data, D2P_arm_rot(ang_1.data));  
  nh.loginfo(msg);  
  
  //nh.loginfo("Entered cb");
}

// Convert radians to Pololu
float rad2Polo(float ang_rad){
  return 4000 + 4000 * ang_rad / PI / 2;
}

// Convert degrees to Pololu (0-180->4000-8000)
float deg2Polo(float ang_deg){
  return 4000 + 4000 * ang_deg / 180;   
}

// Convert degrees to radians
float deg2rad(float ang_deg){
  return ang_deg * 3.14159 / 180;
}

// Convert degrees to Pololu (0 to 140) -> (2400 to 8000)
int D2P_elbow(float ang_deg){
  //return (int) 2400 + ang_deg * 40;
  //return (int) 2880 + ang_deg * 5120/ 135;
  return (int) 2773 + ang_deg * 3111 / 58;
}

// Convert degrees to Pololu (-90 to 50) -> (2774 to 8000)
int D2P_arm_rot(float ang_deg){
  return (int) 6133.6 + ang_deg * 5226 / 140;
}

// Convert degrees to Pololu (-50 to 90) -> (2774 to 8000)
int D2P_arm_fwd(float ang_deg){  
  return (int) 4640.4 + ang_deg * 5226 / 140;
}

// Convert degrees to Pololu (0 to 90) -> (2774 to 8000)
int D2P_arm_out(float ang_deg){  
  return (int) 2948 + ang_deg * 2090 / 80;
}
























