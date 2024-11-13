#include <ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>

//motor kanan bawah
int RPWM =2;
int LPWM =3;
int R_EN=22;
int L_EN=24;

//motor kiri bawah
int RPWM1 =4;
int LPWM1 =5;
int R_EN1=26;
int L_EN1=28;

//motor kanan atas
int RPWM2 =6;
int LPWM2 =7;
int R_EN2=30;
int L_EN2=32;

//motor kiri atas
int RPWM3 =8;
int LPWM3 =9;
int R_EN3=34;
int L_EN3=36;

int vSpeed = 25;
bool isReset = false;
bool isCommandDone = true;

//----
std_msgs::String cmd_status; //-+ message variable for cmd_status_topic_

// initialize ros callback
void commandCallback(const std_msgs::String& msg);

// initialize ros, node, subs, and pub
ros::NodeHandle nh;

ros::Publisher cmd_status_pub("cmd_status_topic_", &cmd_status);
ros::Subscriber<std_msgs::String> command_sub("command_topic", commandCallback);
//----

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM1,OUTPUT);
  pinMode(LPWM1,OUTPUT);
  pinMode(RPWM2,OUTPUT);
  pinMode(LPWM2,OUTPUT);
  pinMode(RPWM3,OUTPUT);
  pinMode(LPWM3,OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN1,OUTPUT);
  pinMode(L_EN1,OUTPUT);
  pinMode(R_EN2,OUTPUT);
  pinMode(L_EN2,OUTPUT);
  pinMode(R_EN3,OUTPUT);
  pinMode(L_EN3,OUTPUT);
  hidupkanDriver();

  Serial.begin(115200);

  // ros init
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.loginfo("Miria is online!!");

  // pub
  nh.advertise(cmd_status_pub);
  // sub
  nh.subscribe(command_sub);
}

void loop() {
  nh.spinOnce();
}
