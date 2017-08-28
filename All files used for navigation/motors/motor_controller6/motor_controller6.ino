
//ROS headers                                                                                         //from here
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include "robot_specs.h"

//Motor Shield headers
#include "DualVNH5019MotorShield.h"                                                                   //to here don't touch
DualVNH5019MotorShield md;                                                                            //from here

                                                                                                      //to here VNH5019 initialize                                                                                                   
#define encodPinA1      18     // first encoder A output                                              //from here
#define encodPinB1      34     // first encoder B output
#define encodPinA2      19     // second encoder A output
#define encodPinB2      35     // second encoder B output
#define LOOPTIME        100    // PID loop time(ms)
#define SMOOTH      10         //??

#define setmotorspeed

#define sign(x) (x > 0) - (x < 0)                                                                     //to here don't touch

unsigned long lastMilli = 0;       // loop timing                                                     //from here
unsigned long lastMilliPub = 0;
double rpm_req1 = 0;
double rpm_req2 = 0;
double rpm_act1 = 0;
double rpm_act2 = 0;
double rpm_req1_smoothed = 0;
double rpm_req2_smoothed = 0;                                                                         //to here don't touch

int PWM_val1 = 0;                                                                                     //from here
int PWM_val2 = 0;
volatile long count1 = 0;          // rev counter
volatile long count2 = 0;
long countAnt1 = 0;
long countAnt2 = 0;
float Kp =   7.0;
float Kd =   0.0;
float Ki =   0.0;
ros::NodeHandle nh;                                                                                  //to here don't touch

void handle_cmd( const geometry_msgs::Twist& cmd_msg) {                                              //from here
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;
  if (z == 0) {     // go straight
    // convert m/s to rpm
    rpm_req1 = x*60/(pi*wheel_diameter);
    rpm_req2 = rpm_req1;
  }
  else if (x == 0) {
    // convert rad/s to rpm
    rpm_req2 = z*track_width*60/(wheel_diameter*pi*2);
    rpm_req1 = -rpm_req2;
  }
  else {
    rpm_req1 = x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
    rpm_req2 = x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
  }
}                                                                                                    //to here don't touch

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle_cmd);                                    //from here
geometry_msgs::Vector3Stamped rpm_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg);
ros::Time current_time;
ros::Time last_time;                                                                                 //to here don't touch

void setup() {                                                                                       //from here

 Serial.begin(57600);                                                                               //this serial.begin starts at 115200 is it right for mega 2560?, all my files are set for 57600 baud rate
 Serial.println("Dual VNH5019 Motor Shield");
 md.init();                                                                                          //md.init() activate all md commands, right?        
 count1 = 0;
 count2 = 0;
 countAnt1 = 0;
 countAnt2 = 0;
 rpm_req1 = 0;
 rpm_req2 = 0;
 rpm_act1 = 0;
 rpm_act2 = 0;
 PWM_val1 = 0;
 PWM_val2 = 0;
 nh.initNode();
 nh.getHardware()->setBaud(57600);
 nh.subscribe(sub);
 nh.advertise(rpm_pub);                                                                              //to here don't touch
  
 pinMode(encodPinA1, INPUT);                                                                         //from here
 pinMode(encodPinB1, INPUT); 
 digitalWrite(encodPinA1, HIGH);                // turn on pullup resistor
 digitalWrite(encodPinB1, HIGH);
 attachInterrupt(5, encoder1, RISING);                                                               //use of interrupt 3 on mega board (pin 20)

 pinMode(encodPinA2, INPUT); 
 pinMode(encodPinB2, INPUT); 
 digitalWrite(encodPinA2, HIGH);                // turn on pullup resistor
 digitalWrite(encodPinB2, HIGH);
 attachInterrupt(4, encoder2, RISING);                                                               //use of interrupt 5 on mega board (pin 18)
                                                                                                     //to here don't touch
 
 md.setM1Speed(0);                                                                                   //change from motor1->setSpeed(0); to "0". If I'd put md.setM1Speed (0) instead 0"I think they do the same"
 md.setM2Speed(0);                                                                                   //same as above

}                                                                                                    

void loop() {                                                                                        //from here 
  nh.spinOnce();
  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME)   {      // enter tmed loop                                          
    getMotorData(time-lastMilli);
    PWM_val1 = updatePid(1, PWM_val1, rpm_req1, rpm_act1);
    PWM_val2 = updatePid(2, PWM_val2, rpm_req2, rpm_act2);
    
    if (rpm_req1 == 0)
       md.setM1Speed(0);
    else 
       md.setM1Speed(PWM_val1/35);
    if (rpm_req2 == 0)
       md.setM2Speed(0);
    else
       md.setM2Speed(PWM_val2/35);

    
    publishRPM(time-lastMilli);                                                                      //from here
    lastMilli = time;
  }
  if(time-lastMilliPub >= LOOPTIME) {
  //  publishRPM(time-lastMilliPub);
    lastMilliPub = time;
  }
}                                                                                                    //to here don't touch

void getMotorData(unsigned long time)  {                                                             //from here
 rpm_act1 = double((count1-countAnt1)*60*1000)/double(time*encoder_pulse*gear_ratio);
 rpm_act2 = double((count2-countAnt2)*60*1000)/double(time*encoder_pulse*gear_ratio);
 countAnt1 = count1;
 countAnt2 = count2;
}                                                                                                    //to here don't touch

int updatePid(int id, int command, double targetValue, double currentValue) {                        //from here
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;
  
  error = targetValue-currentValue;
  if (id == 1) {
    int_error1 += error;
    pidTerm = Kp*error + Kd*(error-last_error1) + Ki*int_error1;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp*error + Kd*(error-last_error2) + Ki*int_error2;
    last_error2 = error;
  }
  new_pwm = constrain(double(command)*MAX_RPM/4096.0 + pidTerm, -MAX_RPM, MAX_RPM);
  new_cmd = 4096.0*new_pwm/MAX_RPM;
  return int(new_cmd);
}                                                                                                    //to here don't touch

void publishRPM(unsigned long time) {                                                                //from here
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}                                                                                                    //to here don't touch

void encoder1() {                                                                                    //from here
  if (digitalRead(encodPinA1) == digitalRead(encodPinB1)) count1++;
  else count1--;
}
void encoder2() {
  if (digitalRead(encodPinA2) == digitalRead(encodPinB2)) count2--;
  else count2++;
}                    
