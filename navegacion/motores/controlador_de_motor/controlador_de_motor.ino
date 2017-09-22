           
//ROS headers                                                                                        
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include "especificaciones_del_robot.h"

#include "DualVNH5019MotorShield.h"                                                                   
DualVNH5019MotorShield md;                                                                            

                                                                                                                                                                                                       
#define encodPinA1      18     // first encoder A output                                              
#define encodPinB1      34     // first encoder B output
#define encodPinA2      19     // second encoder A output
#define encodPinB2      35     // second encoder B output
#define LOOPTIME        100    // PID loop time(ms)
#define SMOOTH          10         //de 10

#define setmotorspeed

#define sign(x) (x > 0) - (x < 0)                                                                     

unsigned long lastMilli = 0;                                                         
unsigned long lastMilliPub = 0;
double rpm_req1 = 0;
double rpm_req2 = 0;
double rpm_act1 = 0;
double rpm_act2 = 0;
double rpm_req1_smoothed = 0;
double rpm_req2_smoothed = 0;                                                                         

int PWM_val1 = 0;                                                                                     
int PWM_val2 = 0;
volatile long count1 = 0;         
volatile long count2 = 0;
long countAnt1 = 0;
long countAnt2 = 0;
float Kp =   13.0; //de 15
float Kd =   0.0;
float Ki =   0.0;
ros::NodeHandle nh;                                                                               

void handle_cmd( const geometry_msgs::Twist& cmd_msg) {                                             
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;
  if (z == 0) {     
    
    rpm_req1 = x*60/(pi*diametro_ruedas);
    rpm_req2 = rpm_req1;
  }
  else if (x == 0) {
    
    rpm_req2 = z*distancia_ruedas*60/(diametro_ruedas*pi*2);
    rpm_req1 = -rpm_req2;
  }
  else {
    rpm_req1 = x*60/(pi*diametro_ruedas)-z*distancia_ruedas*60/(diametro_ruedas*pi*2);
    rpm_req2 = x*60/(pi*diametro_ruedas)+z*distancia_ruedas*60/(diametro_ruedas*pi*2);
  }
}                                                                                                    

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle_cmd);                                  
geometry_msgs::Vector3Stamped rpm_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg);
ros::Time current_time;
ros::Time last_time;                                                                                 

void setup() {                                                                                       

 Serial.begin(57600);                                                                               
 Serial.println("Dual VNH5019 Motor Shield");
 md.init();                                                                                                  
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
 nh.advertise(rpm_pub);                                                                             
  
 pinMode(encodPinA1, INPUT);                                                                        
 pinMode(encodPinB1, INPUT); 
 digitalWrite(encodPinA1, HIGH);                
 digitalWrite(encodPinB1, HIGH);
 attachInterrupt(5, encoder1, RISING);                                                               

 pinMode(encodPinA2, INPUT); 
 pinMode(encodPinB2, INPUT); 
 digitalWrite(encodPinA2, HIGH);               
 digitalWrite(encodPinB2, HIGH);
 attachInterrupt(4, encoder2, RISING);                                                               
                                                                                                    
 
 md.setM1Speed(0);                                                                                   
 md.setM2Speed(0);                                                                                   

}                                                                                                    

void loop() {                                                                                       
  nh.spinOnce();
  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME)   {                                               
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
       md.setM2Speed(PWM_val2/30);  

    
    publishRPM(time-lastMilli);                                                                      
    lastMilli = time;
  }
  if(time-lastMilliPub >= LOOPTIME) {
  //publishRPM(time-lastMilliPub);
    lastMilliPub = time;
  }
}                                                                                                    

void getMotorData(unsigned long time)  {                                                             
 rpm_act1 = double((count1-countAnt1)*60*1000)/double(time*pulso_encoders*relacion_transformacion);
 rpm_act2 = double((count2-countAnt2)*60*1000)/double(time*pulso_encoders*relacion_transformacion);
 countAnt1 = count1;
 countAnt2 = count2;
}                                                                                                    

int updatePid(int id, int command, double targetValue, double currentValue) {                        
  double pidTerm = 0;                            
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
}                                                                                                   

void publishRPM(unsigned long time) {                                                                
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}                                                                                                   

void encoder1() {                                                                                   
  if (digitalRead(encodPinA1) == digitalRead(encodPinB1)) count1++;
  else count1--;
}
void encoder2() {
  if (digitalRead(encodPinA2) == digitalRead(encodPinB2)) count2--;
  else count2++;
}               
