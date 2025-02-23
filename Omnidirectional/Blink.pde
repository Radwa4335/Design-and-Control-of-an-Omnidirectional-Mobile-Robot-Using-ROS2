/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

const int motor_left_front = 5;
const int motor_right_front = 6;
const int motor_left_back = 10;
const int motor_right_back = 11;
int motorA1 = 3;  
int motorA2 = 4;
int motorB1 = 8;  
int motorB2 = 7; 
int motorC1 = 9;  
int motorC2 = 12;
int motorD1 = 2;  
int motorD2 = 13;

int division_x = 0;
int division_y = 0;
int division_z = 0;
int division_angular_z = 0;

void cmdcallback(const geometry_msgs::Twist& twist){
    int speed_left_front = float(twist.linear.x);
    int speed_right_front = float(twist.linear.y);
    int speed_right_back = float(twist.linear.z);
    int speed_left_back = float(twist.angular.z);
   
    
    speed_left_front = constrain(speed_left_front, -255, 255);
    speed_right_front = constrain(speed_right_front, -255, 255);
    speed_right_back = constrain(speed_right_back, -255, 255);
    speed_left_back = constrain(speed_left_back, -255, 255);


  
      if (speed_left_front > 0) {
        digitalWrite(motorA1, LOW);
        digitalWrite(motorA2, HIGH);
      }  
      else if (speed_left_front < 0){
        digitalWrite(motorA1, HIGH);
        digitalWrite(motorA2, LOW);
      }
      else {
        digitalWrite(motorA1, LOW);
        digitalWrite(motorA2, LOW);

      }

      if (speed_right_front > 0) {
        digitalWrite(motorB1, LOW);
        digitalWrite(motorB2, HIGH);
      } 
      else if (speed_right_front < 0){
        digitalWrite(motorB1, HIGH);
        digitalWrite(motorB2, LOW);
      }
      else {
        digitalWrite(motorB1, LOW);
        digitalWrite(motorB2, LOW);

      }


        if (speed_left_back > 0 ) {
          digitalWrite(motorC1, LOW);
          digitalWrite(motorC2, HIGH);
        } 
        else if (speed_left_back < 0){
          digitalWrite(motorC1, HIGH);
          digitalWrite(motorC2, LOW);
        }
      else {
        digitalWrite(motorC1, LOW);
        digitalWrite(motorC2, LOW);

      }



      if (speed_left_back > 0) {
        digitalWrite(motorD1, LOW);
        digitalWrite(motorD2, HIGH);
      } 
      else if (speed_left_back < 0){
        digitalWrite(motorD1, HIGH);
        digitalWrite(motorD2, LOW);
      }
      else {
        digitalWrite(motorD1, LOW);
        digitalWrite(motorD2, LOW);

      }

    analogWrite(motor_left_front, abs(speed_left_front));
    analogWrite(motor_right_front, abs(speed_right_front));
    analogWrite(motor_left_back, abs(speed_left_back));
    analogWrite(motor_right_back, abs(speed_right_back));

/*    digitalWrite(motorA1, LOW);
      digitalWrite(motorA2, HIGH);
      analogWrite(motor_left_front, abs(speed_left_front));

      digitalWrite(motorB1, LOW);
      digitalWrite(motorB2, HIGH);
      analogWrite(motor_right_front, abs(speed_right_front));

      digitalWrite(motorC1, LOW);
      digitalWrite(motorC2, HIGH);
      analogWrite(motor_right_back, abs(speed_right_back));

      digitalWrite(motorD1, LOW);
      digitalWrite(motorD2, HIGH);
      analogWrite(motor_left_back, abs(speed_left_back));
*/
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdcallback);


void setup()
{ 
    pinMode(motor_left_front, OUTPUT);
    pinMode(motor_right_front, OUTPUT);
    pinMode(motor_left_back, OUTPUT);
    pinMode(motor_right_back, OUTPUT);
    pinMode(motorA1, OUTPUT);
    pinMode(motorA2, OUTPUT);
    pinMode(motorB1, OUTPUT);
    pinMode(motorB2, OUTPUT);
    pinMode(motorC1, OUTPUT);
    pinMode(motorC2, OUTPUT);
    pinMode(motorD1, OUTPUT);
    pinMode(motorD2, OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub);

}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

