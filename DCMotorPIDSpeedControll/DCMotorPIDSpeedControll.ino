#include "PID.h"

const boolean DEBUG = true;

const int motor_pin_1 = 3; // digital 3
const int motor_pin_2 = 4; // digital 4
const int enable = 9; // digital 9

const int hall_intterup_pin = 0; // digital 2
double hall_counter = 0;
double rpm = 0;
unsigned long last_time = 0;
const unsigned long hall_sample_rate = 123456; //1.23456 second

double pwm = 0,setpoint = 2136; // 2221 RPM
double Kp=0.1, Ki=0.07, Kd=0.01;
PID pid(&rpm, &pwm, &setpoint, Kp, Ki, Kd,100000,ON,REVERSE,0,255);

void setup()
{
  if(DEBUG){
    Serial.begin(9600);
    
    attachInterrupt(hall_intterup_pin, HallISR, FALLING);
  }
  
  pinMode(motor_pin_1,OUTPUT);
  pinMode(motor_pin_2,OUTPUT);
  pinMode(enable,OUTPUT); 
  
  digitalWrite(motor_pin_1,LOW);
  digitalWrite(motor_pin_2,HIGH);
  
  last_time = micros();
}

void loop()
{
  
  if((micros() - last_time) >= hall_sample_rate){
    rpm = ((hall_counter * (1000000.0f/(double)(micros()-last_time))) * 60);
    hall_counter = 0;
    last_time = micros();
    
    if(DEBUG){
      Serial.print(rpm);
      Serial.print(",");
      Serial.println(pwm);
    }
  }
  pid.Computing();
  analogWrite(enable,pwm);
  

}

void HallISR(){
  ++hall_counter;
}
