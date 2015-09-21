#include "PID.h"

const boolean DEBUG = true;

const int motor_pin_1 = 3; // digital 3
const int motor_pin_2 = 4; // digital 4
const int enable = 9; // digital 9

const int hall_intterup_pin = 0; // digital 2
double hall_counter = 0;
double rpm = 0;
unsigned long last_time = 0;
const double hall_sample_rate = 100;

double pwm = 0,setpoint = 1500;
double Kp=5, Ki=3, Kd=1;
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
  
  last_time = millis();
}

void loop()
{
  if((millis() - last_time) >= hall_sample_rate){
    rpm = hall_counter * 60.0f * (1000.0f/hall_sample_rate);
    hall_counter = 0;
    last_time = millis();
    
    if(DEBUG){
      Serial.println(rpm);
    }
  }
  
  
  pid.Computing();
  analogWrite(enable,pwm);
  

}

void HallISR(){
  hall_counter++;
}
