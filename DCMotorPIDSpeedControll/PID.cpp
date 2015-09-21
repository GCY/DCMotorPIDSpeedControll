#if ARDUINO >= 100
	#include <Arduino.h>
#else
	#include <WProgram.h>
#endif

#include "PID.h"

PID::PID(double *_input,double *_output,double *_setpoint,
      double _Kp,double _Ki,double _Kd,
      unsigned long _sample_time = 100000,
      bool _on_off = OFF,double _direction = DIRECT,
      double _min = 0,double _max = 255)
{
   input = _input;
   output = _output;
   setpoint = _setpoint;

   sample_time = _sample_time;

   on_off = _on_off;

   SetOutputLimits(_min,_max);

   SetDirection(_direction);

   SetTunings(_Kp,_Ki,_Kd);

   last_time = (micros() - sample_time);
}

void PID::Computing()
{
   if(on_off == OFF){
      return ;
   }

   unsigned long now = micros();

   if((now - last_time) >= sample_time){
      double error = (*setpoint - *input);

      integral += (Ki * error);
      if(integral > max){
	 integral = max;
      }
      else if(integral < min){
	 integral = min;
      }

      double diff_input = (*input - last_input);

      *output = (Kp * error) + integral - (Kd * diff_input);

      if(*output > max){
	 *output = max;
      }
      else if(*output < min){
	 *output = min;
      }

      last_input = *input;
      last_time = now;
   }
}

void PID::SetTunings(double _Kp,double _Ki,double _Kd)
{
   if((_Kp < 0) || (_Ki < 0) || (_Kd < 0)){
      return ;
   }

   double SampleTimeInSec = ((double)sample_time) / 1000000.0f;

   Kp = _Kp;
   Ki = _Ki * SampleTimeInSec;
   Kd = _Kd / SampleTimeInSec;

   if(direction != REVERSE){
      Kp = (0 - Kp);
      Ki = (0 - Ki);
      Kd = (0 - Kd);
   }
}

void PID::SetSampleTime(unsigned long _sample_time)
{
   if(_sample_time > 0){
      double ratio = ((double)_sample_time / (double)sample_time);

      Ki *= ratio;
      Kd /= ratio;

      sample_time = _sample_time;
   }
}

void PID::SetOutputLimits(double _min,double _max)
{
   if(_min > _max){
      return ;
   }

   min = _min;
   max = _max;

   if(on_off == ON){
      if(*output > max){
	 *output = max;
      }
      else if(*output < min){
	 *output = min;
      }

      if(integral > max){
	 integral = max;
      }
      else if(integral < min){
	 integral = min;
      }
   }
}

void PID::SetOnOff(bool _on_off)
{
   bool new_state = (_on_off == ON);

   if(new_state == (!on_off)){
      integral = *output;
      last_input = *input;

      if(integral > max){
	 integral = max;
      }
      else if(integral < min){
	 integral = min;
      }
   }

   on_off = new_state;
}

void PID::SetDirection(bool _direction)
{
  if((on_off == ON) && (_direction != direction)){
      Kp = (0 - Kp);
      Ki = (0 - Ki);
      Kd = (0 - Kd);
   }

   direction = _direction;
}
