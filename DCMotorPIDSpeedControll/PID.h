#ifndef __PID__
#define __PID__

const bool DIRECT = true;
const bool REVERSE = false;

const bool ON = true;
const bool OFF = false;

class PID
{
   public:
      PID(double*,double*,double*,double,double,double,unsigned long,bool,double,double,double);
      void Computing();
      void SetTunings(double,double,double);
      void SetSampleTime(unsigned long);
      void SetOutputLimits(double,double);
      void SetOnOff(bool);
      void SetDirection(bool);

   private:
      double Kp,Ki,Kd;
      double *input,*output;
      double *setpoint;

      double integral;
      double last_input;

      double min,max;

      unsigned long last_time;
      unsigned long sample_time;

      bool on_off;
      bool direction;
};

#endif
