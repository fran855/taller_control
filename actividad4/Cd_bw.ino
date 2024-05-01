#include "TimerOne.h"

// putput
#define PWMoutput 9
#define PWMperiod 1000000
#define PWMlow 0
#define PWMhigh 1
// input
#define sensorBajo A1
#define sensorAlto A2
#define sensorMaxBin 1000
#define sensorScaleB 0.05451
#define sensorBiasB -3.853
#define sensorScaleA 0.05869
#define sensorBiasA 37.49 

float error_anterior = 0, error_actual = 0;
float u_anterior = 0, u_actual = 0;
float k = 3.1623, p = 0.00237;
static int T = 1;

void setup()
{
  pinMode(PWMoutput, OUTPUT);
  Timer1.initialize(PWMperiod);              // initialize timer1, and set period
  Timer1.pwm(PWMoutput, 0);                // setup pwm, X duty cycle
  Timer1.attachInterrupt(timer_callback);    // attaches callback() as a timer overflow interrupt

  Serial.begin(115200);
}

void loop()
{
  //h0 eq 0.45m
  //u eq 0.5
  static float u0=0.5, h_ref=0.45, h, u;
  static float t1=0,t2=0,Ts=1;
  
  h_ref = float(matlab_receive(int(h_ref*100)))/100;

  t1=micros();
  
  // get height in [0.1, 0.75]
  h=getHeight();
  
  //=========================
  //CONTROL
  //Cd_bilineal =  -k ((1 + T/2 * p) + (T/2 * p - 1) * z^{-1})(1-z^{-1})
  //h in [0.1,0.75]meters, u in [0,1]-
  
  error = h_ref - h;
  u_actual = u_anterior - k * (1 + T/2 * p) error + k * (1 - T/2 * p) * error_anterior;
  error_anterior = error;
  u_anterior = u_actual;
  //=========================
  
  u = u_actual + u0;
  // set duty in [0, 1]
  PWMSetDuty(u);
  //send data to matlab with header "abcd" and delay till next control loop
  matlab_send(h,u);
  
  //time of control instructions execution
  t2=micros()-t1;
  delay(Ts*1000-t2/1000); //no usar delaymicroseconds con valores mayores a 16000
}

void timer_callback()
{
}

void PWMSetDuty(float duty)
{
  if(duty<PWMlow){duty=PWMlow;}
  else if(duty>PWMhigh){duty=PWMhigh;}
  Timer1.pwm(PWMoutput,duty*1024);
}

float getHeight()
{
  float sB, sA;
  sB = analogRead(sensorBajo);
  sA = analogRead(sensorAlto);
  if(sB<sensorMaxBin){return ((float)sB*sensorScaleB + sensorBiasB)/100;}
  else{return ((float)sA*sensorScaleA + sensorBiasA)/100;}
}

void matlab_send(float h, float u){
  Serial.write("abcd");
  byte * b = (byte *) &h;
  Serial.write(b,4);
  b = (byte *) &u;
  Serial.write(b,4);
}

int matlab_receive(int ref){
  if (Serial.available() > 0) {
    //Serial.println(2);
    signed char temp = Serial.read();
    ref=temp;
    //Serial.println(ref);
    }
  
  return ref;
}
