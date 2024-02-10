#include "Follow1.h"

IR IR;
Ultrasonic Ultrasonic;
extern Balanced Balanced;

void Ultrasonic::Pin_init()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
}


char Ultrasonic ::measure_flag=0;
unsigned long Ultrasonic ::measure_prev_time=0;
double Ultrasonic ::distance_value;
void Function::Follow_Mode1()
{
  IR.Send();
  Ultrasonic.Get_Distance();
     
  if (millis() - follow_prev_time >= 100)
  {
     If_IR_TRIGGERED ? IR.Check():Ultrasonic.Check();
     
     follow_prev_time = millis();
  }
     
}

void IR::Check()
{
    int motion = left_is_obstacle + right_is_obstacle;
    switch(motion)
    {
      case FOLLOW_LEFT:
                        Balanced.Motion_Control(LEFT);
                        left_is_obstacle=0;break;
      case FOLLOW_RIGHT:
                        Balanced.Motion_Control(RIGHT);
                        right_is_obstacle=0;break;                      
      
      case FOLLOW_BACK:
                        Balanced.Motion_Control(BACK);
                        right_is_obstacle=0;
                        left_is_obstacle=0;break;        
    }
}

void Ultrasonic::Check()
{ 
  DISTANCE_JUDAGEMENT ? Balanced.Motion_Control(FORWARD) : Balanced.Stop();
}

void IR::Send()
{
  static unsigned long ir_send_time;

  if (millis() - ir_send_time > 15)
  {
    for (int i = 0; i < 39; i++)
    { 
      digitalWrite(IR_SEND_PIN, LOW);
      delayMicroseconds(9);
      digitalWrite(IR_SEND_PIN, HIGH);
      delayMicroseconds(9);
    }
    ir_send_time=millis();
  }
}
