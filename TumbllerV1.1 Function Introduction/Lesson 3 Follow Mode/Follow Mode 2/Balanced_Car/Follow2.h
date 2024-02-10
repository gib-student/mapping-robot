#ifndef _FOLLOW2_H_
#define _FOLLOW2_H_

#include <Arduino.h>
#include "Balanced.h"


class Ultrasonic 
{
  public:   
          void Pin_init();
          void Get_Distance();
          void Check();
          static void Distance_Measure();
          
  public:
          static char measure_flag;
          static unsigned long measure_prev_time;
          unsigned long get_distance_prev_time;
          static double distance_value;

          unsigned long ir_send_time;

          
  private:
          #define ECHO_PIN 17
          #define TRIG_PIN 11 
          #define Distance_MIN 3
          #define Distance_MAX 35 
          #define DISTANCE_JUDAGEMENT (distance_value > Distance_MIN && distance_value < Distance_MAX)
};


class Function
{
  public:
         void Follow_Mode2();

  public:
          int follow_flag;
          unsigned long follow_prev_time;
};

class IR
{
  public: 
          void Pin_init();
          void Check();
          void Send();
          void Filter();
          static void Left_Receive();
          static void Right_Receive();
          
          
  public:
          static unsigned char left_receive_flag;
          static unsigned int left_count;
          
          static unsigned char right_receive_flag;
          static unsigned int right_count;

          unsigned long left_count_time = 0;
          static int left_is_obstacle;
          static int right_is_obstacle;

  private:
          #define RECV_PIN 9
          #define IR_SEND_PIN 9
          #define LEFT_RECEIVE_PIN A0
          #define RIGHT_RECEIVE_PIN A1
          #define If_IR_TRIGGERED (IR.left_is_obstacle || IR.right_is_obstacle)

};

enum FOLLW_MOTION
{
  FOLLOW_LEFT = 1,
  FOLLOW_RIGHT,
  FOLLOW_BACK,
};



#endif
