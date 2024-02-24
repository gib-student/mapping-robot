/* Copied from Lesson 4 Obstacle Mode */

#ifndef _BALANCED_h
#define _BALANCED_h

#include "MsTimer2.h"
#include "KalmanFilter.h"

enum Direction
{
  FORWARD,
  BACK,
  LEFT,
  RIGHT,
  STOP,
};

class Balanced
{
  public:
          Balanced();
          void Get_EncoderSpeed();
          void PD_VerticalRing();
          void PI_SpeedRing();
          void PI_SteeringRing();
          void Total_Control();

          void Motion_Control(Direction direction);
          void Stop();
          void Forward(int speed);
          void Back(int speed);
          void Left(int speed);
          void Right(int speed);
  
/*Speed value*/
          double pwm_left;
          double pwm_right;
          int encoder_left_pulse_num_speed;
          int encoder_right_pulse_num_speed;
/*Cnt*/
          int interrupt_cnt;

/*PID parameter*/
         /*PD_VerticalRing*/
          double kp_balance, kd_balance;
         /*PI_SpeedRing*/
          double kp_speed, ki_speed;
         /*PI_SteeringRing*/
          double kp_turn, kd_turn;

          double speed_filter;
          double speed_filter_old;
          double car_speed_integeral;
          double balance_control_output;
          double speed_control_output;
          double rotation_control_output;
          int setting_turn_speed;
          int setting_car_speed;
          
   private:
   #define ANGLE_MIN -100
   #define ANGLE_MAX 40
   #define EXCESSIVE_ANGLE_TILT (kalmanfilter.angle < ANGLE_MIN || ANGLE_MAX < kalmanfilter.angle)
   #define PICKED_UP (kalmanfilter.angle6 < -10 || 22 < kalmanfilter.angle6)
};

class Timer2
{
  public:
          void init(int time);
          static void interrupt();
  private:       
          #define TIMER 5
};


class Mpu6050
{
  public:
          void init();
          void DataProcessing();
          Mpu6050();

  public:
         int ax, ay, az, gx, gy, gz;
         float dt, Q_angle, Q_gyro, R_angle, C_0, K1;
};












#endif
