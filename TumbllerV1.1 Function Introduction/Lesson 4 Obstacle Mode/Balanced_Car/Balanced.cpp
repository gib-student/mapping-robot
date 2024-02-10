#include "Balanced.h"
#include "Wire.h"
#include "Motor.h"
#include "Obstacle.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "KalmanFilter.h"
MPU6050 MPU6050;
Mpu6050 Mpu6050;
Balanced Balanced;
KalmanFilter kalmanfilter;
Motor Motor;
extern IR IR;
extern Ultrasonic Ultrasonic;

void Timer2::init(int time)
{
  MsTimer2::set(time,interrupt);
  MsTimer2::start();
}

static void Timer2::interrupt()
{ 
  sei();//enable the global interrupt
  Balanced.Get_EncoderSpeed();
  Mpu6050.DataProcessing();
  Balanced.PD_VerticalRing();
  //Serial.print("Balanced.setting_turn_speed:");Serial.println(Balanced.setting_turn_speed);
  Balanced.interrupt_cnt++;//Serial.print("Balanced.setting_car_speed:");Serial.println(Balanced.setting_car_speed);
  if(Balanced.interrupt_cnt > 8)
  {
    Balanced.interrupt_cnt=0;
    Balanced.PI_SpeedRing();
    Balanced.PI_SteeringRing();
   }
  Balanced.Total_Control();
  
  IR.Send();
  Ultrasonic.Get_Distance();
}

Balanced::Balanced()
{
  kp_balance = 55, kd_balance = 0.75;
  kp_speed = 10, ki_speed = 0.26;
  kp_turn = 2.5, kd_turn = 0.5;
}

void Balanced::Total_Control()
{
  pwm_left = balance_control_output - speed_control_output - rotation_control_output;//Superposition of Vertical Velocity Steering Ring
  pwm_right = balance_control_output - speed_control_output + rotation_control_output;//Superposition of Vertical Velocity Steering Ring
  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);

  while(EXCESSIVE_ANGLE_TILT || PICKED_UP)
  { 
    Mpu6050.DataProcessing();
    Motor.Stop();
  }
  
  (pwm_left < 0) ?  (Motor.Control(AIN1,1,PWMA_LEFT,-pwm_left)):
                    (Motor.Control(AIN1,0,PWMA_LEFT,pwm_left));
  
  (pwm_right < 0) ? (Motor.Control(BIN1,1,PWMB_RIGHT,-pwm_right)): 
                    (Motor.Control(BIN1,0,PWMB_RIGHT,pwm_right));

}

void Balanced::Get_EncoderSpeed()
{
  encoder_left_pulse_num_speed += pwm_left < 0 ? -Motor::encoder_count_left_a : 
                                                  Motor::encoder_count_left_a;
  encoder_right_pulse_num_speed += pwm_right < 0 ? -Motor::encoder_count_right_a : 
                                                    Motor::encoder_count_right_a;
  Motor::encoder_count_left_a=0;
  Motor::encoder_count_right_a=0;
}

void Balanced::Motion_Control(Direction direction)
{
  switch(direction)
  {
    case STOP:
                  Stop();break;
    case FORWARD:
                  Forward(40);break;
    case BACK:
                  Back(40);break;
    case LEFT:
                  Left(50);break;
    case RIGHT:
                  Right(50);break;
    default:      
                  Stop();break;
  }
}

void Balanced::Stop()
{
  setting_car_speed = 0;
  setting_turn_speed = 0;
}

void Balanced::Forward(int speed)
{
  setting_car_speed = speed;
  setting_turn_speed = 0;
}

void Balanced::Back(int speed)
{
  setting_car_speed = -speed;
  setting_turn_speed = 0;
}

void Balanced::Left(int speed)
{
  setting_car_speed = -speed;
  setting_turn_speed = speed;
}

void Balanced::Right(int speed)
{
  setting_car_speed = -speed;
  setting_turn_speed = -speed;
}

void Balanced::PI_SpeedRing()
{
   double car_speed=(encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
   encoder_left_pulse_num_speed = 0;
   encoder_right_pulse_num_speed = 0;
   speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
   speed_filter_old = speed_filter;
   car_speed_integeral += speed_filter;
   car_speed_integeral += -setting_car_speed; 
   car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);

   speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;
}

void Balanced::PD_VerticalRing()
{
  balance_control_output= kp_balance * (kalmanfilter.angle - 0) + kd_balance * (kalmanfilter.Gyro_x - 0);
}

void Balanced::PI_SteeringRing()
{  
   rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;////control with Z-axis gyroscope
}


void Mpu6050::init()
{
   Wire.begin();         
   MPU6050.initialize();    
 }

Mpu6050::Mpu6050()
{
    dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
}

void Mpu6050::DataProcessing()
{  
  MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);// Data acquisition of MPU6050 gyroscope and accelerometer
  kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);// Obtaining Angle by Kalman Filter
  //Serial.println(gx);
}
