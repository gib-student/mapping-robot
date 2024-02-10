#include "Motor.h"
#include "Follow1.h"
#include "PinChangeInt.h"
#include "MsTimer2.h"

extern Balanced Balanced;
static void EncoderCountRightA();
static void EncoderCountLeftA();

unsigned char IR::left_receive_flag;
unsigned char IR::right_receive_flag;

void Motor::Encoder_init()
{
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), EncoderCountLeftA, CHANGE);
  attachPinChangeInterrupt(ENCODER_RIGHT_A_PIN, EncoderCountRightA, CHANGE);
}

unsigned long Motor::encoder_count_right_a;
//Getting Right Wheel Speed.
static void EncoderCountRightA()
{
  Motor::encoder_count_right_a++;
}


unsigned long Motor::encoder_count_left_a;
//Getting Left Wheel Speed.
static void EncoderCountLeftA()
{
  Motor::encoder_count_left_a++;
}

void IR::Pin_init()
{
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(IR_SEND_PIN, OUTPUT);
  attachPinChangeInterrupt(LEFT_RECEIVE_PIN, Left_Receive, FALLING);
  attachPinChangeInterrupt(RIGHT_RECEIVE_PIN, Right_Receive, FALLING);   
}

int IR::left_is_obstacle;
int IR::right_is_obstacle;
unsigned int IR::left_count;
unsigned int IR::right_count;
void IR::Left_Receive()
{ 
   left_is_obstacle=1;
}
void IR::Right_Receive()
{ 
   right_is_obstacle=2;
}

void Ultrasonic::Get_Distance()
{
  if (millis() - get_distance_prev_time > 50)
  { 
    delayMicroseconds(1);
    get_distance_prev_time = millis();
    measure_flag = 0;
    attachPinChangeInterrupt(ECHO_PIN, Distance_Measure, RISING);
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  }
}

void Ultrasonic::Distance_Measure()
{
  if (measure_flag == 0)
  {
    measure_prev_time = micros();
    attachPinChangeInterrupt(ECHO_PIN, Distance_Measure, FALLING);
    measure_flag = 1;
  }
  else if (measure_flag == 1)
  {
    distance_value = (micros() - measure_prev_time) * 0.017; //340.29 m/s / 2 -> (340.29*100 cm) /(1000*1000 us) / 2 = 0.0170145
    measure_flag = 2;//Serial.println(distance_value);
  }
}
