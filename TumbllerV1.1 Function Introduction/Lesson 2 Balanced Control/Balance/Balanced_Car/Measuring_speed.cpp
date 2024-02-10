#include "Motor.h"
#include "PinChangeInt.h"

static void EncoderCountRightA();
static void EncoderCountLeftA();

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
