// Copied from balanced_car.ino in follow2 lesson
#include "Motor.h"
#include "Balanced.h"
#include "Mapping.h"


Timer2 Timer2;
extern Mpu6050 Mpu6050;
extern Motor Motor;
extern Balanced Balanced;
extern Ultrasonic Ultrasonic;
extern IR IR;
Function Function;


void setup() 
{
  Motor.Pin_init();
  IR.Pin_init();
  Ultrasonic.Pin_init();
  Motor.Encoder_init();
  Timer2.init(TIMER);
  Mpu6050.init();
  Serial.begin(9600);
  delay(100);
}

void loop() 
{ 
   Function.Mapping_Mode();
}
