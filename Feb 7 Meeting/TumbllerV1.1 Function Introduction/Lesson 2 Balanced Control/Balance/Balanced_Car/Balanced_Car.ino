#include "Motor.h"
#include "Balanced.h"


Timer2 Timer2;
extern Mpu6050 Mpu6050;
extern Motor Motor;
extern Balanced Balanced;



void setup() 
{
  Motor.Pin_init();
  Motor.Encoder_init();
  Timer2.init(TIMER);
  Mpu6050.init();
  Serial.begin(9600);
  delay(100);
}

void loop() 
{
  int direction_buf[] = {FORWARD,BACK,LEFT,RIGHT,STOP};
  static unsigned long print_time;
  
  for(int i=0;i<5;)
  {
      if(millis() - print_time > 5000)
   { 
      print_time = millis();
      Balanced.Motion_Control(direction_buf[i++]);
   }
  }
}
