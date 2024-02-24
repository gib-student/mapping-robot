/*  This would replace the signature .h file like in each of the lessons. 
    Much of this code is copied from Obstacle.h */ 

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
#define Obstacle_MIN 40
#define DISTANCE_JUDAGEMENT (distance_value > Distance_MIN && distance_value < Distance_MAX)
#define OBSTACLE_JUDAGEMENT (Ultrasonic.distance_value < Obstacle_MIN)
#define OBSTACLE_JUDAGEMENT2 (Ultrasonic.distance_value < 10)
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

/*  The code unique to this feature, like how there is Obstacle mode, 
    Follow Mode, etc.*/
class Function
{
    // declare functions here
    public:
        void Mapping_Mode(); // follows a similar style to Obstacle.h

    // declare member variables here
    public:
        int example; // change this
}


/*  Copied from Obstacle.h, but we may not need it. Thus I commented it oute.
    Similar enums can be found in other of the signature .h files, like 
    follow2.h*/
// enum FOLLW_MOTION
// {
//   Meet_OBSTACLE,
//   FOLLOW_RIGHT,
//   FOLLOW_LEFT,
//   FOLLOW_BACK,
// };
