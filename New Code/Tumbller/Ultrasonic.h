/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-10-08 09:35:07
 * @LastEditTime: 2019-10-10 14:24:57
 * @LastEditors: Please set LastEditors
 */
bool left_flag[10] = {false};
unsigned char left_count_flag = 0;
unsigned char left_index = 0;
bool right_flag[10] = {false};
unsigned char right_count_flag = 0;
unsigned char right_index = 0;

char measure_flag = 0;
double distance_value = 0;
unsigned long measure_prev_time = 0;
unsigned long get_distance_prev_time = 0;
char obstacle_avoidance_flag = 0;
unsigned long obstacle_avoidance_prev_time = 0;
int follow_flag = 0;
unsigned long follow_prev_time = 0;
long int turn_count = 0;

unsigned long ir_send_time = 0;
unsigned long left_receive_time = 0;
unsigned char left_receive_flag = 0;
unsigned int left_count = 0;
unsigned long left_count_time = 0;
unsigned long left_test_time = 0;
bool left_test_flag = false;
bool left_is_obstacle = false;

unsigned long right_receive_time = 0;
unsigned char right_receive_flag = 0;
unsigned int right_count = 0;
unsigned long right_count_time = 0;
unsigned long right_test_time = 0;
bool right_test_flag = false;
bool right_is_obstacle = false;

/* Trevor and Adam variables for nav algorithm */
// Keep track of the available movements the car can make
bool canDoMovement[]  = {false,   false,     false}; 
//                       left,    forward,   right
int numOptions = 0; // track number of direction options available to the car
const int intersectRows = 100; // 100 possible intersections can be recorded
const int intersectCols = 4; // col1: isActiveIntersectionItem?, col2: left, 
// col3: forward, col4: right
// easily create dynamically sizabl arrays in C++, so we must keep track of 
// which items in the array are active and inactive with item 0 of each array.
bool intersections[intersectRows][intersectCols];
// For each array: 
// item 0: is array active (intersection has been discovered and recorded),
// item 1: can turn left
// item 2: can move forward
// item 3: can turn right
int intersectionsPosition[100][2]; // track position of left/right
// item 0: active
// item 1: position
bool left = false;
bool right = false;
bool straight = false;
bool hitDeadEnd = false;
int intersectionsIndex = -1;

// Turning variables
bool turningCommenced = false;
bool turnSequenceInitialized = false;
unsigned long turnSequenceStartTime = 0;

void rightFilter(bool value)
{
  if (right_flag[right_index])
    right_count_flag--;
  if (value)
    right_count_flag++;
  right_flag[right_index] = value;
  right_index++;
  if (right_index >= 10)
    right_index = 0;
}
void leftFilter(bool value)
{
  if (left_flag[left_index])
    left_count_flag--;
  if (value)
    left_count_flag++;
  left_flag[left_index] = value;
  left_index++;
  if (left_index >= 10)
    left_index = 0;
}
/**
 * @brief 发送40个38KHz的脉冲
 * 
 * @param pin 产生脉冲的引脚
 */
void send38K(int pin)
{
  for (int i = 0; i < 39; i++)
  {
    digitalWrite(pin, LOW);
    delayMicroseconds(9);
    digitalWrite(pin, HIGH);
    delayMicroseconds(9);
  }
}

/**
 * @brief 接收引脚中断服务函数
 * 
 */
void leftReceive()
{
  if (left_receive_flag == 0)
  {
    left_receive_time = micros();
    left_receive_flag = 1;
    attachPinChangeInterrupt(LEFT_RECEIVE_PIN, leftReceive, RISING);
  }
  else if (left_receive_flag == 1)
  {
    left_test_time = micros() - left_receive_time;
    left_count++;
    left_receive_flag = 0;
    attachPinChangeInterrupt(LEFT_RECEIVE_PIN, leftReceive, FALLING);
  }
}
void rightReceive()
{
  if (right_receive_flag == 0)
  {
    right_receive_time = micros();
    right_receive_flag = 1;
    attachPinChangeInterrupt(RIGHT_RECEIVE_PIN, rightReceive, RISING);
  }
  else if (right_receive_flag == 1)
  {
    right_test_time = micros() - right_receive_time;
    right_count++;
    right_receive_flag = 0;
    attachPinChangeInterrupt(RIGHT_RECEIVE_PIN, rightReceive, FALLING);
  }
}

void ultrasonicInit()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(IR_SEND_PIN, OUTPUT);
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);
  attachPinChangeInterrupt(LEFT_RECEIVE_PIN, leftReceive, FALLING);
  attachPinChangeInterrupt(RIGHT_RECEIVE_PIN, rightReceive, FALLING);
}

void checkObstacle()
{
  // Do this every 15 milliseconds
  if (millis() - ir_send_time > 15)
  {
    send38K(IR_SEND_PIN);
    ir_send_time = millis();
  }

  // Do this every 50 milliseconds
  if (millis() - left_count_time > 50)
  {
    // Serial.print(left_count);
    if (left_count >= 3)
    {
      // Serial.print("\tLeft");
      left_is_obstacle = true;
      leftFilter(true);
    }
    else
    {
      left_is_obstacle = false;
      leftFilter(false);
    }

    // Serial.print(left_count_flag);
    if (left_count_flag >= 5)
    {
      // Serial.print("\tLeft");
      left_is_obstacle = true;
    }
    else
    {
      left_is_obstacle = false;
    }

    // Serial.println();
    left_count = 0; // this variable resets every 50 ms
    left_count_time = millis();
  }

  if (millis() - right_count_time > 50)
  {
    // Serial.print("\t\t\t");
    // Serial.print(right_count);
    if (right_count >= 3)
    {
      // Serial.print("\tRight");
      rightFilter(true);
    }
    else
    {
      rightFilter(false);
    }

    // Serial.print("\t\t\t");
    // Serial.print(right_count_flag);
    if (right_count_flag >= 5)
    {
      // Serial.print("\tRight");
      right_is_obstacle = true;
    }
    else
    {
      right_is_obstacle = false;
    }
    // Serial.println();

    right_count = 0;
    right_count_time = millis();
  }
}

void obstacleAvoidanceMode()
{
  Serial.println("Enter Obstacle Avoidance Mode");
  /* Keep these four variables */
  unsigned int super_min = 3;
  unsigned int distance_min = 10;
  unsigned int distance_max = 45;
  unsigned int super_max = 400;

  /* New navigation algorithm (Mar 22) */

  // Are there obstacles around me?
  // 1. What is around me?
  if (!turningCommenced) // Only update obstacles while we are not in a 
  // special sequence of movements, to prevent a sequence from being
  // prematuraly terminated
  {
    canDoMovement[0] = !left_is_obstacle;
    canDoMovement[1] = distance_value >= distance_min;
    canDoMovement[2] = !right_is_obstacle;
    left      = canDoMovement[0]; // left
    straight  = canDoMovement[1]; // straight
    right     = canDoMovement[2]; // right

    // Quantify movement options
    // one option
    if ((!left && !straight && right) ||
        (!left && straight && !right) ||
        (left && !straight && !right)) {
      numOptions = 1;
    }
    // 2 options
    else if ((!left && straight && right) ||
            (left && straight && !right) ||
            (left && !straight && right)) {
      numOptions = 2;
    }
    // 3 options
    else if (left && straight && right) {
      numOptions = 3;
    }
    // no options
    else {numOptions = 0;}
  }


  // 2. If one option, do normal movement
  if (numOptions == 1)
  {
    // normal movement logic
    if (left)
    {
      if (turningCommenced == false) {turningCommenced = true;}
      // Turn sequence
      if (!turnSequenceInitialized) {
        turnSequenceStartTime = millis();
        turnSequenceInitialized = true;
      }
      // for the first 1 seconds, go straight
      if (millis() - turnSequenceStartTime < 1000)
      {
        motion_mode = FORWARD;
      }
      // pause for one second 
      if (millis() - turnSequenceStartTime < 2000) {
        motion_mode = STANDBY;
      }
      // then, for the next 2 seconds, turn left
      else if (millis() - turnSequenceStartTime < 4000) {
        motion_mode = TURNLEFT;
        turn_count --;
      }
      // pause for another second
      else if (millis() - turnSequenceStartTime < 5000) {
        motion_mode = STANDBY;
      }
      // continue straight for another 1 second.
      else if (millis() - turnSequenceStartTime < 6000) {
        motion_mode = FORWARD;
      }
      else {
        turnSequenceInitialized = false;
        turningCommenced = false;
      }
    }

    else if (straight)
    {
      // go straight
      motion_mode = FORWARD;
    }
    else if (right)
    {
      if (turningCommenced == false) {turningCommenced = true;} // tell the 
      // rest of our program that we are now in a turn sequence which should
      // not be interrupted.

      // Turn sequence
      if (!turnSequenceInitialized) {
        turnSequenceStartTime = millis();
        turnSequenceInitialized = true;
      }
      // for the first 1 seconds, go straight
      if (millis() - turnSequenceStartTime < 1000)
      {
        motion_mode = FORWARD;
      }
      // pause for one second 
      if (millis() - turnSequenceStartTime < 2000) {
        motion_mode = STANDBY;
      }
      // then, for the next 2 seconds, turn left
      else if (millis() - turnSequenceStartTime < 4000) {
        motion_mode = TURNRIGHT;
        turn_count ++;
      }
      // pause for another second
      else if (millis() - turnSequenceStartTime < 5000) {
        motion_mode = STANDBY;
      }
      // continue straight for another 1 second.
      else if (millis() - turnSequenceStartTime < 6000) {
        motion_mode = FORWARD;
      }
      else {
        turnSequenceInitialized = false;
        turningCommenced = false;
      }
    }
  }
  // else if more than one option, do intersection logic
  else if (numOptions > 1)
  {
    motion_mode = STANDBY; // temporary (may not need it later)
    // intersection logic
    if (!hitDeadEnd) { // assume we have encountered a new intersection.
      intersectionsIndex ++; // move to the next item in the intersections array
      // Create a new array in our intersections array
      intersections[intersectionsIndex][0] = true;      // set to active
      intersections[intersectionsIndex][1] = left;      // record obstacle
      intersections[intersectionsIndex][2] = straight;  // record obstacle
      intersections[intersectionsIndex][3] = right;     // record obstacle
      
      // Make a decision on where to go.
    }
  }
  // This means we have encountered a dead-end, and need to turn around.
  else if (numOptions < 1) {
    // Turn around
    if (turningCommenced == false) {turningCommenced = true;} // tell the 
      // rest of our program that we are now in a turn sequence which should
      // not be interrupted.

      // Turn sequence
      if (!turnSequenceInitialized) {
        turnSequenceStartTime = millis();
        turnSequenceInitialized = true;
      }
      // for the first 3 seconds, turn left
      if (millis() - turnSequenceStartTime < 3000)
      {
        motion_mode = TURNLEFT;
        turn_count --;
      }
      // pause for one second 
      if (millis() - turnSequenceStartTime < 4000) {
        motion_mode = STANDBY;
      }
      // once the turn sequence is finished, tell the rest of our program to 
      // resume as normal.
      else {
        turnSequenceInitialized = false;
        turningCommenced = false;
      }
  }

  /* Old Algorithm below */
  // If obstacles exist on left and right
  // Serial.print("turn_count: ");
  // Serial.println(turn_count);
  
  // if (left_is_obstacle && right_is_obstacle)
  // {
  //   // 
  //   if (distance_value >= distance_min && distance_max <= distance_max)
  //   {
  //     if (turn_count >= 0)
  //     {
  //       motion_mode = TURNRIGHT;
  //       rgb.flashYellowColorRight();
  //       turn_count++;
  //     }
  //     else
  //     {
  //       motion_mode = TURNLEFT;
  //       rgb.flashYellowColorLeft();
  //       turn_count--;
  //     }
  //   }
  //   else
  //   {
  //     motion_mode = BACKWARD;
  //     rgb.flashYellowColorback();
  //   }
  // }
  // else if (left_is_obstacle && !right_is_obstacle)
  // {
  //   motion_mode = TURNRIGHT;
  //   rgb.flashYellowColorRight();
  //   turn_count++;
  // }
  // else if (!left_is_obstacle && right_is_obstacle)
  // {
  //   motion_mode = TURNLEFT;
  //   rgb.flashYellowColorLeft();
  //   turn_count--;
  // }
  // else
  // {
    // switch (obstacle_avoidance_flag)
    // {
    // case 0:
    //   if (distance_value < distance_max || distance_value > super_max)
    //   {
    //     obstacle_avoidance_flag = 1;
    //     motion_mode = STANDBY;
    //     rgb.brightYellowColor();
    //   }
    //   else
    //   {
    //     motion_mode = FORWARD;
    //     rgb.flashYellowColorFront();
    //   }
    //   break;
    // case 1:
    //   if (distance_value >= distance_max && distance_value <= super_max)
    //   {
    //     obstacle_avoidance_flag = 0;
    //     motion_mode = FORWARD;
    //     rgb.flashYellowColorFront();
    //   }
    //   else if (distance_value >= super_min && distance_value <= distance_min)
    //   {
    //     motion_mode = BACKWARD;
    //     rgb.flashYellowColorback();
    //   }
    //   else
    //   {
    //     if (turn_count >= 0)
    //     {
    //       motion_mode = TURNRIGHT;
    //       rgb.flashYellowColorRight();
    //       turn_count++;
    //     }
    //     else
    //     {
    //       motion_mode = TURNLEFT;
    //       rgb.flashYellowColorLeft();
    //       turn_count--;
    //     }
    //   }
    //   break;
    // default:
    //   break;
    // }
  // }
}

void followMode()
{
  unsigned int distance_min = 3;
  unsigned int distance_max = 35;

  switch (follow_flag)
  {
  case 0:
    if (millis() - follow_prev_time < 500)
    {
      motion_mode = STANDBY;
      rgb.brightGreenColor();
    }
    else if (millis() - follow_prev_time >= 500)
    {
      if (left_is_obstacle || right_is_obstacle)
      {
        follow_flag = 1;
      }
      else
      {
        if (distance_value > distance_min && distance_value < distance_max)
        {
          follow_flag = 2;
        }
        else
        {
          follow_prev_time = millis();
        }
      }
    }
    break;
  case 1:
    if (left_is_obstacle && !right_is_obstacle)
    {
      rgb.flashGreenColorLeft();
      motion_mode = TURNLEFT;
    }
    else if (!left_is_obstacle && right_is_obstacle)
    {
      rgb.flashGreenColorRight();
      motion_mode = TURNRIGHT;
    }
    else if (left_is_obstacle && right_is_obstacle)
    {
      rgb.flashGreenColorFront();
      motion_mode = FORWARD;
    }
    else
    {
      follow_flag = 0;
      motion_mode = STANDBY;
      rgb.brightGreenColor();
    }
    break;
  case 2:
    if (distance_value > distance_min && distance_value < distance_max)
    {
      rgb.flashGreenColorFront();
      motion_mode = FORWARD;
    }
    else
    {
      follow_flag = 0;
      motion_mode = STANDBY;
      rgb.brightGreenColor();
    }
    break;
  default:
    break;
  }
}

void followMode2()
{
  unsigned int distance_min = 5;
  unsigned int distance_max = 35;

  switch (follow_flag)
  {
  case 0:
    if (millis() - follow_prev_time < 500)
    {
      motion_mode = STANDBY;
      rgb.brightGreenColor();
    }
    else if (millis() - follow_prev_time >= 500)
    {
      if (left_is_obstacle || right_is_obstacle)
      {
        follow_flag = 1;
      }
      else
      {
        if (distance_value > distance_min && distance_value < distance_max)
        {
          follow_flag = 2;
        }
        else
        {
          follow_prev_time = millis();
        }
      }
    }
    break;
  case 1:
    if (left_is_obstacle && !right_is_obstacle)
    {
      rgb.flashGreenColorLeft();
      motion_mode = TURNLEFT;
    }
    else if (!left_is_obstacle && right_is_obstacle)
    {
      rgb.flashGreenColorRight();
      motion_mode = TURNRIGHT;
    }
    else if (left_is_obstacle && right_is_obstacle)
    {
      rgb.flashGreenColorFront();
      motion_mode = BACKWARD;
    }
    else
    {
      follow_flag = 0;
      motion_mode = STANDBY;
      rgb.brightGreenColor();
    }
    break;
  case 2:
    if (distance_value > distance_min && distance_value < distance_max)
    {
      rgb.flashGreenColorFront();
      motion_mode = BACKWARD;
    }
    else
    {
      follow_flag = 0;
      motion_mode = STANDBY;
      rgb.brightGreenColor();
    }
    break;
  default:
    break;
  }
}

void measureDistance()
{
  if (measure_flag == 0)
  {
    measure_prev_time = micros();
    attachPinChangeInterrupt(ECHO_PIN, measureDistance, FALLING);
    measure_flag = 1;
  }
  else if (measure_flag == 1)
  {
    distance_value = (micros() - measure_prev_time) * 0.017; //340.29 m/s / 2 -> (340.29*100 cm) /(1000*1000 us) / 2 = 0.0170145
    measure_flag = 2;
  }
}

void getDistance()
{
  if (millis() - get_distance_prev_time > 50)
  {
    get_distance_prev_time = millis();
    measure_flag = 0;
    attachPinChangeInterrupt(ECHO_PIN, measureDistance, RISING);
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  }
}
