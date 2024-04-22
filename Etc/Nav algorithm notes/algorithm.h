// 1. What is around me?
  canDoMovement[0] = !left_is_obstacle;
  canDoMovement[1] = distance_value >= distance_min;
  canDoMovement[2] = !right_is_obstacle;
  bool left      = canDoMovement[0]; // left
  bool straight  = canDoMovement[1]; // straight
  bool right     = canDoMovement[2]; // right
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

  // 2. If one option, do normal movement
  if (numOptions == 1)
  {
    // normal movement logic
    if (left)
    {
      // turn left
      // go forward for a second, right left, then move forward for another second.
      // go forward for 1 second
      unsigned long startTime = millis();
      while (millis() - startTime < 1000) { // 1 second
        motion_mode = FORWARD;
      }
      // turn left
      long int startTurnCount = turn_count;
      while (startTurnCount - turn_count < 60) { // the constant should be checked.
        motion_mode = TURNLEFT;
        turn_count --;
      }
      // go forward for another second.
      startTime = millis();
      while (millis() - startTime < 1000) { // 1 second
        motion_mode = FORWARD;
      }
    }
    else if (straight)
    {
      // go straight
      motion_mode = FORWARD;
    }
    else if (right)
    {
      // turn right
      // go forward for a second, right right, then move forward for another second.
      unsigned long startTime = millis();
      // go forward for 1 second
      while (millis() - startTime < 1000) { // 1 second
        motion_mode = FORWARD;
      }
      // turn right
      long int startTurnCount = turn_count;
      while (turn_count - startTurnCount < 60) { // the constant should be checked.
        motion_mode = TURNRIGHT;
        turn_count ++;
      }
      // go forward for another second.
      startTime = millis();
      while (millis() - startTime < 1000) { // 1 second
        motion_mode = FORWARD;
      }
    }
  }
  // else if more than one option, do intersection logic
  else if (numOptions > 1)
  {
    // intersection logic
  }
