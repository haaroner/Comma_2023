#include "Robot.h"


int robot_x, robot_y, a = 0, b = 0;

int main()
{
  Robot::init_robot(1);
  //Robot::set_blinking(0, 1000);
  Robot::motors_on_off(ON);
  Timer test;
  test.save_time();
  Robot::moveRobot(0, 70);
  //Robot::rotateRobot(10, 10);
  while(true)
  {
    if(test.is_duration_reached(1000))
    {
      a = my_abs(a - 1);
      Robot::control_led(1, a);
      test.save_time();
    }
    if(Robot::check_button(3))
      Robot::motors_on_off(OFF);
    if(Robot::check_button(1))
      Robot::motors_on_off(ON);
    Robot::update();
  }
  
}
  
