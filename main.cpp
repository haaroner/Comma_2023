#include "Robot.h"

volatile int16_t robot_x, robot_y, gyro = 0, forward_angle = 0, backward_angle = 0,
a = 0, move_angle, ball_angle, start_attack_point[2];

uint8_t gaming_state = 0, role, attacker_state = 0, defender_state = 0;

int main()
{
  Robot::init_robot(1);
  //Robot::set_blinking(0, 1000);
  Robot::motors_on_off(OFF);
  Timer test;
  PID test2(-0.2, -0.005, 0, 5);
  test.save_time();
  Robot::moveRobot(0, 0);
  Robot::rotateRobot(0, 15);
  
  role = 1;
  
  while(true)
  {
    gyro = Robot::gyro;
    
    forward_angle = Robot::forward_angle;
    
    if(Robot::check_button(ENTER_BUTTON))
    {
      gaming_state = my_abs(gaming_state - 1);
      Robot::control_led(0, OFF);
    }
    
    if(gaming_state == 0)
    {
      Robot::set_blinking(3, 0);
      Robot::motors_on_off(OFF);
      if(Robot::check_button(UP_BUTTON)) Robot::change_side();
      if(Robot::check_button(DOWN_BUTTON)) Robot::callibrate_gyro();
      
      Robot::moveRobot(move_angle, 0);
      Robot::control_led(3, my_abs(gyro) < 5);
      Robot::control_led(1, my_abs(forward_angle - gyro) < 10);
    }
    else if(gaming_state == 1)
    {
      Robot::motors_on_off(ON);
      Robot::set_blinking(3, 200);
      if(role == 1)
      {
        if(/*ball isnt in the dribler(attacker_state == 0*/false)
        {
          Robot::moveRobotAbs(Robot::ball_abs_angle, 25);
          ball_angle = Robot::ball_abs_angle;
          Robot::setAngle(Robot::ball_abs_angle, 20);
        }
        if(/*ball is in the dribler*/true)
        {
          if(Robot::check_button(DOWN_BUTTON))
          {
            start_attack_point[0] = robot_x;
            start_attack_point[1] = robot_y;
          }
          Robot::moveToPoint(0, 100, 20);
        }
      }
      
    }
    
    Robot::update();
  }
  
}
  
