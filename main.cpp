#include "Robot.h"

volatile int16_t robot_x = 0, robot_y = 0, gyro = 0, forward_angle = 0, backward_angle = 0,
a = 0, b = 0, c, move_angle, ball_angle, start_attack_point[2], point[2];

uint8_t gaming_state = 0, role, attacker_state = 0, defender_state = 0;

bool trajectory_started = 0;

int main()
{
  Robot::init_robot(1);
  //Robot::set_blinking(0, 1000);
  Robot::motors_on_off(OFF);
  //Timer test;
  PID test2(-0.2, -0.005, 0, 5);
  //test.save_time();
  Robot::moveRobot(0, 0);
  Robot::rotateRobot(0, 15);
  
  role = 1;
  Queue points_queue;
  
  while(true)
  {
    gyro = Robot::gyro;
    robot_x = Robot::robot_x;
    robot_y = Robot::robot_y;
    
    forward_angle = Robot::forward_angle;
    backward_angle = Robot::backward_angle;
    
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
          //Robot::moveToPoint(20, 50, -1);
          Robot::setAngle(0, 20);
          if(Robot::check_button(DOWN_BUTTON))//start state section
          {
            start_attack_point[0] = Robot::robot_x;
            start_attack_point[1] = Robot::robot_y;
            points_queue.clear();
            points_queue.push(-50);//x
            points_queue.push(50);//y
            points_queue.push(-50);//x
            points_queue.push(100);//y
             points_queue.push(50);//x
            points_queue.push(100);//y
            points_queue.push(50);//x
            points_queue.push(50);//y
            trajectory_started = false;
            a = 0;
            b = 0;
          }
          if(points_queue.get_length() > 1)
          {
//            //c = get_distance_to_point(a, b, Robot::_robot_x, Robot::_robot_y);
            c = Robot::getDistanceToPoint(a, b);
            if(c < 15 || !trajectory_started)
            {
             a = points_queue.pop();
             b = points_queue.pop();
              trajectory_started = true;
            }
          }
          Robot::moveToPoint(a, b, -1);
        }
      }
      //Robot::setAngle(lead_to_degree_borders(forward_angle + 180), 20);
    }
    
    Robot::update();
  }
  
}
  
