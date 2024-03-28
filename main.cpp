#include "Robot.h"
#include "Settings.h"

volatile uint32_t time = 0, ball_grab_timer = 0, super_timer = 0;

volatile uint8_t gaming_state, role, attacker_state = 1, attacker_old_state = 1;

int16_t gyro, forward_angle, backward_angle, ball_angle, 
  ball_loc_angle, ball_abs_angle;

uint16_t forward_distance, backward_distance, ball_distance;
int32_t robot_x = 0, robot_y = 80, ball_loc_x = 0, ball_loc_y = 20,
  ball_abs_x = 0, ball_abs_y = 100, point_distance = 0;

point robot_position, ball_abs_position, attacker_start_state_point;

volatile uint16_t dribler_speed = 200;



int main()
{
  Robot::init_robot();
  role = 1;
  while(true)
  {
    time = time_service::getCurTime();
    
    gyro = Robot::gyro;
    
    ball_abs_angle = Robot::ball_abs_angle;
    ball_loc_angle = Robot::ball_loc_angle;
    ball_distance = Robot::ball_distance;
    
    forward_angle = Robot::forward_angle;
    forward_distance = Robot::forward_distance;
    
    backward_angle = Robot::backward_angle;
    backward_distance = Robot::backward_distance;
    
    robot_x = Robot::robot_x;
    robot_y = Robot::robot_y;
    robot_position.x = robot_x;
    robot_position.y = robot_y;
    
    ball_loc_x = Robot::ball_loc_x;
    ball_loc_y = Robot::ball_loc_y;
    
    ball_abs_x = Robot::ball_abs_x;
    ball_abs_y = Robot::ball_abs_y;
    ball_abs_position.x = ball_abs_x;
    ball_abs_position.y = ball_abs_y;
    
    gaming_state = Robot::_game_state;
    
    if(gaming_state == 0)
    {
      //disable all motors as a safety precoutions
      Robot::motors_on_off(OFF);
      Robot::set_dribler_speed(STOP_DRIBLER_SPEED);
      
      Robot::set_blinking(2, 0); //disable blinking of middle LED
      
      //callibration indication
      Robot::control_led(3, my_abs(gyro) < 5);
      Robot::control_led(2, my_abs(forward_angle) < 10);
      
      //update some timers
      ball_grab_timer = time;
    }
    else if(gaming_state == 1)
    {
      Robot::motors_on_off(ON);
      Robot::set_blinking(2, 200);
      
      if(role == 1) // atacker
      {
        //attacker state change conditions
        if(true) // go to ball
        {
//          if(time - ball_grab_timer > 3000)
//          {
//            Robot::wait(1000);
//            attacker_state = 2;
//          }
          attacker_state = 1;
        }
        else if(attacker_state == 2) // go with ball to gates
        {
          if(attacker_state != 2)
            attacker_start_state_point = robot_position;
          
        }
        else if(attacker_state == 3) //smth idk
        {
          attacker_state = 3;
        }
        
        
        //attacker state bodies
        if(attacker_state == 1) //move to ball and grab it
        {
//          Robot::moveRobotAbs(ball_abs_angle, 7);
//          Robot::setAngle(ball_abs_angle, 13);
//          Robot::set_dribler_speed(270);
//          if(ball_distance > 15 || my_abs(ball_loc_angle) > 20) ball_grab_timer = time;
          Robot::setAngle(0, 15);
        }       
        if(attacker_state == 2) //move to gates with ball
        {
          Robot::set_dribler_speed(270);
          point_distance = Robot::moveToPoint(0, 45, 12);
          if(point_distance < 10)
          {
            Robot::wait(1000);
            super_timer = time_service::getCurTime();
            while(time_service::getCurTime() - super_timer < 3000)
            {
              Robot::moveRobotAbs(0, 0);
              Robot::setAngle(0, 10);
              Robot::update();
            }
            //Robot::moveRobotAbs(0, 0);
            //Robot::wait(1000, true, 180, 15);
            attacker_state = 1;
            ball_grab_timer = time;
          }
//          if(attacker_old_state == 1)
//          {
//            Robot::enable_trajectory(true);
//            Robot::add_stop_to_route(40 * my_sgn(robot_x), 90, 90 * my_sgn(robot_x));
//            Robot::add_stop_to_route(25 * my_sgn(robot_x), 30, 45 * my_sgn(robot_x));
//          }
          attacker_old_state = 2;
          //if(Robot::trajectory_finished) Robot::wait(2000);
        }
        
        if(attacker_state == 3)
        {
        
        }
      }
    } 
    Robot::update();
  }
}
