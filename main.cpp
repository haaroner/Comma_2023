#include "Robot.h"
#include "Settings.h"

volatile uint32_t time = 0, ball_grab_timer = 0, super_timer = 0, point_reached_timer = 0, delay = 0;

volatile uint8_t gaming_state, role, attacker_state = 1, attacker_old_state = 1;

int16_t gyro, forward_angle, backward_angle, ball_angle, 
  ball_loc_angle, ball_abs_angle;

uint16_t forward_distance, backward_distance, ball_distance;
int32_t robot_x = 0, robot_y = 80, ball_loc_x = 0, ball_loc_y = 20,
  ball_abs_x = 0, ball_abs_y = 100, point_distance = 0;

point robot_position, ball_abs_position, attacker_start_state_point;

volatile uint16_t dribler_speed = STOP_DRIBLER_SPEED;



int main()
{
  Robot::init_robot();
  role = 1;
  
  while(true)
  {
    time = time_service::getCurTime();
    
    delay = Robot::loop_delay;
    
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
      //disable all motors as a safety precoution
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
      
      //attacker role
      if(role == 1) // atacker
      {
        /*attacker state change conditions*/
        
        //go to ball condition
        if(attacker_state == 1)
        {
          //if ball in dribler more than 4 scnds go to state 2
          if(time - ball_grab_timer > 4000)
          {
            Robot::set_dribler_speed(270);
            Robot::wait(750);
            attacker_state = 2;
          }
        }
        
        // go with ball to gates
        if(attacker_state == 2) 
        {
          //save start state point
          if(attacker_old_state != 2)
            attacker_start_state_point = robot_position;
          
          //if ball moved out of dribler accidentally go to state 1
          if(ball_distance > 20 || my_abs(ball_loc_angle) > 30) attacker_state = 1;
        }
        if(false) //smth idk
        {
          attacker_state = 3;
        }
        
        
        /*attacker state bodies*/
        
        if(attacker_state == 1) //move to ball and grab it
        {
          
          if(ball_distance > 25) //move quickly
          {
            Robot::moveRobotAbs(ball_abs_angle, 20);
            Robot::set_dribler_speed(STOP_DRIBLER_SPEED);
          }
          else //move slowly
          {
            Robot::moveRobotAbs(ball_abs_angle, 7);
            Robot::set_dribler_speed(270);
          }
          
          Robot::setAngle(ball_abs_angle, 8, -0.12); //turn to ball
          
          if((ball_distance > 15 || my_abs(ball_loc_angle) > 30) && (Robot::is_ball_seen_T(75))) ball_grab_timer = time;
          attacker_old_state = 1;
          
        }       
        
        /*move to gates with ball*/
        
        if(attacker_state == 2) 
        {
          //move to gates
          Robot::set_dribler_speed(270);
          point_reached_timer = Robot::moveToPoint(30 * my_sgn(attacker_start_state_point.x), 175, 7);
          Robot::setAngle(lead_to_degree_borders(forward_angle + 180), 4, -0.08);
          
          //if point reached
          //TODO: put this if statement inside Robot::moveToPoint function
          if(time_service::getCurTime() - point_reached_timer > 150)
          {
            Robot::wait(500);
            Robot::setAngle(0, 0, -0.1);
            Robot::wait(2000, true, 180 - 65 * my_sgn(robot_x), 6);
            Robot::setAngle(0, 0, -0.25);
            Robot::wait(750);
            Robot::wait(1500, true, forward_angle + 15 * my_sgn(attacker_start_state_point.x), 25);
            attacker_state = 1;
            ball_grab_timer = time_service::getCurTime();
          }
//          point_distance = Robot::moveToPoint(30 * my_sgn(attacker_start_state_point.x), 55, 12);
//          Robot::setAngle(lead_to_degree_borders(backward_angle + 180), 5, -0.07);
//          if(point_distance < 8)
//          {
//            Robot::setAngle(0, 0, -0.25);
//            Robot::wait(500);
//            Robot::wait(2000, true, 60, 12);
//            //Robot::set_dribler_speed(260);
//            Robot::wait(1000);
//            super_timer = time_service::getCurTime();
//            //Robot::moveRobotAbs(0, 0);
//            Robot::wait(1500, true, backward_angle - 20 * my_sgn(attacker_start_state_point.x), 23);
//            attacker_state = 1;
//            ball_grab_timer = time_service::getCurTime();
//          }
          
          
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
          Robot::setAngle(ball_abs_angle, 12);
          Robot::moveRobotAbs(0, 0);
        }
        
        //avoiding out of bounds 

        if(my_abs(robot_x) < 30 && robot_y > 187)
          Robot::moveRobotAbs(180, 12);
        
        if(my_abs(robot_x) > 30 && forward_distance < 43)
          Robot::moveRobotAbs(180, 12);
        
        if(robot_y > 190)
          Robot::moveRobotAbs(180, 12);
        
        if(my_abs(robot_x) < 30 && robot_y < 40)
          Robot::moveRobotAbs(0, 12);
        
        if(my_abs(robot_x) > 30 && backward_distance < 45)
          Robot::moveRobotAbs(lead_to_degree_borders(backward_angle + 180), 12);  
        
        if(robot_x > 55)
          Robot::moveRobotAbs(-90, 12);
        
        if(robot_x < -55)
          Robot::moveRobotAbs(90, 12);
        
      }
    } 
    Robot::update();
  }
}
