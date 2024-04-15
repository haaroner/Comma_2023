#include "Robot.h"
#include "Settings.h"

#define TEST_DRIBLER false
#define TEST_MOTORS false

volatile uint32_t time = 0, ball_grab_timer = 0, super_timer = 0,
  point_reached_timer = 0, delay = 0, attacker_2_to_1_tim = 0;

volatile uint8_t gaming_state, role, attacker_state = 1, attacker_old_state = 1;


int16_t move_angle = 0, move_speed = 0, gyro, forward_angle, backward_angle, 
  ball_loc_angle, ball_abs_angle;

uint16_t forward_distance, backward_distance, ball_distance;
int32_t robot_x = 0, robot_y = 80, ball_loc_x = 0, ball_loc_y = 20,
  ball_abs_x = 0, ball_abs_y = 100, point_distance = 0;
  
bool point_reached = false;
  

point robot_position, ball_abs_position, attacker_start_state_point;

volatile uint16_t dribler_speed = 0;
volatile uint8_t to_keck = 0; 
volatile uint32_t pinok_delay = 20;

int main()
{
  Robot::init_robot();
  #if TEST_MOTORS
  while(true)
  {
    Robot::motors_on_off(ON); 
    Robot::moveRobot(0, 10);
    gyro = Robot::gyro;
    robot_x = Robot::robot_x;
    robot_y = Robot::robot_y;
    time_service::delay_ms(1);
    
    Robot::update();
  }
  #endif
  Robot::wait(2000);
  role = 1;
  
  #if TEST_DRIBLER
  while(true)
  {
    Robot::motors_on_off(ON);
    Robot::set_dribler_speed(dribler_speed);
    if(to_keck != 0)
    {
      Robot::direct_keck(pinok_delay);
      to_keck = 0;
    }
    Robot::update();
    time_service::delay_ms(1);
  }
  #endif

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
      Robot::set_dribler_speed(0);
      
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
          Robot::enable_trajectory(false);
          //if ball in dribler more than 4 scnds go to state 2
          if(time - ball_grab_timer > 3000 && attacker_old_state == 1)
          {
            Robot::set_dribler_speed(14);
            Robot::wait(1250);
            attacker_state = 2;
          }
          if(attacker_old_state != 1) ball_grab_timer = time;
        }
        
        // go with ball to gates
        if(attacker_state == 2) 
        {
          //save start state point
          if(attacker_old_state != 2)
            attacker_start_state_point = robot_position;
          
          //if ball moved out of dribler accidentally go to state 1
          if(ball_distance < 25 && my_abs(ball_loc_angle) < 20) attacker_2_to_1_tim = time;
          
          if(time - attacker_2_to_1_tim > 200) attacker_state = 1;
        }
        if(attacker_state == 3)
        {
          if(attacker_old_state != 3)
            attacker_start_state_point = robot_position;
        }
        
        
        
        
        /*attacker state bodies*/
        
        if(attacker_state == 1) //move to ball and grab it
        {         
//          if(ball_distance > 15) //move quickly
//          {
//            Robot::moveRobotAbs(ball_abs_angle, 26);
//            Robot::set_dribler_speed(0);
//          }
//          else //move slowly
//          {
//            Robot::moveRobotAbs(ball_abs_angle, 7);
//            Robot::set_dribler_speed(14);
//          }
          if(ball_distance < 16) Robot::set_dribler_speed(11);
          else Robot::set_dribler_speed(0);
          
          if(ball_abs_x > 65)
          {
            Robot::moveRobot(ball_loc_angle + exponential_detour(ball_loc_angle, ball_distance, 0.066, 0.35, 0.0255, 4.1), constrain(30, 7, ball_distance * 1.2));
            //move_angle = ball_angle + exponential_detour(ball_angle, ball_distance, 0.066, 0.35, 0.0255, 4.1);
            Robot::setAngle(90, 11, -0.2);
          }
          else
          {
            Robot::moveRobotAbs(ball_abs_angle, constrain(30, 7, ball_distance * 1.2));
            Robot::setAngle(ball_abs_angle, 11, -0.2); //turn to ball 0.15
          }
          if((ball_distance > 20 || my_abs(ball_loc_angle) > 20) && (Robot::is_ball_seen_T(75))) ball_grab_timer = time;
          attacker_old_state = 1;
          
        }       
        
        /*move to gates with ball*/
        
        if(attacker_state == 2) 
        {
          //move to gates
//          Robot::set_dribler_speed(14);
//          point_reached = Robot::moveToPoint(35 * my_sgn(attacker_start_state_point.x), 180, 8);
//          Robot::setAngle(lead_to_degree_borders(forward_angle + 180), 3, -0.02);
        
          
          
          if(attacker_old_state == 1)
          {
            Robot::enable_trajectory(true);
            Robot::add_stop_to_route(40 * my_sgn(robot_x), 160, 90 * my_sgn(robot_x));
            Robot::add_stop_to_route(37 * my_sgn(robot_x), 185, 90 * my_sgn(robot_x));
          }
          
          //if point reached
          //if(point_reached)
          if(Robot::trajectory_finished)
          {
            Robot::enable_trajectory(false);
            Robot::set_dribler_speed(22);
            Robot::wait(250);
            Robot::setAngle(0, 0, -0.1);
            
            Robot::wait(1000, true, 130 * my_sgn(attacker_start_state_point.x), 5);
            Robot::setAngle(0, 0, -0.4);
            Robot::wait(250);
            
            Robot::side_keck(130 * my_sgn(attacker_start_state_point.x), 
            10 * my_sgn(attacker_start_state_point.x), 6, 100  * my_sgn(attacker_start_state_point.x));
            
            //side kick
            //Robot::wait(1000, true, 10 * my_sgn(attacker_start_state_point.x), 35, 135);
           // Robot::side_keck(forward_angle + 15 * my_sgn(attacker_start_state_point.x), 20,
             // lead_to_degree_borders(forward_angle + 160 * my_sgn(attacker_start_state_point.x)));
            
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
//            //Robot::set_dribler_speed(20);
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
        
        /*now it is just state to check ball visibility*/
        if(attacker_state == 3)
        {
//          Robot::set_dribler_speed(15);
//          point_reached = Robot::moveToPoint(37 * my_sgn(attacker_start_state_point.x), 205, 9);
//          Robot::setAngle(lead_to_degree_borders(forward_angle + 180), 3, -0.02);
//          
          if(attacker_old_state == 1)
          {
            Robot::enable_trajectory(true);
            Robot::add_stop_to_route(40 * my_sgn(robot_x), 160, 90 * my_sgn(robot_x));
            Robot::add_stop_to_route(35 * my_sgn(robot_x), 207, 90 * my_sgn(robot_x));
          }
          //if(point_reached)
          if(Robot::trajectory_finished)
          {
            Robot::enable_trajectory(false);
            Robot::wait(500);
            Robot::setAngle(0, 0, -0.1);
            Robot::wait(2000, true, 90 * my_sgn(attacker_start_state_point.x), 3);
            Robot::wait(2000, true, -50 * my_sgn(attacker_start_state_point.x), 3);
            Robot::set_dribler_speed(10);
            Robot::wait(500);
            Robot::direct_keck(10);
            attacker_state = 1;
          }
          attacker_old_state = 3;
        }
        
        /*avoiding out of bounds*/
        
        //front gate
        if(my_abs(robot_x) < 30 && robot_y > 193)
        {
          Robot::moveRobotAbs(180, 12);
          if(attacker_state == 1 && robot_y < 203 && ball_distance < 20)
          {
            move_angle = sum_of_vectors(ball_abs_angle, ball_distance * 1.25, 180, 7);
            move_speed = get_len_from_sum_of_vectors();
            
            Robot::moveRobotAbs(move_angle,constrain(13, 0, move_speed));
          }
        }
        if(my_abs(robot_x) > 30 && forward_distance < 35)
        {
          move_angle = lead_to_degree_borders(forward_angle + 180);
          Robot::moveRobotAbs(move_angle, 12);
          if(attacker_state == 1 && forward_distance > 25 && ball_distance < 20)
          {
            move_angle = sum_of_vectors(ball_abs_angle, ball_distance * 1.25, move_angle, 7);
            move_speed = get_len_from_sum_of_vectors();
            
            Robot::moveRobotAbs(move_angle,constrain(13, 0, move_speed));
          }
        }
        
        if(robot_y > 215)
          Robot::moveRobotAbs(180, 12);
        
        //back gate
        if(my_abs(robot_x) < 30 && robot_y < 40)
          Robot::moveRobotAbs(0, 12);
        
        if(my_abs(robot_x) > 30 && backward_distance < 45)
          Robot::moveRobotAbs(lead_to_degree_borders(backward_angle + 180), 12);  
        
        //side outs
        if(robot_x > 82)
          Robot::moveRobotAbs(-90, 12);
        
        if(robot_x < -75)
          Robot::moveRobotAbs(90, 12);     
      }
    } 
    Robot::update();
  }
}
