#include "Robot.h"
#include "Settings.h"
#include "usartik1.h"

#define TEST_DRIBLER false
#define TEST_MOTORS false
#define TEST_BLUETOOTH false
#define BALL_THRESHOLD 4

volatile double a = 0, b = 1, c = 10;
uint8_t Putin = 5;


//timers
volatile uint32_t time = 0, ball_grab_timer = 0, super_timer = 0,
  point_reached_timer = 0, delay = 0, attacker_2_to_1_tim = 0, 
  attacker_3_to_1_tim = 0,
  out_action_tim = 0, defender_start_ball_track_tim = 0;

//general values
volatile uint8_t gaming_state;

int16_t move_angle = 0, move_speed = 0, gyro, forward_angle, backward_angle, 
  ball_loc_angle, ball_abs_angle;

uint16_t forward_distance, backward_distance, ball_distance;
int32_t robot_x = 0, robot_y = 80, ball_loc_x = 0, ball_loc_y = 20,
  ball_abs_x = 0, ball_abs_y = 100, point_distance = 0;

bool point_reached = false;
point robot_position, ball_abs_position;

float bounds_k = 0;

//attacker values
volatile uint8_t role, attacker_state = 1, attacker_old_state = 1;
volatile uint32_t out_diving_start_tim = 0, out_diving_stop_tim = 0,
  attacker_4_to_1_tim = 0, attacker_5_to_1_tim = 0, attacker_angry_tim = 0;

bool out_diving = false;
  
point attacker_start_state_point; 

bool attacker_angry = false;
  
//defender values
volatile uint8_t defender_state = 1, defender_old_state = 0;

int16_t defender_angle_error = 0, defender_y_error = 0,
defender_y_angle = 0;

point ball_track_position, backward_gate_center, ball_start_position;

uint32_t defender_3_to_1_tim = 0, defender_1_to_4_tim = 0, defender_4_to_1_tim, 
  defender_ball_grab_tim = 0, defender_end_keck_time = 0, defender_end_keck_tim = 0,
  defender_2_to_1_tim = 0, defender_1_to_2_tim = 0;
 
bool ball_position_fixed = false;

//values for test
volatile uint16_t dribler_speed = 0;
volatile uint8_t to_keck = 0; 
volatile uint32_t pinok_delay = 20, pinok_wait = 7;

int main()
{
  Robot::init_robot();
  Robot::wait(2000);
  #if TEST_BLUETOOTH
    while(true)
    {
      Robot::wait(1);
      usartik1::abcde(Putin);
      time_service::delay_ms(1);
      if(usartik1::available() > 0)
        b = usartik1::read();
      Robot::wait(500);
      Putin += 1;
      if(Putin > 99) Putin = 0;
    }
  #endif
  
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
  
  #if TEST_DRIBLER
  while(true)
  {
    Robot::motors_on_off(ON);
    Robot::use_dribler(true);
    Robot::set_dribler_speed(dribler_speed);
    if(to_keck != 0)
    {
      Robot::keck(pinok_delay);
      Robot::wait(pinok_wait);
      Robot::set_dribler_speed(0);
      Robot::wait(200);
      to_keck = 0;
    }
    Robot::update();
    time_service::delay_ms(1);
  }
  #endif

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
      Robot::use_dribler(false);
      Robot::set_dribler_speed(0);
      
      Robot::set_blinking(2, 0); //disable blinking of middle LED
      
      //callibration indication
      Robot::control_led(3, my_abs(gyro) < 5);
      Robot::control_led(2, my_abs(forward_angle) < 10);
      
      //update some timers
      ball_grab_timer = time;
      
      //reset robot's states
      attacker_state = 1;
      defender_state = 1;
      defender_old_state = 0;
    }
    else if(gaming_state == 1)
    {
      Robot::motors_on_off(ON);
      Robot::use_dribler(true);
      Robot::set_blinking(2, 200);
      Robot::predict(50);
      
      //attacker role
      if(role == 1) // atacker
      {
        int16_t max_robotx[2] = {87, -72};
        uint16_t max_forward_depth[2] = {193, 35};   
        
        /*attacker state change conditions*/
        
        //go to ball condition
        if(attacker_state == 1)
        {
          Robot::enable_trajectory(false);
          
          if(my_abs(ball_abs_angle - forward_angle) > 45) attacker_angry_tim = time;
          
          if(time - attacker_angry_tim > 400) attacker_angry = true;
          else attacker_angry = false;
          
          if(attacker_angry == false)
          {
            //if ball in dribler more than 3 scnds go to state 2
            if(time - ball_grab_timer > 1000 && attacker_old_state == 1)
            {
              Robot::set_dribler_speed(30);
              Robot::wait(100);
              attacker_state = 2;
            }
          }
          else
          {
            if(time - ball_grab_timer > 500 && attacker_old_state == 1)
            {
              Robot::set_dribler_speed(30);
              Robot::wait(100);
              attacker_state = 5;
            }
          }
          
         //!!!!REMAKE THIS IF STATEMENT AND FROM 2 to 1 state IF!!!!!!!!!!!
         // if(((ball_distance > 20 || my_abs(ball_loc_angle) > 12) && Robot::is_ball_seen_T(200)) || 
           // !(Robot::is_ball_seen_T(200))) ball_grab_timer = time;
          if(my_abs(ball_loc_angle) > 10 || ball_distance > 10) ball_grab_timer = time;
          
          if(attacker_old_state != 1)
          {
            ball_grab_timer = time;
            attacker_angry = false;
          }
        }   
        
        // go with ball to gates
        if(attacker_state == 2) 
        {
          Robot::enable_delay();
          //save start state point
          if(attacker_old_state != 2)
            attacker_start_state_point = robot_position;
          
          //if ball moved out of dribler accidentally go to state 1
          if(ball_distance <= 10 && my_abs(ball_loc_angle) <= 10) attacker_2_to_1_tim = time;
          
          if(Robot::is_ball_seen_T(100))
            if(time - attacker_2_to_1_tim > 100) attacker_state = 1;
          
          out_diving = false;
        }
        
        if(attacker_state == 3)
        {
          if(attacker_old_state != 3)
            attacker_start_state_point = robot_position;
          
          if(ball_distance < 20 && my_abs(ball_loc_angle) < 20) attacker_3_to_1_tim = time;
          
          if(Robot::is_ball_seen_T(100))
            if(time - attacker_3_to_1_tim > 100) attacker_state = 1;
          
          out_diving = false;
        }

        if(attacker_state == 4)
        {
         if(attacker_old_state != 4)
            attacker_start_state_point = robot_position;
         
         if(ball_distance < 20 && my_abs(ball_loc_angle) < 20) attacker_4_to_1_tim = time;
          
          if(Robot::is_ball_seen_T(100))
            if(time - attacker_4_to_1_tim > 100) attacker_state = 1;
        }
        
        if(attacker_state == 5)
        {
          if(ball_distance < 20 && my_abs(ball_loc_angle) < 20) attacker_5_to_1_tim = time;
          
          if(Robot::is_ball_seen_T(100))
            if(time - attacker_5_to_1_tim > 100) attacker_state = 1;
        }
        
        /******attacker state bodies******/
        if(attacker_state == 1) //move to ball and grab it
        {         
          Robot::enable_trajectory(false);
          if(ball_distance < 15 && my_abs(ball_loc_angle) < 15) 
            Robot::set_dribler_speed(30, false);
          else 
          {
            if(my_abs(lead_to_degree_borders(backward_angle + 180) - gyro) > 20)
              Robot::set_dribler_speed(0, false);
          }
          if(attacker_angry == false)
          {
            Robot::moveRobotAbs(ball_abs_angle, constrain(80, 25, (ball_distance - 5) * 2));
            Robot::setAngle(ball_abs_angle + BALL_THRESHOLD, 10, -0.3); //turn to ball 0.15
          }
          else
          {
            Robot::moveRobotAbs(ball_abs_angle, constrain(80, 50, (ball_distance - 4) * 2));
            Robot::setAngle(ball_abs_angle + BALL_THRESHOLD, 10, -0.3); //turn to ball 0.15
          }
          
          if(my_abs(lead_to_degree_borders(Robot::abs_move_angle - forward_angle)) < 45)
          {
            Robot::setRobotSpeed(constrain(40, 16, (ball_distance - 8) * 2));
          }
          
          //detecting ball is out of bounds
          if(out_diving == false && ball_abs_x < 65)
            out_diving_start_tim = time;
          
          //detecting ball is in the game zone
          if(out_diving == true && ball_abs_x >= 65)
            out_diving_stop_tim = time;
          
          //activate ball out of bounds state after 1.5s delay
          if(time - out_diving_start_tim > 1500)
          {
            out_diving = true;
            out_diving_stop_tim = time;
          }
          
          //stop ball out of bounds state after 1.5s delay
          if(time - out_diving_stop_tim > 1500)
          {
            out_diving = false;
            out_diving_start_tim = time;
          }
          
          if(out_diving)
          {
            //TODO make gyro independent movement
            Robot::moveRobot(ball_loc_angle + BALL_THRESHOLD + 
            exponential_detour(ball_loc_angle + BALL_THRESHOLD, ball_distance, 
            0.066, 0.35, 0.0255, 4.1), 7);
            
            Robot::setAngle(90, 11, -0.2);
          }
  
          attacker_old_state = 1;       
        }       
        
        /******move to gates with ball******/       
        if(attacker_state == 2) 
        {     
          if(attacker_old_state == 1)
          {
            Robot::enable_trajectory(true);
            if(robot_y <= 140) Robot::add_stop_to_route(47 * my_sgn(robot_x), 140);
            Robot::add_stop_to_route(49 * my_sgn(robot_x), 165);
          }   
          Robot::set_dribler_speed(35);
          Robot::setAngle(lead_to_degree_borders(forward_angle + 180), 9);
          
          if(Robot::trajectory_finished)
          {
            Robot::ball_distance_disable_delay(true);
            Robot::enable_trajectory(false);
            Robot::set_dribler_speed(45);
            Robot::wait(100);
            Robot::setAngle(0, 0, -0.1);
            
            //Robot::wait(100, true, 130 * my_sgn(attacker_start_state_point.x), 5);
            Robot::setAngle(0, 0, -0.4);
            //Robot::wait(250);
            
            Robot::side_keck(160 * my_sgn(attacker_start_state_point.x), 
            10 * my_sgn(attacker_start_state_point.x), 12, 120  * my_sgn(attacker_start_state_point.x));
            
            attacker_state = 1;
            //ball_grab_timer = time_service::getCurTime();
            Robot::ball_distance_disable_delay(false);
          } 
          attacker_old_state = 2;
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
            Robot::ball_distance_disable_delay(true);
            Robot::wait(500);
            Robot::setAngle(0, 0, -0.1);
            Robot::wait(2000, true, 90 * my_sgn(attacker_start_state_point.x), 3);
            Robot::wait(2000, true, -50 * my_sgn(attacker_start_state_point.x), 3);
            Robot::set_dribler_speed(10);
            Robot::wait(500);
            Robot::direct_keck();
            attacker_state = 1;
          }
          attacker_old_state = 3;
        }
        
        if(attacker_state == 4)
        {
//          Robot::moveRobotAbs(forward_angle, 30);
//          Robot::setAngle(-110, 7);
//          Robot::set_dribler_speed(30);
//          
//          if(forward_distance < 85)
//          {
//            Robot::direct_keck(10);
//            attacker_state = 1;
//            attacker_angry = false;
//          }
//          attacker_old_state = 4;
        }
        
        if(attacker_state == 5)
        {
          Robot::set_dribler_speed(15);
          
          Robot::moveRobotAbs(forward_angle, 90);
          Robot::setAngle(forward_angle, 7);
          
          
          if(forward_distance < 50 && time - attacker_angry_tim > 1500)
          {
            Robot::direct_keck();
            attacker_state = 1;
            attacker_angry = false;
          }
          attacker_old_state = 5;
        }
        /*avoiding out of bounds*/
        
        //front gate
        if(my_abs(robot_x) < 30 && robot_y > max_forward_depth[0])
        {
          Robot::moveRobotAbs(180, 10);
          if(attacker_state == 1 && robot_y < max_forward_depth[0] + 10 && ball_distance < 20)
          {
            move_angle = sum_of_vectors(ball_abs_angle, 7, 180, (robot_y -  max_forward_depth[0]) * 1.3);
            move_speed = get_len_from_sum_of_vectors();
            
            Robot::moveRobotAbs(move_angle,constrain(13, 0, move_speed));
          }
        }
        
//        if(out_diving)
//        {
//          if(robot_x > 90)
//            Robot::moveRobotAbs(-90, 12);
//          
//          if(robot_x < -90)
//            Robot::moveRobotAbs(90, 12); 
//        }
//        else
//        {
          if(robot_y > 215)
            Robot::moveRobotAbs(180, 12);
          
          if(robot_y > 180)
          {
            move_speed = Robot::move_speed;
            move_angle = Robot::abs_move_angle;
            
            if(my_abs(robot_x) <= 35)
            {
              move_angle = sum_of_vectors(move_angle, constrain(30, 0, move_speed),
              180, constrain(50, 0, (robot_y - 180) * 2));
              move_speed = get_len_from_sum_of_vectors();
            }
            else
            {
              move_angle = sum_of_vectors(move_angle, constrain(30, 0, move_speed),
              lead_to_degree_borders(forward_angle + 180), constrain(50, 0, (45 - forward_distance) * 1.5));
              move_speed = get_len_from_sum_of_vectors();
            }
           
            
            Robot::moveRobotAbs(move_angle, move_speed);
          }

          
          //back gate
          if(my_abs(robot_x) < 30 && robot_y < 40)
            Robot::moveRobotAbs(0, 12);
          
          if(my_abs(robot_x) > 30 && backward_distance < 45)
            Robot::moveRobotAbs(lead_to_degree_borders(backward_angle + 180), 12);  
          
          if(robot_x > 65)
          {
            bounds_k = constrainf(1, 0, 1 - my_abs(robot_y - 105) / 105);
            move_angle = sum_of_vectors(Robot::abs_move_angle, constrain(30, 0, Robot::move_speed),
              -90, constrain(50, 0, (robot_x - 85 - int(20 * bounds_k)) * 2));
            move_speed = get_len_from_sum_of_vectors();
            
            Robot::moveRobotAbs(move_angle, move_speed);
          }
          
          if(robot_x < -65)
          {
            bounds_k = constrainf(1, 0, 1 - my_abs(robot_y - 105) / 105);
            move_angle = sum_of_vectors(Robot::abs_move_angle, constrain(30, 0, Robot::move_speed),
              90, constrain(50, 0, (-105 - robot_x + int(17 * bounds_k)) * 2));
            move_speed = get_len_from_sum_of_vectors();
            
            Robot::moveRobotAbs(move_angle, move_speed);
          }
            
          
      }
      
      
      //#######defender role########//
      if(role == 2)
      {
        point middleL;
        middleL.x = -30;
        middleL.y = 42;
        
        point sideL;
        sideL.x = -43;
        sideL.y = 28;    
        const int16_t L_stop_angle = 118;
        const uint16_t L_stop_y = 19;
        
        point middleR;
        middleR.x = 30;
        middleR.y = 42;    
        
        point sideR;
        sideR.x = 35;
        sideR.y = 29;
        const int16_t R_stop_angle = -132;
        const uint16_t R_stop_y = 19;
        
        backward_gate_center.x = 0;
        backward_gate_center.y = 0;
        
        point returnR;
        returnR.x = 30;
        returnR.y = 50;
        
        point returnL;
        returnR.x = -35;
        returnR.y = 50;
         
        point returnM;
        returnR.x = 0;
        returnR.y = 50;
        
        
        //***defender change state conditions***//
        if(defender_state == 1)
        {
          if(backward_distance < 70 && robot_y < 70) defender_1_to_4_tim = time;
          else if (time - defender_1_to_4_tim > 500) defender_state = 4;
          
          if(!Robot::is_ball_seen_T(1500) && my_abs(defender_angle_error) < 80) 
          {
            defender_state = 5;
            ball_start_position = ball_abs_position;
          }
          
         // if(!Robot::predict(70)) defender_1_to_2_tim = time;
          //else if(time - defender_1_to_2_tim > 200) defender_state = 2;
          
          //counting time to go to keck state
         if((ball_distance > 40 || 
           ball_abs_y > 80 || ball_abs_y < 40 ||
           ball_abs_x > 55 || ball_abs_x < -55) || 
           defender_old_state != 1)
          {
            defender_start_ball_track_tim = time;
            ball_position_fixed = false;
          }
          
          //if ball is near and not fixed - save its position
          if(time - defender_start_ball_track_tim > 500 && !ball_position_fixed) 
          {
            ball_track_position = ball_abs_position;
            ball_position_fixed = true;
          }
          
          //if ball's position fixed
          if(ball_position_fixed)
          {
            //if ball moved more than by const value
            if(get_angle_to_point(ball_track_position, ball_abs_position).length > 20)
            {
              //refixe it
              defender_start_ball_track_tim = time;
              ball_position_fixed = false;
            }
            //if ball isnt moving go to keck state
            else if(time - defender_start_ball_track_tim > 1500 &&
                    time - defender_end_keck_time > 3000)
            {
              if(my_abs(defender_angle_error) < 7)
              {
                defender_state = 3;
                defender_3_to_1_tim = time;
              }
            }
          }
        }
        
        if(defender_state == 2)
        {
          defender_state = 1;
          if(Robot::predict()) defender_2_to_1_tim = time;
          else if(time - defender_2_to_1_tim > 750) attacker_state = 1;
        }
          
        if(defender_state == 3)
        {
          if(!Robot::is_ball_seen_T(1000))
          {
            defender_end_keck_time = time;
            defender_state = 1;
          }
          if(time - defender_3_to_1_tim > 3000)
            defender_state = 1;
          
          if(ball_abs_y < 80 || ball_abs_y > 40 || 
            ball_abs_x > 55 || ball_abs_x < -55)          
              defender_end_keck_tim = time;
          else if(time - defender_end_keck_tim > 500)
          {
            defender_end_keck_time = time;
            defender_state = 1;
          }
          
          if(get_angle_to_point(ball_track_position, ball_abs_position).length > 35)
          {
            defender_state = 1;
            defender_end_keck_time = time;
          }
        }
        
        if(defender_state == 4)
        {
          if(backward_distance > 70 && robot_y > 70) defender_4_to_1_tim = time;
          
          if(time - defender_4_to_1_tim > 500) defender_state = 1;
        }
        
        if(defender_state == 5)
        {
          if(Robot::is_ball_seen_T(100)) defender_state = 1;
        }
        
        
        //***defender state bodies***//
        if(defender_state == 1)
        {
          defender_angle_error = lead_to_degree_borders(get_angle_to_point(backward_gate_center, ball_abs_position).angle - lead_to_degree_borders(backward_angle + 180));
          //move_speed = constrain(30, 7, my_abs(defender_angle_error) * 1.7);
          move_speed = constrain(90, 8, pow(90 * my_abs(defender_angle_error), 0.5));
          move_speed *= my_abs(defender_angle_error) >= 5;
          //Robot::display_data(ball_abs_angle, move_speed);
          if(middleR.x >= robot_x && robot_x >= middleL.x)
          {       
            if(defender_angle_error > 5)
              move_angle = Robot::getAngleToPoint(middleR.x, middleR.y);
            else if(defender_angle_error < -5)
              move_angle = Robot::getAngleToPoint(middleL.x, middleL.y);
            defender_y_error = constrain(20, -20, (robot_y - 42) * 0.7);
            //if(defender_y_error > 0) defender_y_angle = 180;
            //else defender_y_angle = 0;
            defender_y_angle = (defender_y_error > 0) * 180;
            a = pow(double(10 * c), double(b));
            move_angle = sum_of_vectors(move_angle, move_speed, defender_y_angle, my_abs(defender_y_error));
            move_speed = get_len_from_sum_of_vectors();
            Robot::setAngle(0, 20, -0.36);
            Robot::moveRobotAbs(move_angle, move_speed);
          }
          else
          {
            //Robot::moveRobotAbs(0, 0);
            
            if(robot_x > middleR.x)
            {
              if(defender_angle_error > 5)
                move_angle = Robot::getAngleToPoint(sideR.x, sideR.y);
              else if(defender_angle_error < -5)
                move_angle = Robot::getAngleToPoint(middleR.x, middleR.y);
              
              if(backward_angle >= R_stop_angle && defender_angle_error >= 0)
                move_speed = 0;
              
              if(robot_y <= R_stop_y)
              {
                move_angle = 0;
                move_speed = 15;
              }
              
              Robot::setAngle(0, 12, -0.35);
              Robot::moveRobotAbs(move_angle, move_speed);
            }
            
            if(robot_x < middleL.x)
            {
              if(defender_angle_error > 5)            
                move_angle = Robot::getAngleToPoint(middleL.x, middleL.y);
              else if(defender_angle_error < -5)
                move_angle = Robot::getAngleToPoint(sideL.x, sideL.y);
              
              if(backward_angle <= L_stop_angle && defender_angle_error <= 0)
                move_speed = 0;
              
              if(robot_y <= L_stop_y)
              {
                move_angle = 0;
                move_speed = 15;
              }
              
              Robot::setAngle(0, 10, -0.3);
              Robot::moveRobotAbs(move_angle, move_speed);
            }
          }
          defender_old_state = 1;
        }
        
        if(defender_state == 2)
        {
          Robot::moveToPoint(Robot::predicted_point, 70);
          Robot::setAngle(0, 12, -0.35);
          defender_old_state = 2;
        }
        
        if(defender_state == 3)
        {
          if(Robot::is_ball_captured())
          {
            Robot::direct_keck();
            defender_state = 1;
            defender_end_keck_time = time;
          }
          else
          {
            Robot::moveRobotAbs(ball_abs_angle, constrain(20, 7, (ball_distance - 5) * 1.8));
            Robot::setAngle(ball_abs_angle, 11, -0.2);
          }
          
          defender_old_state = 3;
        }
        
        if(defender_state == 4)
        {
          move_speed = constrain(50, 15, (robot_y - 35) * 0.8);
          if(my_abs(lead_to_degree_borders(ball_abs_angle - backward_angle)) > 110)
            Robot::moveRobotAbs(backward_angle, 20);
          else
            Robot::moveRobotAbs(ball_abs_angle + 
          exponential_detour(lead_to_degree_borders(ball_abs_angle - backward_angle + 180), ball_distance, 0.08, 0.35, 0.0255, 3.2), move_speed);
          
          Robot::setAngle(lead_to_degree_borders(backward_angle + 180), 11, -0.2);
          defender_old_state = 4;
        }
        
        if(defender_state == 5)
        {
          Robot::moveToPoint(30, 50, 40);
//          Robot::moveRobotAbs(0, 0);
//          if(ball_start_position.x > 20)
//            Robot::moveToPoint(returnR.x, returnR.y, 40, 0);
//          else if(ball_start_position.x < -20)
//            Robot::moveToPoint(returnL.x, returnL.y, 40, 0);
          //else
            //Robot::moveToPoint(returnM, 40);
        }
      }
    } 
    Robot::update();
    Robot::display_data(ball_loc_angle, ball_distance);
  }
}
