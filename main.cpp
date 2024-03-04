#include "Robot.h"
#include "Settings.h"
#include "SSD1306.h"
#include "font.h"

#define TEST_DRIBLER 0

volatile uint32_t _test_flashka = 0, defender_angry_ball_movement_timer = 0, 
  time = 0, defender_angry_state_start_timer = 0, dribler_speed = 200,
  defender_attack_timer = 0, cringe_timer = 0;
volatile int16_t robot_x = 0, robot_y = 0, gyro = 0, forward_angle = 0, backward_angle = 0,
a = 0, b = 0, c, move_angle, ball_angle, defence_angle, 
  start_attack_point[2], point[2], error_angle = 0, 
  ball_distance = 0, x0_gyro = 0;

volatile int32_t defender_y_error = 0;
volatile int defender_ideal_angle = 0, defender_angle_error = 0,
  defender_defence_angle = 0, defender_line_angle = 0, 
  defender_point_angle = 0, defender_point_distance = 0,
  defender_error_len = 0, defender_defence_len = 0,
  defender_ball_distance = 0, defender_ball_angle = 0, 
  defender_local_gate_ball_angle = 0, forward_distance = 0, backward_distance = 0;

uint8_t gaming_state = 0, role = 1, attacker_state = 0, defender_state = 0,
  move_speed, error_speed = 0, defender_mode = 0;

volatile int ball_abs_x = 0, ball_abs_y = 0, ball_loc_x = 0,
  ball_loc_y = 0, d, e = 0, ball_abs_angle;
double testf, defender_ball_state = 0;
bool trajectory_started = 0, defender_angry_ball_movement_enable = 0,
  defender_attack_enable = 0, cringe_enable = 0;

int main()

{
  //write_to_FLASH(2);
  Robot::init_robot();
  
  Robot::dribler_control.pwm(200);
  time_service::delay_ms(500);
  Robot::dribler_control.pwm(300);
  time_service::delay_ms(500);
  Robot::dribler_control.pwm(200);
  time_service::delay_ms(500);
  
  #if TEST_DRIBLER
  pin dribler_control('A', 3, dribler_);
  dribler_control.pwm(200);
  time_service::delay_ms(500);
  dribler_control.pwm(300);
  time_service::delay_ms(500);
  dribler_control.pwm(200);
  time_service::delay_ms(500);
    while(true)
    {
      //time = ball_sen_dma.dataReturn(0);// * 0.00357;
      dribler_control.pwm(dribler_speed/*change_speed(dribling_speed, time_service::getCurTime())*/);
    }
  #endif  
  
  //point abcd;
  //abcd.x = 2;
  //Robot::set_blinking(0, 1000);
  Robot::motors_on_off(ON);
  //Timer test;
  PID test2(-0.2, -0.005, 0, 5);
  //test.save_time();
  //Robot::moveRobot(0, 0);
  //Robot::rotateRobot(0, 15);
  Queue points_queue;
  time_service::delay_ms(1);
  //_test_flashka = read_from_FLASH();
  role = 1;
  
  struct point point, robot_position, attacker_start_state_point;
  struct polar_vector vector;
  defender_attack_timer = time_service::getCurTime();
  
  while(true)
  {
    //Robot::dribler_control.pwm(200);
    //Robot::moveRobot(0, 20);
    //Robot::setAngle(90, 20);
    //Robot::dribler_control.pwm(280);
    time = time_service::getCurTime();
    testf = time_service::getCurTime() - d;
    d = time_service::getCurTime();
//    Robot::display_clear();
//    Robot::display_draw_string("time =");
//    Robot::display_draw_number(testf, 1, 6, 0);
//    Robot::display_update(true);
    Robot::predict();
    gyro = Robot::gyro;
    robot_x = Robot::robot_x;
    robot_y = Robot::robot_y;
    robot_position.x = robot_x;
    robot_position.y = robot_y;
    
    ball_distance = Robot::ball_distance;
    ball_abs_angle = Robot::ball_abs_angle;
    ball_angle = Robot::ball_loc_angle;
    ball_distance = Robot::ball_distance;
    ball_abs_x = Robot::ball_abs_x;
    ball_abs_y = Robot::ball_abs_y;
    
    
    ball_loc_x = Robot::ball_loc_x;
    ball_loc_y = Robot::ball_loc_y;
    
    forward_angle = Robot::forward_angle;
    backward_angle = Robot::backward_angle;
    
    forward_distance = Robot::forward_distance;
    backward_distance = Robot::backward_distance;
    
    
    Robot::predict();
    
    a = Robot::_x1b;
    b = Robot::_y1b;
    if(my_abs(Robot::_alpha * RAD2DEG) < 90)
    c = Robot::_alpha * RAD2DEG;
    //predict();
    forward_angle = Robot::forward_angle;
    backward_angle = Robot::backward_angle;
    
    gaming_state = Robot::_game_state;
//    if(Robot::check_button(ENTER_BUTTON))
//    {
//      gaming_state = my_abs(gaming_state - 1);
//      Robot::control_led(0, OFF);
//    }
    
    if(gaming_state == 0)//gaming_state == 0
    {
      Robot::dribler_control.pwm(200);
      Robot::set_blinking(3, 0);
      Robot::motors_on_off(OFF);
      
     // Robot::moveRobot(move_angle, 0);
      Robot::control_led(3, my_abs(gyro) < 5);
      Robot::control_led(1, my_abs(forward_angle - gyro) < 10);
    }
    else if(gaming_state == 1)//gaming_state == 1
    {
      Robot::motors_on_off(ON);
      Robot::set_blinking(3, 200);
      if(role == 1) // atacker
      {
        if(false) // go to ball
        {
          attacker_state = 1;
        }
        else if(true) // go with ball to gates
        {
          if(attacker_state != 2)
            attacker_start_state_point = robot_position;
          attacker_state = 2;
          
        }
        else if(false) //smth idk
        {
          attacker_state = 3;
        }
        
        if(attacker_state == 2)
        {
          if(!Robot::use_trajectory)
          {
            Robot::enable_trajectory(true);
            if(attacker_start_state_point.y < 110)
            {
              Robot::add_stop_to_route(50 * my_sgn(robot_x), 110, 90 * my_sgn(robot_x));
              Robot::add_stop_to_route(25 * my_sgn(robot_x), 30, 45 * my_sgn(robot_x));
            }
          }
        }
      }
      if(role == 2) // defender
      {
//        if(/* not prediction, standart defend mode*/ false)
//        {
//          Robot::motors_on_off(OFF);
//          const int defender_y0 = 40;
//          int max_defender_x[2] = {-30, 30};
//          static float k_error = 1.85;//3.375
//          static const double k1_ball = 1.65, k2_ball = 0.625, k3_ball = 0.61, k_gyro = 0.017;
//          defender_y_error = robot_y - defender_y0;
//          
//          if((Robot::ball_abs_angle > 0 && robot_x < max_defender_x[1]) ||
//             (Robot::ball_abs_angle < 0 && robot_x > max_defender_x[0]))
//          {
//            if(defender_y_error > 0) defence_angle = 180;
//            else defence_angle = 0;
//            
//            if(Robot::ball_abs_angle > 0) 
//              error_angle = 90;
//            else
//              error_angle = -90;
//            
//            error_speed = my_abs(defender_y_error) * k_error;
//            move_angle = sum_of_vectors(defence_angle, error_speed, error_angle, 20);
//            move_speed = get_len_from_sum_of_vectors();
//            
//            if(move_speed > 80) move_speed = 80;
//          }
//          else
//          {
//            move_speed = 0;
//          }
//          Robot::moveRobotAbs(move_angle, move_speed);
//        }
//        else if(/*prediction*/true)
//        {
//          //if(my_abs(Robot::_defender_predicted_x) < 30)
//          Robot::moveToPoint(Robot::_defender_predicted_x, 40, -1);
//        }
        if(true /* test phormula movement*/)
        {
          const int borderangls[6] = {112, 125, 145, -142, -130, -112};
          const int slow_angles[2] = {135, -135};
          const int stop_angles[2] = {120, -120};
          const int d0[5] = {41, 12, 35, 13, 41};
          const int minR = 25;
          const int side_points[2][2] = {{-29, 23}, {27, 23}};
          const double defence_k1 = 0.000265/*25*/, defence_k2 = 1.53, defence_k3 = 1.3;
          const float kd = 2;
          const uint8_t defender_whole_max_speed = 65, defender_defence_max_speed = 45, defender_error_max_speed = 30;
          const uint32_t defender_start_attack_time = 2000, defender_end_attack_time = 5000, defender_end_returning_time = 6500;
          
          defender_ideal_angle = lead_to_degree_borders(backward_angle + 180);
          vector = get_angle_to_point(0, 0, ball_abs_x, ball_abs_y);
          defender_local_gate_ball_angle = vector.angle;
          
          if(my_abs(backward_angle - ball_abs_angle) < 45)
          {
            defender_angle_error = lead_to_degree_borders(defender_local_gate_ball_angle - defender_ideal_angle);
            defender_ball_state = 1;
          }
          else 
          {
            defender_angle_error = lead_to_degree_borders(ball_abs_angle - defender_ideal_angle);
            defender_ball_state = 0;
          }
          
          if(Robot::_dSSoft < 40 && defender_angle_error < 14 && 
          ((my_abs(ball_abs_x) < 25 && ball_abs_y < 130) || 
          (my_abs(ball_abs_x) < 50 && robot_y < 60)) && 
          Robot::is_ball_seen_T(300) && ball_abs_y > 40)
          {
            if(!defender_attack_enable) defender_attack_timer = time;
              defender_attack_enable = 1;
            
            if(time - defender_attack_timer > defender_start_attack_time && time - defender_attack_timer < defender_end_attack_time)
            {
              defender_mode = 2;
            }
            else
            {
              //defender_attack_enable = 0;
              defender_mode = 1;
            }
          }
          else
          {
            if(time - defender_attack_timer > defender_end_attack_time || 
            ((my_abs(ball_abs_x) < 35 && ball_abs_y > 135) && 
            (my_abs(ball_abs_x) < 60 && robot_y > 80)) ||
             my_abs(ball_abs_x) >= 60 || !Robot::is_ball_seen_T(300))
            {
              defender_mode = 1;
            }
            defender_attack_enable = 0;
          }
          if(true)
          {
            if((backward_angle <= borderangls[1] && backward_angle >= borderangls[0]) ||
               (backward_angle >= borderangls[4] && backward_angle <= borderangls[5]))
            {
              defender_state = 1;
              a = 1;
              if(robot_x > 0) // right side
              {
                if(defender_angle_error > 0)
                  defender_defence_angle = 180;
                else 
                  defender_defence_angle = 0;
               
                if(robot_x > d0[4])
                  defender_line_angle = -90;
                else
                  defender_line_angle = 90;
                defender_error_len = my_abs(my_abs(robot_x) - my_abs(d0[4])) * kd;
              }
              else // left side
              {
                if(defender_angle_error > 0)
                  defender_defence_angle = 0;
                else 
                  defender_defence_angle = 180;
                
                if(robot_x > -d0[0])
                  defender_line_angle = -90;
                else
                  defender_line_angle = 90;
                defender_error_len = my_abs(my_abs(robot_x) - my_abs(d0[0])) * kd;
              }
               //move_angle = sum_of_vectors(defender_defence_angle, 15, defender_line_angle, 15);
              //move_speed = 14;
                
            }
            else if(backward_angle >= borderangls[2] || backward_angle <= borderangls[3]) // central line zone
            {
              defender_state = 12;
              a = 2;
              if(defender_angle_error > 0)
                defender_defence_angle = 90;
              else
                defender_defence_angle = -90;
              
              if(robot_y > d0[2])
                defender_line_angle = 180;
              else
                defender_line_angle = 0;
              //move_angle = sum_of_vectors(defender_defence_angle, 15, defender_line_angle, 15);
              move_speed = 14;
              defender_error_len = my_abs(robot_y - d0[2]) * kd;
            }
            else if((backward_angle > borderangls[3] && backward_angle < borderangls[4]) ||
               (backward_angle > borderangls[1] && backward_angle < borderangls[2])) // quarter ring side zone
            { 
              defender_state = 1;
              a = 3;
              if(robot_x > 0)
              {
                defender_point_angle = lead_to_degree_borders(Robot::getAngleToPoint(side_points[1][0], side_points[1][1]));
                defender_point_distance = Robot::getDistanceToPoint(side_points[1][0], side_points[1][1]);
             
                if(defender_angle_error > 0)
                  defender_defence_angle = lead_to_degree_borders(defender_point_angle - 90);
                else
                  defender_defence_angle = lead_to_degree_borders(defender_point_angle + 90);
                //if(my_abs(lead_to_degree_borders(backward_angle - defender_point_angle)) < 90)
                //{
                  if(defender_point_distance > d0[3])
                    defender_line_angle = defender_point_angle;
                  else
                    defender_line_angle = lead_to_degree_borders(defender_point_angle + 180);
                  //}
                  //else
                  //{
                    
                    //if(defender_point_distance < d0[3])
                     //defender_line_angle = defender_ideal_angle;// it is shit!!!!
                  //}
                defender_error_len = (defender_point_distance - d0[3])  * kd;
              }
              if(robot_x < 0)
              {
                defender_point_angle = Robot::getAngleToPoint(side_points[0][0], side_points[0][1]);
                defender_point_distance = Robot::getDistanceToPoint(side_points[0][0], side_points[0][1]);
                if(defender_angle_error > 0)
                  defender_defence_angle = lead_to_degree_borders(defender_point_angle - 90);
                else
                  defender_defence_angle = lead_to_degree_borders(defender_point_angle + 90);
                  
                if(defender_point_distance > d0[1])
                  defender_line_angle = defender_point_angle;
                else
                  defender_line_angle = lead_to_degree_borders(defender_point_angle + 180);
                defender_error_len = (defender_point_distance - d0[1])  * kd;
              }
                
                //move_angle = sum_of_vectors(defender_defence_angle, 15, defender_line_angle, 15);
              move_speed = 0;
            }
            else
            {
              defender_state = 0;
              defender_line_angle = 0;
              defender_error_len = 15;
              defender_defence_len = 0;
            }
          if(my_abs(defender_angle_error) < 5) //side vertical line
            defender_defence_len = 0;
          defender_ball_angle = constrain(80, 0, my_abs(defender_angle_error));
          defender_ball_distance = constrain(85, 15, ball_distance);     
          defender_defence_len = defence_k1 * pow(my_abs(defender_ball_angle), defence_k2) * pow(double(130 - defender_ball_distance), defence_k3);
            
            //if(a == 3) defender_defence_len = 0;
            //defender_defence_len = constrain(80, 0, defender_defence_len);

          if(defender_ball_state == 1 && defender_angle_error > 7)
          {
            if(ball_distance > 35)
             defender_defence_len = constrain(50, 10, defender_defence_len);
            else
              defender_defence_len = constrain(30, 10, defender_defence_len);
          }
          else if(defender_ball_state == 1) defender_defence_len = constrain(30, 0, defender_defence_len);
            
          if(Robot::is_ball_seen && defender_ball_angle > 15)
            defender_defence_len = constrain(defender_defence_max_speed, 15, defender_defence_len);
          else
            defender_defence_len = constrain(defender_defence_max_speed, 0, defender_defence_len);
          if(Robot::backward_distance <= minR)
            defender_defence_angle = lead_to_degree_borders(Robot::backward_angle + 180);
            
          if((is_in_the_angle_borders(slow_angles[0], 0, backward_angle) && defender_angle_error <= 0) ||
            (is_in_the_angle_borders(0, slow_angles[1], backward_angle) && defender_angle_error >= 0))
            defender_defence_len = constrain(35, 0, defender_defence_len); //slow down
          
          if((is_in_the_angle_borders(stop_angles[0], 0, backward_angle) && defender_angle_error <= 0) ||
            (is_in_the_angle_borders(0, stop_angles[1], backward_angle) && defender_angle_error >= 0))
            defender_defence_len = 0; // side stop
          
          if(Robot::is_ball_seen_T(2000) == 0 && my_abs(robot_x) > 5) //ball not seen
          {
            defender_state = 2;
            if(robot_x > 0)
              Robot::moveToPoint(15, 37, -1);
            else
              Robot::moveToPoint(-15, 37, -1);
            c = 0;
            defender_angry_ball_movement_timer = time;
            defender_angry_ball_movement_enable = 1;
          }
          else 
          {  
            c = 1;
            if(Robot::is_ball_seen_T(2000) == 0) defender_defence_len = 0;
              defender_error_len = constrain(defender_error_max_speed, 0, defender_error_len);
            move_angle = sum_of_vectors(defender_defence_angle, defender_defence_len, defender_line_angle, defender_error_len);
            move_speed = get_len_from_sum_of_vectors();
            move_speed = constrain(defender_whole_max_speed, 0, move_speed);
              
            if(my_abs(backward_angle) > 147 && Robot::predict())
            {
              //move_angle = Robot::getAngleToPoint(Robot::_defender_predicted_x, Robot::_defender_predicted_y);
              if(Robot::getDistanceToPoint(Robot::_defender_predicted_x, Robot::_defender_predicted_y) > 7)
                move_speed = 80;
              else
                move_speed = 0;
              Robot::moveToPoint(Robot::_defender_predicted_x, Robot::_defender_predicted_y, -1);
            }
              
            if(time - defender_angry_ball_movement_timer < 1500 && defender_angry_ball_movement_enable)
            {
              if(((ball_abs_x < -10 && ball_abs_x > -60 && ball_abs_y < 60 && 
                ball_abs_y > 15 && ball_distance > 15) ||
              (ball_abs_x > 10 && ball_abs_x < 60 && ball_abs_y < 60 && 
                ball_abs_y > 15 && ball_distance > 15)))
              {
                move_angle = Robot::getAngleToPoint(ball_abs_x, ball_abs_y + 5);
                move_speed = 95;
              }
              if(my_abs(defender_angle_error) < 15)
                move_speed = 0;
            }
            if((ball_distance <= 15 || time - defender_angry_ball_movement_timer > 1500)/* || my_abs(ball_abs_angle) > 150 || ball_loc_y < 0*/)
              defender_angry_ball_movement_enable = 0;
          }
          
//            if(backward_distance > 50)
//            {
//              if(my_abs(ball_abs_angle) > 90)
//              {
//                move_angle = ball_abs_angle + exponential_detour(ball_abs_angle, 10 - constrain(map(ball_distance, 0, 140, 0, 10), 0, 10), 0.06, 0.32, 0.024, 4.1);
//                if(ball_distance < 35)
//                  move_speed = 30;
//                else
//                  move_speed = 60;
//              }
//              else
//              {
//                move_angle = backward_angle;
//                move_speed = 60;
//              }
//            }
         }
         else
         {
          move_angle = ball_abs_angle + exponential_detour(ball_abs_angle, 10 - constrain(map(ball_distance, 0, 140, 0, 10), 0, 10), 0.06, 0.32, 0.024, 4.1); // 0.066, 0.35, 0.0255, 4.1
          //move_angle = ball_abs_angle;
          if(my_abs(ball_abs_angle) > 10)
            move_speed = 35;
          else
            move_speed = 60;
         }
        }
        Robot::setAngle(0, 20);
        if(Robot::is_ball_seen_T(2000) == 1 || my_abs(robot_x) < 10)
          Robot::moveRobotAbs(move_angle, move_speed);
      }
      if(role == 3)
      {
        if(time - cringe_timer < 5000)
          Robot::setAngle(0, 20);
        else
        {
          if(time - cringe_timer < 6000)
            Robot::rotateRobot(50, 50);
          else
            cringe_timer = time;
        }
      }
    }
    //Robot::setAngle(90, 20);
    Robot::display_data(Robot::move_speed, Robot::point_distance);
    Robot::update();
  }
}
