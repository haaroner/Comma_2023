#include "Robot.h"
#include "Settings.h"
#include "SSD1306.h"
#include "font.h"

#define TEST_DRIBLER 0

volatile uint32_t _test_flashka = 0, defender_angry_ball_movement_timer = 0, 
  time = 0, defender_angry_state_start_timer = 0, dribler_speed = 200,
  defender_attack_timer = 0;
volatile int16_t robot_x = 0, robot_y = 0, gyro = 0, forward_angle = 0, backward_angle = 0,
a = 0, b = 0, c, move_angle, ball_angle, defence_angle, 
  start_attack_point[2], point[2], error_angle = 0, ball_distance = 0;

volatile int32_t defender_y_error = 0;
volatile int defender_ideal_angle = 0, defender_angle_error = 0,
  defender_defence_angle = 0, defender_line_angle = 0, 
  defender_point_angle = 0, defender_point_distance = 0,
  defender_error_len = 0, defender_defence_len = 0,
  defender_ball_distance = 0, defender_ball_angle = 0, defender_local_gate_ball_angle = 0;

uint8_t gaming_state = 0, role, attacker_state = 0, defender_state = 0,
  move_speed, error_speed = 0, defender_mode = 0;

volatile int ball_abs_x = 0, ball_abs_y = 0, ball_loc_x = 0,
  ball_loc_y = 0, d, e = 0, ball_abs_angle;
double testf, defender_ball_state = 0;
bool trajectory_started = 0, defender_angry_ball_movement_enable = 0,
  defender_attack_enable = 0;

int main()
{
  Robot::init_robot();
  
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
  Robot::motors_on_off(OFF);
  //Timer test;
  PID test2(-0.2, -0.005, 0, 5);
  //test.save_time();
  Robot::moveRobot(0, 0);
  Robot::rotateRobot(0, 15);
  Queue points_queue;
  time_service::delay_ms(1);
  //_test_flashka = read_from_FLASH();
  role = Robot::role;
  
  struct point point;
  struct polar_vector vector;
  
  while(true)
  {
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
    
    ball_abs_angle = Robot::ball_abs_angle;
    ball_distance = Robot::ball_distance;
    ball_abs_x = Robot::ball_abs_x;
    ball_abs_y = Robot::ball_abs_y;
    
    ball_loc_x = Robot::ball_loc_x;
    ball_loc_y = Robot::ball_loc_y;
    
    forward_angle = Robot::forward_angle;
    backward_angle = Robot::backward_angle;
    
    
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
    
    if(gaming_state == 0)
    {
      Robot::set_blinking(3, 0);
      Robot::motors_on_off(OFF);
      //if(Robot::check_button(UP_BUTTON)) Robot::change_side();
      //if(Robot::check_button(DOWN_BUTTON)) Robot::callibrate_gyro();
      
      Robot::moveRobot(move_angle, 0);
      Robot::control_led(3, my_abs(gyro) < 5);
      Robot::control_led(1, my_abs(forward_angle - gyro) < 10);
    }
    else if(gaming_state == 1)
    {
      Robot::motors_on_off(ON);
      //Robot::moveToPoint(10, 37, -1);
      Robot::set_blinking(3, 200);
      if(role == 1) // atacker
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
          const float kd = 2.4;
          const uint8_t defender_whole_max_speed = 77, defender_defence_max_speed = 50, defender_error_max_speed = 50;
         
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
          
          if(Robot::_dS < 15 && defender_angle_error < 10)
          {
            if(!defender_attack_enable) defender_attack_timer = time;
            defender_attack_enable = 1;
            
            if(time - defender_attack_timer > 2000 && time - defender_attack_timer < 3500)
            {
              defender_mode = 2;
              a = 0;
              b = 0;
              points_queue.clear();
              points_queue.push(ball_abs_x);//x
              points_queue.push(ball_abs_y - 20);//y
            }
            else
            {
              //defender_attack_enable = 0;
              defender_mode = 1;
            }
          }
          else
          {
            if(time - defender_attack_timer > 3500)
            {
              defender_mode = 1;
              defender_attack_enable = 0;
            }
          }
          if(defender_mode == 1)
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
          
          if(Robot::is_ball_seen_T(2000) == 0 && my_abs(robot_x) > 10) //ball not seen
          {
            defender_state = 2;
            if(robot_x > 0)
              Robot::moveToPoint(10, 37, -1);
            else
              Robot::moveToPoint(-10, 37, -1);
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
              
            if(my_abs(backward_angle) > 140 && Robot::predict())
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
                ball_abs_y > 15 && ball_distance > 15)) && 
                time - defender_angry_state_start_timer > 1500)
              {
                move_angle = Robot::getAngleToPoint(ball_abs_x, ball_abs_y - 5);
                move_speed = 95;
              }
              if(my_abs(defender_angle_error) < 15)
                move_speed = 0;
            }
            if((ball_distance <= 15 || time - defender_angry_ball_movement_timer > 1500) || my_abs(ball_abs_angle) > 150 || ball_loc_y < 15)
              defender_angry_ball_movement_enable = 0;
          }
         }
         else
         {
          points_queue.clear();
          points_queue.push(ball_abs_x);//x
          points_queue.push(ball_abs_y - 20);//y
           
          move_angle = ball_abs_angle;
          move_speed = 70;
         }
            
        }
        Robot::setAngle(0, 20);
      } 
      if(Robot::is_ball_seen_T(2000) == 1 || my_abs(robot_x) < 10)
        Robot::moveRobotAbs(move_angle, move_speed);
    }
    Robot::display_data(Robot::_dS, ball_loc_x);
    Robot::update();
  }
}
