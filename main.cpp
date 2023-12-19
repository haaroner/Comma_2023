#include "Robot.h"
#include "Settings.h"
#include "SSD1306.h"
#include "font.h"

volatile int16_t robot_x = 0, robot_y = 0, gyro = 0, forward_angle = 0, backward_angle = 0,
a = 0, b = 0, c, move_angle, ball_angle, defence_angle, start_attack_point[2], point[2], error_angle;

volatile int32_t defender_y_error = 0;

uint8_t gaming_state = 0, role, attacker_state = 0, defender_state = 0, move_speed, error_speed = 0;

volatile int ball_abs_x = 0, ball_abs_y = 0, ball_loc_x = 0, ball_loc_y = 0, d, e = 0;
double testf;
bool trajectory_started = 0;

int main()
{
  Robot::init_robot(1);
  
  //point abcd;
  //abcd.x = 2;
  //Robot::set_blinking(0, 1000);
  Robot::motors_on_off(OFF);
  //Timer test;
  PID test2(-0.2, -0.005, 0, 5);
  //test.save_time();
  Robot::moveRobot(0, 0);
  Robot::rotateRobot(0, 15);
  role = 2;
  Queue points_queue;
  
  
  while(true)
  {
    testf = time_service::getCurTime() - d;
    d = time_service::getCurTime();
//    Robot::display_clear();
//    Robot::display_draw_string("time =");
//    Robot::display_draw_number(testf, 1, 6, 0);
//    Robot::display_update(true);
    Robot::predict();
    Robot::draw_menu();
    gyro = Robot::gyro;
    robot_x = Robot::robot_x;
    robot_y = Robot::robot_y;
    
    ball_abs_x = Robot::ball_abs_x;
    ball_abs_y = Robot::ball_abs_y;
    
    ball_loc_x = Robot::ball_loc_x;
    ball_loc_y = Robot::ball_loc_y;
    
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
        if(/* not prediction, standart defend mode*/ false)
        {
          Robot::motors_on_off(OFF);
          const int defender_y0 = 40;
          int max_defender_x[2] = {-30, 30};
          static float k_error = 1.85;//3.375
          static const double k1_ball = 1.65, k2_ball = 0.625, k3_ball = 0.61, k_gyro = 0.017;
          defender_y_error = robot_y - defender_y0;
          
          if((Robot::ball_abs_angle > 0 && robot_x < max_defender_x[1]) ||
             (Robot::ball_abs_angle < 0 && robot_x > max_defender_x[0]))
          {
            if(defender_y_error > 0) defence_angle = 180;
            else defence_angle = 0;
            
            if(Robot::ball_abs_angle > 0) 
              error_angle = 90;
            else
              error_angle = -90;
            
            error_speed = my_abs(defender_y_error) * k_error;
            move_angle = sum_of_vectors(defence_angle, error_speed, error_angle, 20);
            move_speed = get_len_from_sum_of_vectors();
            
            if(move_speed > 80) move_speed = 80;
          }
          else
          {
            move_speed = 0;
          }
          Robot::moveRobotAbs(move_angle, move_speed);
        }
        else if(/*prediction*/true)
        {
          //if(my_abs(Robot::_defender_predicted_x) < 30)
          Robot::moveToPoint(Robot::_defender_predicted_x, 40, -1);
        }
        Robot::setAngle(0, 20);
      }
      
    }
    
    Robot::update();
  }
  
}
  
