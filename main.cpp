#include "Robot.h"
#include "Settings.h"
#include "usartik1.h"

#define TEST_DRIBLER false
#define TEST_MOTORS false
#define TEST_BLUETOOTH false
#define BALL_THRESHOLD 2

volatile double a = 0, b = 1, c = 10;
volatile int Putin = 0, Medved = 0;
uint32_t Mish = 0;
double defender_ball_tan = 0;
int defender_move_x = 0, defender_move_y = 0;
volatile int test1 = 0, test2 = 0, test3 = 0;

//timers
volatile uint32_t time = 0, ball_grab_timer = 0, super_timer = 0,
  point_reached_timer = 0, delay = 0, attacker_2_to_1_tim = 0, 
  attacker_3_to_1_tim = 0,
  out_action_tim = 0, defender_start_ball_track_tim = 0;

//general values
volatile uint8_t gaming_state;

int16_t move_angle = 0, move_speed = 0, gyro, forward_angle, backward_angle, 
  ball_loc_angle, ball_abs_angle, side_keck_angle;

uint16_t forward_distance, backward_distance, ball_distance;
int32_t robot_x = 0, robot_y = 80, ball_loc_x = 0, ball_loc_y = 20,
  ball_abs_x = 0, ball_abs_y = 100, point_distance = 0;

bool point_reached = false;
point robot_position, ball_abs_position;

point forward_gate_center, forward_gate_right_end, forward_gate_left_end; 

float bounds_k = 0;

//attacker values
volatile uint8_t role, attacker_state = 1, attacker_old_state = 1, attacker_trajectory_type;
volatile uint32_t out_diving_start_tim = 0, out_diving_stop_tim = 0,
  attacker_4_to_1_tim = 0, attacker_5_to_1_tim = 0, 
  attacker_start_2_state_tim = 0, attacker_angry_tim = 0,
  attacker_6_to_1_tim = 0, attacker_3_to_2_tim = 0, attacker_5_to_2_tim = 0;
uint8_t test_data[3];
volatile int16_t attack_angle = 0;
point attack_point;

volatile uint16_t out_of_bounds_victor = 0, ball_data = 0;

point attacker_defence_point;

bool out_diving = false;
  
point attacker_start_state_point; 

bool attacker_angry = false;

int8_t attack_side = 0;
  
//defender values
volatile uint8_t defender_state = 1, defender_old_state = 0;

int16_t defender_angle_error = 0, defender_y_error = 0,
defender_y_angle = 0;

point ball_track_position, backward_gate_center, ball_start_position,
defender_moving_point;

uint32_t defender_3_to_1_tim = 0, defender_1_to_4_tim = 0, defender_4_to_1_tim, 
  defender_ball_grab_tim = 0, defender_end_keck_time = 0, defender_end_keck_tim = 0,
  defender_2_to_1_tim = 0, defender_1_to_2_tim = 0;
 
bool ball_position_fixed = false;

//values for test
volatile uint16_t dribler_speed = 0, debug_state = 0;
volatile uint8_t to_keck = 0; 
volatile uint32_t pinok_delay = 20, pinok_wait = 7;

PID forward_out(0.25, 0, -2);
PID backward_out(0.25, 0, -2);
PID left_out(0.4, 0, -3);
PID right_out(0.4, 0, -3);

int main()
{
  Robot::init_robot();
  Robot::wait(2000);
  
   Adc adc_voltage(ADC1, 1, 15, RCC_APB2Periph_ADC1, Robot::battery_charge, 5);
  adc_voltage.sendMeChannel(15);
  
  Dma dma_voltage(RCC_AHB1Periph_DMA2, adc_voltage);
  dma_voltage.dmaInit(DMA2_Stream0, DMA_Channel_0, 1);
  dma_voltage.adcInitInDma(5);
    
  Adc ADC_ball_sen(ADC2, 1, 3, RCC_APB2Periph_ADC2, Robot::ball_sen_adc, 5);
  ADC_ball_sen.sendMeChannel(3);
//    
  Dma DMA_ball_sen(RCC_AHB1Periph_DMA2, ADC_ball_sen);
  DMA_ball_sen.dmaInit(DMA2_Stream2, DMA_Channel_1, 1);
  DMA_ball_sen.adcInitInDma(5);

  
   #if TEST_BLUETOOTH
    while(true)
    {
      Robot::wait(1);
      usartik1::abcde(255);
      time_service::delay_ms(1);
      usartik1::abcde(Putin / 2 + 100);
      time_service::delay_ms(1);
      usartik1::abcde(Medved / 2 + 100);
      time_service::delay_ms(1);
      usartik1::abcde((Putin / 2 + 100 + Medved / 2 + 100) / 2);
      //usartik1::abcde(10 / 2 + 100);
      time_service::delay_ms(1);
      test_data[0] = 200;
      test_data[1] = 100;
      test_data[2] = 10;
      //usartik1::abcde(Robot::crc8(test_data, 3));
      if(usartik1::available() > 0)
        b = usartik1::read();
      Robot::wait(500);
      //if(Putin > 99) Putin = 0;
      //Robot::display_data(b);
    }
  #endif
  
  #if TEST_MOTORS
  while(true)
  {
    Robot::motors_on_off(ON); 
    Robot::moveRobot(0, 100);
    Robot::rotateRobot(0, 0);
    gyro = Robot::gyro;
    robot_x = Robot::robot_x;
    robot_y = Robot::robot_y;
    time_service::delay_ms(1);
    
    Robot::update();
  }
  #endif
  Robot::wait(5000);
  
  #if TEST_DRIBLER
  while(true)
  {
    Robot::motors_on_off(ON);
    Robot::use_dribler(true);
    Robot::set_dribler_speed(dribler_speed);
    //Robot::moveRobot(0, 20);
    //Robot::set_dribler_speed(20);
//    if(time_service::getCurTime() - Mish > 2000)
//    {
//      Robot::control_led(0, ON);
//      Robot::Ilya();
//      Robot::control_led(0, OFF);
//      Mish = time_service::getCurTime();
//    }
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

  forward_gate_center.x = 0; forward_gate_center.y = 220;
  forward_gate_right_end.x = 12; forward_gate_right_end.y = 220;
  forward_gate_left_end.x = -12; forward_gate_left_end.y = 220;
  
  point left_attack_point;
  left_attack_point.x = -30; 
  left_attack_point.y = 172;
  
  point right_attack_point;
  right_attack_point.x = 27; 
  right_attack_point.y = 177;
  
  side_keck_angle = 140;
  
  point middleL;
        middleL.x = -35;
        middleL.y = 40;
        
        point sideL;
        sideL.x = -40;
        sideL.y = 25;    
        const int16_t L_stop_angle = 128;
        const uint16_t L_stop_y = 19;
        
        point middleR;
        middleR.x = 35;
        middleR.y = 40;    
        
        point sideR;
        sideR.x = 40;
        sideR.y = 25;
        const int16_t R_stop_angle = -128;
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
        
        point gate_right_point;
        gate_right_point.x = 30;
        gate_right_point.y = 0;
        
        point gate_left_point;
        gate_left_point.x = -30;
        gate_left_point.y = 0;
        
        point defender_detour_point;
        point defender_detour_left_point;
        point defender_detour_right_point;
 
  role = 2;/////////////////////////////////////////////////
  
  Robot::add_stop_to_route(0, 180);
  Robot::add_stop_to_route(30, 180);
  
  attacker_defence_point.x = 0;
  attacker_defence_point.y = 180;

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
    
    ball_data = Robot::ball_data;
    
    gaming_state = Robot::_game_state;
    
    if(gaming_state == 0)
    {
      //disable all motors as a safety precoution
      Robot::motors_on_off(OFF);
      Robot::use_dribler(false);
     // Robot::set_dribler_speed(0);
      
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
      Robot::display_data(ball_data, ball_abs_angle);
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
        Robot::motors.change_smoothness(6);
        int16_t max_robotx[2] = {87, -72};
        uint16_t max_forward_depth[2] = {193, 35};   
        
        /*attacker state change conditions*/
        
        //go to ball condition
        if(attacker_state == 1)
        {
          Robot::enable_trajectory(false);
          
          if(my_abs(ball_abs_angle - forward_angle) > 45 && robot_y < 150)
            attacker_angry_tim = time;
          
          if(time - attacker_angry_tim > 400) attacker_angry = true;
          else attacker_angry = false;
          
          if(ball_data < BALL_DETECTION_LIGHTNESS)
            ball_grab_timer = time;
          
          if(time - ball_grab_timer > 100 && attacker_old_state == 1)
          {
            Robot::set_dribler_speed(30);
            if(time - ball_grab_timer > 1300)
            {
              debug_state = 11;
              //Robot::wait(200);
              attacker_state = 2;
            }
          }
           
          //if((my_abs(ball_loc_angle + BALL_THRESHOLD) > 10) || ball_distance > 11) 
           // ball_grab_timer = time;
          
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
          {
            attacker_start_state_point = robot_position;
            if(time % 10 <= 6)
            {
              if(robot_x >= 0) attack_side = 1;
              else attack_side = -1;
            }
            else
            {
              if(robot_x >= 0) attack_side = -1;
              else attack_side = 1;
            }
          }
          
          //if ball moved out of dribler accidentally go to state 1
          if(ball_data > BALL_DETECTION_LIGHTNESS)
            attacker_2_to_1_tim = time;
          //if(ball_distance <= 20 && my_abs(ball_loc_angle) <= 20) attacker_2_to_1_tim = time;
          
          //if(Robot::is_ball_seen_T(100))
          if(time - attacker_2_to_1_tim > 1000) attacker_state = 1;
          
          out_diving = false;
        }
        
        if(attacker_state == 3)
        {
          attacker_2_to_1_tim = time;
          attacker_state = 2;
//          if(attacker_old_state != 3)
//            attacker_start_state_point = robot_position;
//          
//          if(ball_distance < 20 && my_abs(ball_loc_angle) < 20) attacker_3_to_1_tim = time;
//          
//          if(time - attacker_3_to_2_tim > 6000) attacker_state = 2;
//          
//          if(Robot::is_ball_seen_T(100))
//            if(time - attacker_3_to_1_tim > 100) attacker_state = 1;
//          
//          out_diving = false;
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
          attacker_2_to_1_tim = time;
          attacker_state = 2;
          if(ball_distance < 20 && my_abs(ball_loc_angle) < 20) attacker_5_to_1_tim = time;
          
          if(time - attacker_5_to_2_tim > 6000) attacker_state = 5;
          
          if(Robot::is_ball_seen_T(100))
            if(time - attacker_5_to_1_tim > 100) attacker_state = 1;
        }

        if(attacker_state == 6)
        {
          if(ball_distance < 20 && my_abs(ball_loc_angle) < 20) attacker_6_to_1_tim = time;
          
          if(Robot::is_ball_seen_T(100))
            if(time - attacker_6_to_1_tim > 100) attacker_state = 1;
        }
        
        /******attacker state bodies******/
        if(attacker_state == 1) //move to ball and grab it
        {      
          debug_state = 1;          
          Robot::enable_trajectory(false);
          if(ball_distance < 45 && my_abs(ball_loc_angle) < 20) 
            Robot::set_dribler_speed(13, false);
          else 
          {
            if(my_abs(lead_to_degree_borders(backward_angle + 180) - gyro) > 20)
              Robot::set_dribler_speed(0, false);
          }
          
          if(ball_data > BALL_DETECTION_LIGHTNESS)
          {
            Robot::set_dribler_speed(30, false);
          }
          
          if(time - ball_grab_timer > 100)
          {
            Robot::moveRobot(0, 14); 
            Robot::setAngle(ball_abs_angle + BALL_THRESHOLD, 10, -0.3);
          }
          else
          {
            if(ball_distance < 18.5)
            {
              move_speed = constrain(30, 15, 3 * (ball_distance - 8.7));
              if(my_abs(ball_loc_angle) > 30)
                move_speed *= 0.75;
            }
            else
              move_speed = constrain(70, 30, 1.7 * (ball_distance - 1));
            
            Robot::moveRobotAbs(ball_abs_angle, move_speed);
            Robot::setAngle(ball_abs_angle + BALL_THRESHOLD, 15, -0.3); //turn to ball 0.15
          }
//          if(!Robot::is_ball_seen_T(100))
//            Robot::constrainRobotSpeed(20, 0);
          
          if(my_abs(lead_to_degree_borders(Robot::abs_move_angle - forward_angle)) < 45 && forward_distance < 65)
          {
            Robot::setRobotSpeed(constrain(40, 12, (ball_distance - 8) * 2));
          }
          
          if(!Robot::is_ball_seen_T(1000))
          {
            Robot::moveToPoint(ball_abs_position, -1, Robot::getAngleToPoint(ball_abs_position));
          }
          
          if(my_abs(robot_x) > 40)
          {
            if(robot_x > 0 && Robot::abs_move_angle > 0)
              Robot::setRobotSpeed(constrain(30, 0, Robot::move_speed));
            else if(robot_x < 0 && Robot::abs_move_angle < 0)
              Robot::setRobotSpeed(constrain(30, 0, Robot::move_speed));
          }
  
          attacker_old_state = 1;       
        }       
        
        /******move to gates with ball******/       
        if(attacker_state == 2) 
        {     
          debug_state = 2;
          if(attacker_old_state == 1)
          {
            attacker_start_2_state_tim = time;
            if(my_sgn(attacker_start_state_point.x) == my_sgn(attack_side))
            {
              if(robot_y <= 120) Robot::add_stop_to_route(35 * my_sgn(attack_side), 135, -255, 0);
              attacker_trajectory_type = short_trajectory;
            }
            else
            {
              Robot::add_stop_to_route(20 * my_sgn(attacker_start_state_point.x), 130, -255, 0/*int(attacker_start_state_point.y / 2)*/);
              Robot::add_stop_to_route(50 * my_sgn(attack_side), 130, -255, 1);
              attacker_trajectory_type = long_trajectory;
            }
            
            if(my_sgn(attack_side) == 1)
              Robot::add_stop_to_route(right_attack_point.x, right_attack_point.y, -255, 1);
            else
              Robot::add_stop_to_route(left_attack_point.x, left_attack_point.y, -255, 1);
            
            Robot::enable_trajectory(true);
          } 
            
//          if(int64_t(time) - int64_t(attacker_start_2_state_tim) > 
//            constrain(9000, 6000, 50 * (Robot::start_trajectory_length - 55)))
//          {
//            Robot::enable_trajectory(false);
//            attack_side *= -1;
//            
//            Robot::add_stop_to_route(10, 120, -255, 0);
//            Robot::add_stop_to_route(44 * my_sgn(attack_side), 165, -255, 2);
//            Robot::enable_trajectory(true);
//            attacker_start_2_state_tim = time + 1000;
//          }
          
          if(Robot::trajectory.calculate_distance(robot_position) < 30)
            Robot::set_dribler_speed(40);
          else
            Robot::set_dribler_speed(35);
          if(Robot::trajectory.get_length() > 0)
              Robot::setAngle(lead_to_degree_borders(Robot::getAngleToPoint(attacker_defence_point) + 180), 8, -0.2);//14
          else
            Robot::setAngle(125 * my_sgn(attack_side), 8, -0.2);
          //          if(attacker_trajectory_type == short_trajectory || Robot::trajectory.get_length() < 60)
//            Robot::setAngle(90 * my_sgn(robot_x), 8, -0.3);
//          else if(attacker_trajectory_type == long_trajectory)
//          {
//           
//          }
          
          if(Robot::trajectory_finished)
          {
            Robot::ball_distance_disable_delay(false);
            Robot::enable_trajectory(false);
            Robot::set_dribler_speed(45);
            Robot::wait(100);
            //Robot::setAngle(0, 0, -0.1);
            
            //Robot::wait(100, true, 130 * my_sgn(attacker_start_state_point.x), 5);
            Robot::setAngle(0, 0, -0.4);
            //Robot::wait(250);
            if(my_sgn(attack_side) == 1)
              Robot::side_keck(140 * my_sgn(attack_side), 
                10 * my_sgn(attack_side), 15, 120  * my_sgn(attack_side));
            else
              Robot::side_keck(140 * my_sgn(attack_side), 
                10 * my_sgn(attack_side), 15, 120  * my_sgn(attack_side));
            attacker_state = 1;
            //ball_grab_timer = time_service::getCurTime();
            //Robot::ball_distance_disable_delay(false);
          } 
          attacker_old_state = 2;
        }
        
        
        if(attacker_state == 3)
        {       
          if(attacker_old_state == 1)
          {
            Robot::add_stop_to_route(40 * my_sgn(robot_x), 160, 90 * my_sgn(robot_x), 0);
            if(robot_x >= 0)
              Robot::add_stop_to_route(38 * my_sgn(robot_x), 200, 90 * my_sgn(robot_x), 2);
            else
              Robot::add_stop_to_route(-30, 205, 90 * my_sgn(robot_x), 2);
            Robot::enable_trajectory(true);
          }
          Robot::set_dribler_speed(35);
          //if(point_reached)
          if(Robot::trajectory_finished)
          {
            Robot::set_dribler_speed(15);
            Robot::enable_trajectory(false);
            Robot::ball_distance_disable_delay(true);
            Robot::setAngle(0, 0, -0.3);
            Robot::wait(2000, true, 90 * my_sgn(attacker_start_state_point.x), 12);
            Robot::wait(2000, true, -20 * my_sgn(attacker_start_state_point.x), 7);
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
          Robot::set_dribler_speed(constrain(35, 25, 0.55 * (forward_distance - 70)));
          
          if(my_abs(robot_x) <= 20) attack_point = forward_gate_center;
          if(robot_x > 20) attack_point = forward_gate_right_end;
          if(robot_x < -20) attack_point = forward_gate_left_end;
          
          Robot::moveRobotAbs(forward_angle, 80);
          if(my_abs(robot_x) <= 20)
            Robot::setAngle(0, 12, -0.5);
          else
            Robot::setAngle(Robot::getAngleToPoint(attack_point), 12, -0.5);
                   
          if((forward_distance < 65 && time - attacker_angry_tim > 1000 && my_abs(gyro - Robot::getAngleToPoint(attack_point)) < 25) ||
             (forward_distance < 50 && time - attacker_angry_tim > 1000))
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
              out_of_bounds_victor = forward_out.calculate_lineary(182, robot_y/* - int(20 * bounds_k)*/);
              move_angle = sum_of_vectors(move_angle, constrain(15, 0, move_speed),
              180, constrain(35, 0, out_of_bounds_victor));
              move_speed = get_len_from_sum_of_vectors();
            }
            else if(forward_distance < 45)
            {
              out_of_bounds_victor = forward_out.calculate_lineary(0, 45 - forward_distance/* - int(20 * bounds_k)*/);
              move_angle = sum_of_vectors(move_angle, constrain(15, 0, move_speed),
              lead_to_degree_borders(forward_angle + 180), constrain(35, 0, out_of_bounds_victor));
              move_speed = get_len_from_sum_of_vectors();
            }
            else if(robot_y > 190)
            {
              out_of_bounds_victor = forward_out.calculate_lineary(190, robot_y/* - int(20 * bounds_k)*/);
              move_angle = sum_of_vectors(move_angle, constrain(15, 0, move_speed),
              180, constrain(32, 0, out_of_bounds_victor));
              move_speed = get_len_from_sum_of_vectors();
            }
           
            
            Robot::moveRobotAbs(move_angle, move_speed);
          }

          
          //back gate
          if(robot_y < 20)
            Robot::moveRobotAbs(0, 12);
          
          if(robot_y < 60)
          {
            move_speed = Robot::move_speed;
            move_angle = Robot::abs_move_angle;
            
            if(my_abs(robot_x) <= 35)
            {
              out_of_bounds_victor = backward_out.calculate_lineary(60, 60 - robot_y);
              move_angle = sum_of_vectors(move_angle, constrain(15, 0, move_speed),
              0, constrain(35, 0, out_of_bounds_victor));
              move_speed = get_len_from_sum_of_vectors();
            }
            else if(backward_distance < 45)
            {
              out_of_bounds_victor = forward_out.calculate_lineary(0, 45 - backward_distance);
              move_angle = sum_of_vectors(move_angle, constrain(15, 0, move_speed),
              lead_to_degree_borders(backward_angle + 180), constrain(35, 0, out_of_bounds_victor));
              move_speed = get_len_from_sum_of_vectors();
            }
            else if(robot_y < 35)
            {
              out_of_bounds_victor = forward_out.calculate_lineary(30, 30 - robot_y/* - int(20 * bounds_k)*/);
              move_angle = sum_of_vectors(move_angle, constrain(15, 0, move_speed),
              0, constrain(32, 0, out_of_bounds_victor));
              move_speed = get_len_from_sum_of_vectors();
            }      
            Robot::moveRobotAbs(move_angle, move_speed);
          }
         
          
          if(robot_x > ROBOT_MAX_X)
          {
            bounds_k = constrainf(1, 0, 1 - my_abs(robot_y - 105) / 105);
            //out_of_bounds_victor = (robot_x - 85 - int(20 * bounds_k)) * 5;
            out_of_bounds_victor = right_out.calculate_lineary(ROBOT_MAX_X, robot_x/* - int(20 * bounds_k)*/);
            move_angle = sum_of_vectors(Robot::abs_move_angle, constrain(10, 0, Robot::move_speed),
              -90, constrain(25, 0, out_of_bounds_victor));
            move_speed = get_len_from_sum_of_vectors();
            
            Robot::moveRobotAbs(move_angle, move_speed);
          }
          
          if(robot_x < ROBOT_MIN_X)
          {
            bounds_k = constrainf(1, 0, 1 - my_abs(robot_y - 105) / 105);
            out_of_bounds_victor = left_out.calculate_lineary(ROBOT_MIN_X, -robot_x/* - int(20 * bounds_k)*/);
            move_angle = sum_of_vectors(Robot::abs_move_angle, constrain(10, 0, Robot::move_speed),
              90, constrain(25, 0, out_of_bounds_victor));
            move_speed = get_len_from_sum_of_vectors();
            
            Robot::moveRobotAbs(move_angle, move_speed);
          }    
      }
            
      //#######defender role########//
      if(role == 2)
      { 
        Robot::motors.change_smoothness(3);
        //***defender change state conditions***//
        if(defender_state == 1)
        {
          //if(backward_distance < 70 && robot_y < 70) defender_1_to_4_tim = time;
          //else if (time - defender_1_to_4_tim > 500) defender_state = 4;
          
          if(!Robot::is_ball_seen_T(1500) && my_abs(defender_angle_error) < 80) 
          {
            defender_state = 5;
            ball_start_position = ball_abs_position;
          }

          if((my_abs(get_angle_to_point(backward_gate_center, ball_abs_position).angle) > 70) ||
            get_angle_to_point(backward_gate_center, ball_abs_position).length > 100 ||
            my_abs(robot_x) > 55) 
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
            if(get_angle_to_point(ball_track_position, ball_abs_position).length > 30)
            {
              //refixe it
              defender_start_ball_track_tim = time;
              ball_position_fixed = false;
            }
            //if ball isnt moving go to keck state
            else if(time - defender_start_ball_track_tim > 2500 &&
                    time - defender_end_keck_time > 5000)
            {
              if(my_abs(defender_angle_error) < 12)
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
          defender_end_keck_tim = time;
          if(ball_abs_y < 45 || my_abs(ball_abs_angle) > 75) defender_state = 1;
          
          if(!Robot::is_ball_seen_T(1000))
          {
            defender_end_keck_time = time;
            defender_state = 1;
          }
          if(((time - defender_3_to_1_tim > 6500) && (!Robot::is_ball_captured(500))) ||
            ((time - defender_3_to_1_tim > 8500) && (Robot::is_ball_captured(500))))
            defender_state = 1;
          
          if((my_abs(get_angle_to_point(backward_gate_center, ball_abs_position).angle) < 70) &&
            my_abs(robot_x) < 55)          
              defender_end_keck_tim = time;
          else if(time - defender_end_keck_tim > 500)
          {
            defender_end_keck_time = time;
            defender_state = 1;
          }
          
          if(get_angle_to_point(ball_track_position, ball_abs_position).length > 50)
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
        
        if(defender_state == 6)
        {
          if(get_angle_to_point(backward_gate_center, ball_abs_position).length > 110)
            defender_state = 1;
          defender_old_state = 6;
        }
          
        if(defender_state != 3) defender_state = 1;
        
        //***defender state bodies***//
        if(defender_state == 1)
        {
          Robot::set_dribler_speed(0);
          defender_angle_error = lead_to_degree_borders(
          get_angle_to_point(backward_gate_center, ball_abs_position).angle - 
          lead_to_degree_borders(backward_angle + 180));
          
          
          if(middleR.x >= robot_x && robot_x >= middleL.x)
          {       
            defender_move_y = 40;
            defender_ball_tan = ball_abs_y / ball_abs_x;
            defender_move_x = ball_abs_x - (ball_abs_y - defender_move_y) / defender_ball_tan;
            defender_move_x = ball_abs_x * (defender_move_y / ball_abs_y);
            if(ball_abs_y <= 50)
            {
              defender_moving_point.x = ball_abs_x;
              defender_moving_point.y = 40;
            }
            else
            {
              defender_moving_point.x = defender_move_x;
              defender_moving_point.y = defender_move_y;
            }
            defender_moving_point.angle = Robot::getAngleToPoint(defender_moving_point);
            defender_moving_point.distance = Robot::getDistanceToPoint(defender_moving_point);
            
            if(defender_moving_point.distance <= 7)
              move_speed = constrain(15, 0, 2.8 * defender_moving_point.distance);
            else
              move_speed = constrain(70, 15, 2.3 * (defender_moving_point.distance));
            
            if(defender_moving_point.distance <= 5) move_speed *= 0;
            
            move_angle = defender_moving_point.angle;
            
          }
          else
          {
            //Robot::moveRobotAbs(0, 0);
            
            if(robot_x > middleR.x)
            {
              //move_speed = constrain(90, 8, pow(90 * my_abs(defender_angle_error), 0.5));
              //move_speed *= my_abs(defender_angle_error) >= 5;
              
              //if(defender_angle_error > 0)
                //move_angle = Robot::getAngleToPoint(sideR.x, sideR.y);
              //else if(defender_angle_error < 0)
                //move_angle = Robot::getAngleToPoint(middleR.x, middleR.y);
              
              
              
              //if(backward_angle >= R_stop_angle && defender_angle_error >= 0)
                //move_speed = 0;
              
              //if(backward_angle >= R_stop_angle - 10 && get_angle_to_point(backward_gate_center, ball_abs_position).angle >= R_stop_angle)
                //move_speed = 0;
              
              defender_moving_point = find_lines_crosspoint(get_line_from_points(middleR, sideR), 
              get_line_from_points(backward_gate_center, ball_abs_position));
            
              defender_moving_point.angle = Robot::getAngleToPoint(defender_moving_point);
              defender_moving_point.distance = Robot::getDistanceToPoint(defender_moving_point);
            
              move_angle = defender_moving_point.angle;
              if(defender_moving_point.distance <= 15)
                move_speed = constrain(10, 0, 2.8 * defender_moving_point.distance);
              else
                move_speed = constrain(60, 10, 8 * (defender_moving_point.distance - 5));
            
              
              if(robot_y <= R_stop_y)
              {
                move_angle = 0;
                move_speed = 15;
              }
              
              //Robot::setAngle(0, 12, -0.35);
              //Robot::moveRobotAbs(move_angle, move_speed);
            }
            
            if(robot_x < middleL.x)
            {           
              defender_moving_point = find_lines_crosspoint(get_line_from_points(middleL, sideL), 
              get_line_from_points(backward_gate_center, ball_abs_position));
            
              defender_moving_point.angle = Robot::getAngleToPoint(defender_moving_point);
              defender_moving_point.distance = Robot::getDistanceToPoint(defender_moving_point);
            
              move_angle = defender_moving_point.angle;
              if(defender_moving_point.distance <= 15)
                move_speed = constrain(10, 0, 2.8 * defender_moving_point.distance);
              else
                move_speed = constrain(60, 10, 8 * (defender_moving_point.distance - 5));
                         
              if(robot_y <= L_stop_y)
              {
                move_angle = 0;
                move_speed = 15;
              }
              
              //Robot::setAngle(0, 10, -0.3);
              //Robot::moveRobotAbs(move_angle, move_speed);
            }
          }
          defender_old_state = 1;
          Robot::setAngle(0, 20, -0.4);
          if(robot_y > 55)
          {
            move_speed = constrain(40, 0, move_speed);
            if(lead_to_degree_borders(Robot::getAngleToPoint(ball_abs_position) - Robot::getAngleToPoint(gate_left_point)) < 0 &&
              lead_to_degree_borders(Robot::getAngleToPoint(ball_abs_position) - Robot::getAngleToPoint(gate_right_point)) > 0)
              {
                if(ball_distance < 30)
                  move_speed = constrain(30, 0, move_speed);
                defender_detour_left_point.x = ball_abs_x - 25;
                defender_detour_left_point.y = ball_abs_y;
                
                defender_detour_right_point.x = ball_abs_x + 25;
                defender_detour_right_point.y = ball_abs_y; 

                if(defender_detour_left_point.x < -60)
                  defender_detour_point.x = defender_detour_right_point.x;
                else if(defender_detour_right_point.x > 60)
                  defender_detour_point.x = defender_detour_left_point.x;
                else
                {
                  if(Robot::getDistanceToPoint(defender_detour_right_point) < Robot::getDistanceToPoint(defender_detour_left_point))
                      defender_detour_point.x = defender_detour_right_point.x;
                  else
                      defender_detour_point.x = defender_detour_left_point.x;
                }
                defender_detour_point.y = ball_abs_y;
                move_angle = Robot::getAngleToPoint(defender_detour_point);
              }
          }
          Robot::moveRobotAbs(move_angle, move_speed);
        }
        
        if(defender_state == 2)
        {
          Robot::moveToPoint(Robot::predicted_point, 70);
          Robot::setAngle(0, 12, -0.35);
          defender_old_state = 2;
        }
        
        if(defender_state == 3)
        {
          //Robot::set_dribler_speed(10);
//          if(Robot::is_ball_captured(1200))
//          {
////            if(my_abs(robot_x) < 35)
////            {
////              Robot::direct_keck(20, 5);
////              defender_state = 1;
////              defender_end_keck_time = time;
////            }
////            else
////            {
////              defender_state = 6;
////            }
//            defender_state = 6;
//            Robot::set_dribler_speed(30);
//          }
//          else
//          {
            if((my_abs(ball_loc_angle + BALL_THRESHOLD) > 10) || ball_distance > 7) ball_grab_timer = time;
          
            if(time - ball_grab_timer > 700) 
            {
              Robot::direct_keck();
              defender_state = 1;
              defender_end_keck_tim = time;
            }
            
            
            Robot::moveRobotAbs(ball_abs_angle, constrain(40, 14, (ball_distance - 10) * 2));
            Robot::setAngle(ball_abs_angle + BALL_THRESHOLD, 14, -0.3); //turn to ball 0.15
          //}      
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
          if(ball_start_position.x > 20)
            Robot::moveToPoint(30, 45, -1, 0);
          else if(ball_start_position.x < -20)
            Robot::moveToPoint(-30, 45, -1, 0);
          else
            Robot::moveToPoint(0, 45, -1, 0);
//          Robot::moveRobotAbs(0, 0);
//          if(ball_start_position.x > 20)
//            Robot::moveToPoint(returnR.x, returnR.y, 40, 0);
//          else if(ball_start_position.x < -20)
//            Robot::moveToPoint(returnL.x, returnL.y, 40, 0);
          //else
            //Robot::moveToPoint(returnM, 40);
        }
        
        if(defender_state == 6)
        {
          Robot::set_dribler_speed(30);
          if(defender_old_state != 6)
          {
            Robot::enable_trajectory(true);
            if(my_abs(robot_x) <= 30)
              Robot::add_stop_to_route(30 * my_sgn(robot_x), 40, 160 * my_sgn(robot_x));
            Robot::add_stop_to_route(55 * my_sgn(robot_x), 40, 170 * my_sgn(robot_x));
            attack_side = my_sgn(robot_x);
          }
          
          if(Robot::trajectory_finished)
          {
            Robot::ball_distance_disable_delay(true);
            Robot::enable_trajectory(false);
            Robot::set_dribler_speed(45);
            Robot::wait(500);
            Robot::setAngle(0, 0, -0.4);
            Robot::wait(100, true, 160 * my_sgn(attack_side), 10);
            //Robot::wait(250);
            
            Robot::side_keck(-170 * my_sgn(attack_side), 
            40 * my_sgn(attack_side), 15, 150  * my_sgn(attack_side));
            
            defender_state = 1;
            //ball_grab_timer = time_service::getCurTime();
            Robot::ball_distance_disable_delay(false);
          }
          defender_old_state = 6;
          defender_end_keck_time = time;
        }
        if(robot_x > ROBOT_MAX_X)
          {
            bounds_k = constrainf(1, 0, 1 - my_abs(robot_y - 105) / 105);
            //out_of_bounds_victor = (robot_x - 85 - int(20 * bounds_k)) * 5;
            out_of_bounds_victor = right_out.calculate_lineary(ROBOT_MAX_X, robot_x/* - int(20 * bounds_k)*/);
            move_angle = sum_of_vectors(Robot::abs_move_angle, constrain(10, 0, Robot::move_speed),
              -90, constrain(25, 0, out_of_bounds_victor));
            move_speed = get_len_from_sum_of_vectors();
            
            Robot::moveRobotAbs(move_angle, move_speed);
          }
          
          if(robot_x < ROBOT_MIN_X)
          {
            bounds_k = constrainf(1, 0, 1 - my_abs(robot_y - 105) / 105);
            out_of_bounds_victor = left_out.calculate_lineary(ROBOT_MIN_X, -robot_x/* - int(20 * bounds_k)*/);
            move_angle = sum_of_vectors(Robot::abs_move_angle, constrain(10, 0, Robot::move_speed),
              90, constrain(25, 0, out_of_bounds_victor));
            move_speed = get_len_from_sum_of_vectors();
            
            Robot::moveRobotAbs(move_angle, move_speed);
          }    
      }
      Robot::display_data(defender_state, Robot::move_speed);
    } 
    Robot::update();
    //get_line_from_points(backward_gate_center, ball_abs_position)
    //get_line_from_points(middleR, sideR)
    //Robot::display_data(ball_abs_position.x, ball_abs_position.y);
    a = get_line_from_points(backward_gate_center, ball_abs_position).a;
    b = get_line_from_points(backward_gate_center, ball_abs_position).c;
  }
}