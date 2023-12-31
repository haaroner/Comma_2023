#pragma once

#include "Settings.h"
#include "libs.h"
#include "tools.h"

namespace Robot
{
  
  pin led1('A', 0, write_UP); 
  pin led2('A', 1, write_UP); 
  pin led3('A', 2, write_UP); 
  
  Motor m1('D', 12, tim4, CHANNEL1, 'D', 13, tim4, CHANNEL2);					
  Motor m3('B', 3, tim2, CHANNEL2, 'B', 4, tim3, CHANNEL1);
  Motor m2('E', 13, tim1, CHANNEL3, 'E', 14, tim1, CHANNEL4);
  Motor m4('D', 14, tim4, CHANNEL3, 'D', 15, tim4, CHANNEL4);
  motors motors(m1, m2, m3, m4, 100, 7);//1.5
  
  pin spi2_sck('B', 13, spi2);
  pin spi2_mosi('B', 15, spi2);
  pin spi2_miso('B', 14, spi2);
  pin spi2_ck('B', 12, write_DOWN);
  
  pin usart6_tx('C', 6,  uart6);	
  pin usart6_rx('C', 7,  uart6);    
    
  pin motors_move('B', 1, read_UP);
  pin gyro_reset('B', 0, read_UP);	
  pin cap_charge('B', 9, write_);
  pin cap_discharge('E', 0, write_);
  pin dribler_control('A', 3, dribler_);
  pin adc_voltage_pin('C', 5, adc);
  pin ball_sen('A', 7, adc);
    
  pin tssp_write_4('D', 8, write_DOWN);
  pin tssp_write_3('D', 9, write_DOWN);
  pin tssp_write_2('D', 10, write_DOWN);
  pin tssp_write_1('D', 11, write_DOWN);
  
  pin tssp_left_read('C', 3, read_DOWN);
  pin tssp_right_read('C', 2, read_DOWN);  
  
  pin up('B', 5, read_UP);
  pin reset_gyro('B', 6, read_UP);
  pin enter('B', 7, read_UP);
  
  pin spi3_mosi('C', 12, spi3);
  pin spi3_sck('C', 10, spi3);
  pin spi3_dc('A', 15, write_DOWN);
  pin spi3_cs('E', 0, write_DOWN);
  pin spi3_rst('C', 11, write_DOWN);
  
  SSD1306 display(3, spi3_dc, spi3_rst, spi3_cs, 1, 1, 1);
    
  camera camera(usart6_tx, usart6_rx);
    
  TSSP ball(digital ,tssp_write_4, tssp_write_3, tssp_write_2, 
  tssp_write_1, tssp_left_read, tssp_right_read, 1//,// tssp_left_read_dma, 
  /*tssp_right_read_dma*/);
  mpu9250_spi mpu_sensor(spi2_ck);
  IMU mpu(mpu_sensor);
  
  PID angle_pid(-0.2, -0.005, 0, 5);
  
  void init_robot(uint8_t role);
  void callibrate_gyro();
  void control_led(uint8_t _led_num, bool _data);
  void set_blinking(uint8_t _led_num, uint32_t _duration);
  void moveRobot(int16_t _angle, uint8_t _speed);
  void rotateRobot(int16_t _angular_speed);
  void check_buttons();
  bool check_button(uint8_t _button_num);
  void change_side();
  void update();

  int a = 0;
  
  volatile uint32_t time, button_timers[3], blinking_timer = 0, blinking_durations,
    init_timer = 0, display_timer = 0;
  
  volatile int move_angle = 0, angular_speed = 0, max_angular_speed, gyro = 0,
  robot_x = 0, robot_y = 100, ball_loc_angle = 0, ball_abs_angle = 0, ball_loc_x = 0,
  ball_loc_y = ball_loc_x = 20, forward_angle = 0, backward_angle = 180, ball_angle,
  moving_point[4] = {0, 0, 0, 0};
  
  int ball_distance = 20, forward_distance = 100, backward_distance = 100, point_distance = 0, 
    old_b_x = 0, old_b_y = 0, ball_abs_x = 0, ball_abs_y = 0,
  _dS = 0, _x1b = 0, _y1b = 0, _defender_predicted_x = 0, _defender_predicted_y = 0;
  
  double _dxb = 0, _dyb = 0;
  
  uint8_t move_speed = 0;
  
  double _alpha = 0;
  unsigned select_arrow_y = 0, menu_level = 0, main_menu_arrow_y = 0;
  
  bool side = 0, buttons_data[3] = {1, 1, 1}, buttons_old_data[3] = {1, 1, 1}, 
  pressed_buttons[3] = {0, 0, 0}, blinking_leds[3] = {0, 0, 0}, leds_state = 0, motors_state = 0,
  init_image = true, _game_state = 0, _callibrated = 0, moving_to_point = false;
  
  void init_robot(uint8_t role)
  { 
    time_service::init();
    time_service::startTime();
    
    control_led(0, OFF);
    
    usart2::usart2Init(115200, 8, 1);//gyro
    usart6::usart6Init(460800, 8, 1);//camera
       
    display.begin();
    //display.clear();
    //drawString(display, "Comma,", 6, 2);
    display.display();
    
    init_timer = time;
    
    mpu.init();
    mpu.update();
  }
  
  void check_buttons()
  {
    buttons_data[0] = up.read();
    buttons_data[1] = reset_gyro.read();
    buttons_data[2] = enter.read();
    for(int i = 0; i < 3; i++)
    {
      if(buttons_data[i] == 0 && buttons_old_data[i] == 1)//is pressed
        button_timers[i] = time;
      if(time - button_timers[i] > BUTTON_MIN_PRESSING_TIME_MS && buttons_data[i] == 1 && buttons_old_data[i] == 0)
      {
        pressed_buttons[i] = 1;
        button_timers[i] = time;
      }
    }
    buttons_old_data[0] = buttons_data[0];
    buttons_old_data[1] = buttons_data[1];
    buttons_old_data[2] = buttons_data[2];
  }
  
  bool check_button(uint8_t _button_num)
  {
    a = pressed_buttons[_button_num - 1];
    pressed_buttons[_button_num - 1] = 0;
    return a;
  }
    
  void change_side()
  {
    side = my_abs(side - 1);
  }
    
  void change_game_state()
  {
    if(_callibrated)
      _game_state = my_abs(_game_state - 1);
  }

  void motors_on_off(bool _state)
  {
    motors_state = _state; 
  }
  
  void callibrate_gyro()
  {
    motors.moveRobot(0, 0, 0, 0, 0, 0);
    control_led(2, ON);
    time_service::delay_ms(200);
    mpu.calibrate(1000);
    mpu.setZeroAngle();
    control_led(2, OFF);
    motors_on_off(OFF);
    _callibrated = true;
  }
  
  inline void display_draw_string(const char* data, unsigned x = 0, unsigned y = 0)
  {
    drawString(display, data, x, y);
  }
  
  template <typename N>
    
  inline void display_draw_number(N data,unsigned num_of_digits_after_dot = 3, unsigned x = 0, unsigned y = 0)
  {
    printTml(display, data, num_of_digits_after_dot, x, y);
  }
  
  template <typename N>
  
  void display_data(const char* str_data, unsigned _str_len, N num_data,  unsigned num_of_digits_after_dot = 3, unsigned x = 0, unsigned y = 0)
  {
    drawString(display, str_data, x, y);
    printTml(display, num_data, num_of_digits_after_dot, x + _str_len, y);
  }  
  
  inline void display_clear()
  {
    display.clear();
  }
  
  inline void display_update(bool auto_clear = true)
  {
    if(time_service::getCurTime() - init_timer > 1000 && (time_service::getCurTime() - display_timer) > 30) //30fps
    {
      if(init_image) display.clear();//to clear before first .display()
      display.display();
      if(auto_clear) display.clear();// to clear buffer for following .display()
      display_timer = time_service::getCurTime();
      init_image = false;
    }
  }
  
  void draw_menu()
  {
    display_clear();
    if(_game_state)
    {
      if(check_button(ENTER_BUTTON))
        change_game_state();
      
      display_data("Gyro & gates", 11, gyro, 1, 0, 0);
      display_draw_number(lead_to_degree_borders(forward_angle - gyro), 1, 14, 0);
      display_data("Robot XY", 9, robot_x, 1, 0, 1);
      display_draw_number(robot_y, 1, 12, 1);
      
      display.drawLine(64, 40, constrain(128, 0, _x1b + 64), 64);
    }
    else
    {
      if(check_button(UP_BUTTON))
        select_arrow_y = constrain(4, 0, select_arrow_y - 1);
      if(check_button(DOWN_BUTTON))
        select_arrow_y = constrain(4, 0, select_arrow_y + 1);
      if(menu_level == 0)
      {
        if(check_button(ENTER_BUTTON))
        {
          switch(select_arrow_y)
          {
            case 0: callibrate_gyro(); break;
            case 1: change_side(); break;
            case 2: change_game_state(); break;
            case 4: 
              menu_level = 11;
              main_menu_arrow_y = select_arrow_y;
              select_arrow_y = 0;
            break;
          }
        }
        
        display_data("Gyro & gates", 11, gyro, 1, 0, 0);
        display_draw_number(lead_to_degree_borders(forward_angle - gyro), 1, 14, 0);
        if(side)
          display_draw_string("Side  Yellow", 0, 1);
        else
          display_draw_string("Side  Blue", 0, 1);
        display_draw_string("RUN", 0, 2);
        display_data("Robot XY", 9, robot_x, 1, 0, 3);
        display_draw_number(robot_y, 1, 12, 3);
        display_draw_string("Test data", 0, 4);
      }
      else if(menu_level == 11)
      {
        if(check_button(ENTER_BUTTON)) 
        {
          switch(select_arrow_y)
          {
            case 0: 
              menu_level = 0;
              select_arrow_y = main_menu_arrow_y;
              break;
          }
        }
        
        display_draw_string("...", 0, 0);
        display_data("Ball abs XY", 11, ball_abs_x, 1, 0, 1);
        display_draw_number(ball_abs_y, 1, 14, 1);
        display_data("Predicted_x", 11, _x1b, 1, 0, 2);
      }
      display_draw_string("<", 17, select_arrow_y);
    }
    display_update();
  }
  void control_led(uint8_t _led_num, bool _data)
  {    
    switch(_led_num)
    {
      case 0: led1.write(_data); led2.write(_data); led3.write(_data); break;
      case 1: led1.write(_data); break;
      case 2: led2.write(_data); break;
      case 3: led3.write(_data); break;
    }
  }
  
  
  void set_blinking(uint8_t _led_num, uint32_t _duration)
  {
   for(int i = 0; i < 3; i++)
      blinking_leds[i] = OFF;
    if(_duration > 0)
    {
      switch(_led_num)
      {
        case 0: blinking_leds[0] = ON; blinking_leds[1] = ON; blinking_leds[2] = ON; break;
        case 1: blinking_leds[0] = ON; break;
        case 2: blinking_leds[1] = ON; break;
        case 3: blinking_leds[2] = ON; break;
      }
    }
    else
    {
      control_led(_led_num, OFF);
    }
    blinking_durations = _duration;
  }
  
  void moveRobot(int16_t _angle, uint8_t _speed)
  {
    move_angle = lead_to_degree_borders(_angle);
    if(_speed > 100) _speed = 0;
    move_speed = _speed;
  }
  
  void moveRobotAbs(int16_t _angle, uint16_t _speed)
  {
    move_angle = lead_to_degree_borders(_angle - gyro);
    if(_speed > 100) _speed = 100;
    move_speed = _speed;
  }
  
  int getDistanceToPoint(int _x, int _y)
  {
    return get_distance_to_point(robot_x, robot_y, _x, _y);
  }
  
  bool moveToPoint(int32_t _x, int32_t _y, int16_t _speed)
  {
    moving_point[0] = _x;
    moving_point[1] = _y;
    moving_point[2] = 0; //choosing angle will be added in the future
    moving_point[3] = _speed;
    //move_angle = get_angle_to_point(robot_x, robot_y, _x, _y);
    //point_distance = get_distance_to_point(robot_x, robot_y, _x, _y);
    // -1 - speed from reg
    // 0 - turn to point
    // 1 - standart speed
    if(_speed == 0 || point_distance < 5) move_speed = 0;
    else if(_speed == -1) move_speed = point_distance * 1.25;
    else move_speed = _speed;
    
    moveRobotAbs(move_angle, move_speed);
    
    return point_distance < 20;
  }
  void rotateRobot(int16_t _angular_speed, int16_t _max_angular_speed)
  {
    angular_speed = _angular_speed;
    max_angular_speed = _max_angular_speed;
  }
  
  void setAngle(int16_t x0_angle, int16_t _max_angular_speed)
  {
    angular_speed = angle_pid.calculate(x0_angle, gyro, 180);
    max_angular_speed = _max_angular_speed;
  }
  
  void predict()
  {
   _dS = sqrt(my_pow(_dxb, 2) + my_pow(_dyb, 2));
   _alpha = atan2(_dxb, _dyb);
    
   if(_dS > 15 && _dyb > 0)
   {
     _y1b = ball_abs_y - 40;
     _x1b = tan(_alpha) * _y1b;
     if(my_abs(_x1b + ball_abs_x) < 40)
     {
      _defender_predicted_x = _x1b * 1.2 + ball_abs_x;
      _defender_predicted_y = 40;
     }
   }
   else
   {
    if(my_abs(ball_abs_x) < 40)
    {
      _defender_predicted_x = ball_abs_x;
      _defender_predicted_y = 40;
    }
   }
   
  }
  
  void update()
  {
    time = time_service::getCurTime();
    mpu.update();
    gyro = lead_to_degree_borders(mpu.getAngle());
    
    ball.get_data();
    //ball_angle = lead_to_degree_borders(ball.get_angle() + gyro);
    
    camera.getData();
    camera.calculate_pos(gyro, side);
    
    robot_x = camera.get_x();
    robot_y = camera.get_y();
    
    ball_loc_angle = camera.get_ball_angle();
    ball_abs_angle = camera.get_abs_ball_angle();
    ball_distance = ball.get_distance();
    
    ball_loc_x = camera.get_ball_loc_x();
    ball_loc_y = camera.get_ball_loc_y();
    
    ball_abs_x = camera.get_ball_abs_x();
    ball_abs_y = camera.get_ball_abs_y();
    
    old_b_x = camera.get_old_b_x();
    old_b_y = camera.get_old_b_y();
    
    _dxb = camera.get_dbx();
    _dyb = camera.get_dby();
    
    forward_angle = camera.get_forward_angle();
    forward_distance = camera.get_forward_distance();
    backward_angle = camera.get_backward_angle();
    backward_distance = camera.get_backward_distance();
    
    check_buttons();
    if(time - blinking_timer > blinking_durations)
    {
      leds_state = my_abs(leds_state - 1);
      for(int i   = 0; i < 3; i++)
      {
        if(blinking_leds[i])
        {
          control_led(i + 1, leds_state);
          
        }
      }
      blinking_timer = time;
    }
    
    if(motors_state)
    {
      if(moving_to_point)
      {
        move_angle = get_angle_to_point(robot_x, robot_y, moving_point[0], moving_point[1]);
        point_distance = get_distance_to_point(robot_x, robot_y, moving_point[0], moving_point[1]);
        // -1 - speed from reg
        // 0 - turn to point
        // 1 - standart speed
        if(moving_point[3] == 0 || point_distance < 5) move_speed = 0;
        else if(moving_point[3] == -1) move_speed = point_distance * 1.25;
        else move_speed = moving_point[3];
      }
      motors.moveRobot(move_speed, max_angular_speed, move_angle, angular_speed, time, 0);
    }
    else
      motors.disableMotors();
  }
}
