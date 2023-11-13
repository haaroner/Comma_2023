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
    
  pin spi3_mosi('C', 12, spi3);
  pin spi3_sck('C', 10, spi3);
  pin OLED_DC('A', 15, write_UP);
  pin spi3_nss('E', 0, write_UP);
  pin OLED_res('C', 11, write_UP);
  
  pin usart6_tx('C', 6,  uart6);	
  pin usart6_rx('C', 7,  uart6);    
    
  pin motors_move('B', 1, read_UP);
  pin gyro_reset('B', 0, read_UP);	
  pin cap_charge('B', 9, write_);
  pin cap_discharge('E', 0, write_);
  pin dribler_control('A', 3, dribler_);
  pin adc_voltage_pin('C', 5, adc);
  pin ball_sen('A', 7, adc);
    
  pin up('B', 5, read_UP);
  pin reset_gyro('B', 6, read_UP);
  pin enter('B', 7, read_UP);
    
  camera camera(usart6_tx, usart6_rx);
    
  mpu9250_spi mpu_sensor(spi2_ck);
  IMU mpu(mpu_sensor);
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
  
  volatile uint32_t time, button_timers[3], blinking_timer = 0, blinking_durations;
  
  int16_t move_angle = 0, angular_speed = 0, max_angular_speed, gyro = 0,
  robot_x = 0, robot_y = 100, ball_loc_angle = 0, ball_abs_angle = 0, ball_loc_x = 0,
  ball_loc_y = ball_loc_x = 20, forward_angle = 0, backward_angle = 180;
  
  uint16_t ball_distance = 20, forward_distance = 100, backward_distance = 100;
  
  uint8_t move_speed = 0;
  bool side = 0, buttons_data[3] = {1, 1, 1}, buttons_old_data[3] = {1, 1, 1}, 
  pressed_buttons[3] = {0, 0, 0}, blinking_leds[3] = {0, 0, 0}, leds_state = 0, motors_state = 0;
  
  void init_robot(uint8_t role)
  { 
    time_service::init();
    time_service::startTime();
    
    control_led(0, OFF);
    
    usart2::usart2Init(115200, 8, 1);//gyro
    usart6::usart6Init(460800, 8, 1);//camera
       
    mpu.init();
    mpu.update();
  }
  
  
  void motors_on_off(bool _state)
  {
    motors_state = _state; 
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
    switch(_led_num)
    {
      case 0: blinking_leds[0] = ON; blinking_leds[1] = ON; blinking_leds[2] = ON; break;
      case 1: blinking_leds[0] = ON; break;
      case 2: blinking_leds[1] = ON; break;
      case 3: blinking_leds[2] = ON; break;
    }
    
    blinking_durations = _duration;
  }
  
  void moveRobot(int16_t _angle, uint8_t _speed)
  {
    move_angle = lead_to_degree_borders(_angle);
    if(_speed > 100) _speed = 0;
    move_speed = _speed;
  }
  void rotateRobot(int16_t _angular_speed, int16_t _max_angular_speed)
  {
    if(_angular_speed > 100) _angular_speed = 0;
    angular_speed = _angular_speed;
    
    max_angular_speed = _max_angular_speed;
  }
  
  void callibrate_gyro()
  {
    motors.moveRobot(0, 0, 0, 0, 0, 0);
    control_led(3, ON);
    time_service::delay_ms(500);
    mpu.calibrate(1000);
    mpu.setZeroAngle();
    control_led(3, OFF);
    motors_on_off(OFF);
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
        control_led(i + 1, ON);
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
  
  void update()
  {
    time = time_service::getCurTime();
    mpu.update();
    gyro = lead_to_degree_borders(mpu.getAngle());
    
    camera.getData();
    camera.calculate_pos(gyro, side);
    
    robot_x = camera.get_x();
    robot_y = camera.get_y();
    
    ball_loc_angle = camera.get_ball_angle();
    ball_abs_angle = camera.get_abs_ball_angle();
    ball_distance = camera.get_ball_distance();
    
    ball_loc_x = camera.get_ball_loc_x();
    ball_loc_y = camera.get_ball_loc_y();
    
    forward_angle = camera.get_forward_angle();
    forward_distance = camera.get_forward_distance();
    backward_angle = camera.get_backward_angle();
    backward_distance = camera.get_backward_distance();
    
    check_buttons();
    if(time - blinking_timer > blinking_durations)
    {
      leds_state = my_abs(leds_state - 1);
      for(int i = 0; i < 3; i++)
      {
        if(blinking_leds[i])
        {
          control_led(i + 1, leds_state);
          
        }
      }
      blinking_timer = time;
    }
    
    if(motors_state)
      motors.moveRobot(move_speed, max_angular_speed, move_angle, angular_speed, time, 0);
    else
      motors.disableMotors();
  }
}
