#include "project_config.h"
#include "pin_setup.h"
#include "motor.h"
#include "time_service.h"
#include "usart6.h"
#include "usart3.h"
#include "usart2.h"
#include "soft_i2c.h"
#include "motors.h"
#include "OpenMv.h"
#include "kicker.h"
//#include "MPU9250.h"
#include "SPI_1.h"
#include "SPI_2.h"
#include "SPI_3.h"
#include "ball_detour.h"
#include "usart2.h"
#include "hc-05.h"
//#include "SSD1306.h"
#include "robot_math.h"
#include "dribler.h"
#include "adc.h"
#include "dma.h"
#include "voltmeter.h"
#include "IMU_SPI.h"
#include "TSSP.h"
//#include "Buttons.h"
#include "tools.h"

#define CHANNEL1 1
#define CHANNEL2 2
#define CHANNEL3 3
#define CHANNEL4 4

#define OTLADKA 1 // 0 - not moving
#define COMPETITION 1 // 1 - 11V fault 0 - 9.5V fault
#define DRIBLER 0 // 0 - not using
#define BLUETOOTH 0 // 0 - not using
#define BALL_DETECTION_LIGHTNESS 1800
#define USE_GYRO_OFFSET 1 // not using
#define STANDART_DRIBLER_SPEED 345//345
#define MINIMAL_DRIBLER_SPEED 315//315
#define KECK_DRIBLER_SPEED 235//235
#define MIN_KECK_SPEED 265//265
#define STOP_DRIBLER_SPEED 295//295
#define TEST_DRIBLER 0 // 1 - test


int lead_to_degree_borders(int _num)
{
  while(_num < -180 || _num > 180)
  {
    if(_num < -180)
        _num += 360;
    else if(_num > 180)
        _num -= 360;
  }
  return _num;
}

int main(){
  time_service::init();
  time_service::startTime();
  
  volatile bool old_role, motor_move, seeker_state, dist_state, sub_dist, 
    gyro_reset_en, kick_en, see_ball, block_motor, robot = 0,
  attacker_change_state_en = 0, attacker_state_1_en = 0, attacker_state_2_en = 0, 
    attacker_state_0_en = 0, defenders_attack_lock = 0, defence_mode = 0,
    defence_en = 0, defence_mode_dis = 0, callibrate_en = 0;
  
  volatile float kp_angle = -0.7, ki_angle = 0.0002, kd_angle = 0,//-28 0.007
    angle_change_const = 180/3, voltage = 13.05;
  
  uint8_t gates_state, dist, long_dist, role;
  
  volatile int16_t robot_x, robot_y, gyro, x0_gyro, e_gyro, e_old, start_gyro;
  
  volatile int16_t gyro_zero_angle = 0, ball_angle, old_ball_angle, abs_ball_angle, robot_angle, forward_angle, forward_distance, backward_angle, 
    backward_distance, p_angle, i_angle, d_angle, u_angle, k_detour = 1,raw_gyro, old_gyro = 0, gyro_const = 10, gyro_offset = 0;
  
  volatile uint32_t time, dist_state_tim,kick_over_tim, d_timer, 
  angle_control_tim, out_kick_timer, end_out_kick_timer, enter_tim,
  hanging_timer, kick_tim, reset_gyro_tim, up_timer, test_tim, 
  start_pwm = 1500, test_pwm = 1500, start_timer, voltage_timer = 0, 
  attacker_change_state_tim, turn_on_tim, attacker_state_1_tim = 0, 
  attacker_state_2_tim = 0, attacker_state_0_tim = 0, take_ball_tim = 0,
  defence_time, gyro_zero_angle_tim = 0, defence_dis_tim = 0, 
  callibration_tim = 0, instant_start_timer = 0, test_rotate_timer = 0;
  //attacker
  volatile uint16_t speed_seeker, speed_angle, ball_distance, x0_backward_distance, 
    defender_y, dribling_speed = 300;
  
  //defender
  bool out_kick_en = false, hang_signal = 0, old_reset = 1, 
    my_gate = 1, kick_state = 0;
  
  volatile uint8_t attacker_state = 0, defender_state = 0, state, 
    old_state, voltage_fault = 0, take_ball_en = 2, bluetooth_data = 255,
    defence_state = 0;
  
  volatile int32_t x_result, y_result, x_ball, y_ball, x_error, y_error, 
    kp_error, error_angle, defence_angle, error_speed, result_angle, 
    move_speed, move_angle, ball_dist_formulka, gates_x0, p_seeker, e_seeker,
    backward_distance_e, x0_seeker, seeker, angle_speed, defender_error = 0;
  
  volatile double seeker_error;
  uint8_t game_state = 0, side = 1, use_dribler = 1;
  
  bool old_enter = 1, old_up = 1, old_down = 0, callibrated = false;
  
  float k_base_speed, k_angle_speed, k_dist_speed;
  double k_voltage = 0.00357;  int16_t playing_out_borders[2][2] = {{-22, 58}, {22, 150}}; //borders of central playing out zone
  
  
  pin power('E', 12, write_);
  power.resetBit();
	
  pin led1('A', 0, write_UP);
  pin led2('A', 1, write_UP);
  pin led3('A', 2, write_UP);
  
	Motor m1('D', 12, tim4, CHANNEL1, 'D', 13, tim4, CHANNEL2);					
	Motor m3('B', 3, tim2, CHANNEL2, 'B', 4, tim3, CHANNEL1);
  Motor m2('E', 13, tim1, CHANNEL3, 'E', 14, tim1, CHANNEL4);
	Motor m4('D', 14, tim4, CHANNEL3, 'D', 15, tim4, CHANNEL4);
    
	//m1.motorMove(2000);
  
  pin spi2_sck('B', 13, spi2);
  pin spi2_mosi('B', 15, spi2);
  pin spi2_miso('B', 14, spi2);
  pin spi2_ck('B', 12, write_DOWN);
  
  pin spi3_mosi('C', 12, spi3);
  pin spi3_sck('C', 10, spi3);
  pin OLED_DC('A', 15, write_UP);
  pin spi3_nss('E', 0, write_UP);
  pin OLED_res('C', 11, write_UP);
  OLED_res.resetBit();
  
	pin usart6_tx('C', 6,  uart6);	
  pin usart6_rx('C', 7,  uart6);
  
	
	pin motors_move('B', 1, read_UP);
	pin gyro_reset('B', 0, read_UP);	
  pin cap_charge('B', 9, write_);
  pin cap_discharge('E', 0, write_);
  pin dribler_control('A', 3, dribler_);//tim8!!! prsc = 419
  dribler_control.pwm(0);
  pin adc_voltage_pin('C', 5, adc);
  pin ball_sen('A', 7, adc);
  
  dribler_control.pwm(300);
  time_service::delay_ms(1);
  dribler_control.pwm(0);
  pin usart2_rx('D', 6, uart2);
  pin usart2_tx('D', 7, uart2);
  
  usart2::usart2Init(115200, 8, 1);//115200
  
  led1.setBit();
  led2.setBit();
  led3.setBit();

  mpu9250_spi mpu_sensor(spi2_ck);
  IMU mpu(mpu_sensor);
  mpu.init();
  
  turn_on_tim = time_service::getCurTime();
//  while(time_service::getCurTime() - turn_on_tim < 2000)
//  {
//    mpu.update();
//  }
  
  
  pin up('B', 5, read_UP);
  //pin down('B', 6, read_UP);
  pin enter('B', 7, read_UP);
  
  pin tssp_write_4('D', 8, write_DOWN);
  pin tssp_write_3('D', 9, write_DOWN);
  pin tssp_write_2('D', 10, write_DOWN);
  pin tssp_write_1('D', 11, write_DOWN);
  
  pin tssp_left_read('C', 3, read_DOWN);
  pin tssp_right_read('C', 2, read_DOWN);
  
  pin reset_gyro('B', 6, read_UP);
  
  //button reset_button(reset_gyro);
  
  Adc adc_voltage(ADC2, 1, 15, RCC_APB2Periph_ADC2, adc_voltage_pin);
  adc_voltage.sendMeChannel(15);
  Dma dma_voltage(RCC_AHB1Periph_DMA2, adc_voltage);
  dma_voltage.dmaInit(DMA2_Stream2, DMA_Channel_1, 1);
  dma_voltage.adcInitInDma(5);
  
  Adc ball_sen_adc(ADC1, 1, 7, RCC_APB2Periph_ADC1, ball_sen);
  ball_sen_adc.sendMeChannel(7);
  Dma ball_sen_dma(RCC_AHB1Periph_DMA2, ball_sen_adc);
  ball_sen_dma.dmaInit(DMA2_Stream0, DMA_Channel_0, 1);
  ball_sen_dma.adcInitInDma(5);
  
//  
//  Adc tssp_left_read_adc(ADC1, 1, 14, RCC_APB2Periph_ADC1, tssp_left_read);
//  tssp_left_read_adc.sendMeChannel(15);
//  Dma tssp_left_read_dma(RCC_AHB1Periph_DMA1, tssp_left_read_adc);
//  tssp_left_read_dma.dmaInit(DMA1_Stream0, DMA_Channel_0, 1);
//  tssp_left_read_dma.adcInitInDma();
//  
//  Adc tssp_right_read_adc(ADC2, 1, 14, RCC_APB2Periph_ADC2, tssp_right_read);
//  tssp_right_read_adc.sendMeChannel(15);
//  Dma tssp_right_read_dma(RCC_AHB1Periph_DMA2, tssp_right_read_adc);
//  tssp_right_read_dma.dmaInit(DMA2_Stream3, DMA_Channel_1, 1);
//  tssp_right_read_dma.adcInitInDma();
  
  role = 2; //1 - attacker!!!!!!!!!!!!!!!!!!!!!!
  
  //usart2::
  
//  if(role == 1)
//  {
////    dribler_control.pwm(200);
////    time_service::delay_ms(2000);
////    dribler_control.pwm(400);
////    time_service::delay_ms(2000);
//    time_service::delay_ms(2000);
//    dribler_control.pwm(300);
//    time_service::delay_ms(2000);
////    dribler_control.pwm(400);
////    time_service::delay_ms(2000);
//    //m1.motorMove(-500);
//  }
  

  
  
  //voltmeter voltmeter(dma_voltage);
  
  pin robot_change('B', 9, read_UP);
  
  pin usart3_tx('B', 10, uart3);
  pin usart3_rx('B', 11, uart3);
  
  usart3::usart3Init(9600, 8, 1);
  
  //pin test_dribler('A', 7, dribler_);
  
  camera camera(usart6_tx, usart6_rx);
  motors motors(m1, m2, m3, m4, 100, 2.5);//1.5
  
  TSSP ball(digital ,tssp_write_4, tssp_write_3, tssp_write_2, 
  tssp_write_1, tssp_left_read, tssp_right_read, role//,// tssp_left_read_dma, 
  /*tssp_right_read_dma*/);
    
  #if TEST_DRIBLER
    while(true)
    {
      time = ball_sen_dma.dataReturn(0);// * 0.00357;
      dribler_control.pwm(change_speed(dribling_speed, time_service::getCurTime()));
      ball.get_data();
      ball_angle = ball.get_angle();
      ball_distance = ball.get_distance();
    }
  #endif  
    
  kicker kicker(cap_charge, cap_discharge);
  
  speed_seeker = 0;
  motor_move = true;
  
  //mpu.calibrate(2000);
  mpu.update();
  start_gyro = mpu.getAngle();
  robot = (robot_change.getGPIOx()->IDR & robot_change.getPinNumber());
  start_timer = time_service::getCurTime();
  bluetooth_init(robot, 250);
  led1.setBit(); 
  led2.setBit();
  led3.setBit();
  move_speed = 0;
  
  while(true)
  {
    time = time_service::getCurTime();
    robot = 0;
    if(role != 1)
    {
      mpu.update();
      raw_gyro = mpu.getAngle();
      if(robot == 1)
      {
        if(old_gyro > 0 && raw_gyro < 0) gyro_offset = lead_to_degree_borders(gyro_offset - gyro_const);
        else if(old_gyro < 0 && raw_gyro > 0) gyro_offset = lead_to_degree_borders(gyro_offset + gyro_const);
      }
      if(USE_GYRO_OFFSET)
        gyro = lead_to_degree_borders(raw_gyro - gyro_offset);
      else
        gyro = lead_to_degree_borders(raw_gyro);
    }
    else
    {
      gyro = lead_to_degree_borders((usart2::read() * -2) - gyro_zero_angle);
    }
    
    ball.get_data();
    ball_angle = lead_to_degree_borders(ball.get_angle());
    abs_ball_angle = lead_to_degree_borders(ball_angle + gyro);
    ball_distance = ball.get_distance();
    
    camera.getData();
    camera.calculate_pos(gyro, side);
    robot_x = camera.get_x();
    robot_y = camera.get_y();
    forward_angle = camera.get_forward_angle();
    forward_distance = camera.get_forward_distance();
    backward_angle = camera.get_backward_angle();
    backward_distance = camera.get_backward_distance();
    gates_state = camera.get_data_state();
    if(game_state == 1)
    {
      if(role == 1)
      {
        if(attacker_state == 2 || take_ball_en == 2 || ball_sen_dma.dataReturn(0) > BALL_DETECTION_LIGHTNESS)
        {        
          usart3::write(100);
        }
        else
          usart3::write(255);
      }
      if(role == 0 || role == 2) usart3::write(255);
    }
    
//    bluetooth_send_data(game_state, defenders_attack_lock);//!!!!!!!!!!
//    bluetooth_read_data();
//    
//    defence_mode = get_defence_mode();//!!!!!!!!!!
//    defenders_attack_lock = is_attack_available();//change this before adding changing roles
    
    //semicolon_advise_send(game_state);
    
    //defence_mode = semicolon_advise_read();
    
     if(usart3::available() < 1 && BLUETOOTH)
     {
      if(defence_en == 0)
        defence_time = time;
      defence_en = 1;
      
      if(time - defence_time > 1500)
      {
        defence_mode = 1;
      }
      else
        defence_mode = 0;
    }
    else
    {
      if(BLUETOOTH) bluetooth_data = usart3::read();
      
      if(defence_mode == 1)
      {
        if(defence_mode_dis == 0)
          defence_dis_tim = time;
        defence_mode_dis = 1;
        if(time - defence_dis_tim > 1500)
          defence_mode = 0;
      }
      else
      {
        defence_mode = 0;
        defence_en = 0;
      }
    }
    if(time - callibration_tim < 20000)
      defence_mode = 0;
    
    voltage = float(dma_voltage.dataReturn(0) * k_voltage);
    
    kp_angle = -0.6;
    angle_speed = 25;
    
    motors.disableMotors();
    if(voltage > 6 && voltage < 11 && game_state == 0)
    {
      if(voltage_fault != 1) voltage_timer = time;
      if(voltage_fault != 2) voltage_fault = 1;
      
        
      if(time - voltage_timer > 2500)
      {
        while(voltage_fault == 1)
        {
          time = time_service::getCurTime();
          mpu.update();
          if(enter.read() != old_enter && time - enter_tim > 1000)
          {
            if(old_enter == 0)
              voltage_fault = 2;
            enter_tim = time;
          }
          old_enter = enter.read();
          if(time - hanging_timer > 500)
          {
            hang_signal = abs(double(hang_signal - 1));
            led1.write(hang_signal);
            led2.write(hang_signal);
            led3.write(hang_signal);
            hanging_timer = time;
          }
          motors.moveRobot(0, 0, 0, 0, time, time, 0);
        }
      }
    }
//    if((voltage < 10 && COMPETITION) || (voltage < 9.5 && !COMPETITION))
//    {
//      if(voltage_fault != 2) voltage_fault = 1;
//      while(voltage_fault == 1)
//      {
//        time = time_service::getCurTime();
//        mpu.update();
//        if(enter.read() != old_enter && time - enter_tim > 1000)
//        {
//          if(old_enter == 0)
//            voltage_fault = 2;
//          enter_tim = time;
//        }
//        old_enter = enter.read();
//        if(time - hanging_timer > 500)
//        {
//          hang_signal = abs(double(hang_signal - 1));
//          led1.write(hang_signal);
//          led2.write(hang_signal);
//          led3.write(hang_signal);
//          hanging_timer = time;
//        }
//        motors.moveRobot(0, 0, 0, 0);
//      }
//    }
    
//    if(time - voltage_timer > 100)
//    {
//      voltage = voltage * k_voltage + (1 - k_voltage) * voltmeter.get_voltage(dma_voltage.dataReturn(0));
//      voltage_timer = time;
//    }
        
    if(enter.read() != old_enter && time - enter_tim > 150)
    {
    if(old_enter == 0)
    {
      if(game_state == 0 && callibrated) 
      {
        game_state = 1;
//        if(robot_x > playing_out_borders[0][0] && robot_x < playing_out_borders[1][0] &&
//          robot_y > playing_out_borders[0][1] && robot_x < playing_out_borders[1][1])
//        time_service::delay_ms(2000);
      }
      else 
      {
        game_state = 0;
        voltage_fault = 0;
        led1.resetBit();
        led2.resetBit();
        led3.resetBit();
        motors.moveRobot(0, 0, 0, 0, time, time, 0);
        time_service::delay_ms(1);
      }
      instant_start_timer = time;
    }
      enter_tim = time;
    }
    old_enter = enter.read();
    led2.write(game_state);
    
    if(game_state == 0) //motors off state
    {
      //indication of acceleration of mpu
      dribler_control.pwm(change_speed(300, time_service::getCurTime()));
      if(callibrated)
      {
        if(abs(double(gyro)) < 5)    
          led3.setBit();
        else led3.resetBit();
      }
      else
      {
        if(my_abs(mpu.get_acceleration()) < 0.1)
          if(time - hanging_timer > 250)
          {
            hang_signal = abs(double(hang_signal - 1));
            led3.write(hang_signal);
            hanging_timer = time;
          }
      }
      
      if(my_abs(forward_angle - gyro) < 20)
        led1.setBit();
      else
        led1.resetBit();
      
      if(reset_gyro.read() != old_reset && time - reset_gyro_tim > 150)
      {
      if(old_reset == 0)
      {
        led2.setBit();
        if(role == 1)
        {
          gyro_zero_angle_tim = time;
          while(time_service::getCurTime() < 3000 || usart2::available() < 1) usart2::available();
          gyro_zero_angle = lead_to_degree_borders((usart2::read() * -2));
        }
        else
        {
          //motors.stopRobot(75);
          mpu.calibrate(100);
          mpu.setZeroAngle();
        }
          led2.resetBit();
        callibrated = true;
        callibration_tim = time_service::getCurTime();
      }
      reset_gyro_tim = time;
    }
    old_reset = reset_gyro.read();
    
    if(up.read() != old_up && time - up_timer > 150)
    {
      if(old_up == 0)
        side = abs(double(side - 1));
      up_timer = time;
    }
    old_up = up.read();
    
    //motors.disableMotors();
      
      //if(up_button.is_pressed())
        //role = abs(double(role - 1));
      
//      if(up.read() != old_reset && time - up_timer > 180)
//      {
//      if(old_up == 0)
//      {
//        //if(camera.get_raw_yellow_angle() < 10)
//        //{
//          //my_gate = abs(double(my_gate - 1));
//          //led1.setBit();
//        //}
//        //else
//          led1.resetBit();
//      }
//      up_timer = time;
//    }
      old_up = up.read();
      //led1.write(camera.get_raw_yellow_angle() < 10); //is turned on yellow gates
      
      //led2.write(role); //indicates role of robot
      
      //motors.disableMotors();
    }
    else if(game_state == 1) //game state
    {
      if(time - hanging_timer > 100)
      {
        hang_signal = abs(double(hang_signal - 1));
        led3.write(hang_signal);
        hanging_timer = time;
      }
      
      if(up.read() != old_up && time - up_timer > 150)
      {
        if(old_up == 0)
          use_dribler = abs(double(use_dribler - 1));
        up_timer = time;
      }
      old_up = up.read();
      
      if(role == 1 && (defence_mode == 0 || attacker_state == 2 || take_ball_en != 2 || 
        ((my_abs(ball_angle) < 30 || ball_sen_dma.dataReturn(0) > BALL_DETECTION_LIGHTNESS) && attacker_state == 0 && my_abs(forward_angle - gyro) < 35 && ball_distance > 3)))
      {
        if(my_abs(lead_to_degree_borders(abs_ball_angle - forward_angle)) < 90 || (ball_distance == 0 && attacker_state == 0)/* && forward_distance < 100) || (my_abs(abs_ball_angle) < 90)// && (forward_distance >= 100)))/* || 
          //(my_abs(forward_angle - gyro) < 90 && ball_sen_dma.dataReturn(0) > BALL_DETECTION_LIGHTNESS)*/)
        {  
          if(attacker_state_0_en == 0)
            attacker_state_0_tim = time;
          attacker_state_0_en = 1;
          if((attacker_state == 1 && (time - attacker_state_0_tim) > 350) || 
             (attacker_state == 2 && (time - attacker_state_0_tim) > 500))
          {
            attacker_state = 0;
            attacker_change_state_en = 0;
            attacker_state_1_en = 0;
            attacker_state_2_en = 0;
            //dribler_control.pwm(300);
          }
        }
        else
        {
          if(ball_sen_dma.dataReturn(0) < BALL_DETECTION_LIGHTNESS)
          {
             if(attacker_state_1_en == 0)
               attacker_state_1_tim = time;
             attacker_state_1_en = 1;
             if((attacker_state == 2 && (time - attacker_state_1_tim) > 750) || 
               (attacker_state == 0 && (time - attacker_state_1_tim) > 350))
             {
               attacker_state = 1;
               attacker_state_2_en = 0;
               attacker_state_0_en = 0;
             }
          }
          else   
          {
            if(attacker_state_2_en == 0)
               attacker_state_2_tim = time;
             attacker_state_2_en = 1;
             if((attacker_state == 1 && (time - attacker_state_2_tim > 500)) || 
               (attacker_state == 0 && (time - attacker_state_2_tim > 500)))
             {
               attacker_state = 2;
               attacker_state_1_en = 0;
               attacker_state_0_en = 0;
             }
          }
            
        }
        //if(use_dribler == 0 || DRIBLER == 0) attacker_state = 0;
        attacker_state = 0;
        if(attacker_state == 0)
        {
          
          x0_gyro = forward_angle;
          
          if(robot_y < 70) x0_gyro = 0;
          
          
          
            if(ball_sen_dma.dataReturn(0) > BALL_DETECTION_LIGHTNESS)
            {
              if(robot_y < 65)
                move_angle = lead_to_degree_borders(0 - gyro);
              else
                move_angle = lead_to_degree_borders(forward_angle - gyro);
              move_speed = 95;
            }
            else
            {
              move_angle = ball_angle + exponential_detour(ball_angle, ball_distance, 0.066, 0.35, 0.0255, 4.1);// 0.07, 0.37, 0.026, 4.2
              move_speed = 90;
              //move_angle = lead_to_degree_borders(move_angle - gyro);
            }
            //move_angle = abs_ball_angle + exponential_detour(abs_ball_angle, ball_distance, 0.07, 0.35, 0.023, 4.5);
          //if(ball_distance >= 3)
          //{
          //}
          //else
//          {
//            if(ball_distance > 0)
//            {
//              if(my_abs(lead_to_degree_borders(abs_ball_angle - forward_angle)) < 20)
//                x0_gyro = abs_ball_angle + 20;
//              else
//              {
//                if(robot_y < 50)
//                  x0_gyro = 0;
//                else
//                  x0_gyro = forward_angle;
//              }
//            }
            
            
         // }
          kp_angle = -0.35;//-0.45
          
//          if(my_abs(ball_angle) < 21 && my_abs(forward_angle - gyro) < 21)
//            move_speed = 95;
//          else
//            move_speed = 90;
            
          
          if((robot_x > 25 && abs_ball_angle > 0 ) || (robot_x < -25 && abs_ball_angle < 0))
            move_speed = 30;
          else
          {
            if(forward_distance < 70)
              move_speed = 70;
            if(ball_distance > 4 && my_abs(ball_angle) > 20 && robot_y > 135)
              move_speed = 65;
            if(ball_distance > 4 && my_abs(ball_angle) > 20 && robot_y <= 135)
              move_speed = 80;
          }     
          
          if(ball_distance >= 3 && my_abs(ball_angle) < 45)
            dribler_control.pwm(change_speed(MIN_KECK_SPEED, time_service::getCurTime()));
          else
            dribler_control.pwm(change_speed(STOP_DRIBLER_SPEED, time_service::getCurTime()));
          
//          if(my_abs(ball_angle) < 15 && ball_distance >= 3 &&(robot_y < 150 || (robot_y >= 150 && take_ball_en == 1)) && OTLADKA && DRIBLER)
//          {
//            take_ball_en = 1;
//            
//            dribler_control.pwm(MINIMAL_DRIBLER_SPEED);
//            move_angle = lead_to_degree_borders(forward_angle - gyro);
//            
//            if(ball_sen_dma.dataReturn(0) > BALL_DETECTION_LIGHTNESS)
//              move_speed = 95;
//          }
//          
//          if(forward_distance < 75 && (take_ball_en == 1 || take_ball_en == 0) && (my_abs(gyro - forward_angle) < 35 || forward_distance < 48) && OTLADKA && DRIBLER)
//          {
//            if(take_ball_en == 1)
//              take_ball_tim = time;
//            take_ball_en = 0;
//            
//            if(time - take_ball_tim < 750)
//              dribler_control.pwm(KECK_DRIBLER_SPEED);
//            else
//            {
//              take_ball_en = 2;
//              dribler_control.pwm(STOP_DRIBLER_SPEED);
//            }
//          }
//          
//          if(forward_distance < 55 && take_ball_en == 2 && my_abs(forward_angle - gyro) < 10 && my_abs(ball_angle) < 10)
//            dribler_control.pwm(MIN_KECK_SPEED);
          
//          if(ball_sen_dma.dataReturn(0) < BALL_DETECTION_LIGHTNESS)
//          {
//            kp_angle = -0.6;
//            angle_speed = 25;
//          }
//          else
//          {
//            kp_angle = -0.6;
//            angle_speed = 25;
//          }
//          if(my_abs(lead_to_degree_borders(gyro - forward_angle)) < 30 && my_abs(ball_angle) < 30 && forward_distance < 130 && DRIBLER != 0 && use_dribler)
//            dribler_control.pwm(250);
//          else
//            dribler_control.pwm(300);
          
        }
        else if(attacker_state == 1)
        {
          //if(ball_distance <= 2)
          // {
//            if(robot_y < 130)
//              move_angle = lead_to_degree_borders(0 - gyro);
//            else
//              move_angle = lead_to_degree_borders(180 - gyro);
//            x0_gyro = 20;
//          }
//          else
//          {
            //  game beginning behaviour
            move_angle = lead_to_degree_borders(ball_angle + exponential_detour(ball_angle, ball_distance, 0.07, 0.35, 0.023, 4.5));
            angle_speed = 30;
            if(robot_y > 150)
              x0_gyro = lead_to_degree_borders(forward_angle + 180);
            else
              x0_gyro = 180;
            //if(robot_y < 70 && my_abs(abs_ball_angle) > 145) x0_gyro = abs_ball_angle;
          //}
          if(OTLADKA == 0 || ball_distance < 1 || (!DRIBLER))
            dribler_control.pwm(change_speed(STOP_DRIBLER_SPEED, time_service::getCurTime()));
          else
            dribler_control.pwm(change_speed(340, time_service::getCurTime()));
          
          if(ball_distance > 1 || (backward_distance < 90 && my_abs(backward_angle - gyro) < 45))
            move_speed = 25;
          else
            move_speed = 40;
  
          
        }
        else if(attacker_state == 2)
        {
          attacker_change_state_tim = time;
          if(OTLADKA == 0 || (!DRIBLER))
          {
            dribler_control.pwm(STOP_DRIBLER_SPEED);
          }
          else
            dribler_control.pwm(STANDART_DRIBLER_SPEED);
          
          if(time - attacker_state_2_tim < 2000)
          {
            move_speed = 0;
            x0_gyro = lead_to_degree_borders(forward_angle + 180);
          }
         else
          {
            if(robot_y < 75)
            {
//              if(my_abs(robot_x) < 30)
//              {
//                if(robot_x > 0 && robot_x < 30) move_angle = lead_to_degree_borders(-90 - gyro); 
//                if(robot_x < 0 && robot_x > -30) move_angle = lead_to_degree_borders(90 - gyro); 
//                move_speed = 2000;
//              }
              
                
              if(robot_x > 0) x0_gyro = lead_to_degree_borders(forward_angle + 50);
              else x0_gyro = lead_to_degree_borders(forward_angle - 50);
              
              kp_angle = -30;
              angle_speed = 63;
              
                
            }
            else if(robot_y > 133)
            {
              move_angle = lead_to_degree_borders(forward_angle - gyro);
              
              if(my_abs(robot_x) > 25)
              {
                if(my_abs(lead_to_degree_borders(gyro - forward_angle)) > 90)
                {
                  if(robot_x > 0)
                  {
                    x0_gyro = lead_to_degree_borders(forward_angle - 50);
                  }
                  else
                  {
                    x0_gyro = lead_to_degree_borders(forward_angle + 50);
                  }
                }
                else
                  x0_gyro = lead_to_degree_borders(forward_angle + 50);
              }
              else
              {
                if(robot_x > 0)
                  x0_gyro = lead_to_degree_borders(forward_angle + 50);
                else
                  x0_gyro = lead_to_degree_borders(forward_angle - 50);
              }
              
              kp_angle = -0.6;
              angle_speed = 50;
              move_speed = 0;
              
//              if(my_abs(lead_to_degree_borders(forward_angle + 180) - gyro < 30) && robot_y > 133)
//              {
//                if(my_abs(forward_angle) < 25 && my_abs(forward_angle) > 15)
//                {
//                  if(robot_x > 0)
//                    x0_gyro = lead_to_degree_borders(forward_angle + 50);
//                  else
//                    x0_gyro = lead_to_degree_borders(forward_angle - 50);
//                  
//                  kp_angle = -30;
//                  angle_speed = 2500;
//                }
//              }
            }
            else
            {
              move_speed = 38;
              kp_angle = -0.375;
              angle_speed = 25;
              x0_gyro = lead_to_degree_borders(forward_angle + 180);
              
              if(my_abs(forward_angle) < 45 && my_abs(forward_angle) > 15)
                x0_gyro = 180;
              
              move_angle = lead_to_degree_borders(forward_angle - gyro);
            } 
          }
        }
        
        
//        if(robot_x > 53)
//        {
//          move_angle = lead_to_degree_borders(270 - gyro);
//          move_speed = 2000;
//        }
//        if(robot_x < -48)
//        {
//          move_angle = lead_to_degree_borders(90 - gyro);
//          move_speed = 2000;
//        }
//        if((forward_distance < 34 && my_abs(robot_x) <= 20) || (forward_distance < 41 && my_abs(robot_x) > 20))
//        {
//          move_angle = lead_to_degree_borders(180 - gyro);
//        }
//        if(forward_distance < 50)
//        {
//          move_speed = 2200;
//        }
        
//        if(ball_distance == 0 && ball_angle == 255 && ball_sen_dma.dataReturn(0) < BALL_DETECTION_LIGHTNESS)
//        {
//          //x0_gyro = 90;
//          if(robot_y < 65)
//          {
//            move_angle = lead_to_degree_borders(0 - gyro);
//            move_speed = 50;
//          }
//          else
//            move_speed = 0;
//        }  
        
        if((my_abs(robot_x) <= 20 && forward_distance < 54) ||
            (my_abs(robot_x) > 20 && forward_distance < 56))
          {
            if(attacker_state == 1 || attacker_state == 2)
              move_angle = ball_angle;
            
            if(my_abs(robot_x) <= 25)
               move_angle = sum_of_vectors(180, 68, move_angle, 15);
            else
               move_angle = sum_of_vectors(
  lead_to_degree_borders(forward_angle - 180), 68, move_angle, 15);
            move_speed = get_len_from_sum_of_vectors();
            
            move_angle = sum_of_vectors(move_angle, move_speed, 
            lead_to_degree_borders(180 - gyro), 15);
            move_speed = get_len_from_sum_of_vectors();
////              move_angle = lead_to_degree_borders(move_angle - gyro);
//            if(ball_distance > 0)
//            {
//              if(my_abs(robot_x) <= 20)
//                move_angle = lead_to_degree_borders(180 - gyro);
//              else
//                move_angle = lead_to_degree_borders(forward_angle - 180 - gyro);
//              
//              move_speed = 2000;
////              if(my_abs(robot_x) <= 20)
////                move_angle = sum_of_vectors(180, 2700, abs_ball_angle, 900);
////              else
////                move_angle = sum_of_vectors(
////  lead_to_degree_borders(forward_angle - 180), 2700, abs_ball_angle, 900);
////              move_speed = get_len_from_sum_of_vectors();
////              move_angle = lead_to_degree_borders(move_angle - gyro);
//            }
//            else
//            {
//              move_angle = lead_to_degree_borders(forward_angle - 180);
//              move_speed = 2500;
//            }
          }
          
          if((robot_x > 45 && attacker_state == 0 && robot_y > 100) || (robot_x > 44 && attacker_state == 0 && robot_y <= 100) || ((robot_x > 37 && attacker_state != 0))) 
          {
            move_angle = lead_to_degree_borders(-90 - gyro);
            if(attacker_state == 0) move_speed = 63;
          }
          else if((robot_x < -41 && attacker_state == 0 && robot_y > 100)|| (robot_x < -42 && attacker_state == 0 && robot_y <= 100) || (robot_x < -40 && attacker_state != 0))
          {
             move_angle = lead_to_degree_borders(90 - gyro);
             if(attacker_state == 0) move_speed = 63;
          }
          
        if(robot_y > 210)
        {
          move_angle = lead_to_degree_borders(180 - gyro);
        }
        
        if(robot_y < 60)
        {
          move_angle = lead_to_degree_borders(0 - gyro);
          
          if(my_abs(ball_angle) < 90 && my_abs(ball_angle) > 10)//!!!!!!!!!!!!!!
          {
            move_angle = ball_angle;
            
            if(attacker_state == 0) move_speed = 68;
          }
          else if(my_abs(ball_angle) < 125)
          {
            if(robot_y < 45)
            {
              if(ball_angle > 0)
                move_angle = lead_to_degree_borders(ball_angle + 10);
              else
                move_angle = lead_to_degree_borders(ball_angle - 10);
              move_angle = ball_angle;
              if(attacker_state == 0) move_speed = 60;
            }
            else
            {
              if(ball_angle > 0)
                move_angle = lead_to_degree_borders(ball_angle + 20);
              else
                move_angle = lead_to_degree_borders(ball_angle - 20);
              if(attacker_state == 0) move_speed = 43;
            }
          }      
        }
        
        if(robot_y < 47 || backward_distance < 47)
        {
            move_angle = lead_to_degree_borders(0 - gyro);
            if(attacker_state == 0) move_speed = 43;  
        }
        
        if(robot_y < 70) move_speed = 50;
        
        if(ball_distance <= 1)
        {
          //x0_gyro = 90;
          if(robot_y < 65)
          {
            move_angle = lead_to_degree_borders(0 - gyro);
            move_speed = 50;
          }
          else
            move_speed = 0;
        } 
        
//        if(my_abs(robot_x) > 30)
//          move_speed = 1700;
        
//        if(abs(double(seeker)) < 10)
//          p_seeker = seeker;
//        else
//        {
//          if(ball_distance > 40)
//          {
//            if(seeker > 0)
//              p_seeker = seeker + 90;
//            else
//              p_seeker = seeker - 90;
//          }
//          else 
//            p_seeker = seeker;
//        }
//        if(seeker > 0)
//        {
//          if(ball_distance < 50)    
//            p_seeker = seeker + (seeker * ball_distance / 0.56) * 0.011;
//          else
//            p_seeker = seeker + 90;
//        }
//        if(seeker < 0)
//        {
//          if(ball_distance < 50)    
//            p_seeker = seeker + (seeker * ball_distance / 0.56) * 0.011;
//          else
//            p_seeker = seeker - 90;
//        }
        
       // p_seeker = seeker * 1.5;
        
        
        move_angle = lead_to_degree_borders(move_angle);
          
        
//        if(forward_distance < 43)
//          move_angle = 180 - gyro;
        
//        if(robot_x < -50) 
//          move_angle = 90 - gyro;

//        if(robot_x > 50)
//          move_angle = 270 - gyro;
//        
//        if(robot_y > 205)
//          p_seeker = 180 - gyro;
      
      }
      else if(role == 0 || role == 2 || defence_mode == 1)
      {
        ball_angle = lead_to_degree_borders(ball_angle);
        abs_ball_angle = lead_to_degree_borders(abs_ball_angle);
        static const uint8_t R0 = 42;
        static const uint8_t max_deviation[2] = {25, 25};
        static const int16_t max_angles[2] = {-135, 135};
        static float k_error = 1.85;//3.375
        static const double k1_ball = 1.65, k2_ball = 0.625, k3_ball = 0.61, k_gyro = 0.017;//2.825 0.765 0.39
        
        if(defender_state == 0) //defending state
        {
          kp_angle = -0.375;
          dribler_control.pwm(change_speed(300, time_service::getCurTime()));
          angle_speed = 41;
          
          defender_error = lead_to_degree_borders(lead_to_degree_borders(backward_angle + 180) - 
                abs_ball_angle);
          
          if(my_abs(backward_angle) > 145) //robot on the center and moving near the straight line
          {
            defence_state = 0;
            defender_y = robot_y;
            if(defender_y > R0) error_angle = lead_to_degree_borders(180 - gyro);//!!!
            else error_angle = lead_to_degree_borders(0 - gyro);//!!!
            if(ball_distance > 1 && ball_distance < 100)
            {
              if((my_abs(defender_error) > 2 && ball_distance >= 4) ||
                (my_abs(defender_error) > 5 && ball_distance < 4))
              {
                if(defender_error < 0)
                  defence_angle = lead_to_degree_borders(90 - gyro);//!!!
                else 
                  defence_angle = lead_to_degree_borders(-90 - gyro);//!!!
              }
             }
             else
             {
               if(backward_angle < 175 && backward_angle > 0) defence_angle = lead_to_degree_borders(90 - gyro);//!!!
               else if(backward_angle > -175 && backward_angle < 0) defence_angle = lead_to_degree_borders(-90 - gyro);//!!!
             }           
          }
          else //robot moving near the circular line with R = 35
          {
            defence_state = 1;
            defender_y = backward_distance;
            if(defender_y > R0) error_angle = lead_to_degree_borders(backward_angle);//!!!
            else error_angle = lead_to_degree_borders(backward_angle + 180);
            if(ball_distance > 1 && ball_distance < 100)
            {
              if((my_abs(defender_error) > 2 && ball_distance >= 4) ||
                (my_abs(defender_error) > 5 && ball_distance < 4))
              {
                if(defender_error < 0) 
                  defence_angle = lead_to_degree_borders(backward_angle - 90);//!!!
                else
                  defence_angle = lead_to_degree_borders(backward_angle + 90);             
              }  
            }  
            else
            {
              if(backward_angle < 175 && backward_angle > 0) defence_angle = lead_to_degree_borders(backward_angle + 90 - gyro);
              else if(backward_angle > -175 && backward_angle < 0) defence_angle = lead_to_degree_borders(backward_angle - 90);
            }  
          }
          if(my_abs(lead_to_degree_borders(backward_angle - 180)) > 65) x0_gyro = lead_to_degree_borders(backward_angle + 180);
          else x0_gyro = lead_to_degree_borders(backward_angle + 180) * my_abs(lead_to_degree_borders(backward_angle - 180)) * k_gyro;
          
          //x0_gyro = lead_to_degree_borders(backward_angle + 180);
          
          move_speed = k1_ball * pow(my_abs(ball_angle), k2_ball) * pow(double(ball_distance), k3_ball);
          if(my_abs(backward_angle) > 150)
          {
            if(move_speed > 85) move_speed = 85;//83
          }
          else
          {
            if(move_speed > 60) move_speed = 60;
          }
          
////          if(my_abs(ball_angle) > 20 && ball_distance > 6)
////            move_speed = 3500;
          
          if(time - kick_over_tim < 2000 && backward_distance > R0 * 2)
            error_speed = 60;
          else
          {
            if(my_abs(robot_x > 20))
              y_error = defender_y - (R0 + 7);
            else
              y_error = defender_y - R0;
            error_speed = my_abs(y_error) * k_error;
          }
          
//          x_ball = int(move_speed * sin(double(defence_angle * DEG2RAD)));
//          y_ball = int(move_speed * cos(double(defence_angle * DEG2RAD)));
//          
//          x_error = int(error_speed * sin(double(error_angle * DEG2RAD)));
//          y_error = int(error_speed * cos(double(error_angle * DEG2RAD)));
//          
//          x_result = x_ball + x_error;
//          y_result = y_ball + y_error;
          
          
//          if(error_speed == 0) move_angle = defence_angle;
//          else move_angle = atan2(double(x_result), double(y_result)) * RAD2DEG;
//          
//          move_angle = lead_to_degree_borders(move_angle - gyro);
//          
//          move_speed = sqrt(pow(double(x_result), 2) + pow(double(y_result), 2));
          
          
          move_angle = sum_of_vectors(defence_angle, move_speed, error_angle, error_speed);
          move_speed = get_len_from_sum_of_vectors();
          if(move_speed < 5) move_speed = 0;
          
          if(ball_angle > 10 && backward_angle > -155&& backward_angle < 0)
            move_speed = 25;
          if(ball_angle < -10 && backward_angle < 155 && backward_angle > 0)
            move_speed = 25;
          
          if(ball_angle > 0 && backward_angle > -135 && backward_angle < 0)
            move_speed = 0;
          if(ball_angle < 0 && backward_angle < 135 && backward_angle > 0)
            move_speed = 0;
          
//          if(ball_angle > 0 && backward_angle > -117 && backward_angle < 0)
//          {
//            move_angle = lead_to_degree_borders(0 - gyro);
//            if(backward_distance > R0 * 1.5)
//              move_angle = backward_angle;
//            move_speed = 63;
//          }
//          if(ball_angle < 0 && backward_angle < 117 && backward_angle > 0)
//          {
//            move_angle = lead_to_degree_borders(0 - gyro);
//            if(backward_distance > R0 * 1.5)
//              move_angle = error_angle;
//            move_speed = 63;
//          }
//          if(my_abs(lead_to_degree_borders(abs_ball_angle - backward_angle)) < 13 && ball_distance > 5)
//          {
//            if(backward_distance < 80)
//            {
//              if(ball_distance > 6)
//              {
//                move_angle = lead_to_degree_borders(0 - gyro);
//                move_speed = 38;
//              }
//              else
//                move_speed = 0;
//            }
//          }
//          if(defence_mode == 0)
//          {
//            if(bluetooth_data != 100)
//            {
//              if(my_abs(ball_angle) < 7 && ((ball_distance > 3 && my_abs(backward_angle) > 160) || (ball_distance > 5 && my_abs(backward_angle) > 130)) && time - kick_over_tim > 1500)/* || (ball_sen_dma.dataReturn(0) > BALL_DETECTION_LIGHTNESS)*/
//              {
//                if(!kick_en)
//                  kick_tim = time;
//                kick_en = true;
//                if((time - kick_tim > 3500 && ball_distance < 5) || (time - kick_tim > 1700 && ball_distance >= 5))
//                {
//                  defender_state = 1;
//                  kick_tim = time;
//                }
//              }
//            }
//          }
//          else
//          {
            if(my_abs(ball_angle) < 7 && ((ball_distance > 2 && my_abs(backward_angle) > 147) || (ball_distance > 3 && my_abs(backward_angle) > 127)))/* || (ball_sen_dma.dataReturn(0) > BALL_DETECTION_LIGHTNESS)*/
            {
              if(!kick_en)
                kick_tim = time;
              kick_en = true;
              if((time - kick_tim > 2000 && ball_distance < 2) || (time - kick_tim > 1000 && ball_distance >= 2))
              {
                defender_state = 1;
                kick_tim = time;
              }
            }
         // }
          
          if((backward_distance - R0 < -13 && defence_state == 0) || 
            (backward_distance - R0 < -8 && defence_state == 1 && robot_x < 0) ||
            (backward_distance - R0 < -8 && defence_state == 1 && robot_x > 0))
          {
            //move_angle = 0 - gyro;
            move_angle = lead_to_degree_borders(backward_angle + 180 - gyro);
            move_speed = 75;
          }
//          
          if(backward_distance > 1.5 * R0)
          {
            x0_gyro = 0;
            if(robot_y < 130)
            {
              if(my_abs(lead_to_degree_borders(backward_angle - 180)) > 65) x0_gyro = lead_to_degree_borders(backward_angle + 180);
              else x0_gyro = lead_to_degree_borders(backward_angle + 180) * my_abs(lead_to_degree_borders(backward_angle - 180)) * k_gyro;
            }
          
            if(my_abs(abs_ball_angle) > 45 && ball_distance > 1)
            {
              move_angle = (abs_ball_angle/* + 5*/) + exponential_detour((abs_ball_angle/* + 5*/), ball_distance, 0.07, 0.35, 0.02, 4.3);            
              move_angle = lead_to_degree_borders(move_angle - gyro);
              if(robot_y < 60)
                move_speed = 35;
            }
            else
            {
              if(robot_y > 180)
              {
                if(forward_distance < 40)
                  move_angle = lead_to_degree_borders(180 - gyro);
                else
                {
                  if(robot_x > 30)
                    move_angle = lead_to_degree_borders(-165 - gyro);
                  else if(robot_x < -30)
                    move_angle = lead_to_degree_borders(165 - gyro);
                  else
                    move_angle = lead_to_degree_borders(180 - gyro);
                }
              }
              else
                move_angle = lead_to_degree_borders(backward_angle - gyro); 
            }
            
            if(ball_distance <= 2)
              move_speed = 61;
            else if(my_abs(abs_ball_angle) < 35)
              move_speed = 66;
            else
              move_speed = 70;
          }
          else
          {
            if(my_abs(ball_angle) > 65 && ball_distance > 1)
            {
              move_speed = 25;
              
             if(ball_angle > 0 && backward_angle > -135 && backward_angle < 0)
              move_speed = 0;
             if(ball_angle < 0 && backward_angle < 135 && backward_angle > 0)
              move_speed = 0;   
            }
          }         
          
          if(robot_x > 45)
            move_angle = lead_to_degree_borders(-90 - gyro);
          else if(robot_x < -45)
            move_angle = lead_to_degree_borders(90 - gyro);
          
          if(robot_y < 11)
          {
            move_angle = lead_to_degree_borders(0 - gyro);
            move_speed = 63;
          }
          
          if(ball_distance <= 1 && my_abs(backward_angle) > 150 && my_abs(y_error) < 10) move_speed = 0;
          
        }
        else if(defender_state == 1)
        {
          if(role == 1 && ball_distance >= 3 && my_abs(ball_angle) < 20) 
            dribler_control.pwm(250);
          else 
            dribler_control.pwm(300);
          
          move_angle = ball_angle + exponential_detour(ball_angle, ball_distance, 0.072, 0.33, 0.018, 3.8);
          
          if(my_abs(ball_angle) < 25)
            move_speed = 70;
          else
          {
            move_speed = 70;
            if(defence_mode == 1) move_speed = 87;
          }
          if(my_abs(robot_x) > 37 || time - kick_tim > 2200  || robot_y > 130 || 
           my_abs(ball_angle) > 40 || backward_distance - R0 < -17 || ball_distance <= 1 || 
          ((ball_distance > 2 && my_abs(backward_angle) <= 147) || (ball_distance > 3 && my_abs(backward_angle) <= 127)))
          {
            defender_state = 0;
            kick_over_tim = time;
          }        
          
         if(time - kick_tim > 1700 && backward_distance - R0 < 25) kick_tim = time;
          
         if(my_abs(robot_x) < 20)
           x0_gyro = forward_angle;
         else
          x0_gyro = lead_to_degree_borders(backward_angle - 180);
        
        }
        
        if(move_speed <= 13) move_speed = 0;
      }
      else if(role == 123)//test state
      {
        x0_gyro = 0;
        if(time - test_rotate_timer < 2000)
          move_angle = 180;
        else if(time - test_rotate_timer < 4000)
          move_angle = 0;
        else
          test_rotate_timer = time;       
        move_speed = 35;
//        x0_gyro = 0;
//        move_angle = get_angle_to_point(robot_x, robot_y, 0, 100);
//        move_speed = 115 * sqrt(pow(double(get_distance_to_point(robot_x, robot_y, 0, 100)), 1.25));
//        if(get_distance_to_point(robot_x, robot_y, 0, 100) < 20) move_speed = 0;
      }
      //x0_gyro = 0;
      //x0_gyro = lead_to_degree_borders(backward_angle + 180);
       if(d_timer != time)
       { 
        e_gyro = x0_gyro - gyro;
        if(e_gyro < -180)
          e_gyro += 360;
        else if(e_gyro > 180)
          e_gyro -= 360;	
        p_angle = e_gyro * kp_angle;

        if(i_angle <= 100 && i_angle >= -100)
          i_angle += e_gyro * ki_angle;
        else 
        {
          if(i_angle > 100)
            i_angle = 100;
          else
           i_angle = -100;
        }

        d_angle = (e_old - e_gyro) * kd_angle;

        u_angle = p_angle + i_angle + d_angle;

        e_old = e_gyro;

        d_timer = time;
       }
          
       if(OTLADKA == 0)
        motors.moveRobot(0, 0, 0, 0, time, instant_start_timer, 0);
       else if(OTLADKA == 1)
        motors.moveRobot(move_speed, angle_speed, move_angle, p_angle, time, instant_start_timer, gyro);
       
       if(ball_distance >= 1)
        old_ball_angle = ball_angle;
    }  
    old_gyro = raw_gyro;
  }
}
