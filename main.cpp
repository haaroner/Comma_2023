#include "project_config.h"
#include "pin_setup.h"
#include "motor.h"
#include "time_service.h"
#include "usart6.h"
#include "usart3.h"
#include "soft_i2c.h"
#include "IRLocator.h"
#include "motors.h"
#include "OpenMv.h"
#include "kicker.h"
#include "MPU9250.h"
#include "SPI_1.h"
#include "SPI_2.h"
#include "SPI_3.h"
#include "ball_detour.h"
#include "usart2.h"
#include "hc-05.h"
#include "SSD1306.h"

#define CHANNEL1 1
#define CHANNEL2 2
#define CHANNEL3 3
#define CHANNEL4 4

#define Usart1 01
#define Usart2 02
#define Usart3 03
#define Usart6 04
#define i2c 05
#define read_UP 06
#define write_ 07
#define spi 9
#define spi1 91
#define spi2 92
#define spi3 93

#define tim1 81
#define tim2 82
#define tim3 83
#define tim4 84
#define tim5 85
#define tim8 88
#define tim9 89
#define tim10 810
#define tim11 811
#define tim12 812
#define tim13 813
#define tim14 814


int main(){
  volatile int dop = 0;
  
	volatile bool change_role = false, dist_state = false, line_data[4] = {0, 0, 0, 0}, out_side_en = false, out_side = false, go_zone = 0, stop = 0, gate_color = 0,
	line_zone[4] = {0, 0, 0, 0}, side_kick = false, gyro_reset_en = false,
  side_kick_en = false, kick_en = false, kick_over_time_en = false,
  block = false, detour_side_ch_en = false, sub_dist = true, motor_move,
  kick_state = 0, dopusk = false, gyro_reset_on, motor_stop_on = false;
	
	volatile int state = 0, dist, long_dist, seeker_state = 0, role = 0, old_role = 0, 
		new_role = 0, u = 0, priority_state = 0;
	
	volatile int gyro = 0, seeker = 0, x0_gyro = 0,
		 locator_offset = 0, sign = 1, last_robot_x = 0, 
		 robot_x = 0, last_robot_y = 0, robot_y = 0, forward_angle = 0,
		 backward_angle = 0, goal_atan = 0, state_ob = 0, gate_dist = 0, ball_angle,
     round_dist = 11, detour_side = 0, detour_side_ch = 0;
	
	volatile uint32_t sub = 0, kick_start_tim = 0, kick_over_tim = 0,
	 kick_tim = 0, gyro_reset_tim = 0, change_role_tim = 0, time = 0,
	 dist_state_tim = 0, d_timer = 0, angle_control_tim = 0, detour_side_ch_tim;
	
	volatile int e_gyro = 0, move_seeker = 0, u_angle = 0, e_gate = 0, k_gate = 0, p_gate = 0, x0_back_gate = 180, x0_forward_gate = 0, 
		move_gate = 0, speed_seeker = 3095, start_gyro, speed_angle = 1000, 
    ring = 0, move_x = 0, move_y = 0, e_old, center_error, center_er_x,
    center_error_y, forward_distance, backward_distance, x0_backward_distance = 20, backward_distance_e;
	
	volatile float e_seeker = 0, p_seeker = 0, i_seeker = 0, pi_seeker = 0, k_seeker = 1,
		ki_seeker, x0_seeker = 0,p_angle, d_angle, i_angle = 0, kp_angle = -28, kd_angle = -0, 
    ki_angle = -0.007, actual_dist = 0, last_dist = 0, 
		robot_pos_change = 1,
    angle_change_const = (180 / 3), move_angle = 0;
	volatile char back_gate;
  volatile uint8_t gates_state = 1;
	volatile int test, test_old, test_speed;
  volatile float Pitch, Roll, Yaw;
	
	Motor m1('E', 5, tim9, CHANNEL1, 'E', 6, tim9, CHANNEL2);				
	Motor m2('B', 5, tim3, CHANNEL2, 'B', 3, tim2, CHANNEL2);
	Motor m3('A', 0, tim2, CHANNEL1, 'A', 1, tim5, CHANNEL2);
	Motor m4('E', 11, tim1, CHANNEL2, 'E', 13, tim1, CHANNEL3);
    
	pin usart6_tx('C', 6, Usart6);		
	pin usart6_rx('C', 7,  Usart6);		
	pin usart3_tx('B', 10, Usart3);		
	pin usart3_rx('B', 11, Usart3);		
	pin i2c3_scl('A', 8, i2c);		
	pin i2c3_sda('C', 9, i2c);		
	pin motors_move('B', 1, read_UP);
	pin gyro_reset('B', 0, read_UP);	
  pin cap_charge('B', 9, write_);
  pin cap_discharge('E', 0, write_);
  
  soft_i2c ir(i2c3_scl, i2c3_sda);
	IRlocator locator360(ir, 0x0E);
  
	camera camera(usart3_tx, usart3_rx);
  MPU9250_ mpu(ir);
	
	motors motors(m1, m2, m3, m4);
  kicker kicker(cap_charge, cap_discharge);
	time_service::init();
  
  role = 0;
  speed_seeker = 0;
  motor_move = true;
  
  usart3::usart3Init(115200, 8, 1);
  usart6::usart6Init(115200, 8, 1);
  
  hc_05 bluetooth_1(uart_3);
  hc_05 bluetooth_2(uart_6);
  
//  while(!mpu.setup(0x68) || time!!! < 1000);
//  mpu.calibrateGyro(500, 2500);
//  mpu.update();
//  start_gyro = mpu.getRealYaw();
  
  while(true)
  {
    if(speed_seeker > 99) speed_seeker = 0;
    else speed_seeker++;
    
    bluetooth_1.send(speed_seeker);
    speed_angle = bluetooth_2.read();
    
    //usart3::write(speed_seeker);
    //speed_angle = usart6::read();
    
    if(speed_seeker == speed_angle || speed_seeker - 1 == speed_angle)
      ring++;
    time_service::delay_ms(300);
  }
  
  time_service::delay_ms(30000);
  while(usart6::available() < 1);
  start_gyro = usart6::read();
  
  while(true)
  {
    time = time_service::getCurTime();
    gyro = (usart6::read() * 2) - start_gyro;
		if(gyro < -180)
      gyro += 360;
    else if(gyro > 180)
      gyro -= 360;
		camera.getData();
    camera.calculate_pos(gyro);
		robot_x = camera.get_x();
		robot_y = camera.get_y();
    forward_angle = camera.get_forward_angle();
    forward_distance = camera.get_forward_distance();
    backward_angle = camera.get_backward_angle();
    backward_distance = camera.get_backward_distance();
    gates_state = camera.get_data_state();

    dist = locator360.getData(0x07);

    long_dist = locator360.getData(0x05);
		if(dist < 11)
		{
			seeker = locator360.getData(0x04) * 5;
			seeker_state = 1;
			if(time - dist_state_tim > 500)
				sub_dist = false;
		}
		else
		{
			sub_dist = true;
			dist_state_tim = time;
			seeker = locator360.getData(0x06) * 5;
			seeker_state = 0;
		}
    if(long_dist < 27 && (seeker > 345 || seeker < 15) && forward_distance < 50)
      dist_state = true;
    else
      dist_state = false;

    last_dist = long_dist;
    
    block = false;
    
    ball_angle = seeker + gyro;
		if(ball_angle < -180)
      ball_angle += 360;
    if(seeker > 180)
      ball_angle -= 360;
//    if(seeker < -180)
//      ball_angle = seeker + 360;
//    if(seeker > 180)
//      ball_angle = seeker - 360;
    
    motor_move = !(motors_move.getGPIOx()->IDR & motors_move.getPinNumber());
    gyro_reset_en = !(gyro_reset.getGPIOx()->IDR & gyro_reset.getPinNumber());
    if(gyro_reset_en)
    {
      motor_move = 1;
      start_gyro = usart6::read() * 2;
    }
    
    kicker.check(time);
    
    if(role == 0)
    {
      //p_seeker = seeker + (seeker * k) / (dist * k2);
      if(seeker < -180)
        seeker += 360;
      if(seeker > 180)
        seeker -= 360;
      p_seeker = seeker + exponential_detour(seeker, long_dist, 0.055, 0.67, 0.017, 6.4);
      
      if(gates_state == 8)
      {
        if(robot_x > 0)
          x0_gyro = 340;
        else
          x0_gyro = 20;
      }
      else
      {        
        if(gates_state != 0)
          x0_gyro = forward_angle;
        else
          x0_gyro = 0;
      }
      if(robot_y < 90 || backward_distance < 90)
        x0_gyro = 0;
			if(d_timer != time)
      { 
         if(seeker == 1275)
         {
           ki_angle = -0.001;
           kd_angle = -0;
           kp_angle = -14;
         }
         e_gyro = x0_gyro - gyro;
         if(e_gyro < -180)
            e_gyro += 360;
         else if(e_gyro > 180)
            e_gyro -= 360;	
         p_angle = e_gyro * kp_angle;
          
         if(i_angle <= 300 && i_angle >= -300)
         {
           i_angle += e_gyro * ki_angle;
         }
         else 
         {
           if(i_angle > 300)
            i_angle = 300;
           else
             i_angle = -300;
         }
         
         d_angle = (e_old - e_gyro) * kd_angle;
         
         u_angle = p_angle + i_angle + d_angle;
         
         e_old = e_gyro;
         
         d_timer = time;
      }
      
      if(angle_control_tim != time)
      {
        if(abs(double(move_angle - p_seeker)) > angle_change_const)
          move_angle += angle_change_const * ((move_angle - p_seeker) / abs(double(move_angle - p_seeker))) * -1;        
        else
          move_angle = p_seeker;
        angle_control_tim = time;
      }

      if(block)
        motors.stopRobot();
      else if(motor_move)
        motors.moveRobot(0, 0, 0, 0);
      else if(seeker == 1275)
        motors.moveRobot(0, 1500, 0, u_angle);
      else
        motors.moveRobot(3000, 1000, p_seeker, u_angle);
    }
    time_service::delay_ms(1);
  }
}