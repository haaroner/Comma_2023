#include "project_config.h"
#include "libs.h"

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
#define adc_ 11

#define tim1 81
#define tim2 82
#define tim3 83
#define tim4 84
#define tim5 85
#define tim7 87
#define tim8 88
#define tim9 89
#define tim10 810
#define tim11 811
#define tim12 812
#define tim13 813
#define tim14 814

#define dribler_ 10

int main(){
  time_service::init();
  time_service::startTime();
  
  //time_service::init();
  bool role, old_role, motor_move, seeker_state, dist_state, sub_dist, 
    gyro_reset_en, see_ball, block_motor;
  
  float kp_angle = -28, ki_angle = -0.007, kd_angle = 0,
    angle_change_const = 180/3, seeker;
  
  uint8_t gates_state, dist, long_dist;
  
  volatile int16_t robot_x, robot_y,gyro, x0_gyro, e_gyro, e_old, start_gyro;
  
  volatile int16_t ball_angle, robot_angle, forward_angle, forward_distance, backward_angle, 
    backward_distance, p_angle, i_angle, d_angle, u_angle;
  
  volatile uint32_t time, dist_state_tim,kick_over_tim, d_timer, 
  angle_control_tim, out_kick_timer, end_out_kick_timer;
  //attacker
  volatile uint16_t speed_seeker, speed_angle;
  
  //defender
  bool out_kick_en = false;
  volatile uint8_t defender_state = 0;
  volatile int32_t x_result, y_result, x_ball, y_ball, x_error, y_error, 
    kp_error, error_angle, defence_angle, error_speed, result_angle, 
    move_speed, move_angle, ball_dist_formulka, gates_x0;
  
  volatile double seeker_error;
  
  float k_base_speed, k_angle_speed, k_dist_speed;
  
	
	Motor m1('E', 5, tim9, CHANNEL1, 'E', 6, tim9, CHANNEL2);				
	Motor m2('B', 5, tim3, CHANNEL2, 'B', 3, tim2, CHANNEL2);
	Motor m3('A', 0, tim2, CHANNEL1, 'A', 1, tim5, CHANNEL2);
	Motor m4('E', 11, tim1, CHANNEL2, 'E', 13, tim1, CHANNEL3);
    
	//m1.motorMove(2000);
	pin usart6_tx('C', 6,  Usart6);	
  pin usart6_rx('C', 7,  Usart6);	
	pin usart3_tx('B', 10, Usart3);		
	pin usart3_rx('B', 11, Usart3);		
	pin i2c3_scl('A', 8, i2c);		
	pin i2c3_sda('C', 9, i2c);		
	pin motors_move('B', 1, read_UP);
	pin gyro_reset('B', 0, read_UP);	
  pin cap_charge('B', 9, write_);
  pin cap_discharge('E', 0, write_);
  pin dribler_control('C', 8, tim3);//tim8!!! prsc = 419
  pin adc_test('A', 3, adc_);
  
  Adc test_ADC(ADC1, 1, 3, RCC_APB2Periph_ADC1, adc_test);
  test_ADC.sendMeChannel(3);
  Dma test_DMA(RCC_AHB1Periph_DMA2, test_ADC);
  test_DMA.dmaInit(DMA2_Stream0, DMA_Channel_0, 5);
  test_DMA.adcInitInDma();
  //308
  //dribler_control.setBit();
  //dribler dribler(dribler_control, 8);
  //dribler_control.pwmInit(RCC_APB1ENR_TIM3EN, 159, 2000, 0, CHANNEL3, TIM3, 1);	
  //dribler_control.pwm(150);
  //time_service::delay_ms(5000);
//  while(true)
//  {
//    dribler_control.pwm(165);
//    time_service::delay_ms(2000);
//    dribler_control.pwm(150);
//    time_service::delay_ms(1000);
//  }
  soft_i2c ir(i2c3_scl, i2c3_sda);
	IRlocator locator360(ir, 0x0E);
  
	camera camera(usart3_tx, usart3_rx);
  MPU9250_ mpu(ir);
	
	motors motors(m1, m2, m3, m4);
  kicker kicker(cap_charge, cap_discharge);
  
  role = 1;
  speed_seeker = 0;
  motor_move = true;
  
  usart3::usart3Init(460800, 8, 1);
  usart6::usart6Init(115200, 8, 1);
  
  //hc_05 bluetooth_1(uart_3);
  //hc_05 bluetooth_2(uart_6);
  
//  while(!mpu.setup(0x68) || time!!! < 1000);
//  mpu.calibrateGyro(500, 2500);
//  mpu.update();
//  start_gyro = mpu.getRealYaw();
  //SSD1306 display(3, );
  
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
    
    gyro_reset_en = !(gyro_reset.getGPIOx()->IDR & gyro_reset.getPinNumber());
    
		camera.getData();
    camera.calculate_pos(gyro, gyro_reset_en);
		robot_x = camera.get_x();
		robot_y = camera.get_y();
    forward_angle = camera.get_forward_angle();
    forward_distance = camera.get_forward_distance();
    backward_angle = camera.get_backward_angle();
    backward_distance = camera.get_backward_distance();
    gates_state = camera.get_data_state();

    dist = locator360.getData(0x07);

    long_dist = locator360.getData(0x05);
    if(long_dist == 0) long_dist = 1;
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
    
    if(seeker == 1275) see_ball = false;
    else see_ball = true;
    
    if(seeker < -180)
      seeker += 360;
    if(seeker > 180)
      seeker -= 360;
    
    ball_angle = seeker + gyro;
		if(ball_angle < -180)
      ball_angle += 360;
    if(seeker > 180)
      ball_angle -= 360;
    
    block_motor = false;

    
    motor_move = !(motors_move.getGPIOx()->IDR & motors_move.getPinNumber());
//    if(gyro_reset_en)
//    {
//      motor_move = 1;
//      start_gyro = usart6::read() * 2;
//    }
    
    block_motor = false;
      
    if(motor_move)
    {
      kicker.discharge();
    }
      kicker.check(time);
    
    if(role == 0)
		{
      if((old_role == 1 && time - kick_over_tim > 1500)/* || seeker > 270 || seeker < 90 */)
      {
        role = 1;
        old_role = 0;
      }
      
      if(gates_state == 8)
      {
        if(robot_x > 0)
          x0_gyro = -20;
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
       move_angle = seeker + exponential_detour(seeker, long_dist, 0.055, 0.67, 0.017, 6.4);
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
      
      if((backward_distance < 51 && gates_state != 2) || robot_y < 51)
      {
        move_angle = 0 - gyro;
        speed_seeker = 2500;

        if(abs(double(seeker)) < 90)
        {
          move_angle = seeker;
          speed_seeker = 2700;
        }
        else if(abs(double(seeker)) < 110)
        {
          if(robot_y < 41)
          {
            move_angle = seeker;
            speed_seeker = 2750;
          }
          else
          {
            if(seeker > 0)
              move_angle = seeker + 20;
            else
              move_angle = seeker - 20;
            speed_seeker = 2400;
          }
        }
        if(robot_y < 30 || backward_distance < 30)
          move_angle = 0;
          speed_seeker = 2100;
      }

      if(robot_x < -47)
      { 
        if(robot_x > -52 &&  ball_angle < 90 && ball_angle > 0)
          move_angle = seeker;
        else
        {
          move_angle = 90 - gyro;
        }
      }
      
      if(robot_x > 40)
      {
        if(robot_x < 40 && ball_angle > -90 && ball_angle < 0)
          move_angle = seeker;
        else
        {
          move_angle = 270 - gyro;
        }
      }
      
      if(abs(double(move_angle)) < 11 && abs(double(e_gyro)) < 25)
            speed_seeker = 3900;
      
      if(dist < 9)
        if(abs(double(move_angle)) < 135 || sub_dist == false)
          speed_seeker = 3900;
      
      if(abs(double(robot_x)) > 30)
      {
        if(robot_x > 0 && ball_angle > 0 && ball_angle < 180)
          speed_seeker = 2100;
        else if(robot_x < 0 && ball_angle < 0 && ball_angle > -180)
          speed_seeker = 2100;
        else
          speed_seeker = 3700;
      }
      if(robot_y < 100 && seeker < 260 && seeker > 100)
        speed_seeker = 2000;
      if((abs(double(robot_x)) < 20 && robot_y > 169) || (abs(double(robot_x)) > 20 && forward_distance < 25) || robot_y > 185 || (forward_angle > 65 && forward_angle < 295))
        move_angle = 180 - gyro;
      
      if(gates_state == 8 && robot_x > 0)
        move_angle = 270 - gyro;
      if(gates_state == 8 && robot_x < 0)
        move_angle = 90 - gyro;
      
      if(robot_y < 30)
        move_angle = 0;
      
      if(motor_move)
        motors.moveRobot(0, 0, 0, 0);
      else if(seeker == 1275)
        motors.moveRobot(0, 1500, 0, u_angle);
      else
        motors.moveRobot(speed_seeker, 1500, int(move_angle), u_angle);
		}
    else if(role == 1)
    {
      static const uint8_t R0 = 35;
      static const int linear_error_x[2] = {-17, 17};
      static const int max_degree[2][2] = {{-130, -110},
                                           {130, 110}};
      static int ball_norm;
      static const double k1_defender = 1350, k2_defender = 0.64, k3_defender = 0.21;
      
      kp_error = 300;
      
                                           
      if(backward_distance > 90) defender_state = 1;
      else 
      {
        if(long_dist > 50 && abs(double(x0_gyro)) < 45)
        {
          if(out_kick_en == false)
            out_kick_timer = time;
          out_kick_en = true;
          if(time - out_kick_timer > 1700)
          {
            if(defender_state != 2)
              end_out_kick_timer = time;
            defender_state = 2;
          }
        }
        else
        {
          defender_state = 0;
          out_kick_en = false;
        }
      }
      
      if(defender_state == 0)
      {
        x0_gyro = backward_angle + 180;

        ball_norm = (gyro - x0_gyro) + seeker;
        
        if(ball_norm < -180)
          ball_norm += 360;
        if(ball_norm > 180)
          ball_norm -= 360;
        
        if(x0_gyro < -180)
          x0_gyro += 360;
        if(x0_gyro > 180)
          x0_gyro -= 360;
        //motor_move = false;
        if(see_ball == false)
        {
          seeker = backward_angle + 180;
          
          seeker *= -1;
          long_dist = 50;//!!!
        }
        
        seeker_error = seeker / 90;
        if(seeker_error > 1) seeker_error = 1;
        else if(seeker_error < -1) seeker_error = -1;
        
        ball_dist_formulka = long_dist;
        if(ball_dist_formulka > 90) ball_dist_formulka = 90;
        else if(ball_dist_formulka < 10) ball_dist_formulka = 10;
        
        speed_seeker = k1_defender * pow(abs(double(seeker_error)), k2_defender) * pow(ball_dist_formulka, k3_defender);
        
        if(speed_seeker > 3000) speed_seeker = 3000;
        
        if((backward_angle > max_degree[0][0] && backward_angle < 0) || (backward_angle < max_degree[1][0] && backward_angle > 0))
        {
         
         if(robot_x > 0) move_angle = -45;
         else move_angle = 45;
         speed_seeker = 2000;
         
        }
        else if((backward_angle > max_degree[0][1] && backward_angle < 0) || (backward_angle < max_degree[1][1] && backward_angle > 0))
        {
          if(x0_gyro > 0 && defence_angle > 0)
            speed_seeker = 0;
          else if(x0_gyro < 0 && defence_angle < 0)
            speed_seeker = 0;
        }
        else
        {
          if(seeker >= x0_gyro) defence_angle = backward_angle - 90;
          else defence_angle = backward_angle + 90;
          
          if(defence_angle < -180)
            defence_angle += 360;
          if(defence_angle > 180)
            defence_angle -= 360;     
        }
                
        if(robot_x > linear_error_x[1] || robot_x < linear_error_x[0])
        {
          if(R0 > backward_distance) error_angle = backward_angle + 180;
          else error_angle = backward_angle;

          if(error_angle < -180)
            error_angle += 360;
          if(error_angle > 180)
            error_angle -= 360;
          
          y_error = R0 - backward_distance;
          
        }
        else 
        {  
          y_error = R0 - robot_y;
          if(y_error > 0) error_angle = 0;
          else error_angle = 180;
        }
        
        error_speed = abs(double(y_error)) * kp_error; 
        
        x_ball = int(speed_seeker * sin(double(defence_angle / 57.3)));
        y_ball = int(speed_seeker * cos(double(defence_angle / 57.3)));
        
        x_error = int(error_speed * sin(double(error_angle / 57.3)));
        y_error = int(error_speed * cos(double(error_angle / 57.3)));
        
        x_result = x_ball + x_error;
        y_result = y_ball + y_error;
        
        if(y_result == 0) move_angle = defence_angle;
        else move_angle = atan2(double(x_result), double(y_result)) * 57.3;
        
        move_speed = sqrt(pow(double(x_result), 2) + pow(double(y_result), 2));
          
        if(robot_y < 15)
          move_angle = 0;
        
        move_angle -= gyro;
        
      }
      
      if(defender_state == 1)
      {
        if(gates_state != 2)
        {
          x0_gyro = backward_angle + 180;
          
          if(x0_gyro < -180)
            x0_gyro += 360;
          else if(x0_gyro > 180)
            x0_gyro -= 360;	
        }
        else
          x0_gyro = 0;
        
        if(abs(double(seeker)) < 35)
        {
          if(robot_x > 20)
            move_angle = -145;
          else if(robot_x < -20)
            move_angle = 145;
          else
            move_angle = 180;
        }
        else
        {
          move_angle = seeker + exponential_detour(seeker, long_dist, 0.055, 0.67, 0.017, 6.4);
        }
      }
      
      if(defender_state == 2)
      {
        if(abs(double(seeker)) < 30 && time - end_out_kick_timer < 4000)
        {
          if(long_dist > 80 && abs(double(forward_angle)) < 20)
          {
            move_angle = seeker + exponential_detour(seeker, long_dist, 0.055, 0.67, 0.017, 6.4);
            x0_gyro = forward_angle;
          }
          else
          {
            move_angle = seeker;
          }
          move_speed = 3900;
        }
        else 
        {
          out_kick_en = false;
        }
      }
      
      if(d_timer != time)
      {  
         if(seeker == 1275)
         {
           ki_angle = -0.01;
           kd_angle = -0;
           kp_angle = -14;
         }
         else
           kp_angle = -35;
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
      
      if(motor_move)
        motors.moveRobot(0, 0, 0, 0);
      else if(block_motor)
        motors.stopRobot(1500);
      else
        motors.moveRobot(move_speed, 1500, int(move_angle), u_angle);

    }
  }
}