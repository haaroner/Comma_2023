#pragma once
#include <project_config.h>
#include <motor.h>

class motors 
{
	public:
		motors(Motor& m1, 
					Motor& m2,
					Motor& m3, 
					Motor& m4);
	void moveMotor(int32_t power);
	void stopRobot(uint16_t power);
	void moveRobot(double _maxPower,
										 double _maxPower_angle,
										 double _angle,
										 double _inc);
                     
	private:
		Motor _m1;
		Motor _m2;
		Motor _m3;
		Motor _m4;
		signed int opowers;
		uint16_t _maxPowerMove;
		double ang;
};
