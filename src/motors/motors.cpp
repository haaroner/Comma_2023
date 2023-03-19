#include "motors.h"

motors::motors(Motor& m1, 
					 Motor& m2,
					 Motor& m3, 
					 Motor& m4): _m1(m1),
											 _m2(m2), 
											 _m3(m3), 
											 _m4(m4)
{
		
}
void motors::moveMotor(int32_t power)
{
		_m1.motorMove(-power);//without
		_m2.motorMove(-power);//green right
		_m3.motorMove(-power);//white
		_m4.motorMove(power);//blue right
}

void motors::stopRobot(uint16_t _power)
{
  if(_power > 3000) _power = 3000;
	_m1.blockMotor(_power);
	_m2.blockMotor(_power);
	_m3.blockMotor(_power);
	_m4.blockMotor(_power);
}

void motors::moveRobot(double _maxPower,
										 double _maxPower_angle,
										 double _angle,
										 double _inc)
{
	if(_inc > _maxPower_angle)
		_inc =  _maxPower_angle;
	else if(_inc < -_maxPower_angle)
		_inc =  -_maxPower_angle;
  
	ang = (_angle + 135) * DEG2RAD;
	opowers = _maxPower * cos(ang);
	opowers += _inc;
	_m1.motorMove(opowers);
	
	ang = (_angle - 135) * DEG2RAD;
	opowers = _maxPower * -cos(ang);
	opowers += _inc;
	_m2.motorMove(-opowers);

	
	ang = (_angle + 45) * DEG2RAD;
	opowers = _maxPower * cos(ang);
	opowers -= _inc;
	_m3.motorMove(-opowers);

	
	ang = (_angle - 45) * DEG2RAD;
	opowers = _maxPower * cos(ang);
	opowers += _inc;
	_m4.motorMove(opowers);
}
// you are clown
