#include "Settings.h"
#include "libs.h"
#include "tools.h"
#include "Robot.h"

int robot_x, robot_y, a = 0;

int main()
{
  Robot::init_robot(1);
  Robot::moveRobot(0, 0);
  Robot::rotateRobot(0, 0);
  while(true)
  {
    Robot::update();
  }
  
}
  
