//#include "Bounds.h"

//Bounds::Bounds()
//{}

//polar_vector Bounds::detect_out_of_bounds(bool enable_diving)
//{
//  _robot_x = Robot::robot_x;
//  _robot_y = Robot::robot_y;
//  _move_angle = Robot::abs_move_angle;
//  _move_speed = Robot::move_speed;
//  if(!enable_diving)
//  {
//    if(_robot_x > ROBOT_MAX_X)
//    {
//      _move_angle = sum_of_vectors(_move_angle, constrain(30, 0, _move_speed),
//        -90, constrain(50, 0, (_robot_x - ROBOT_MAX_X) * 2));
//      _move_speed = get_len_from_sum_of_vectors();
//    }
//          
//    if(_robot_x < ROBOT_MIN_X)
//    {
//      _move_angle = sum_of_vectors(_move_angle, constrain(30, 0, _move_speed),
//        90, constrain(50, 0, (ROBOT_MIN_X - _robot_x) * 2));
//      _move_speed = get_len_from_sum_of_vectors();
//    }
//  }
//}