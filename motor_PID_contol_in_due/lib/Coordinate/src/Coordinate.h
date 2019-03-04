#include<Arduino.h>
/*
 *Coordinate_operation.h
 *Coordinate_operation
 *坐标验算的头文件，进行坐标的初步计算
 */
float distance_get(float precision_now[3], float precision_goal[3])

void motor_goal(double goal_xyzs[3], double motor_xyzs[3][3], double dgoal[3]);//运动转换函数 输入目标坐标和电机坐标，输出该电机坐标量

