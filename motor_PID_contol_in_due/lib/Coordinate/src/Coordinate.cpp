#include<Arduino.h>
#include<iostream>
#include<cmath>
#include<cstdlib>
#include<cstring>
#include<cfloat>
#include"Coordinate.h"

void motor_goal(double goal_xyzs[3], double motor_xyzs[3][3], double dgoal[3])
{
	for (int ivm = 0; ivm <= 2; ivm++)
	{
		dgoal[ivm] = sqrt(pow(goal_xyzs[0] - motor_xyzs[ivm][0], 2)
			+ pow(goal_xyzs[1] - motor_xyzs[ivm][1], 2)
			+ pow(goal_xyzs[2] - motor_xyzs[ivm][2], 2));
		if (dgoal[ivm] < 0)
		{
			dgoal[ivm] = -dgoal[ivm];
		}
	}
}

float distance_get(float precision_now[3], float precision_goal[3])
{
	float distance_output = sqrt(pow(precision_goal[0] - precision_now[0], 2)
		+ pow(precision_goal[1] - precision_now[1], 2)
		+ pow(precision_goal[2] - precision_now[2], 2));
	return distance_output;
}

float Coordinate_difference_get(float coordinate_now[3], float coordinate_goal[3])
{

}

float motorsteps_get(float motorxyzs[3][3], float coordinate_now[3], float coordinate_goal[3], int motor_number)
{
	int motorsteps = 0;
	if(motor_number == 0)
	{
		motorsteps = sqrt(pow(coordinate_goal[0] - motorxyzs[0][0], 2)
		+ pow(coordinate_goal[1] - motorxyzs[0][1], 2)
		+ pow(coordinate_goal[2] - motorxyzs[0][2], 2))
		- sqrt(pow(coordinate_now[0] - motorxyzs[0][0], 2)
		+ pow(coordinate_now[1] - motorxyzs[0][1], 2)
		+ pow(coordinate_now[2] - motorxyzs[0][2], 2));
	}
	if(motor_number == 1)
	{
		motorsteps = sqrt(pow(coordinate_goal[0] - motorxyzs[1][0], 2)
		+ pow(coordinate_goal[1] - motorxyzs[1][1], 2)
		+ pow(coordinate_goal[2] - motorxyzs[1][2], 2))
		- sqrt(pow(coordinate_now[0] - motorxyzs[1][0], 2)
		+ pow(coordinate_now[1] - motorxyzs[1][1], 2)
		+ pow(coordinate_now[2] - motorxyzs[1][2], 2));
	}
	if(motor_number == 2)
	{
		motorsteps = sqrt(pow(coordinate_goal[0] - motorxyzs[2][0], 2)
		+ pow(coordinate_goal[1] - motorxyzs[2][1], 2)
		+ pow(coordinate_goal[2] - motorxyzs[2][2], 2))
		- sqrt(pow(coordinate_now[0] - motorxyzs[2][0], 2)
		+ pow(coordinate_now[1] - motorxyzs[2][1], 2)
		+ pow(coordinate_now[2] - motorxyzs[2][2], 2));
	}
	return motor_number;
}


void carve_linedirve(float precision_mm, float coordinate_now[3] ,float coordinate_goal[3], float motor_coordinate[3], float target[3])
{
	float goal_distance = distance_get(coordinate_now,coordinate_goal);
	float step_x = goal_distance / precision_mm;
	for (target != coordinate_goal)
	{

	}

}
