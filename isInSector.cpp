// version 1.0 RFID perception model

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <math.h>

using namespace Eigen;


bool isInSector(float x_center,float y_center,float angle_offset,float angle, float x,float y){

VectorXd first_line(3),second_line(3);
first_line(0)=sin(angle_offset);
first_line(1)=-cos(angle_offset);
first_line(2)=-x_center*sin(angle_offset)+y_center*cos(angle_offset);
second_line(0)=sin(angle_offset+angle);
second_line(1)=-cos(angle_offset+angle);
second_line(2)=-x_center*sin(angle_offset+angle)+y_center*cos(angle_offset+angle);


if((x*first_line(0)+y*first_line(1)+first_line(2)<0)&&(x*second_line(0)+y*second_line(1)+second_line(2)<0))
 return true;
else return false;

}
