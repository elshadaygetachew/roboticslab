#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include "arm_lib/msg_collection.h"

using namespace std;

double *rotateX(double x, double y, double z, double a)
{
  double *arr = new double[5];
  double xx = x;
  double yx = y * cos(a) - z * sin(a);
  double zx = y * sin(a) + z * cos(a);

  /* Some operations on arr[] */
  arr[0] = xx;
  arr[1] = yx;
  arr[2] = zx;

  return arr;
}

double *rotateY(double x, double y, double z, double b)
{

  double *arr = new double[5];

  double xy = z * sin(b) + x * cos(b);
  double yy = y;
  double zy = z * cos(b) - x * sin(b);

  arr[0] = xy;
  arr[1] = yy;
  arr[2] = zy;

  return arr;
}

double *rotateZ(double x, double y, double z, double g)
{
  double *arr = new double[100];

  double xz = x * cos(g) - y * sin(g);
  double yz = x * sin(g) + y * cos(g);
  double zz = z;

  /* Some operations on arr[] */
  arr[0] = xz;
  arr[1] = yz;
  arr[2] = zz;

  return arr;
}

double *translate(double x, double y, double z, double a, double b,double g, int d)
{

  double *arr = new double[7];

  arr[0] = x;
  arr[1] = y;
  arr[2] = z;
  arr[3] = a;
  arr[4] = b;
  arr[5] = g;
  arr[6] = d;
    double *rPtr = rotateX(x, y, z, a);
    double *rPtr1 = rPtr;
    double firstV = rPtr[0];
    double secondV = rPtr[1];
    double thirdV = rPtr[2];
     double *rPtr2 = rotateY(firstV, secondV, thirdV, b);
    double *rPtr3 = rPtr2;
    double first_1V = rPtr2[0];
    double second_2V = rPtr2[1];
    double third_3V = rPtr2[2];
    double *rPtr4 = rotateZ(first_1V, second_2V, third_3V, g);
    double *rPtr5 = rPtr4;
    double lastV_1 = rPtr4[0];
    double lastV_2 = rPtr4[1];
    double lastV_3 = rPtr4[2];


  double zy = lastV_1 + d;
  double xz = lastV_2 + d;
  double xy = lastV_3 + d;

  return arr;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_node");
  ros::NodeHandle n;
  ros::Publisher vectorPublisher = n.advertise<arm_lib::msg_collection>("/publishVector", 1000);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    arm_lib::msg_collection vector;
    double *values = new double[7];
    for (int i = 0; i < 7; i++)
    {
      cout << "Enter value: " << endl;
      cin >> values[i];
    }

    vector.x = values[0];
    vector.y = values[1];
    vector.z = values[2];
    vector.alpha = values[3];
    vector.beta = values[4];
    vector.gamma = values[5];
    vector.distance = values[6];


    double *tPtr = translate(vector.x, vector.y, vector.z, vector.alpha, vector.beta, vector.gamma, vector.distance);


    vector.newX = tPtr[0];
    vector.newY = tPtr[1];
    vector.newZ = tPtr[2];

  
    ROS_INFO("\nx', y', z'[ROTATED THEN TRANSLATED] = %f, %f, %f", vector.newX, vector.newY, vector.newZ);
    vectorPublisher.publish(vector);
   
    ros::spinOnce();
    loop_rate.sleep();
  }
}