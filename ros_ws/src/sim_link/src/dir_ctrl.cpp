#include "ros/ros.h"
#include <std_msgs/Float32MultiArray.h>
#include "mavros_msgs/ActuatorControl.h"
#include <eigen3/Eigen/Dense>

std_msgs::Float32MultiArray l;
std_msgs::Float32MultiArray L;

void l_cb(const std_msgs::Float32MultiArrayConstPtr& msg)
{
  l = *msg;
}

void L_cb(const std_msgs::Float32MultiArrayConstPtr& msg)
{
  L = *msg;
}

float deadband_check(float cmd,float deadband) {
  if (abs(cmd) < deadband)
  {
    cmd = 0;
  }
  else
  {
    float a = copysign(1, cmd);

    cmd = a * (abs(cmd) - deadband);
  }
  return cmd;
}

float fenced_update(float lim[], float cmd, float current, float hz) {
  float value = current + (cmd/hz);

  if (value < lim[0]){
    value = lim[0];
  }
  else if (value > lim[1])
    value = lim[1];
  else {
    // Carry on
  }
  return value;
}

void update_pos_cmd()
{
  joy_cmd[0] = deadband_check(joy_cmd[0],deadband);
  joy_cmd[1] = deadband_check(joy_cmd[1],deadband);
  joy_cmd[3] = deadband_check(joy_cmd[3],deadband);

  pos_cmd.point.x = fenced_update(x_lim,joy_cmd[1],pos_cmd.point.x,hz);
  pos_cmd.point.y = fenced_update(y_lim,joy_cmd[0],pos_cmd.point.y,hz);
  pos_cmd.point.z = fenced_update(z_lim,joy_cmd[3],pos_cmd.point.z,hz);

  //std::cout << reset_cmd << std::endl;
  if (reset_cmd > 0.5) {
    pos_cmd.point.x = 0;
    pos_cmd.point.y = 0;
    pos_cmd.point.z = 0;
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_link");
  ros::NodeHandle n;

  ros::Subscriber js_sub = n.subscribe<sensor_msgs::Joy>("/sim_link/joy", 20, js_cb);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PointStamped>("/sim_link/pos_cmd", 20);

  ros::Rate rate(hz);

  //send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    // local_vel_pub.publish(cmd_vel);
    pose_pub.publish(pos_cmd);
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok())
  {
    update_pos_cmd();
    pose_pub.publish(pos_cmd);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
