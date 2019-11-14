#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>

std::vector<float> joy_cmd(4);
geometry_msgs::PointStamped pos_cmd;

static float hz = 20.0; // Hz
static float deadband = 0.1;
static float x_lim[] = {-2, 2};
static float y_lim[] = {-1, 1};
static float z_lim[] = {0, 2};
bool fs_trigger = 0;

void js_cb(const sensor_msgs::Joy msg)
{
  joy_cmd = msg.axes;
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
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_link");
  ros::NodeHandle n;

  ros::Subscriber js_sub = n.subscribe<sensor_msgs::Joy>("/gquad/joy", 20, js_cb);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PointStamped>("/gquad/pos_cmd", 20);

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
