#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <turtlesim/Spawn.h>
int main(int argc, char *args[]) {
  ros::init(argc, args, "tf_listener");
  ros::NodeHandle node;

  // 等待 /spawn 服务添加到ros系统
  ros::service::waitForService("/spawn");
  // 创建客户端
  ros::ServiceClient add_turtle =
      node.serviceClient<turtlesim::Spawn>("/spawn");
  turtlesim::Spawn turtle;
  turtle.request.name = "/turtle2";
  add_turtle.call(turtle);

  // 用于控制 /turtle2 的速度
  ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>(
      turtle.request.name + "/cmd_vel", 10);

  // 创建 tf 的框架变换的监视者，在监视者创建后将会监视 tf
  // 总线中的坐标变换消息，并将坐标变换缓存10s,
  tf::TransformListener listener;
  // 一秒钟执行10次
  ros::Rate rate(10.0);
  while (ros::ok()) {
    tf::StampedTransform transform;
    try {
      // 获得一个从 /turtle1 框架到　turtle.name 框架的　tf 变换, GF
      listener.lookupTransform(turtle.request.name, "/turtle1", ros::Time(0),
                               transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("lookupTransform failed, %s", ex.what());
      // 等待一秒后继续
      ros::Duration(1.0).sleep();
      continue;
    }
    geometry_msgs::Twist vel_msg;
    // 设置运动的角度
    double x = transform.getOrigin().x(), y = transform.getOrigin().y();
    vel_msg.angular.z = 4.0 * atan(y / x);
    // 设置运动的距离
    vel_msg.linear.x = 0.5 * sqrt(y * y + x * x);

    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
}