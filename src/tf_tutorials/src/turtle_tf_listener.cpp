#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <turtlesim/Spawn.h>
int main(int argc, char *args[]) {
  ros::init(argc, args, "tf_listener");
  ros::NodeHandle node;

  // 等待 /spawn 服务添加到ros系统
  ros::service::waitForService("/spawn");
  // 创建客户端
  ros::ServiceClient add_turtle =
      node.serviceClient<turtlesim::Spawn>("/spawn");
  std::string targetTurtleName = "turtle1";
  std::string turtleName = "turtle2";
  turtlesim::Spawn turtle;
  turtle.request.x = 4;
  turtle.request.y = 2;
  turtle.request.theta = 0;
  turtle.request.name = "/" + turtleName;
  add_turtle.call(turtle);

  // 用于控制 /turtle2 的速度
  ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>(
      turtle.request.name + "/cmd_vel", 10);

  // 创建坐标变换的缓存，并设置坐标变换缓存10s
  tf2::BufferCore tf_buffer(ros::Duration(10));
  // 用于监听 tf 框架坐标变换， 创建以后开始监听ros消息
  // 总线中的坐标变换消息
  tf2_ros::TransformListener listener(tf_buffer);
  // 一秒钟执行2次
  ros::Rate rate(100);
  while (ros::ok()) {
    std::string error;
    geometry_msgs::TransformStamped transformStamped;
    try {
      // 获得一个从 turtle1 框架到　turtle12 框架的　tf 变换
      transformStamped =
          tf_buffer.lookupTransform(turtleName, targetTurtleName, ros::Time(0));
    } catch (tf2::TransformException ex) {
      ROS_ERROR("lookupTransform failed, %s", ex.what());
      // 等待一秒后继续
      ros::Duration(1.0).sleep();
      continue;
    }
    geometry_msgs::Twist vel_msg;

    // 设置 turtle2 的运动方向
    double x = transformStamped.transform.translation.x,
           y = transformStamped.transform.translation.y;

    vel_msg.angular.z = 4.0 * atan(y / x);
    // 设置运动的距离
    vel_msg.linear.x = 0.5 * sqrt(pow(y, 2) + pow(x, 2));

    turtle_vel.publish(vel_msg);
    rate.sleep();
  }
  return 0;
}