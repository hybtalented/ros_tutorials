#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle handle;
  ros::Publisher marker_pub =
      handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate rate(30);
  visualization_msgs::Marker points, line_strip, line_list;
  points.header.frame_id = line_strip.header.frame_id =
      line_list.header.frame_id = "/my_frame";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp =
      ros::Time::now();
  points.action = line_strip.action = line_list.action =
      visualization_msgs::Marker::ADD;
  points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points.id = 0;
  line_strip.id = 1;
  line_list.id = 2;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  points.pose.orientation.w = line_strip.pose.orientation.w =
      line_list.pose.orientation.w = 1.0;
  // 点通过 scale.x, scale.y 属性设置宽度和高度
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  // 线和线段通过 scale.x 设置线宽
  line_strip.scale.x = 0.1;
  line_list.scale.x = 0.1;
  // 设置点为绿色
  points.color.g = 1.0f;
  points.color.a = 1.0f;

  // 设置线为蓝色
  line_strip.color.b = 1.0f;
  line_strip.color.a = 1.0f;

  // 设置线段为红色
  line_list.color.r = 1.0f;
  line_list.color.a = 1.0f;
  double start_angle = 0.0f;

  while (ros::ok()) {
    points.points.clear();
    line_strip.points.clear();
    line_list.points.clear();
    // 添加圆 x^2 + y^2 = 25 上的点
    for (int32_t i = 0; i < 100; ++i) {
      double a = start_angle + i / 100.0 * 2 * M_PI;
      geometry_msgs::Point p;
      p.x = i - 50;
      p.y = 5 * sin(a);
      p.z = 5 * cos(a);
      points.points.push_back(p);
      line_strip.points.push_back(p);
      line_list.points.push_back(p);
      p.z += 1.0;
      line_list.points.push_back(p);
    }
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);
    rate.sleep();
    start_angle += 0.04;
  }
  return 0;
}