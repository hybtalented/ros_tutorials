#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>

/**
 * @brief  用户交互处理函数
 * @param feedback 交互的具体内容
 *
 * 这个处理函数仅仅打印了图形的当前位置
 */
void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  geometry_msgs::Point position = feedback->pose.position;
  ROS_INFO_STREAM(feedback->marker_name << "is now at (" << position.x << ","
                                        << position.y << "," << position.z
                                        << ")");
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "simple_interactive_marker");
  ros::NodeHandle handle;

  // 在命名空间 simple_marker 上创建一个交互服务器
  interactive_markers::InteractiveMarkerServer server("simple_marker");

  // 创建一个可交互的图形
  visualization_msgs::InteractiveMarker int_marker;

  // 设置图形的框架和时间戳
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp = ros::Time::now();

  // 设置图形的名称以及描述
  int_marker.name = "my_marker";
  int_marker.description = "simple 1-DOF Control";

  // 创建一个灰色的基本盒子
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // 创建一个不可交互的控制器, 用于放置盒子
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(box_marker);

  // 创建一个控制器用于移动可交互的图形, 这个控制器中不包含任何基本图形, 这会让
  // rviz 生成一对箭头.
  visualization_msgs::InteractiveMarkerControl move_control_x;
  move_control_x.name = "move_x";
  move_control_x.orientation.x = 1.0;
  move_control_x.orientation.y = 0.0;
  move_control_x.orientation.z = 0.0;
  move_control_x.orientation.w = 1.0;
  move_control_x.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  // 创建一个y 方向的移动控制器
  visualization_msgs::InteractiveMarkerControl move_control_y;
  move_control_y.name = "move_y";
  move_control_y.orientation.x = 0.0;
  move_control_y.orientation.y = 0.0;
  move_control_y.orientation.z = 1.0;
  move_control_y.orientation.w = 1.0;
  move_control_y.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  // 在可交互图形上添加各个控制器
  int_marker.controls.push_back(box_control);
  int_marker.controls.push_back(move_control_x);
  int_marker.controls.push_back(move_control_y);

  // 将可交互图形放到交互服务器中, 并告诉服务器在与该图形交互时调用
  // processFeedback
  server.insert(int_marker, &processFeedback);

  server.applyChanges();

  ros::spin();
  return 0;
}
