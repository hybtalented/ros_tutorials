#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "basic_shape");
  ros::NodeHandle handle;
  // 一秒钟运行一次
  ros::Rate r(1);
  ros::Publisher marker_pub =
      handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  visualization_msgs::Marker marker;
  // 设置标记的框架
  marker.header.frame_id = "/my_frame";
  // 设置标记的命名空间和 id, 如果多次发布同一个 命名空间和 id 的标记到 rviz,
  marker.ns = "basic_shape";
  marker.id = 0;
  // 将会覆盖掉之前发送的标记 设置标记的初始形状
  marker.type = visualization_msgs::Marker::CUBE;
  // action 表示操作, ADD 表示添加标记, DELETE 表示删除标记, DELETEALL
  // 表示删除所有标记, MODIFY 表示修改标记
  marker.action = visualization_msgs::Marker::ADD;
  // 设置标记的位置
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  // 设置标记的方向
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  // 设置标记的颜色
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;

  // 设置标记的大小
  marker.scale.x = 5.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // lifetime 用于告诉 rviz 多长时间后自动删除该标记,  ros::Duration()
  // 表示永不删除
  marker.lifetime = ros::Duration();
  while (ros::ok()) {
    // 如果尚未有任何主题订阅在, 则进行等待
    while (marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscribe to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    r.sleep();
    switch (marker.type) {
    case visualization_msgs::Marker::CUBE:
      marker.type = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      marker.type = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      marker.type = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      marker.type = visualization_msgs::Marker::CUBE;
      break;
    }
  }
  return 0;
}