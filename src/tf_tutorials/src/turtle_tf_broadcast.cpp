#include <ros/ros.h>
// tf 包提供的一种简化坐标变换广播功能的类 TransformBroadcaster
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/Pose.h>
/**
 * @brief 获得乌龟的位置后，将二维位置作为tf消息广播
 */
void poseTransformCallback(std::string turtleName,
                           const turtlesim::PoseConstPtr pose) {
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStampled;
  // 设置坐标变换的基本内容
  transformStampled.header.frame_id = "world";
  transformStampled.header.stamp = ros::Time::now();
  transformStampled.child_frame_id = turtleName;
  // 设置坐标的原点位置，因为是二维空间移动，将z坐标固定为常数
  transformStampled.transform.translation.x = pose->x;
  transformStampled.transform.translation.y = pose->y;
  transformStampled.transform.translation.z = 0;
  // 设置围绕z轴的旋转角度
  tf2::Quaternion rotation;
  rotation.setRPY(0, 0, pose->theta);
  transformStampled.transform.rotation.x = rotation.x();
  transformStampled.transform.rotation.y = rotation.y();
  transformStampled.transform.rotation.z = rotation.z();
  transformStampled.transform.rotation.w = rotation.w();
  // 广播对应框架的坐标变换消息
  br.sendTransform(transformStampled);
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "turtle_broadcast");
  if (argc != 2) {
    ROS_ERROR("need turtle name as argument");
    return -1;
  }
  // 可以通过命令行参数获取想要绑定框架的　turtle 的名称
  std::string turtleName = argv[1];

  ros::NodeHandle node;
  // 订阅对应 turtle 的位置消息，在乌龟创建后，每隔一段时间会自动发送该消息
  ros::Subscriber sub = node.subscribe<turtlesim::Pose>(
      turtleName + "/pose", 10,
      boost::bind(&poseTransformCallback, turtleName, boost::placeholders::_1));
  // 进入 ros 的消息循环, 已处理订阅的消息
  ros::spin();
  return 0;
}