#include <beginner_tutorials/AddTwoInts.h>
#include <ros/ros.h>
/**
 * @brief 服务处理函数, sum = a + b 
 * @param req 请求参数
 * @param res 返回参数
 * 
 * @returns 服务需要返回 true 通知服务调用成功
 */
bool add(beginner_tutorials::AddTwoIntsRequest &req,
         beginner_tutorials::AddTwoIntsResponse &res) {
  res.sum = req.a + req.b;
  ROS_INFO("add_two_ints: %ld + %ld, response %ld", req.a, req.b, res.sum);
  return true;
}

int main(int argc, char **argv) {
  // 初始化 ros 节点
  ros::init(argc, argv, "add_two_ints_server");
  // 创建一个 ros 节点句柄
  ros::NodeHandle handle;
  /**
   * 创建一个服务提供者对象
   * 
   * 通过 NodeHandle::advertiseService 向 ros 的 master 节点 注册一个 名称为 add_two_ints 的 服务,
   * add 函数作为服务的处理函数. advertiseService 返回一个 服务提供者实例, 在所有服务提供者实例及其拷贝
   * 被销毁后, 相应的服务将会从 ros master 中注销.
   */
  ros::ServiceServer server = handle.advertiseService("add_two_ints", add);

  ROS_INFO("ready to serve add_two_ints service");
  // 进入 ros 的消息循环
  ros::spin();
  return 0;
}