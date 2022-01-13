#include <beginner_tutorials/AddTwoInts.h>
#include <ros/ros.h>

#include <cstdlib>
int main(int argc, char **argv) {
  /**
   * 初始化 ros 节点
   */
  ros::init(argc, argv, "add_two_ints_client");
  /**
   * 创建 ros 节点句柄
   */
  ros::NodeHandle handle;

  /**
   * 创建 ros 客户端
   *
   * 创建一个客户端用于调用 add_two_ints 服务,
   * 可以指定第二个参数以提高服务的调用效率, 以及第三个参数
   * 指定请求连接的连接握手时的请求头.
   *
   * serviceClient 方法返回一个服务调用客户端, 通过该客户端可以调用服务.
   */
  ros::ServiceClient client =
      handle.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  /**
   * 创建一个服务对象, 用于传递请求以及获取返回值
   */
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv)) {
    // 服务调用成功
    ROS_INFO("sum is %ld", srv.response.sum);
    return 0;
  } else {
    // 服务调用失败
    ROS_ERROR("Failed to call service add_two_ints");
    return -1;
  }
}