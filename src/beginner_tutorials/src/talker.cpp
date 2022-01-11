#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

int main(int argc, char **argv) {
  /**
   * 初始化 ros 节点
   *
   * ros 节点是通过 ros::init 函数进行初始化的,
   * 初始化参数可以这里一样通过命令行闯入, 也可以通过该函数的重载通过程序来配置.
   * 最后一个参数 "talker" 指定了节点的名字.
   *
   * 这里指定的节点名称不能包含命名空间, 即节点名称中不能包含 "/"
   */
  ros::init(argc, argv, "talker");
  /**
   * ros 节点通过 ros::NodeHandle 与 ros 系统交互, 在一个进程中可以多次构造
   * ros::NodeHandle, 但是只有第一次构造会将ros节点注册到ros系统中, 而最后一个
   * ros::NodeHandle 的销毁或导致节点的关闭.
   */
  ros::NodeHandle handle;

  /**
   * 通过 NodeHandle::advertise 可以通知 ros 的 master
   * 节点该节点将发布指定主题的消息, 而 master 节点会通知
   * 所有订阅了该主题的其他节点与该节点协商建立一个点对点的连接. advertise
   * 方法会返回一个发布者对象, 通过该对象可以 发布对应主题的消息,
   * 当该对象以及所有该对象的拷贝销毁后, 将会通知 master
   * 节点该节点不在发布对应主题的消息.
   *
   * advertise 方法的第二个参数指定了消息发布队列的大小,
   * 当待发送消息的数量大于消息发送的速度时, 未发送的消息将缓存在 消息队列中,
   * 而当消息队列中的消息到达这个大小后, 后续的消息将会被丢弃.
   */
  ros::Publisher chatter_pub =
      handle.advertise<std_msgs::String>("chatter", 1000);

  /**
   * 通过 loop_rate 可以控制循环的调用频率
   *
   * 构造参数为调用调用频率(Hz)
   */
  ros::Rate loop_rate(10);
  int count = 0;
  /**
   * ok 方法将一直返回true , 直到
   * 1. 收到 SIGINT 信号(如用户按下了 Ctrl+C)
   * 2. 应用程序调用了 ros::shutdown 方法
   * 3. 所有的 NodeHandle 实例都被销毁了
   * 4. 另一个同名的节点启动了
   */
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;

    ss << "hello world " << count++;
    msg.data = ss.str();

    ROS_INFO("publishing %s", msg.data.c_str());

    /**
     * 发布一个消息, 发布的消息类型必须要与 advertise 创建是指定的模板参数一致.
     */
    chatter_pub.publish(msg);

    /**
     * 完成一次 ros 消息循环
     */
    ros::spinOnce();
    /**
     * 等待剩余的时间后再次运行循环
     *
     * ros 的消息循环将会执行指定小消息订阅回调或者服务回调
     */
    loop_rate.sleep();
  }
  ROS_INFO("ros chatter node is shutdown");
  return 0;
}