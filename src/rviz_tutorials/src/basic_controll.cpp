#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <locale>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
using namespace visualization_msgs;
void processCallback(InteractiveMarkerFeedbackConstPtr feedback);
/**
 * @brief 创建一个交互控制盒子
 * 盒子的大小根据父交互图形的大小决定，
 * @param parent 相应的交互图形
 */
Marker makeBox(InteractiveMarker &parent) {
  Marker marker;
  marker.type = Marker::CUBE;
  marker.scale.x = parent.scale * 0.45;
  marker.scale.y = parent.scale * 0.45;
  marker.scale.z = parent.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  return marker;
}
/**
 * @brief 在交互图形中添加一个交互盒子
 * @param parent 父交互图形
 * @returns 相应盒子的控制器
 */
InteractiveMarkerControl &makeBoxControl(InteractiveMarker &parent) {
  InteractiveMarkerControl control;
  control.name = "box";
  control.always_visible = true;
  control.markers.push_back(makeBox(parent));
  parent.controls.push_back(control);
  return parent.controls.back();
}

/**
 * @brief  创建一个6维控制图形
 * @param server 交互服务器实例
 * @param fixed 图形的方向是否可以改变
 * @param interactive_mode 控制器的模式，包括
 *     MOVE_3D 3维平移控制
 *     ROTATE_3D 旋转控制器，可以在父框架中进行旋转
 *     MOVE_ROTATE_3D 平移旋转可以在
 * @param position 交互图形所在位置
 * @param show_6dof 是否添加6自由度的控制器
 */
void make6DofMaker(interactive_markers::InteractiveMarkerServer &server,
                   bool fixed, unsigned int interactive_mode,
                   const tf::Vector3 &position, bool show_6dof) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1.0;

  int_marker.name = "simple_6doc";
  int_marker.description = "Simple 6 Dof Control";
  // 创建一个盒子
  InteractiveMarkerControl &boxControl = makeBoxControl(int_marker);
  boxControl.interaction_mode = interactive_mode;

  InteractiveMarkerControl control;
  if (fixed) {
    int_marker.name += "_fix";
    int_marker.description += "\n(fix direction)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interactive_mode != InteractiveMarkerControl::NONE) {
    std::string mode_str;
    switch (interactive_mode) {
    case InteractiveMarkerControl::MOVE_3D:
      // 3维平移控制
      mode_str = "MOVE_3D";
      break;
      // 3维旋转控制
    case InteractiveMarkerControl::ROTATE_3D:
      mode_str = "ROTATE_3D";
      break;
      // 3维平移加旋转
    case InteractiveMarkerControl::MOVE_ROTATE_3D:
      mode_str = "MOVE_ROTATE_3D";
      break;
    default:
      break;
    }
    int_marker.name += "_" + mode_str;
    int_marker.description += std::string("3D Control") +
                              (show_6dof ? " + 6-DOF Control" : "") + "\n" +
                              mode_str;
    ;
  }
  // 创建6自由度的控制器
  if (show_6dof) {
    control.orientation.w = 1.0;
    control.orientation.x = 1.0;
    control.orientation.y = 0.0;
    control.orientation.z = 0.0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1.0;
    control.orientation.x = 0.0;
    control.orientation.y = 0.0;
    control.orientation.z = 1.0;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1.0;
    control.orientation.x = 0.0;
    control.orientation.y = 1.0;
    control.orientation.z = 0.0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server.insert(int_marker);
  server.setCallback(int_marker.name, &processCallback);
}
/**
 * @brief 创建一个基于视觉平面（ViewFacing） 控制器
 */
void makeViewFacingMarker(interactive_markers::InteractiveMarkerServer &server,
                          const tf::Vector3 &position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;
  int_marker.name = "view_facing";
  int_marker.description = "View facing 6-DOF";
  InteractiveMarkerControl control;
  // 创建一个围绕着与摄像头平面垂直的轴的旋转控制器
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation.w = 1;
  control.name = "rotate";

  int_marker.controls.push_back(control);
  // 创建一个盒子，拖动盒子可以在摄像头平面内运动
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  // 如果设置为 false 盒子将会一直面向摄像机平面
  control.independent_marker_orientation = true;

  control.markers.push_back(makeBox(int_marker));
  control.always_visible = true;
  control.name = "box_move";
  int_marker.controls.push_back(control);

  server.insert(int_marker);
  server.setCallback(int_marker.name, processCallback);
}

/**
 * @brief  创建一个直升机模型 Quadrocopter 控制器
 */
void makeQuadrocopterMarker(
    interactive_markers::InteractiveMarkerServer &server,
    const tf::Vector3 &position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "quadrocopter";
  int_marker.description = "Quadrocopter";

  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.orientation.w = 1;
  // 创建一个圆环，拖动圆环可以进行在 orientation 垂直的平面内旋转和移动
  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  control.name = "move_rotate_xy";
  int_marker.controls.push_back(control);

  // 创建一个一维平移移动轴， orientation 为轴的方向
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move_z";
  int_marker.controls.push_back(control);

  server.insert(int_marker);
  server.setCallback(int_marker.name, &processCallback);
}

/**
 * @brief 创建一个棋盘对齐控制图形
 */
void makeChessPieceMarker(
    interactive_markers::InteractiveMarkerServer &server,
    const tf::Vector3 &position,
    const interactive_markers::InteractiveMarkerServer::FeedbackCallback
        &alignMarkerFeedback) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "chess_piece";
  int_marker.description = "Chess Piece\n(2D MOVE and Align)";

  InteractiveMarkerControl control;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.orientation.w = 1;

  // 创建一个二维移动圆盘
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.name = "move_plane";
  int_marker.controls.push_back(control);

  // 创建一个可以二维移动的盒子
  control.markers.push_back(makeBox(int_marker));
  control.always_visible = true;
  control.name = "box";
  int_marker.controls.push_back(control);

  server.insert(int_marker);
  server.setCallback(int_marker.name, &processCallback);
  server.setCallback(int_marker.name, alignMarkerFeedback,
                     InteractiveMarkerFeedback::POSE_UPDATE);
}
/**
 * @brief 创建二维控制转台控制器
 */
void makePanTiltMarker(interactive_markers::InteractiveMarkerServer &server,
                       const tf::Vector3 &posiiton) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(posiiton, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "pan_tilt";
  int_marker.description = "Pan / Tilt";

  makeBoxControl(int_marker);

  InteractiveMarkerControl control;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.orientation.w = 1;

  // 创建一个法向量为 y 方向的倾斜旋转控制器
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.y = 1;
  control.orientation.z = 0;
  // 创建一个垂直于 z 方向并且方向固定的平面旋转控制器
  control.orientation_mode = InteractiveMarkerControl::FIXED;
  int_marker.controls.push_back(control);

  server.insert(int_marker);
  server.setCallback(int_marker.name, &processCallback);
}

void makeMenuMarker(interactive_markers::InteractiveMarkerServer &server,
                    const tf::Vector3 &position,
                    interactive_markers::MenuHandler &menu_handler) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1.0;

  int_marker.name = "context_menu";
  int_marker.description = "Context Menu\bRight Click";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";

  control.markers.push_back(makeBox(int_marker));
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server.insert(int_marker);
  server.setCallback(int_marker.name, &processCallback);

  // 将右键菜单添加到 对应的名称的marker上
  menu_handler.apply(server, int_marker.name);
}
/**
 * @brief 创建一个按钮
 */
void makeButtonControl(interactive_markers::InteractiveMarkerServer &server,
                       const tf::Vector3 &position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "button";
  int_marker.description = "Button\nLeft Click";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;

  control.markers.push_back(makeBox(int_marker));
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server.insert(int_marker);
  server.setCallback(int_marker.name, &processCallback);
}
/**
 * @brief 创建一个可以在移动框架中的盒子
 */
void makeMovingMarker(interactive_markers::InteractiveMarkerServer &server,
                      const tf::Vector3 &position, bool isRotate = false) {
  InteractiveMarker int_marker;

  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1.0;

  if (isRotate) {
    int_marker.header.frame_id = "rotating_frame";
    int_marker.name = "rotating_marker";
    int_marker.description = "Marker in Rotating frame";
  } else {
    int_marker.header.frame_id = "moving_frame";
    int_marker.name = "moving_marker";
    int_marker.description = "Marker in moving frame";
  }

  InteractiveMarkerControl control;

  control.orientation.x = 1.0;
  control.orientation.y = 0.0;
  control.orientation.z = 0.0;
  control.orientation.w = 1.0;
  // 创建一个可以在 y-z 平面的旋转控制器
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  // 创建一个可以在 y-z 平面内平移的盒子
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.always_visible = true;
  control.markers.push_back(makeBox(int_marker));
  int_marker.controls.push_back(control);

  server.insert(int_marker);
  server.setCallback(int_marker.name, &processCallback);
}
void frameCallback(const ros::TimerEvent &event);
int main(int argc, char *argv[]) {
  setlocale(LC_CTYPE, "zh_CN.utf-8");
  ros::init(argc, argv, "basic_controll");
  ros::NodeHandle handle;

  ros::Timer frame_timer =
      handle.createTimer(ros::Duration(0.01), &frameCallback);
  interactive_markers::InteractiveMarkerServer server("basic_controls", "",
                                                      false);
  tf::Vector3 position;
  position = tf::Vector3(-3, 3, 0);
  // 简单的6自由与控制器
  make6DofMaker(server, false, InteractiveMarkerControl::NONE, position, true);

  position = tf::Vector3(0, 3, 0);
  // 固定角度的自由度控制器
  make6DofMaker(server, true, InteractiveMarkerControl::NONE, position, true);

  position = tf::Vector3(3, 3, 0);
  // 3D 平移控制
  make6DofMaker(server, false, InteractiveMarkerControl::MOVE_3D, position,
                false);

  position = tf::Vector3(-3, 0, 0);
  // 3D 平移及旋转控制
  make6DofMaker(server, false, InteractiveMarkerControl::MOVE_ROTATE_3D,
                position, false);

  position = tf::Vector3(0, 0, 0);
  // 3D 旋转控制并添加6自由度控制器
  make6DofMaker(server, false, InteractiveMarkerControl::ROTATE_3D, position,
                true);

  position = tf::Vector3(3, 0, 0);
  // 创建一个 ViewFacing 控制图形
  makeViewFacingMarker(server, position);

  position = tf::Vector3(-3, -3, 0);
  // 创建一个 Quadrocopter 控制模式的盒子
  makeQuadrocopterMarker(server, position);

  interactive_markers::InteractiveMarkerServer::FeedbackCallback
      alignMarkerFeedback =
          [&](const InteractiveMarkerFeedbackConstPtr &feedback) {
            geometry_msgs::Pose pose = feedback->pose;
            pose.position.x = round(pose.position.x - 0.5) + 0.5;
            pose.position.y = round(pose.position.y - 0.5) + 0.5;

            ROS_INFO_STREAM(
                feedback->marker_name
                << ": aligning position =" << feedback->pose.position.x << ","
                << feedback->pose.position.y << "," << feedback->pose.position.z
                << " to position " << pose.position.x << "," << pose.position.y
                << "," << pose.position.z);
            server.setPose(feedback->marker_name, pose);
            server.applyChanges();
          };
  position = tf::Vector3(0, -3, 0);
  // 创建一个 Chess Piece 控制模式的盒子
  makeChessPieceMarker(server, position, alignMarkerFeedback);

  position = tf::Vector3(3, -3, 0);
  // 创建一个二维转台控制模式的盒子
  makePanTiltMarker(server, position);

  // 菜单创建
  interactive_markers::MenuHandler menu_handler;
  menu_handler.insert("First Entry", &processCallback);
  // 创建一个包括子选项的菜单
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle =
      menu_handler.insert("Submenu Entry");
  // 创建子选项
  menu_handler.insert(sub_menu_handle, "First Entry", &processCallback);

  position = tf::Vector3(-3, -6, 0);
  // 创建可以弹出右键菜单的图形
  makeMenuMarker(server, position, menu_handler);

  position = tf::Vector3(0, -6, 0);
  // 创建一个可以点击的图形
  makeButtonControl(server, position);

  position = tf::Vector3(-3, -9, 0);
  // 在平移框架中创建一个图形
  makeMovingMarker(server, position);

  position = tf::Vector3(0, -9, 0);
  // 在旋转框架中创建一个图形
  makeMovingMarker(server, position, true);

  server.applyChanges();
  ros::spin();
  return 0;
}
void frameCallback(const ros::TimerEvent &event) {
  static u_int32_t counter;
  static tf::TransformBroadcaster br;

  tf::Transform t;
  ros::Time now = ros::Time::now();
  // 设置平移框架的坐标转换
  t.setOrigin(tf::Vector3(0.0, 0.0, 2.0 * sin(counter / 140.0)));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, now, "base_link", "moving_frame"));
  // 设置旋转的框架 rotating_frame 的相对与固定框架的坐标转换
  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, counter / 140.0, 0.0));
  br.sendTransform(tf::StampedTransform(t, now, "base_link", "rotating_frame"));

  ++counter;
}
void processCallback(InteractiveMarkerFeedbackConstPtr feedback) {
  std::string basic_info = "来自图形-" + feedback->marker_name + "/控制器-" +
                           feedback->control_name + "的反馈消息:";
  geometry_msgs::Point position = feedback->pose.position;
  geometry_msgs::Quaternion orientation = feedback->pose.orientation;

  std::stringstream mouse_stream;
  if (feedback->mouse_point_valid) {
    geometry_msgs::Point mouse_point = feedback->mouse_point;
    mouse_stream << "鼠标位置: " << mouse_point.x << ", " << mouse_point.y
                 << ", " << mouse_point.z << "\n";
  }
  switch (feedback->event_type) {
  case InteractiveMarkerFeedback::POSE_UPDATE:
    ROS_INFO_STREAM(basic_info << "位置改变\n    位置：" << position.x << ", "
                               << position.y << ", " << position.z
                               << "\n    方向：" << orientation.x << ", "
                               << orientation.y << ", " << orientation.z
                               << "\n    框架:" << feedback->header.frame_id
                               << "\n    时间:" << feedback->header.stamp);
    break;
  case InteractiveMarkerFeedback::MENU_SELECT:
    ROS_INFO_STREAM(basic_info << "菜单选择\n"
                               << "菜单名称:" << feedback->menu_entry_id
                               << std::endl
                               << mouse_stream.str());
    break;
  case InteractiveMarkerFeedback::MOUSE_DOWN:
    ROS_INFO_STREAM(basic_info << "鼠标按下\n" << mouse_stream.str());
    break;
  case InteractiveMarkerFeedback::MOUSE_UP:
    ROS_INFO_STREAM(basic_info << "鼠标释放\n" << mouse_stream.str());
    break;
  case InteractiveMarkerFeedback::BUTTON_CLICK:
    ROS_INFO_STREAM(basic_info << "按钮点击\n" << mouse_stream.str());
    break;
  default:
    break;
  }
}