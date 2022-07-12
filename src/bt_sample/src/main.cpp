#include "movebase_client.h"
#include "sensor_nodes.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

using namespace BT;

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_bt");

  ros::NodeHandle nh("~");
  std::string xml_filename;
  nh.param<std::string>("file", xml_filename, "../bt/patrolling_battery.xml");
  ROS_INFO("Loading XML : %s", xml_filename.c_str());

  // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;

  factory.registerNodeType<MoveBase>("MoveBase");
  factory.registerNodeType<BatteryCondition>("BatteryCondition");
  factory.registerNodeType<BatteryCondition_charging>("BatteryCondition_charging");

  // Trees are created at deployment-time (i.e. at run-time, but only once at
  // the beginning). The currently supported format is XML. IMPORTANT: when the
  // object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromFile(xml_filename);

  // Create a logger
  StdCoutLogger logger_cout(tree);

  // This logger publish status changes using ZeroMQ. Used by Groot
  PublisherZMQ publisher_zmq(tree);
  ros::Rate rate(10);

  NodeStatus status = NodeStatus::RUNNING;
  // Keep on ticking until you get either a SUCCESS or FAILURE state
  while (ros::ok() && status == NodeStatus::RUNNING) {
    status = tree.tickRoot();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}