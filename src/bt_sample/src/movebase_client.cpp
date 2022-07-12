#include "movebase_client.h"

BT::NodeStatus MoveBase::tick() {
  // if no server is present, fail after 2 seconds
  if (!_client.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact move_base server");
    return BT::NodeStatus::FAILURE;
  }

  auto text= getInput<std::string>("goal");
  std::cout<< "----------------------------------------------------------------Sending the robot to : " << text.value() << std::endl;

  


  Pose2D goal;
  if (text.value() == "Goal_a"){

    // std::string goal_in_string = "-4.1;1.15;0.0;1.0";
    auto parts = BT::splitString(goal_a, ';');
    goal.x     = BT::convertFromString<double>(parts[0]);
    goal.y     = BT::convertFromString<double>(parts[1]);
    goal.quaternion_z = BT::convertFromString<double>(parts[2]);
    goal.quaternion_w = BT::convertFromString<double>(parts[3]);
        
  }
  else if (text.value() == "Goal_b"){

    // std::string goal_in_string = "-2.504;4.455;0.0;1.0";
    auto parts = BT::splitString(goal_b, ';');
    goal.x     = BT::convertFromString<double>(parts[0]);
    goal.y     = BT::convertFromString<double>(parts[1]);
    goal.quaternion_z = BT::convertFromString<double>(parts[2]);
    goal.quaternion_w = BT::convertFromString<double>(parts[3]);
  }
  else if (text.value() == "charging_dock"){

    // std::string goal_in_string = "0.386;4.669;0.0;1.0";
    auto parts = BT::splitString(charging_dock, ';');
    goal.x     = BT::convertFromString<double>(parts[0]);
    goal.y     = BT::convertFromString<double>(parts[1]);
    goal.quaternion_z = BT::convertFromString<double>(parts[2]);
    goal.quaternion_w = BT::convertFromString<double>(parts[3]);
  }
  else {
    ROS_INFO("Wrong goal name in the BT xml file!!!!");
  }

  setOutput<Pose2D>("goal", goal);

  // Reset this flag
  _aborted = false;
 
  // Build the message from Pose2D
  move_base_msgs::MoveBaseGoal msg;
  msg.target_pose.header.frame_id = "map";
  msg.target_pose.header.stamp = ros::Time::now();
  msg.target_pose.pose.position.x = goal.x;
  msg.target_pose.pose.position.y = goal.y;
  msg.target_pose.pose.orientation.z = goal.quaternion_z;
  msg.target_pose.pose.orientation.w = goal.quaternion_w;

  _client.sendGoal(msg);

  while (!_aborted && !_client.waitForResult(ros::Duration(0.02))) {
    // polling at 50 Hz. No big deal in terms of CPU
  }

  if (_aborted) {
    // this happens only if method halt() was invoked
    //_client.cancelAllGoals();
    ROS_ERROR("MoveBase aborted");
    return BT::NodeStatus::FAILURE;
  }

  if (_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("MoveBase failed");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("Target reached");
  return BT::NodeStatus::SUCCESS;
}