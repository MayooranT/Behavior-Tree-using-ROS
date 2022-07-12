#pragma once

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <tf/transform_datatypes.h>
#include <yaml-cpp/yaml.h> 
#include <fstream>
#include <iostream>

// Custom type
struct Pose2D
{
    double x, y, quaternion_z, quaternion_w;
};


namespace BT
{
template <> inline
Pose2D convertFromString(StringView key)
{
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 4)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Pose2D output;
        output.x     = convertFromString<double>(parts[0]);
        output.y     = convertFromString<double>(parts[1]);
        output.quaternion_z = convertFromString<double>(parts[2]);
        output.quaternion_w = convertFromString<double>(parts[3]);
        return output;
    }
}
} // end namespace BT

//----------------------------------------------------------------

class MoveBase : public BT::AsyncActionNode
{
public:
    MoveBase(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config),
          _client("move_base", true)
    {
        YAML::Node test = YAML::LoadFile("/home/mayooran/BT/behavior-tree/src/bt_sample/param/bt_goals.yaml");
        goal_a = test["goal_a"].as<std::string>();
        goal_b = test["goal_b"].as<std::string>();
        charging_dock = test["charging_dock"].as<std::string>();
        std::ofstream fout("/home/mayooran/BT/behavior-tree/src/bt_sample/param/bt_goals.yaml");
        fout << test;
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("goal") };
    }

    virtual BT::NodeStatus tick() override;

    virtual void halt() override
    {
        _aborted = true;
    }

private:
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    MoveBaseClient _client;
    bool _aborted;

    std::string goal_a;
    std::string goal_b;
    std::string charging_dock;
    
};