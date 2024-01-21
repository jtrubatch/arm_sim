#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

#pragma once

namespace ur_mtc {

using namespace moveit::task_constructor;

struct poses{
    float x;
    float y;
    float z;
    float rx;
    float py;
    float yz;      
};

// Utility Functions
Eigen::Isometry3d toEigen(const poses& val);
geometry_msgs::msg::Pose toPose(const poses& val);

class TaskControl {
public:
    TaskControl(const rclcpp::NodeOptions &options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
private:

}; // END CLASS TASKCONTROL
} // END NAMESPACE UR_MTC