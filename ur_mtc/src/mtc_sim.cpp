#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur_control");
namespace mtc = moveit::task_constructor;

struct poses{
    float x;
    float y;
    float z;
    float rx;
    float py;
    float yz;      
};
float cup_radius = 0.035; // meters
float cup_height = 0.09;
// UR Gazebo Pose 13.9, -18.56, 1.032
// Cup Gazebo Pose 14.2, -18.1, 1.09
// Coffee Machine Gazebo Pose 14, -17.7, 1
poses cup_pose = {0.3, 0.45, 0.045, 0.0, 0.0, 0.0};
poses fill_pose = {0.4, 0.6, 0.1, 0.0, 0.0, 0.0}; // TEST VALUES ADJUST AS NEEDED
poses regrasp_pose = {0.3, 0.45, 0.065, 0.0, 0.0, 0.0}; // TEST VALUES ADJUST AS NEEDED
poses grasp_frame_transform = {0.0, 0.02, 0.135, M_PI/2, 0.0, 0.0}; // {x, y, z, rx, py, yz} Orients gripper and open/close horizontal
poses regrasp_frame_transform = {0.0, 0.0, 0.05, M_PI, 0.0, 0.0}; // TEST VALUE ADJUST AS NEEDED
// Utility Functions
Eigen::Isometry3d toEigen(const poses& val) {
	return Eigen::Translation3d(val.rx, val.py, val.yz) *
	       Eigen::AngleAxisd(val.x, Eigen::Vector3d::UnitX()) *
	       Eigen::AngleAxisd(val.y, Eigen::Vector3d::UnitY()) *
	       Eigen::AngleAxisd(val.z, Eigen::Vector3d::UnitZ());
}
geometry_msgs::msg::Pose toPose(const poses& val) {
	return tf2::toMsg(toEigen(val));
}

class UrControl
{
public:
    UrControl(const rclcpp::NodeOptions & options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

    void doTask();
    void setupPlanningScene();
private:
    mtc::Task createTask();
    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
    void connectorStage(std::string stage_name, float timeout);
    
}; // END CLASS


// Definitions
UrControl::UrControl(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("control_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr UrControl::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}


void UrControl::setupPlanningScene()
{
    moveit_msgs::msg::CollisionObject cup;
    cup.id = "cup";
    cup.header.frame_id = "world";
    cup.primitives.resize(1);
    cup.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cup.primitives[0].dimensions = { cup_height, cup_radius }; // FINAL ADJUSTMENT NEEDED

    geometry_msgs::msg::Pose pose = toPose(cup_pose);
    cup.primitive_poses.push_back(pose);

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(cup);
}

void UrControl::doTask()
{
    task_ = createTask();

    try
    {
        task_.init();
    }
    catch (mtc::InitStageException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return;
    }

    if (task_.plan(10))
    {
        
        RCLCPP_INFO(LOGGER, "Task planning succeded");
        
    } else {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
    }
    task_.introspection().publishSolution(*task_.solutions().front());
    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }

    return;
}

void UrControl::connectorStage(std::string stage_name, float timeout)
{
    auto stage = std::make_unique<mtc::stages::Connect>(
            stage_name, mtc::stages::Connect::GroupPlannerVector{{ arm_group_name, sampling_planner }}); 
        stage->setTimeout(timeout);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
}
mtc::Task UrControl::createTask()
{
    mtc::Task task;
    task.stages()->setName("Coffee PnP");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "ur_manipulator";
    const auto& gripper_group_name = "gripper";
    const auto& gripper_frame = "tool0"; 

    task.setProperty("group", arm_group_name);
    task.setProperty("eef", gripper_group_name); 
    task.setProperty("hand", gripper_group_name);
	task.setProperty("hand_grasping_frame", gripper_frame);
    task.setProperty("ik_frame", gripper_frame); 

    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
    
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(0.01);

    // ***STAGES***
    // -> Current State Pointer ->
    mtc::Stage* current_state_ = nullptr;
    {
		auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");

		// Verify that object is not attached
		auto applicability_filter =
		    std::make_unique<mtc::stages::PredicateFilter>("applicability test", std::move(current_state));
		applicability_filter->setPredicate([object = "cup"](const mtc::SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				comment = "object is already attached and cannot be picked";
				return false;
			}
			return true;
		});
		task.add(std::move(applicability_filter));
	}
    // ADD TO READY POSITION?
    // ***Close Gripper <MoveTo>***
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
        stage->setGroup(gripper_group_name);
        stage->setGoal("close");       
        task.add(std::move(stage));    
    }

    // ***Open Gripper <MoveTo>***
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
        stage->setGroup(gripper_group_name);
        stage->setGoal("open"); 
        // -> Set Current State Pointer ->
        current_state_ = stage.get();
        task.add(std::move(stage));
    }

    // ***Move to Pre-Grasp Position <Connector>***
    connectorStage("pre-grasp position", 10.0);
    // -> Grasp Stage Pointer ->
    mtc::Stage* grasp_stage_ = nullptr;
    
    // ***Initial Grasp Container***
    {
        auto grasp = std::make_unique<mtc::SerialContainer>("initial grasp");
        // Pass Properties from Task to Container
        task.properties().exposeTo(grasp->properties(), {"eef", "hand", "group", "ik_frame"});
        grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});
        
        // ***Approach Cup <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
            stage->properties().set("marker_ns", "approach cup");
            stage->properties().set("link", gripper_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.1, 0.2);

             // Set hand forward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = gripper_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        // ***Grasp Pose <Generator>***
        {
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate initial grasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "initial_grasp_pose"); 
            stage->setPreGraspPose("open");
            stage->setObject("cup");
            stage->setAngleDelta(M_PI / 12); 
            stage->setMonitoredStage(current_state_);
            
            // ***Compute IK <Wrapper>***
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("initial grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(toEigen(grasp_frame_transform), gripper_frame); // Pose and Frame
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            grasp->insert(std::move(wrapper));
        }
        
        // ***Allow Collision <PlanningScene>*** 
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
            stage->allowCollisions(
                "cup", 
                task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
                true);
            grasp->insert(std::move(stage));
        }    
        
        // ***Close Gripper <MoveTo>***
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
            stage->setGroup(gripper_group_name);
            stage->setGoal("open");       
            grasp->insert(std::move(stage));    
        }
        
        // ***Attach Cup <PlanningScene>***  
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cup");
            stage->attachObject("cup", gripper_frame);
            grasp->insert(std::move(stage));
        }
        
        // *** Lift Cup <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift cup", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            //stage->setMinMaxDistance(0.01, 0.1); 
            stage->setIKFrame(gripper_frame);
            stage->properties().set("marker_ns", "lift_cup");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 0.1;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }
        // -> Set Grasp Stage Pointer ->
        grasp_stage_ = grasp.get();
        task.add(std::move(grasp));
    } // END INITIAL GRASP CONTAINER

    // *** Move to Coffee Machine <Connector>***
    connectorStage("move to coffee machine", 10.0);
    /*
    {
        auto stage = std::make_unique<mtc::stages::Connect>("move to coffee machine", 
            mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
                                                    //{gripper_group_name, sampling_planner}
        stage->setTimeout(10.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }   
    */
    // ***Fill Coffee Container***
    // -> Fill Stage Pointer ->
    mtc::Stage* fill_stage_ = nullptr;
    {
        auto fill = std::make_unique<mtc::SerialContainer>("fill coffee");
        task.properties().exposeTo(fill->properties(), {"eef", "hand", "group", "ik_frame"});
        fill->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});
        
        // *** Fill Coffee Pose <Generator>***
        {
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate fill pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "fill_pose");
            stage->setObject("cup");
            // Define Pose 
            geometry_msgs::msg::PoseStamped fill_pose_msg;
            fill_pose_msg.header.frame_id = "cup";
            fill_pose_msg.pose = toPose(fill_pose);
            stage->setPose(fill_pose_msg);
            stage->setMonitoredStage(grasp_stage_);

            // Compute IK
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("fill pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame("cup");
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            fill->insert(std::move(wrapper));
        }

        // ***Retract from Coffee Machine <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retract coffee", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            //stage->setMinMaxDistance(0.1, 0.2);
            stage->setIKFrame(gripper_frame);
            stage->properties().set("marker_ns", "retract_coffee");
            // Set Direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = gripper_frame;
            vec.vector.z = -0.1;
            stage->setDirection(vec);
            fill->insert(std::move(stage));
        }
        // -> Set Fill Stage Pointer ->
        fill_stage_ = fill.get();
        task.add(std::move(fill));
    } // END FILL COFFEE CONTAINER

    // *** Place for Regrasp <Connector>***
    connectorStage("move to regrasp", 10.0);

    // ***Place Coffee for Regrasp Container***
    // -> Place Stage Pointer ->
    mtc::Stage* place_stage_ = nullptr;
    {
        auto place = std::make_unique<SerialContainer>("place for regrasp");
        task.properties().exposeTo(place->properties(), {"eef", "hand", "group", "ik_frame"});
        place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

        // ***Lower Cup <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lower cup", cartesian_planner);
            stage->properties().set("marker_ns", "lower_cup");
            stage->properties().set("link", gripper_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            //stage->setMinMaxDistance(0.05, 0.15);

            // Set down direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = -0.02; // ADJUST IF NEEDED 
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }
        
        //  ***Place Coffee for Regrasp <Generator>***
        {
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate regrasp place pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "regrasp_place");
            stage->setObject("cup");
            // Define Pose
            geometry_msgs::msg::PoseStamped regrasp_pose_msg;
            regrasp_pose_msg.header.frame_id = "cup";
            regrasp_pose_msg.pose = toPose(regrasp_pose);
            stage->setPose(regrasp_pose_msg);
            stage->setMonitoredStage(fill_stage_);

            // ***Compute IK***
            auto wrapper = std::make_unique<stages::ComputeIK>("regrasp place pose IK". std::move(stage));
            wrapper->setMaxIKSolutions(4);
            wrapper->setIKFrame("cup");
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            place->insert(std::move(wrapper));
        }

        // ***Open Gripper <MoveTo>***
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
            stage->setGroup(gripper_group_name);
            stage->setGoal("open");
            place->insert(std::move(stage));
        }
        
        // ***Disable Collision <PlanningScene>***
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("disable collision");
            stage->allowCollisions("cup", task.getRobotModel()->getJointModelGroup(gripper_group_name),
                false); 
            place->insert(std::move(stage));
        }
        // ***Detach Cup <PlanningScene>***
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach cup");
            stage->detachObject("cup", gripper_frame);
            place->insert(std::move(stage));
        }

        // ***Retreat from Cup <MoveRelative>***
        {
            auto stage = make_unique<mtc::stages::MoveRelative>("retreat from cup", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            //stage->setMinMaxDistance(0.1, 0.15);
            stage->setIKFrame(gripper_frame);
            stage->properties().set("marker_ns", "retreat");

            //Set Retreat Direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = gripper_frame;
            vec.vector.z = -0.1;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }
        // -> Set Place Stage Pointer ->
        place_stage_ = place.get();
        task.add(std::move(place));
    } // END PLACE CONTAINER

    // ***Move to Regrasp <Connector>***
    connectorStage("move to regrasp", 10.0);
    
    // -> Regrasp Stage Pointer ->
    mtc::Stage* regrasp_stage_ = nullptr;
    // ***Regrasp Container***
    {
        auto regrasp = std::make_unique<SerialContainer>("regrasp cup");
        task.properties().exposeTo(regrasp->properties(), {"eef", "hand", "group", "ik_frame"});
        regrasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

        // ***Approach Cup <MoveRelative>***
        {
            auto stage = std::make_unique<MoveRelative>("approach regrasp", cartesian_planner);
            stage->properties().set("marker_ns", "approach regrasp");
            stage->properties().set("link", gripper_frame);
            stage->properties().ConfigureInitFrom(mtc::Stage::PARENT, {"group"});
            //stage->setMinMaxDistance(0.075, 0.15);

            // Set Direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = -0.075; // ADJUST
            stage->setDirection(vec);
            regrasp->insert(std::move(stage));
        }

        // ***Generate Grasp Pose <Generator>***
        {
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate regrasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
			stage->properties().set("marker_ns", "regrasp_pose");
			stage->setPreGraspPose("open");
			stage->setObject("cup");
			stage->setAngleDelta(M_PI / 12);
			stage->setMonitoredStage(place_stage_);

            // ***Compute IK***
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("regrasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(toEigen(regrasp_frame_transform), gripper_frame);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			regrasp->insert(std::move(wrapper)); 
        }

        // *** Allow Collision <PlanningScene>***
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
			stage->allowCollisions(
			    "cup",
			    task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
			    true);
			regrasp->insert(std::move(stage));
        }

        // ***Close Gripper <MoveTo>***
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
			stage->setGroup(gripper_group_name);
			stage->setGoal("close");
			regrasp->insert(std::move(stage));    
        }

        // ***Attach Cup <PlanningScene>***
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cup");
			stage->attachObject("cup", gripper_frame);
			regrasp->insert(std::move(stage));
        }

        // ***Lift Cup <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift cup", cartesian_planner);
			stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
			//stage->setMinMaxDistance(0.05, 0.15);
			stage->setIKFrame(gripper_frame);
			stage->properties().set("marker_ns", "lift_cup");

			// Set direction
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = "world";
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			regrasp->insert(std::move(stage));
        }
        
        // -> Set Regrasp Stage Pointer ->
        regrasp_stage_ = regrasp.get();

        task.add(std::move(regrasp));
    } // END REGRASP CONTAINER

    // ***Move to Serve <Connector>
    connectorStage("move to serve", 10.0);

    // -> Serve Stage Pointer ->
    //mtc::Stage* serve_stage_ = nullptr;

    // ***Serve Container***
    {
        auto serve = std::make_unique<SerialContainer>("serve coffee");
        task.properties().exposeTo(serve->properties(), {"eef", "hand", "group", "ik_frame"});
        serve->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

        // ***Position Over Baristabot <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("position for serve", cartesian_planner);
			stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
			//stage->setMinMaxDistance(0.05, 0.15);
			stage->setIKFrame(gripper_frame);
			stage->properties().set("marker_ns", "position_cup");

			// Set direction
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = "world";
			vec.vector.x = -0.15; // DUMMY VARIABLES  TEST AND ADJUST
            vec.vector.y = 0.15
			stage->setDirection(vec);
			serve->insert(std::move(stage));
        }    

        // ***Serve to Baristabot <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("move to baristabot", cartesian_planner);
			stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
			//stage->setMinMaxDistance(0.05, 0.15);
			stage->setIKFrame(gripper_frame);
			stage->properties().set("marker_ns", "serve_cup");

			// Set direction
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = "world";
			vec.vector.z = -0.3; // DUMMY VARIABLE FILL WITH TEST AND ADJUST
			stage->setDirection(vec);
			serve->insert(std::move(stage));
        }   

        //  ***Open Gripper <MoveTo>***
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
            stage->setGroup(gripper_group_name);
            stage->setGoal("open");       
            serve->insert(std::move(stage));    
        }
        // ***Retract from Serve Pose
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("move to baristabot", cartesian_planner);
			stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
			//stage->setMinMaxDistance(0.05, 0.15);
			stage->setIKFrame(gripper_frame);
			stage->properties().set("marker_ns", "serve_cup");

			// Set direction
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = "world";
			vec.vector.z = 0.3; // DUMMY VARIABLE FILL WITH TEST AND ADJUST
			stage->setDirection(vec);
			serve->insert(std::move(stage));
        }   
    } // END SERVE CONTAINER
    // RETURN HOME
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setGoal("home");
        task.add(std::move(stage));
    }

    return task;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_node = std::make_shared<UrControl>(options);
    rclcpp::executors::MultiThreadedExecutor exec;

    auto spin_thread = std::make_unique<std::thread>([&exec, &mtc_node]() {
        exec.add_node(mtc_node->getNodeBaseInterface());
        exec.spin();
        exec.remove_node(mtc_node->getNodeBaseInterface());
    });
    mtc_node->setupPlanningScene();
    mtc_node->doTask();

    spin_thread->join();
    rclcpp::shutdown();

    return 0;
} // END MAIN

