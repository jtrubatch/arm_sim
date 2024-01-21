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

float grasp_approach = 0.085;
float grasp_retract = 0.1;
float fill_approach = 0.1;
float fill_retract = -0.1;
float place_approach = -0.02;
float place_retreat = 0.1;
float regrasp_approach = -0.075;
float regrasp_retract = 0.05;
float serve_approach = -0.05;
float serve_retract = 0.25; 

float cup_radius = 0.035; // meters
float cup_height = 0.09;
// UR Gazebo Pose 13.8, -18.56, 1.032
// Cup Gazebo Pose 14.1, -18.2, 1.09
// Coffee Machine Gazebo Pose 14.15, -17.9, 1
poses cup_pose = {0.2, 0.35, 0.045, 0.0, 0.0, 0.0};
poses fill_pose = {0.15, 0.6, 0.125, 0.0, 0.0, 0.0}; // TEST VALUES ADJUST AS NEEDED
poses serve_pose = {-0.25, 0.1, -0.2, 0.0, 0.0, 0.0};
poses regrasp_pose = {0.1, 0.45, 0.065, 0.0, 0.0, 0.0}; // TEST VALUES ADJUST AS NEEDED
poses grasp_frame_transform = {0.0, 0.02, 0.135, M_PI/2, 0.0, 0.0}; // {x, y, z, rx, py, yz} Orients gripper and open/close horizontal
poses regrasp_frame_transform = {0.0, 0.0, 0.15, M_PI, 0.0, 0.0}; // TEST VALUE ADJUST AS NEEDED


// Utility Functions
Eigen::Isometry3d toEigen(const poses& val) {
	return Eigen::Translation3d(val.x, val.y, val.z) *
	       Eigen::AngleAxisd(val.rx, Eigen::Vector3d::UnitX()) *
	       Eigen::AngleAxisd(val.py, Eigen::Vector3d::UnitY()) *
	       Eigen::AngleAxisd(val.yz, Eigen::Vector3d::UnitZ());
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
    RCLCPP_INFO(LOGGER, "%f, %f, %f",pose.position.x, pose.position.y, pose.position.z);
    cup.primitive_poses.push_back(pose);
    rclcpp::sleep_for(std::chrono::microseconds(500)); 
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
    //auto result = task_.execute(*task_.solutions().front());
    /*If you want to inspect the goal message, use this instead:
	actionlib::SimpleActionClient<moveit_task_constructor_msgs::msg::ExecuteTaskSolutionAction>
	execute("execute_task_solution", true); 
    execute.waitForServer();
	moveit_task_constructor_msgs::msg::ExecuteTaskSolution::Goal execute_goal;
	task_->solutions().front()->toMsg(execute_goal.solution);
	execute.sendGoalAndWait(execute_goal);
	execute_result = execute.getResult()->error_code;
    
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }
    */
    return;
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
    sampling_planner->setProperty("goal_joint_tolerance", 1e-4);
    
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(0.001);
    cartesian_planner->setJumpThreshold(1.5);
    
    // Constraints  TODO: ADD DATABASE GENERATOR TO LAUNCH FILE 
    moveit_msgs::msg::Constraints serve_constraint;
    serve_constraint.name = "serve_coffee";
    serve_constraint.orientation_constraints.resize(1);
    {
        moveit_msgs::msg::OrientationConstraint &c = serve_constraint.orientation_constraints[0];
        c.link_name = "tool0"; // TODO: POSSIBLY ADD INVERTED(Z DOWN) TF AT BASE LINK TO USE TOOL0 AND NEW TF
        c.header.frame_id = "invert_ref";
        c.orientation.x = 0.0; // 
        c.orientation.y = 0.0;
        c.orientation.z = 0.707;
        c.orientation.w = 0.707; 
        c.absolute_x_axis_tolerance = 0.68;
        c.absolute_y_axis_tolerance = 0.68;
        c.absolute_z_axis_tolerance = 3.15;
        c.weight = 1.0;
    }
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
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
        "pre-grasp position", mtc::stages::Connect::GroupPlannerVector{{ arm_group_name, sampling_planner }}); 
        stage->setTimeout(10.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }
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
            //stage->setMinMaxDistance(0.01, 0.2);

             // Set hand forward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = gripper_frame;
            vec.vector.z = grasp_approach;
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
            stage->setAngleDelta(M_PI / 16); 
            stage->setMonitoredStage(current_state_);
            
            // ***Compute IK <Wrapper>***
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("initial grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(12);
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
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
            stage->setGroup(gripper_group_name);
            stage->setGoal("close");       
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
            vec.vector.z = grasp_retract;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }
        // -> Set Grasp Stage Pointer ->
        grasp_stage_ = grasp.get();
        task.add(std::move(grasp));
    } // END INITIAL GRASP CONTAINER

    // *** Move to Coffee Machine <Connector>***
    {
        auto stage = std::make_unique<mtc::stages::Connect>("move to coffee machine", 
            mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
                                                    //{gripper_group_name, sampling_planner}
        stage->setTimeout(10.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }   
    // ***Fill Coffee Container***
    // -> Fill Stage Pointer ->
    mtc::Stage* fill_stage_ = nullptr;
    {
        auto fill = std::make_unique<mtc::SerialContainer>("fill coffee");
        task.properties().exposeTo(fill->properties(), {"eef", "hand", "group", "ik_frame"});
        fill->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});
        // *** Move to Coffee Machine <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("fill cup", cartesian_planner);
            stage->properties().set("marker_ns", "fill_cup");
            stage->properties().set("link", gripper_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            //stage->setMinMaxDistance(0.02, 0.15);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = gripper_frame;
            vec.vector.z = fill_approach;
            stage->setDirection(vec);
            fill->insert(std::move(stage));
        }
        // *** Fill Coffee Pose <Generator>***
        {
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate fill pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "fill_pose");
            stage->setObject("cup");
            // Define Pose 
            geometry_msgs::msg::PoseStamped fill_pose_msg;
            fill_pose_msg.header.frame_id = "world";
            fill_pose_msg.pose = toPose(fill_pose);
            stage->setPose(fill_pose_msg);
            stage->setMonitoredStage(grasp_stage_);

            // Compute IK
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("fill pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(12);
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
            vec.vector.z = fill_retract;
            stage->setDirection(vec);
            fill->insert(std::move(stage));
        }
        // -> Set Fill Stage Pointer ->
        fill_stage_ = fill.get();
        task.add(std::move(fill));
    } // END FILL COFFEE CONTAINER

    // *** Place for Regrasp <Connector>***
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
        "place for regrasp", mtc::stages::Connect::GroupPlannerVector{{ arm_group_name, sampling_planner }}); 
        stage->setTimeout(10.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));    
    }

    // ***Place Coffee for Regrasp Container***
    // -> Place Stage Pointer ->
    mtc::Stage* place_stage_ = nullptr;
    {
        auto place = std::make_unique<mtc::SerialContainer>("place for regrasp");
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
            vec.vector.z = -place_approach; // ADJUST IF NEEDED 
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
            regrasp_pose_msg.header.frame_id = "world";
            regrasp_pose_msg.pose = toPose(regrasp_pose);
            stage->setPose(regrasp_pose_msg);
            stage->setMonitoredStage(fill_stage_);

            // ***Compute IK***
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("regrasp place pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(12);
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
            stage->allowCollisions("cup", 
                task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
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
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat from cup", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            //stage->setMinMaxDistance(0.1, 0.15);
            stage->setIKFrame(gripper_frame);
            stage->properties().set("marker_ns", "retreat");

            //Set Retreat Direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = place_retreat;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }
        // -> Set Place Stage Pointer ->
        place_stage_ = place.get();
        task.add(std::move(place));
    } // END PLACE CONTAINER

    // ***Move to Regrasp <Connector>***
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
        "move to regrasp", mtc::stages::Connect::GroupPlannerVector{{ arm_group_name, sampling_planner }}); 
        stage->setTimeout(10.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }
    
    // -> Regrasp Stage Pointer ->
    mtc::Stage* regrasp_stage_ = nullptr;
    // ***Regrasp Container***
    {
        auto regrasp = std::make_unique<mtc::SerialContainer>("regrasp cup");
        task.properties().exposeTo(regrasp->properties(), {"eef", "hand", "group", "ik_frame"});
        regrasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

        // ***Approach Cup <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("approach regrasp", cartesian_planner);
            stage->properties().set("marker_ns", "approach regrasp");
            stage->properties().set("link", gripper_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            //stage->setMinMaxDistance(0.075, 0.15);

            // Set Direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = regrasp_approach; // ADJUST
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
			stage->setAngleDelta(M_PI / 16);
			stage->setMonitoredStage(place_stage_);

            // ***Compute IK***
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("regrasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(12);
			wrapper->setMinSolutionDistance(1.0);
			wrapper->setIKFrame(toEigen(regrasp_frame_transform), gripper_frame);
			wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
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
			vec.vector.z = regrasp_retract;
			stage->setDirection(vec);
			regrasp->insert(std::move(stage));
        }
        
        // -> Set Regrasp Stage Pointer ->
        regrasp_stage_ = regrasp.get();

        task.add(std::move(regrasp));
    } // END REGRASP CONTAINER

    // ***Move to Serve <Connector>
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
        "move to serve", mtc::stages::Connect::GroupPlannerVector{{ arm_group_name, sampling_planner }}); 
        stage->setTimeout(15.0);
        stage->setPathConstraints(serve_constraint); 
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }

    // -> Serve Stage Pointer ->
    //mtc::Stage* serve_stage_ = nullptr;

    // ***Serve Container***
    {
        auto serve = std::make_unique<mtc::SerialContainer>("serve coffee");
        task.properties().exposeTo(serve->properties(), {"eef", "hand", "group", "ik_frame"});
        serve->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

        // ***Serve to Baristabot <MoveRelative>***
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("move to baristabot", cartesian_planner);
			stage->properties().set("marker_ns", "serve_cup");
            stage->properties().set("link", gripper_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
			//stage->setMinMaxDistance(0.05, 0.15);
			
			// Set direction
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = "world";
			vec.vector.z = serve_approach; // DUMMY VARIABLE FILL WITH TEST AND ADJUST
			stage->setDirection(vec);
			serve->insert(std::move(stage));
        }   
        // ***Position Over Baristabot <Generator>***
        {
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("position for serve");
			stage->properties().configureInitFrom(mtc::Stage::PARENT);
			//stage->setMinMaxDistance(0.05, 0.15);
			stage->setObject("cup");
			stage->properties().set("marker_ns", "position_cup");

            // Define Pose 
            geometry_msgs::msg::PoseStamped serve_pose_msg;
            serve_pose_msg.header.frame_id = "world";
            serve_pose_msg.pose = toPose(serve_pose);
            stage->setPose(serve_pose_msg);
            stage->setMonitoredStage(regrasp_stage_);

            // Compute IK
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("serve pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(12);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame("cup");
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            serve->insert(std::move(wrapper));
        }    

        //  ***Open Gripper <MoveTo>***
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
            stage->setGroup(gripper_group_name);
            stage->setGoal("open");       
            serve->insert(std::move(stage));    
        }
        
        // ***Disable Collision <PlanningScene>***
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("disable collision");
            stage->allowCollisions("cup", 
                task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
                false); 
            serve->insert(std::move(stage));
        }
        
        // ***Detach Cup <PlanningScene>***
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach cup");
            stage->detachObject("cup", gripper_frame);
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
			vec.vector.z = serve_retract; // DUMMY VARIABLE FILL WITH TEST AND ADJUST
			stage->setDirection(vec);
			serve->insert(std::move(stage));
        }  
        task.add(std::move(serve));
    } // END SERVE CONTAINER
    // RETURN HOME
    /*
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setGoal("home");
        stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
        task.add(std::move(stage));
    }
    */
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

