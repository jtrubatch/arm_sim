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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur_control");
namespace mtc = moveit::task_constructor;

struct poses{
        float x;
        float y;
        float z;
        float w;
        
};
// UR Gazebo Pose 13.9, -18.56, 1.032
// Cup Gazebo Pose 14.2, -18.1, 1.09
poses cup_pose = {0.3, 0.45, 0.0675, 1.0};
poses grasp_pose = {0.0, 0.0, 0.0, 1.0};
// Coffee Machine Gazebo Pose 14, -17.7, 1
poses fill_pose = {0.0, 0.0, 0.0, 1.0};
float cup_radius = 0.035; // meters
float cup_height = 0.13;

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

// DEFINITIONS
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
    cup.primitives[0].dimensions = { cup_height, cup_radius }; // ESTIMATES 

    geometry_msgs::msg::Pose pose;
    pose.position.x = cup_pose.x; // GET ACTUAL VALUES 
    pose.position.y = cup_pose.y;
    pose.position.z = cup_pose.z;
    pose.orientation.w = cup_pose.w;
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

    if (task_.plan(5))
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
    mtc::Stage* grasp_cup_stage = nullptr;
    
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

        // ***Grasp Pose <Generator>
        {
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate initial grasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "initial_grasp_pose"); 
            stage->setPreGraspPose("open");
            stage->setObject("cup");
            stage->setAngleDelta(M_PI / 12); 
            stage->setMonitoredStage(current_state_);

            // Define Frame and Pose
            /*
            geometry_msgs::msg::PoseStamped grasp_pose_msg;
            grasp_pose_msg.header.frame_id = "ik_frame"; // VERIFY PROPERTY FOR UR ARM *Prop1
            grasp_pose_msg.pose.position.x = grasp_pose.x; // DUMMY VALUES FOR POSITION and ORIENTATION REPLACE WITH REAL VALUES
            grasp_pose_msg.pose.position.y = grasp_pose.y;
            grasp_pose_msg.pose.position.z = grasp_pose.z;
            grasp_pose_msg.pose.orientation.w = grasp_pose.w; // ADD ADDITIONAL VALUES FOR PROPER HORIZONTAL AND/OR VERTICAL ORIENTATIONS
            stage->setPose(grasp_pose_msg); // VERIFY
            */
            Eigen::Isometry3d grasp_frame_transform;
            Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2 , Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
            grasp_frame_transform.linear() = q.matrix();
            grasp_frame_transform.translation().z() = 0.135;
            grasp_frame_transform.translation().y() = 0.02;
            
            // Compute IK <Wrapper>
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("initial grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(grasp_frame_transform, gripper_frame); 
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            grasp->insert(std::move(wrapper));
        }
        
        // ***Allow Collision <PlanningScene> 
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
            stage->allowCollisions(
                "cup", 
                task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
                true);
            grasp->insert(std::move(stage));
        }    
        
        // ***Close Gripper <MoveTo>
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
            stage->setGroup(gripper_group_name);
            stage->setGoal("close");
            grasp->insert(std::move(stage));
        }
        
        // ***Attach Cup <PlanningScene>  
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cup");
            stage->attachObject("cup", gripper_frame);
            // Set Stage Pointer
            grasp_cup_stage = stage.get();
            grasp->insert(std::move(stage));
        }
        
        // *** Lift Cup <MoveRelative>
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift cup", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            //stage->setMinMaxDistance(0.01, 0.1); //ADJUST IF NEEDED
            stage->setIKFrame(gripper_frame);
            stage->properties().set("marker_ns", "lift_cup");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 0.1;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }
        grasp_cup_stage = grasp.get();
        task.add(std::move(grasp));
    } // END INITIAL GRASP CONTAINER
    /*
    // *** Move to Coffee Machine <Connector>
    {
        auto stage_move_to_coffee = std::make_unique<mtc::stages::Connect>("move to coffee machine",
                                            mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner},
                                                                                     {gripper_group_name, sampling_planner}});
        stage_move_to_coffee->setTimeout(5.0);
        stage_move_to_coffee->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage_move_to_coffee));
    }    
    // ***Fill Coffee Container
    {
        auto fill_coffee = std::make_unique<mtc::SerialContainer>("fill coffee");
        task.properties().exposeTo(fill_coffee->properties(), {"eef", "group", "ik_frame"});
        fill_coffee->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});
        // Fill Stage Pointer
        mtc::Stage* fill_cup_stage = nullptr;
        // *** Fill Coffee Pose <Generator>
        {
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate fill pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "fill_pose");
            stage->setObject("cup");
            // Define Pose
            geometry_msgs::msg::PoseStamped fill_pose_msg;
            fill_pose_msg.header.frame_id = "cup";
            fill_pose_msg.pose.position.x = fill_pose.x; // DUMMY VALUES ADD REAL VALUES
            fill_pose_msg.pose.position.y = fill_pose.y;
            fill_pose_msg.pose.position.z = fill_pose.z;
            fill_pose_msg.pose.orientation.w = fill_pose.w;
            stage->setPose(fill_pose_msg);
            stage->setMonitoredStage(grasp_cup_stage);

            // Compute IK
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("fill pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(2);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame("cup");
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            fill_coffee->insert(std::move(wrapper));
        }
        // ADD PAUSE
        // ***Retract from Coffee Machine <MoveRelative>
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retract coffee", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setMinMaxDistance(0.1, 0.2);
            stage->setIKFrame(gripper_frame);
            stage->properties().set("market_ns", "retract_coffee");
            // Set Direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.x = -0.1;
            //vec.vector.y = -0.1;  // UNCOMMENT IF NEEDED
            stage->setDirection(vec);
            fill_coffee->insert(std::move(stage));
        }
        task.add(std::move(fill_coffee));
    } // END FILL COFFEE CONTAINER
 
    // RETURN HOME
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setGoal("home");
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


/* NOTES 
Needed Info
*1 Coordinate System Orientation of Robot Base in Environment
*2 Cup Initial Position
*3 Fill Position 
*4 Baristabot Cupholder Positions
Stages
    Open Gripper / Verify Gripper Open
    Set EE Orientation Horizontal and Co-Linear to ~45deg from face of Coffee Machine
    Move to Cup
    Move to Grasp
    Grasp
    Lift to ~ 1cm Higher than Coffee Machine Work Surface  
        Verify Cup, Coffee Machine, and Black Handle Dimensions for Specific Grasp Placement and Clearances
    Move to Fill Position
    Pause for Simulated Fill
    Move to Serve Position
    Lower to Counter
    Release Cup
    Retract 
    Set EE Orientation to Vertical 
    Move to Pre-Grasp Position Above Cup
    Move to Grasp
    Grasp
    Lift ~5cm from Counter
    Move to Above Baristabot
    Lower to Baristabot
    Release
*/