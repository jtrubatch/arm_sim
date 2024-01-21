#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("perception");
using std::placeholders::_1;

class Perception : public rclcpp::Node
{
public:
    Perception() : Node("perception")
    {
        this->p_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    pc_topic, 10, std::bind(&Perception::cloudCallback, this, _1));
        RCLCPP_INFO(LOGGER,"Node Created");
    }
    void addCup()
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::CollisionObject cup;
        geometry_msgs::msg::Pose pose;
        //Eigen::Vector3d cup_z(cylinder.orientation[0], cylinder.orientation[1], cylinder.orientation[2]);
        //Eigen::Vector3d world_z(0.0, 0.0, 1.0);
        //Eigen::Vector3d axis = cup_z.cross(world_z);
        //axis.normalize();
        //double angle = acos(world_z.dot(cup_z));
        // Using Known Camera Frame Orientation
        
        cup.id = "cup";
        cup.header.frame_id = "wrist_rgbd_camera_depth_optical_frame";
        cup.primitives.resize(1);
        cup.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
	    cup.primitives[0].dimensions = { cylinder.height, cylinder.radius };
        pose.orientation.x = -2.35619; //axis.x() * sin(angle / 2);
        pose.orientation.y = 0.0; //axis.y() * sin(angle / 2);
        pose.orientation.z = 0.0; //axis.z() * sin(angle / 2);
        // RCLCPP_INFO(LOGGER, "Rot X: %f, Rot Y: %f, Rot Z: %f, Angle: %f", pose.orientation.x, pose.orientation.y, pose.orientation.z, angle);
        pose.orientation.w = 1.0;
        pose.position.x = cylinder.center[0];
        pose.position.y = cylinder.center[1];
        pose.position.z = cylinder.center[2];
        cup.primitive_poses.push_back(pose);

        psi.applyCollisionObject(cup);
        RCLCPP_INFO(LOGGER,"Cup Created");
    }
    struct CylinderParams{
        double radius;
        double orientation[3];
        double center[3];
        double height;
        double cLine[3];
    };
    CylinderParams cylinder;

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr p_cloud_;
    pcl::PassThrough<pcl::PointXYZRGB> filterZ_;
    pcl::PassThrough<pcl::PointXYZRGB> filterX_;
    double filter_minZ = 0.0; // Distance from Camera
    double filter_maxZ = 0.75;
    double filter_minX = -0.2;
    double filter_maxX = 0.2;
    bool cylinder_found = false;
    std::string pc_topic = "/wrist_rgbd_depth_sensor/points";
    
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcmsg)
    {

        // Convert Format
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::fromROSMsg(*pcmsg, *cloud_);
    
        // Filter Z
        filterZ_.setInputCloud(cloud_);
        filterZ_.setFilterFieldName("z");
        filterZ_.setFilterLimits(filter_minZ, filter_maxZ);
        filterZ_.filter(*cloud_);
        // Filter X(Y in World)
        filterX_.setInputCloud(cloud_);
        filterX_.setFilterFieldName("x");
        filterX_.setFilterLimits(filter_minX, filter_maxX);
        filterX_.filter(*cloud_);

        // Normals
        pcl::PointCloud<pcl::Normal>::Ptr normals_(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_(new pcl::search::KdTree<pcl::PointXYZRGB>());
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> estimation;
        
        estimation.setSearchMethod(tree_);
        estimation.setInputCloud(cloud_);
        estimation.setKSearch(50);
        estimation.compute(*normals_);

        // Remove Planar Surface
        pcl::PointIndices::Ptr inliers_plane_(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;

        segmentor.setOptimizeCoefficients(true);
        segmentor.setModelType(pcl::SACMODEL_PLANE);
        segmentor.setMethodType(pcl::SAC_RANSAC);
        segmentor.setMaxIterations(1000);
        segmentor.setDistanceThreshold(0.01);
        segmentor.setInputCloud(cloud_);

        pcl::ModelCoefficients::Ptr coeff_plane_(new pcl::ModelCoefficients);
        segmentor.segment(*inliers_plane_, *coeff_plane_);
        
        pcl::ExtractIndices<pcl::PointXYZRGB> extracted_indices;
        extracted_indices.setInputCloud(cloud_);
        extracted_indices.setIndices(inliers_plane_);
        extracted_indices.setNegative(true);
        extracted_indices.filter(*cloud_);

        // Extract Normals
        pcl::ExtractIndices<pcl::Normal> extracted;

        extracted.setNegative(true);
        extracted.setInputCloud(normals_);
        extracted.setIndices(inliers_plane_);
        extracted.filter(*normals_);

        // Extract Cylinder
        pcl::ModelCoefficients::Ptr coeff_cylinder_(new pcl::ModelCoefficients);
        pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segment;
        pcl::PointIndices::Ptr inliers_cylinder_(new pcl::PointIndices);

        segment.setOptimizeCoefficients(true);
        segment.setModelType(pcl::SACMODEL_CYLINDER);
        segment.setMethodType(pcl::SAC_RANSAC);
        segment.setNormalDistanceWeight(0.1);
        segment.setMaxIterations(10000);
        segment.setDistanceThreshold(0.05);  // Tolerance for Variation from Model
        segment.setRadiusLimits(0, 0.06); // Min/Max in Meters to Extract
        segment.setInputCloud(cloud_);
        segment.setInputNormals(normals_);
        segment.segment(*inliers_cylinder_, *coeff_cylinder_);

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud_);
        extract.setIndices(inliers_cylinder_);
        extract.setNegative(false);
        extract.filter(*cloud_);

        if (cloud_->points.empty())
        {
        RCLCPP_ERROR_STREAM(LOGGER, "Cylinder Not Found");
        rclcpp::shutdown();
        return;
        } else if(!cylinder_found){
            cylinder.radius = coeff_cylinder_->values[6];
            cylinder.orientation[0] = coeff_cylinder_->values[3];
            cylinder.orientation[1] = coeff_cylinder_->values[4];
            cylinder.orientation[2] = coeff_cylinder_->values[5];
            cylinder.cLine[0] = coeff_cylinder_->values[0];
            cylinder.cLine[1] = coeff_cylinder_->values[1];
            cylinder.cLine[2] = coeff_cylinder_->values[2];


            double max_angle_y = -std::numeric_limits<double>::infinity();
            double min_angle_y = std::numeric_limits<double>::infinity();
            double lowest_point[3] = { 0.0, 0.0, 0.0 };
            double highest_point[3] = { 0.0, 0.0, 0.0 };

            for (auto const point : cloud_->points)
            {
                const double angle = atan2(point.z, point.y);
                if (angle < min_angle_y)
                {
                    min_angle_y = angle;
                    lowest_point[0] = point.x;
                    lowest_point[1] = point.y;
                    lowest_point[2] = point.z;
                } else if (angle > max_angle_y){ 
                    max_angle_y = angle;
                    highest_point[0] = point.x;
                    highest_point[1] = point.y;
                    highest_point[2] = point.z;
                }
            }

            cylinder.center[0] = (highest_point[0] + lowest_point[0]) / 2; 
            cylinder.center[1] = (highest_point[1] + lowest_point[1]) / 2;
            cylinder.center[2] = (highest_point[2] + lowest_point[2]) / 2;

            cylinder.height =
                sqrt(pow((lowest_point[0] - highest_point[0]), 2) + pow((lowest_point[1] - highest_point[1]), 2) +
                pow((lowest_point[2] - highest_point[2]), 2));

            RCLCPP_INFO(LOGGER, "Cup Radius %.3f, Cup Height %.3f, Cup X %.3f Cup Y %.3f Cup Z %.3f", 
                cylinder.radius, cylinder.height, cylinder.center[0], cylinder.center[1], cylinder.center[2]);
            
            addCup();
            cylinder_found = true;
        }
    } // END PC CALLBACK
}; // END PERCEPTION CLASS

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto perception_node = std::make_shared<Perception>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(perception_node);
    exec.spin();
    rclcpp::shutdown();

    return 0;
}

/*

*/