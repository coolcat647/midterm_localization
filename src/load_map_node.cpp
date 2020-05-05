// C++ STL
#include <iostream>
#include <boost/filesystem.hpp> // Support in STL since C++17
// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h> // RemoveNaN
#include <pcl/kdtree/kdtree_flann.h> // KD tree
#include <pcl/filters/voxel_grid.h> // VG
#include <pcl/registration/icp.h> // ICP
#include <pcl/filters/passthrough.h> // passThrough
// TF
#include <tf/transform_broadcaster.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;


class LoadMapNode{
public:
    LoadMapNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pub_map_;

    PointCloudXYZPtr map_pc_ptr_;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;  // Voxel grid filter
};

LoadMapNode::LoadMapNode(ros::NodeHandle nh, ros::NodeHandle pnh):nh_(nh), pnh_(pnh) {
    // ROS parameters
    double voxel_grid_size;
    ros::param::param<double>("~vg_size", voxel_grid_size, 0.25);

    // ROS publisher, subscriber
    pub_map_ = nh.advertise<sensor_msgs::PointCloud2>("map_pc", 1);

    // Filter parameters setting
    voxel_grid_.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);

    // Find map pcd files
    string package_path = ros::package::getPath("midterm_localization");
    string maps_dir = package_path + "/../test_data/map";
    // string maps_dir = package_path.substr(0, package_path.size() - strlen("midterm_localization")) + "test_data/map";
    std::vector<std::string> maps_list;
    for(const auto &entry:boost::filesystem::directory_iterator(maps_dir)) {
        string map_file = entry.path().string();
        if(map_file.substr(map_file.find_last_of(".") + 1) == "pcd"){
            maps_list.push_back(map_file);
        }
    }

    // Load map pointcloud from pcd files 
    map_pc_ptr_ = PointCloudXYZPtr(new PointCloudXYZ);
    PointCloudXYZPtr tempPtr(new PointCloudXYZ);
    for(auto file_path: maps_list) {
        cout << file_path << endl;
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *tempPtr) == -1){
            ROS_ERROR("Cannot find %s, aborting...", file_path.c_str());
            ros::shutdown();
        } 
        *map_pc_ptr_ += *tempPtr;
    }
    ROS_INFO("Load all maps successfully, there are %d sub maps and %d points", \
            maps_list.size(), \
            (int)map_pc_ptr_->points.size());

    // Downsampling
    voxel_grid_.setInputCloud (map_pc_ptr_);
    voxel_grid_.filter(*map_pc_ptr_);
    ROS_INFO("After voxel grid filtering, there are %d points", \
            (int)map_pc_ptr_->points.size());

    // Publish map pointcloud
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_pc_ptr_, map_msg);
    map_msg.header.frame_id = "map";
    pub_map_.publish(map_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "load_map_node");
    ros::NodeHandle nh, pnh("~");
    LoadMapNode node(nh, pnh);
    ros::spin();
    return 0;
}