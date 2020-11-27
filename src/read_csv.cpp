#include <signal.h>
#include <fstream>
#include <string>
#include <sstream>
// C++ STL
#include <iostream>
#include <boost/filesystem.hpp> // Support in STL since C++14
#include <boost/date_time/posix_time/posix_time.hpp>
// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
// Eigen
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h> // RemoveNaN
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/registration/icp.h> 
#include <pcl/filters/passthrough.h>
// TF
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
// PCL_ROS
#include <pcl_ros/transforms.h>
// GeographicLib
// #include <GeographicLib/Geocentric.hpp>
// #include <GeographicLib/LocalCartesian.hpp>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;

#define kLatOrigin 24.7855644226
#define kLonOrigin 120.997009277
#define kAltOrigin 127.651
#define kGlobalFrame "map"

static const string COLOR_RED = "\e[0;31m";
static const string COLOR_GREEN = "\e[0;32m";
static const string COLOR_YELLOW = "\e[0;33m"; 
static const string COLOR_NC = "\e[0m";

class ReadCSV
{
    private:
        ros::NodeHandle nh_, pnh_;
        ros::Publisher pub_map_;                // whole map pointcloud
        ros::Publisher pub_result_path_g_;      //Ground truth
        ros::Publisher pub_result_path_r_;      //result
        std::vector<double> matrix_g_, matrix_r_;
        PointCloudXYZPtr map_pc_ptr_;
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;  // Voxel grid filter
        nav_msgs::Path result_path_g_, result_path_r_;
        
        string map_name_;               // options: nctu, itri, nuscene
        string result_csv_;

        void map_setup(void) 
        {
            // Find map pcd files
            string package_path = ros::package::getPath("midterm_localization");
            string maps_dir = package_path + "/../test_data/" + map_name_ + "_map";
            std::vector<std::string> maps_list;
            for(const auto &entry:boost::filesystem::directory_iterator(maps_dir)) 
            {
                string map_file = entry.path().string();
                if(map_file.substr(map_file.find_last_of(".") + 1) == "pcd")
                {
                    maps_list.push_back(map_file);
                }
            }
            // Load map pointcloud from pcd files 
            map_pc_ptr_ = PointCloudXYZPtr(new PointCloudXYZ);
            PointCloudXYZPtr tempPtr(new PointCloudXYZ);
            for(auto file_path: maps_list) 
            {
                cout << file_path << endl;
                if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *tempPtr) == -1)
                {
                    ROS_ERROR("Cannot load map: %s, aborting...", file_path.c_str());
                    ros::shutdown();
                } 
                *map_pc_ptr_ += *tempPtr;
            }
            ROS_INFO("Load all maps successfully, there are %d sub maps and %d points", (int)maps_list.size(), (int)map_pc_ptr_->points.size());

            // Remove nan
            std::vector<int> indice;
            pcl::removeNaNFromPointCloud(*map_pc_ptr_, *map_pc_ptr_, indice);

            ROS_INFO("After filtering, there are %d points", (int)map_pc_ptr_->points.size());

            // Publish map pointcloud
            // if(pub_map_.getNumSubscribers() > 0) 
            // {
                sensor_msgs::PointCloud2 map_msg;
                pcl::toROSMsg(*map_pc_ptr_, map_msg);
                map_msg.header.frame_id = kGlobalFrame;
                pub_map_.publish(map_msg);
            // }
        }
        void read_file(void)
        {
            // Find map pcd files
            string package_path = ros::package::getPath("midterm_localization");
            string maps_dir = package_path + "/../test_data/" + map_name_ + "_map";
            std::ifstream f_g_, f_r_;
            std::string line_g_, line_r_;
            if(map_name_ == "nuscene")
            {
                f_g_.open(maps_dir + "/Nu_Public_Ground_Truth.csv", std::ifstream::in);
            }
            else if(map_name_ == "itri")
            {
                f_g_.open(maps_dir + "/ITRI_Public_Ground_truth.csv", std::ifstream::in);
            }
            else
            {
                // Show ERROR because this version of code no longer support libgeograpic
                cout << COLOR_RED << "This code doesn't support Geographic transform. Aborting..." << COLOR_NC << endl;  
                ros::shutdown();
            }
            
            f_r_.open(package_path + "/csv_files/" + result_csv_, std::ifstream::in);

            if(!f_g_ && !f_r_)
            {
                printf("Can't open the file!!!\n");
                ros::shutdown();
            }

            while(std::getline(f_g_, line_g_))
            {
                std::istringstream templine(line_g_);
                std::string data_g_;
                while(std::getline(templine, data_g_, ','))
                {
                    matrix_g_.push_back(atof(data_g_.c_str()));
                    cout << COLOR_RED << atof(data_g_.c_str()) << COLOR_NC << " ";
                    //printf("%f ",atof(data_g_.c_str()));
                }
                cout << endl;
            }
            //printf("---------------------------------------------------------------------------------\n");
            while(std::getline(f_r_, line_r_))
            {
                std::istringstream templine(line_r_);
                std::string data_r_;
                while(std::getline(templine, data_r_, ','))
                {
                    matrix_r_.push_back(atof(data_r_.c_str()));
                    cout << COLOR_YELLOW << atof(data_r_.c_str()) << COLOR_NC << " ";
                    //printf("%f ",atof(data_r_.c_str()));
                }
                cout << endl;
            }
            f_g_.close();
            f_r_.close();
        }
    public:
        ReadCSV(ros::NodeHandle nh, ros::NodeHandle pnh):nh_(nh), pnh_(pnh)
        {
            // ROS parameters
            double voxel_grid_size;
            ros::param::param<double>("~vg_size", voxel_grid_size, 0.25);
            ros::param::param<string>("~map_name", map_name_, "nctu");
            ros::param::param<string>("~result_csv", result_csv_, "csv");

            // ROS publisher, subscriber
            pub_map_ = nh.advertise<sensor_msgs::PointCloud2>("map_pc", 1);
            pub_result_path_g_ = nh.advertise<nav_msgs::Path>("result_path_g_", 1);
            pub_result_path_r_ = nh.advertise<nav_msgs::Path>("result_path_r_", 1);

            if(map_name_ == "nctu") 
            { 
                // Show ERROR because this version of code no longer support libgeograpic
                cout << COLOR_RED << "This code doesn't support Geographic transform. Aborting..." << COLOR_NC << endl;  
                ros::shutdown();
            } 
            map_setup();
            read_file();
        }
};


int main(int argc, char** argv)
{
    if(argc!=3)
    {
        printf("Not enough input, please provide input file!!! \n");
        return -1;
    }
    cout << COLOR_GREEN << "Start:" << COLOR_NC << endl;
    ros::init(argc, argv, "read_csv_node");
    ros::NodeHandle nh, pnh("~");
    ReadCSV r_csv(nh, pnh);
    printf("-----------------------------END!!!-----------------------------\n");
    ros::spin();
    return 0;
}