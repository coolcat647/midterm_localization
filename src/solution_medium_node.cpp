#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
// C++ STL
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <mutex>
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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h> 
#include <pcl/filters/filter.h> // RemoveNaN
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
// TF
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
// PCL_ROS
#include <pcl_ros/transforms.h>

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

class InitGuessInfo {
public:
    InitGuessInfo(){
        status_ = 0;    // 0: idle, 1:busy, 2:finished
        score_ = 0.0;
        guess_matrix_ = Eigen::Matrix4d::Identity(4, 4);
        original_matrix_ = Eigen::Matrix4d::Identity(4, 4);
    }
    int status_;
    double score_;
    Eigen::Matrix4d guess_matrix_;
    Eigen::Matrix4d original_matrix_;
    vector<PointCloudXYZPtr, Eigen::aligned_allocator<PointCloudXYZPtr> > clouds_list_;
};


class Solution3Node{
public:
    Solution3Node(ros::NodeHandle nh, ros::NodeHandle pnh);
    static void sigint_cb(int sig);
    void gnss_cb1(const sensor_msgs::NavSatFixConstPtr &msg);
    void gnss_cb2(const geometry_msgs::PointStampedConstPtr &msg);
    void pose_cb(const geometry_msgs::PoseStampedPtr &msg);
    void pc_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void imu_cb(const sensor_msgs::ImuConstPtr &msg);
    void map_setup(void);
    void filters_setup(double voxel_grid_size);
    void visualization_setup(void);
    void renounce_guess_info(void);
    void publish_path_from_matrix(const Eigen::Matrix4d &m, const ros::Time &t, bool show_info);
    void extract_straight_objects(PointCloudXYZPtr cloud_in, PointCloudXYZPtr cloud_out, int num_threshold);
    void remove_plane(PointCloudXYZPtr cloud_in, PointCloudXYZPtr cloud_out);
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pub_gps_marker_;
    ros::Publisher pub_map_;        // whole map pointcloud
    ros::Publisher pub_submap_;     // sub map pointcloud
    ros::Publisher pub_result_pc_;     // sub map pointcloud
    ros::Publisher pub_local_pc_;   // local velodyne pointcloud
    ros::Publisher pub_result_path_;
    ros::Publisher pub_imu_pose_;
    ros::Subscriber sub_gnss_;
    ros::Subscriber sub_pc_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_trypose_;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;  // Voxel grid filter
    PointCloudXYZPtr map_pc_ptr_;
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond_;
    pcl::ConditionalRemoval<pcl::PointXYZ> cond_removal_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    vector<PointCloudXYZPtr, Eigen::aligned_allocator<PointCloudXYZPtr> > clouds_list_;
    nav_msgs::Path result_path_;
    visualization_msgs::Marker line_strip_;

    // Sensor related
    int cnt_imu_msgs_;
    int cnt_gps_msgs_;
    bool flag_init_guess_;
    ros::Time first_timestamp_;

    // TF related
    tf::TransformListener listener_;

    string map_name_;               // options: nctu, itri, nuscene
    string result_filename_;
    Eigen::Matrix4d guess_matrix_;
    int submap_size_;
    int local_sizex_, local_sizey_;
    int remove_sizey_l_, remove_sizey_h_;

    vector<InitGuessInfo> guess_list_;

    vector<sensor_msgs::Imu> imu_tmp_list_;
    vector<sensor_msgs::Imu> imu_list_;
    vector<geometry_msgs::PointStamped> gps_list_;
};


Solution3Node::Solution3Node(ros::NodeHandle nh, ros::NodeHandle pnh):
    nh_(nh), 
    pnh_(pnh), 
    cnt_gps_msgs_(0),
    cnt_imu_msgs_(0),
    flag_init_guess_(false),
    guess_matrix_(Eigen::Matrix4d::Identity(4, 4)) {

    // Signal callback
    signal(SIGINT, sigint_cb);

    // ROS parameters
    boost::posix_time::ptime posix_time(boost::posix_time::second_clock::local_time()); // for filename
    std::string time_str = boost::posix_time::to_iso_string(posix_time);
    double voxel_grid_size;
    ros::param::param<double>("~vg_size", voxel_grid_size, 0.4);
    ros::param::param<int>("~local_sizex", local_sizex_, 100);
    ros::param::param<int>("~local_sizey", local_sizey_, 24);
    ros::param::param<int>("~remove_sizey_l", remove_sizey_l_, -20);
    ros::param::param<int>("~remove_sizey_h", remove_sizey_h_, 5);
    ros::param::param<int>("~submap_size", submap_size_, 200);
    ros::param::param<string>("~map_name", map_name_, "nctu");
    ros::param::param<string>("~result_name", result_filename_, map_name_ + "_" + time_str + ".csv");
    
    // ROS publisher, subscriber
    pub_map_ = nh.advertise<sensor_msgs::PointCloud2>("map_pc", 1);
    pub_submap_ = nh.advertise<sensor_msgs::PointCloud2>("submap_pc", 1);
    pub_local_pc_ = nh.advertise<sensor_msgs::PointCloud2>("local_pc_filtered", 1);
    pub_gps_marker_ = nh.advertise<visualization_msgs::Marker>("gps_marker", 1);
    pub_result_path_ = nh.advertise<nav_msgs::Path>("result_path", 1);
    pub_imu_pose_ = nh.advertise<geometry_msgs::PoseStamped>("imu_pose", 1);
    pub_result_pc_ = nh.advertise<sensor_msgs::PointCloud2>("result_pc", 1);
    sub_imu_ = nh_.subscribe("imu/data", 250, &Solution3Node::imu_cb, this);
    sub_trypose_ = nh_.subscribe("move_base_simple/goal", 1, &Solution3Node::pose_cb, this); 
    if(map_name_ == "nctu") { 
        // Show ERROR because this version of code no longer support libgeograpic
        cout << COLOR_RED << "This code doesn't support Geographic transform. Aborting..." << COLOR_NC << endl;  
        ros::shutdown();
    } else {
        sub_gnss_ = nh.subscribe("fix", 250, &Solution3Node::gnss_cb2, this);
        sub_pc_ = nh_.subscribe("lidar_points", 250, &Solution3Node::pc_cb, this);
    }
    
    // ICP constraints
    icp_.setMaximumIterations(1000);
    icp_.setTransformationEpsilon(1e-14);
    icp_.setEuclideanFitnessEpsilon(1e-14); 
    //icp_.setMaxCorrespondenceDistance(50);

    filters_setup(voxel_grid_size);
    map_setup();
    visualization_setup();
    
    cout << COLOR_GREEN << "solution1_node is ready." << COLOR_NC << endl;
}


void Solution3Node::sigint_cb(int sig) {
    cout << "\nNode name: " << ros::this_node::getName() << " is shutdown." << endl;
    ros::shutdown();
}


void Solution3Node::filters_setup(double voxel_grid_size) {
    // Voxel grid filter parameters setting
    voxel_grid_.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    // Range condition filter parameters setting
    range_cond_ = pcl::ConditionAnd<pcl::PointXYZ>::Ptr(new pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond_->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
        new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -1.5)));
    cond_removal_.setCondition(range_cond_);
    cond_removal_.setKeepOrganized(true);
}


void Solution3Node::map_setup(void) {
    // Find map pcd files
    string package_path = ros::package::getPath("midterm_localization");
    string maps_dir = package_path + "/../test_data/" + map_name_ + "_map";
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
            ROS_ERROR("Cannot load map: %s, aborting...", file_path.c_str());
            ros::shutdown();
        } 
        *map_pc_ptr_ += *tempPtr;
    }
    ROS_INFO("Load all maps successfully, there are %d sub maps and %d points", \
            (int)maps_list.size(), \
            (int)map_pc_ptr_->points.size());

    // Remove nan
    std::vector<int> indice;
    pcl::removeNaNFromPointCloud(*map_pc_ptr_, *map_pc_ptr_, indice);

    ROS_INFO("After filtering, there are %d points", \
            (int)map_pc_ptr_->points.size());

    // Publish map pointcloud
    if(pub_map_.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*map_pc_ptr_, map_msg);
        map_msg.header.frame_id = kGlobalFrame;
        pub_map_.publish(map_msg);
    }
}

void Solution3Node::renounce_guess_info(void) {
    if(cnt_imu_msgs_ > 0 && cnt_gps_msgs_ > 0) {
        cout << "init_guess from GPS & IMU:" << endl;
        publish_path_from_matrix(guess_matrix_, first_timestamp_, true);
        flag_init_guess_ = true;
    }
}

void Solution3Node::visualization_setup(void) {
    // GPS line strip
    line_strip_.header.frame_id = kGlobalFrame;
    line_strip_.ns = "linestrip";
    line_strip_.action = visualization_msgs::Marker::ADD;
    line_strip_.pose.orientation.w = 1.0;
    line_strip_.id = 1;
    line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip_.scale.x = 0.4;
    line_strip_.color.r = 1.0;
    line_strip_.color.a = 1.0;

    // Result path
    result_path_.header.frame_id = kGlobalFrame;
}

void Solution3Node::pc_cb(const sensor_msgs::PointCloud2ConstPtr &msg){
    PointCloudXYZPtr pc_raw(new PointCloudXYZ);
    PointCloudXYZPtr pc_filtered(new PointCloudXYZ);
    pcl::fromROSMsg(*msg, *pc_raw);

    // Apply conditional filter
    // cond_removal_.setInputCloud(pc_raw);
    // cond_removal_.filter (*pc_filtered);
    pcl::copyPointCloud(*pc_raw, *pc_filtered);

    // Downsampling
    // voxel_grid_.setInputCloud(pc_filtered);
    // voxel_grid_.filter(*pc_filtered);
    
    // Transform to base_link frame
    tf::StampedTransform tf_stamped;
    geometry_msgs::TransformStamped tf_msg;
    listener_.lookupTransform("/car", "/nuscenes_lidar", ros::Time(0), tf_stamped);
    tf::transformStampedTFToMsg(tf_stamped, tf_msg);
    Eigen::Affine3d tf_eigen = Eigen::Affine3d::Identity();
    tf::transformMsgToEigen(tf_msg.transform, tf_eigen);
    pcl::transformPointCloud(*pc_filtered, *pc_filtered, tf_eigen);

    // Passthrough filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pc_filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-local_sizex_ / 2, local_sizex_ / 2);
    pass.filter(*pc_filtered);
    pass.setInputCloud(pc_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-local_sizey_ / 2, local_sizey_ / 2);
    pass.filter(*pc_filtered);

    // For nuscene private 1
    pass.setFilterFieldName("y");
    pass.setFilterLimits(remove_sizey_l_, remove_sizey_h_);
    pass.setFilterLimitsNegative(true);
    pass.filter(*pc_filtered);

    // For nuscene private 2
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(0.2, 200);
    // pass.filter(*pc_filtered);
    

    // Remove nan
    std::vector<int> indice; 
    pcl::removeNaNFromPointCloud(*pc_filtered, *pc_filtered, indice);

    // Only publish topic when there are any subscriber
    if(pub_local_pc_.getNumSubscribers() > 0){
        sensor_msgs::PointCloud2 local_pc_msg;
        pcl::toROSMsg(*pc_filtered, local_pc_msg);
        local_pc_msg.header.frame_id = msg->header.frame_id;
        pub_local_pc_.publish(local_pc_msg);
    }

    clouds_list_.push_back(pc_filtered);

    // Find the IMU message whose timestamp is closest to LiDAR message 
    if(clouds_list_.size() > 1) {
        double min_time_diff = 10;
        double lidar_t = pcl_conversions::fromPCL(clouds_list_[clouds_list_.size()- 2]->header.stamp).toSec();
        int index_closest = -1;
        for (int i = 0; i < imu_tmp_list_.size(); ++i) {
            double imu_t = imu_tmp_list_[i].header.stamp.toSec();
            if((abs(lidar_t - imu_t) < min_time_diff) && (abs(lidar_t - imu_t) >= 0)){
                min_time_diff = abs(lidar_t - imu_t);
                index_closest = i;
            }
        }
        // TEST
        if(index_closest > 0) index_closest--;
        imu_list_.push_back(imu_tmp_list_[index_closest]);
        imu_tmp_list_.erase(imu_tmp_list_.begin(), imu_tmp_list_.begin() + index_closest + 1);
    }
}


void Solution3Node::imu_cb(const sensor_msgs::ImuConstPtr &msg) {
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(msg->orientation, q);
    Eigen::Matrix3d m_q;
    m_q = q.toRotationMatrix();

    // Get initial orientaion from IMU
    if(cnt_imu_msgs_ == 0) {
        guess_matrix_.topLeftCorner<3, 3>() = m_q.topLeftCorner<3, 3>();
    }
    cnt_imu_msgs_++;

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "imu";
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.pose.orientation = msg->orientation;

    pub_imu_pose_.publish(pose_msg);
    imu_tmp_list_.push_back(*msg);
}

// Deprecated, manually initall guess
void Solution3Node::pose_cb(const geometry_msgs::PoseStampedPtr &msg) {
    if(flag_init_guess_ == false) {
        Eigen::Affine3d try_matrix;
        tf::poseMsgToEigen(msg->pose, try_matrix);
        cout << "init_guess manually:" << endl;
        cout << try_matrix.matrix() << endl;
    
        double z = guess_matrix_(2, 3);
        guess_matrix_ = try_matrix.matrix();
        guess_matrix_(2, 3) = z;
        renounce_guess_info();
    }
}

void Solution3Node::gnss_cb2(const geometry_msgs::PointStampedConstPtr &msg) {
    // Get initial position from GPS
    if(cnt_gps_msgs_ == 0) {
        guess_matrix_(0, 3) = msg->point.x; 
        guess_matrix_(1, 3) = msg->point.y;
        guess_matrix_(2, 3) = msg->point.z;
        first_timestamp_ = msg->header.stamp;
    }

    // Continune showing the GPS path
    line_strip_.points.push_back(msg->point);
    if(line_strip_.points.size()>15000)
        line_strip_.points.clear();
    pub_gps_marker_.publish(line_strip_);

    cnt_gps_msgs_++;
    gps_list_.push_back(*msg);
}

void Solution3Node::publish_path_from_matrix(const Eigen::Matrix4d &m, const ros::Time &t, bool show_info) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = t;
    pose_msg.header.frame_id = kGlobalFrame;
    Eigen::Affine3d affine_matrix(m);
    tf::poseEigenToMsg(affine_matrix, pose_msg.pose);
    
    // Collect path
    result_path_.poses.push_back(pose_msg);
    result_path_.header.stamp = t;
    pub_result_path_.publish(result_path_);

    // Show infomation
    if(show_info == true){
        cout << "timestamp: " << t << endl;
        cout << "x: " << pose_msg.pose.position.x
             << ", y: " << pose_msg.pose.position.y
             << ", z: " << pose_msg.pose.position.z << endl;
        tf::Matrix3x3 tf_m;
        double yaw, pitch, roll;
        tf::matrixEigenToTF(m.topLeftCorner<3, 3>(), tf_m);
        tf_m.getEulerYPR(yaw, pitch, roll);
        cout << "yaw: " << yaw
             << ", pitch: " << pitch
             << ", roll: " << roll << endl;
        cout << "\n=============================================\n" << endl;
    }
}

template <typename T>
std::vector<T> &operator+=(std::vector<T> &A, const std::vector<T> &B) {
    A.reserve( A.size() + B.size() );                // preallocate memory without erase original data
    A.insert( A.end(), B.begin(), B.end() );         // add B;
    return A;                                        // here A could be named AB
}

template<typename T> 
std::vector<std::vector<std::vector<T>>> make_3d_vector(int z, int y, int x, T value = T{}) {
    return std::vector<std::vector<std::vector<T>>>(z, std::vector<std::vector<T>>(y, std::vector<T>(x, value)));
}

void Solution3Node::remove_plane(PointCloudXYZPtr cloud_in, PointCloudXYZPtr cloud_out) {
    // RANSAC remove plane 
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (1.0);
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_out);    

}

void Solution3Node::extract_straight_objects(PointCloudXYZPtr cloud_in, PointCloudXYZPtr cloud_out, int num_threshold) {
    static auto vec = make_3d_vector<int>(submap_size_ * 2, submap_size_ * 2, 0, 0);
    for(int i = 0; i < vec.size(); i++) 
        for(int j = 0; j < vec[i].size(); j++)
            vec[i][j].clear();
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // Calculate the dense of pointcloud from top view
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud_in, min_pt, max_pt);
    for (int i = 0; i < cloud_in->size(); i++) {
        int idx_x = (int)((cloud_in->points[i].x - min_pt.x) * 2);
        int idx_y = (int)((cloud_in->points[i].y - min_pt.y) * 2);
        vec[idx_y][idx_x].push_back(i);
    }

    // Extract the high density area of pointcloud
    for(int i = 0; i < vec.size(); i++) { 
        for(int j = 0; j < vec[i].size(); j++) {
            // if(vec[i][j].size() >= 10) cout << vec[i][j].size() << ", ";
            if(vec[i][j].size() >= num_threshold)
                inliers->indices += vec[i][j];
        }
    }
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_out);
}

// The thread for maintaining the ROS stuff and checking work progress 
void check_guess_finished(Solution3Node& node) {
    while(true) {
        ros::spinOnce();

        bool flag_finished = true;
        for (int i = 0; i < node.guess_list_.size(); i++) {
            if(node.guess_list_[i].status_ != 2)
                flag_finished = false;
        }
        if(flag_finished == true)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// The thread for work conducting, note that Mutex is used for avoiding task repeat 
mutex gMutex;
void multi_thread_guess(Solution3Node& node) {
    for (int i = 0; i < node.guess_list_.size(); i++) {
        int index_todo = -1;

        // Critical section
        gMutex.lock();
        if(node.guess_list_[i].status_ == 0) {
            node.guess_list_[i].status_ = 1;    // status 0 -> 1(busy)
            index_todo = i;
        }
        gMutex.unlock();
        
        // If get job, do it
        if(index_todo != -1){
            node.guess_list_[index_todo].guess_matrix_ = node.guess_list_[index_todo].original_matrix_;
            for(int j = 0; j < node.guess_list_[index_todo].clouds_list_.size(); j++){
                pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> quick_icp;
                quick_icp.setMaximumIterations(10);
                quick_icp.setTransformationEpsilon(1e-14);
                quick_icp.setEuclideanFitnessEpsilon(1e-14); 

                PointCloudXYZPtr lidar_pc(new PointCloudXYZ);
                PointCloudXYZPtr submap_pc(new PointCloudXYZ);
                PointCloudXYZPtr result_pc(new PointCloudXYZ);
                lidar_pc = node.guess_list_[index_todo].clouds_list_[j];

                // Extract sub map from guessed position
                double current_x = node.guess_matrix_(0, 3);
                double current_y = node.guess_matrix_(1, 3);
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud(node.map_pc_ptr_);
                pass.setFilterFieldName("x");
                pass.setFilterLimits(current_x - node.submap_size_/2, current_x + node.submap_size_/2);
                pass.filter(*submap_pc);
                pass.setInputCloud(submap_pc);
                pass.setFilterFieldName("y");
                pass.setFilterLimits(current_y - node.submap_size_/2, current_y + node.submap_size_/2);
                pass.filter(*submap_pc);

                // ICP
                quick_icp.setInputSource(lidar_pc);
                quick_icp.setInputTarget(submap_pc); // map_pc_ptr_
                Eigen::Matrix4f guess = node.guess_list_[index_todo].guess_matrix_.matrix().cast<float>();
                quick_icp.align(*result_pc, guess);
                node.guess_list_[index_todo].score_ += quick_icp.getFitnessScore();
                node.guess_list_[index_todo].guess_matrix_ = quick_icp.getFinalTransformation().cast<double>();
            }
            node.guess_list_[index_todo].status_ = 2;
        } 
    }
}

int main(int argc, char** argv){
    // Random seed
    srand(time(NULL));

    ros::init(argc, argv, "solution1_node");
    ros::NodeHandle nh, pnh("~");
    Solution3Node node(nh, pnh);

    // ROS parameters
    int kNumThreads;
    int kNumPretestFrames;    // Number of pointcloud frames used to pre-test initial guess
    int kNumPretestPoses;
    ros::param::param<int>("~num_threads", kNumThreads, 4);
    ros::param::param<int>("~num_pretest_frames", kNumPretestFrames, 4);
    ros::param::param<int>("~num_pretest_poses", kNumPretestPoses, 200);
    
    // Create csv result file
    std::fstream file;
    string package_path = ros::package::getPath("midterm_localization") + "/";
    string result_root = package_path + "csv_files/";
    if(!boost::filesystem::exists(result_root))         // Check the directory
        boost::filesystem::create_directory(result_root);
    file.open(result_root + node.result_filename_, std::fstream::out);
    file << setiosflags(ios::fixed);

    // Create thread vector
    std::vector<std::thread> threads_list;

    int index_lidar_frame = 0;
    while(ros::ok()) {
        ros::spinOnce();
        if(node.flag_init_guess_ == false) {
            // Initial guess stategy
            if(node.cnt_imu_msgs_ > 0 && node.cnt_gps_msgs_ > 0 
                && node.clouds_list_.size() >= kNumPretestFrames) {

                for(int i = 0; i < kNumPretestPoses; i++) {
                    // Generate random transformation matries
                    double rot_x = ((double)rand() / RAND_MAX * 2.0 - 1.0) *  M_PI;
                    double rot_y = ((double)rand() / RAND_MAX * 2.0 - 1.0) *  M_PI;
                    double rot_z = ((double)rand() / RAND_MAX * 2.0 - 1.0) *  M_PI;
                    double trans_x = ((double)rand() / RAND_MAX * 2.0 - 1.0);
                    double trans_y = ((double)rand() / RAND_MAX * 2.0 - 1.0);
                    double trans_z = ((double)rand() / RAND_MAX * 2.0 - 1.0);
                    Eigen::Matrix4d small_tf = Eigen::Matrix4d::Identity(4, 4);
                    Eigen::Matrix3d m;
                    m = Eigen::AngleAxisd(rot_x, Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(rot_y,  Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(rot_z, Eigen::Vector3d::UnitZ());
                    small_tf.topLeftCorner<3, 3>() = m;
                    small_tf(0, 3) = trans_x;
                    small_tf(1, 3) = trans_y;
                    small_tf(2, 3) = trans_z;

                    // Custom object to collect guess infomations
                    InitGuessInfo info;
                    info.original_matrix_ = node.guess_matrix_ * small_tf;
                    for(int j = 0; j < kNumPretestFrames; j++)
                        info.clouds_list_.push_back(node.clouds_list_[j]);
                    node.guess_list_.push_back(info);

                    // Display guesses
                    // node.publish_path_from_matrix(info.original_matrix_, node.first_timestamp_, false);
                }

                // The cadidate is from original IMU / GPS info
                InitGuessInfo info;
                info.original_matrix_ = node.guess_matrix_;
                for(int j = 0; j < kNumPretestFrames; j++)
                    info.clouds_list_.push_back(node.clouds_list_[j]);
                node.guess_list_.push_back(info);

                // Create multi-threads for guessing task
                for(int i = 0; i < kNumThreads; i++)
                    threads_list.push_back(std::thread(multi_thread_guess, std::ref(node)));

                // Additional thread for maintain ROS
                threads_list.push_back(std::thread(check_guess_finished, std::ref(node)));

                // Wait for threads
                for(int i = 0; i < threads_list.size(); i++)
                    threads_list[i].join();

                // Choose the best initial guess candidate
                int index_best = 0;
                for(int i = 0; i < kNumPretestPoses; i++){
                    double score = node.guess_list_[i].score_;
                    if(score < node.guess_list_[index_best].score_)
                        index_best = i;
                    cout << "The " << i << "th job get total score: " << score << endl;
                }
                cout << "The best candidate is: " << index_best
                        << ", score: " << node.guess_list_[index_best].score_ << endl;
                node.guess_matrix_ = node.guess_list_[index_best].original_matrix_;
                node.renounce_guess_info();
            }
        }else {
            // Main ICP process
            if(!node.clouds_list_.empty() && node.imu_list_.size() >= 2) {
                // cout << "Stored clouds queue size: " << node.clouds_list_.size() << endl;
                PointCloudXYZPtr lidar_pc(new PointCloudXYZ);
                PointCloudXYZPtr submap_pc(new PointCloudXYZ);
                PointCloudXYZPtr result_pc(new PointCloudXYZ);
                
                // Get lidar pc from clouds_list
                lidar_pc = node.clouds_list_.front();
                node.clouds_list_.erase(node.clouds_list_.begin());
                index_lidar_frame++;

                // Extract sub map from guessed position
                double current_x = node.guess_matrix_(0, 3);
                double current_y = node.guess_matrix_(1, 3);
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud(node.map_pc_ptr_);
                pass.setFilterFieldName("x");
                pass.setFilterLimits(current_x - node.submap_size_/2, current_x + node.submap_size_/2);
                pass.filter(*submap_pc);
                pass.setInputCloud(submap_pc);
                pass.setFilterFieldName("y");
                pass.setFilterLimits(current_y - node.submap_size_/2, current_y + node.submap_size_/2);
                pass.filter(*submap_pc);

                // node.voxel_grid_.setInputCloud(lidar_pc);
                // node.voxel_grid_.filter(*lidar_pc);
                // node.voxel_grid_.setInputCloud(submap_pc);
                // node.voxel_grid_.filter(*submap_pc);

                // Remove plane
                // node.remove_plane(lidar_pc, lidar_pc);

                // Extract trees
                // node.extract_straight_objects(submap_pc, submap_pc, 5);
                // node.extract_straight_objects(lidar_pc, lidar_pc, 10);

                // Publish sub-map pointcloud in green color
                if(node.pub_submap_.getNumSubscribers() > 0) {
                    PointCloudXYZRGBPtr pc_colored(new PointCloudXYZRGB);
                    pcl::copyPointCloud(*submap_pc, *pc_colored);
                    for(auto& point: *pc_colored) {
                        point.r = 0;
                        point.g = 255;
                        point.b = 0;
                    }
                    sensor_msgs::PointCloud2 map_msg;
                    pcl::toROSMsg(*pc_colored, map_msg);
                    map_msg.header.frame_id = kGlobalFrame;
                    node.pub_submap_.publish(map_msg);
                }

                // Take GPS position into initial guess consideration
                // geometry_msgs::PointStamped tmp_pose = node.gps_list_[index_lidar_frame - 1];
                // node.guess_matrix_(0, 3) = tmp_pose.point.x;
                // node.guess_matrix_(1, 3) = tmp_pose.point.y;
                // node.guess_matrix_(2, 3) = tmp_pose.point.z;

                // Publish guessed lidar pointcloud in blue color
                if(node.pub_result_pc_.getNumSubscribers() > 0) {
                    Eigen::Affine3d affine_matrix(node.guess_matrix_);
                    PointCloudXYZPtr pc_transformed(new PointCloudXYZ);
                    pcl::transformPointCloud(*lidar_pc, *pc_transformed, affine_matrix);

                    PointCloudXYZRGBPtr pc_colored(new PointCloudXYZRGB);
                    pcl::copyPointCloud(*pc_transformed, *pc_colored);
                    for(auto& point: *pc_colored) {
                        point.r = 0;
                        point.g = 0;
                        point.b = 255;
                    }
                    sensor_msgs::PointCloud2 map_msg;
                    pcl::toROSMsg(*pc_colored, map_msg);
                    map_msg.header.frame_id = kGlobalFrame;
                    node.pub_result_pc_.publish(map_msg);
                }

                // Main icp process
                node.icp_.setInputSource(lidar_pc);
                node.icp_.setInputTarget(submap_pc); // node.map_pc_ptr_
                node.icp_.align(*result_pc, node.guess_matrix_.cast<float>());
                cout << "Size of clouds_list: " << node.clouds_list_.size() << endl;
                cout << "Index of frame: " << index_lidar_frame << endl;
                cout << "ICP has converged:" << node.icp_.hasConverged();
                cout << COLOR_GREEN << " score: " << node.icp_.getFitnessScore() << COLOR_NC << std::endl;
                node.guess_matrix_ = node.icp_.getFinalTransformation().cast<double>();
                
                // Get icp's transform matrix
                // TEST
                /*
                Eigen::Matrix4d result_transform = node.icp_.getFinalTransformation().cast<double>();
                Eigen::Affine3d affine_guess(node.guess_matrix_);
                Eigen::Matrix4d guess_inverse = affine_guess.inverse().matrix();
                Eigen::Matrix4d icp_transform = result_transform * guess_inverse;
                ///cout << "guess:\n" << node.guess_matrix_ << endl;
                ///cout << "icp:\n" << icp_transform << endl;
                ///cout << "result\n" << result_transform << endl;
                Eigen::Vector3d icp_rot_euler = icp_transform.topLeftCorner<3, 3>().eulerAngles(2, 1, 0);
                    cout << "icp tell you (d_yaw, d_pitch, d_roll): ";
                    cout << icp_rot_euler[0] << ", ";
                    cout << icp_rot_euler[1] << ", ";
                    cout << icp_rot_euler[2] << endl;

                if(index_lidar_frame > 1) {
                    Eigen::Quaterniond imu_ori_q1, imu_ori_q2;
                    ///cout << "stamp1: " << node.imu_list_.front().header.stamp;
                    ///cout << ", stamp2: " << node.imu_list_.back().header.stamp << endl;
                    tf::quaternionMsgToEigen(node.imu_list_.front().orientation, imu_ori_q1);
                    tf::quaternionMsgToEigen(node.imu_list_.back().orientation, imu_ori_q2);
                    imu_ori_q1.normalize();
                    imu_ori_q2.normalize();
                    Eigen::Matrix3d imu_transform = imu_ori_q1.toRotationMatrix().inverse()
                                                     * imu_ori_q2.toRotationMatrix();
                    ///cout << "imu_transform\n" << imu_transform << endl;     
                    ///cout << "imu_orientation\n" << imu_ori_q2.toRotationMatrix() << endl;
                    
                    Eigen::Vector3d imu_rot_euler = imu_transform.eulerAngles(2, 1, 0);
                    cout << "imu tell you (d_yaw, d_pitch, d_roll): ";
                    cout << imu_rot_euler[0] << ", ";
                    cout << imu_rot_euler[1] << ", ";
                    cout << imu_rot_euler[2] << endl;

                    if(node.imu_list_.front().header.stamp == ros::Time(0)
                        || node.imu_list_.back().header.stamp == ros::Time(0))
                        exit(-1);


                    // Simple weight sharing method
                    double sensor_confidence;
                    double icp_score = node.icp_.getFitnessScore();
                    sensor_confidence = 0.1;
                    // if(icp_score >= 1) sensor_confidence = 0.95;
                    // else if(icp_score >= 0.1 && icp_score < 1) sensor_confidence = 0.8;
                    // else if(icp_score >= 0.01 && icp_score < 0.1) sensor_confidence = 0.7;
                    // else sensor_confidence = 0.6;

                    Eigen::Vector3d imu_euler = imu_ori_q2.toRotationMatrix().eulerAngles(2, 1, 0);
                    Eigen::Vector3d icp_euler = result_transform.topLeftCorner<3, 3>().eulerAngles(2, 1, 0);
                    Eigen::Vector3d estimate_euler = icp_euler + sensor_confidence * (imu_euler - icp_euler);
                    Eigen::Matrix3d m;
                    m = Eigen::AngleAxisd(estimate_euler[2], Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(estimate_euler[1],  Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(estimate_euler[0], Eigen::Vector3d::UnitZ());

                    
                    // node.guess_matrix_.topLeftCorner<3, 3>() = m;
                    // node.guess_matrix_.topLeftCorner<3, 3>() = imu_ori_q2.toRotationMatrix();
                    ///cout << "final:\n" << node.guess_matrix_ << endl; 

                    node.imu_list_.erase(node.imu_list_.begin());
                    node.guess_matrix_ = result_transform;
                }
                else{
                    node.guess_matrix_ = result_transform;
                }
                */


                // Visualize path
                ros::Time tt = pcl_conversions::fromPCL(lidar_pc->header.stamp);
                node.publish_path_from_matrix(node.guess_matrix_, tt, true);
                
                // Publish result pointcloud in red color
                if(node.pub_result_pc_.getNumSubscribers() > 0) {
                    PointCloudXYZRGBPtr pc_colored(new PointCloudXYZRGB);
                    pcl::copyPointCloud(*result_pc, *pc_colored);
                    for(auto& point: *pc_colored) {
                        point.r = 255;
                        point.g = 0;
                        point.b = 0;
                    }
                    sensor_msgs::PointCloud2 map_msg;
                    pcl::toROSMsg(*pc_colored, map_msg);
                    map_msg.header.frame_id = kGlobalFrame;
                    node.pub_result_pc_.publish(map_msg);
                }

                // Export result to CSV
                Eigen::Vector3d transition = node.guess_matrix_.block(0, 3, 3, 1); // extract i,j,row,col
                tf::Matrix3x3 tf_m;
                double yaw, pitch, roll;
                tf::matrixEigenToTF(node.guess_matrix_.block(0, 0, 3, 3).topLeftCorner<3, 3>(), tf_m);
                tf_m.getEulerYPR(yaw, pitch, roll); 
                file << tt.toSec() << ",";
                file << transition(0) << "," << transition(1) << "," << transition(2) << ",";
                file << yaw << "," << pitch << "," << roll << endl;
            }
        }
    }
    file.close();
    return 0;
}