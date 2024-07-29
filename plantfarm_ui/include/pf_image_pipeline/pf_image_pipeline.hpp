#ifndef PF_IMAGE_PIPELINE_HPP
#define PF_IMAGE_PIPELINE_HPP

#include <vector>
#include <cmath>
#include <ctime>
#include <string>
#include <algorithm>
#include <thread>
#include <memory>
#include <chrono>
#include <eigen3/Eigen/Dense>

// ROS Headers
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "plantfarm_msgs/msg/yolo_kpt.hpp"
#include "plantfarm_msgs/msg/yolo_result.hpp"
#include "plantfarm_msgs/msg/yolo_result_list.hpp"
#include "plantfarm_msgs/msg/pos_results.hpp"
#include "dsr_msgs2/srv/get_current_posx.hpp"
#include "dsr_msgs2/srv/get_current_rotm.hpp"
#include "dsr_msgs2/srv/get_current_pose.hpp"
#include "dsr_msgs2/msg/current_rotm.hpp"
// #include "visualization_msgs/msg/marker.hpp"

// DSR Headers

// OpenCV Headers
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// PCL Headers
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

class pf_image_pipeline : public rclcpp::Node
{
private:

    /* 콜백 함수 선언 */
    void depth_image_sub_cb(const sensor_msgs::msg::Image::SharedPtr image_raw);
    void color_camera_info_sub_cb(const sensor_msgs::msg::CameraInfo::SharedPtr depth_camera_info);
    void yolo_cb(const plantfarm_msgs::msg::YoloResultList::SharedPtr yolo);
    void yolo_kpt_cb(const plantfarm_msgs::msg::YoloKPT::SharedPtr  yolo_kpt);
    void dsr_rotm_cb(const dsr_msgs2::msg::CurrentRotm::SharedPtr dsr_rotm);
    void pipeline_pub_cb();

    /* 멤버 변수 선언 */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr AlignedDepth;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr CamInfo;
    rclcpp::Subscription<plantfarm_msgs::msg::YoloResultList>::SharedPtr YoloResultList;
    rclcpp::Subscription<plantfarm_msgs::msg::YoloKPT>::SharedPtr YoloKPT;
    rclcpp::Subscription<dsr_msgs2::msg::CurrentRotm>::SharedPtr DsrRotm;

    rclcpp::Publisher<plantfarm_msgs::msg::PosResults>::SharedPtr PosPub;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ImgPub;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Client<dsr_msgs2::srv::GetCurrentRotm>::SharedPtr RotmClient;
    rclcpp::Client<dsr_msgs2::srv::GetCurrentPosx>::SharedPtr PosxClient;
    rclcpp::Client<dsr_msgs2::srv::GetCurrentPose>::SharedPtr PoseClient;

    /* 기타 함수 선언 */
    cv::Mat make_contour_mask(cv::Mat &depth_image, std::vector<std::vector<cv::Point>> contour, std::vector<std::vector<cv::Point>> kpt, int idx);
    cv::Mat dot_kpt_mask(cv::Mat &depth_image, std::vector<std::vector<cv::Point>> contour, std::vector<std::vector<cv::Point>> kpt, int idx);
    pcl::PointCloud<pcl::PointXYZ> depth_to_pointcloud(cv::Mat depth_image);
    void abnormal_pointcloud(pcl::PointCloud<pcl::PointXYZ> abnormal_depth, pcl::PointCloud<pcl::PointXYZ> cloud1, pcl::PointCloud<pcl::PointXYZ> cloud2 , cv::Point direction);
    pcl::PointXYZ move_point_towards(const pcl::PointCloud<pcl::PointXYZ>& cloud1, const pcl::PointCloud<pcl::PointXYZ>& cloud2, double distance);
    void compute_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base, Eigen::Matrix4d camera2endeffector, Eigen::Matrix4d endeffector2base, Eigen::Matrix<float, 4, 1> centroid_point2,
                        Eigen::Vector4f centroid1, Eigen::Matrix4d camera2base, pcl::PointXYZ moved_point, pcl::PointXYZ origin_point, cv::Point direction);
                        
    /* 전역 변수 선언 */
    int color_info_count;
    int image_w, image_h;
    std::string color_profile;
    std::array<double, 9> K;        // camera intrinsics
    std::vector<double> D;            // distortion coefficients
    cv::Mat aligned_depth_image;
    std::vector<std::vector<cv::Point>> abnormal_contours;
    std::vector<std::vector<cv::Point>> keypoint;
    std::vector<std::vector<cv::Point>> keypoint2;
    std::array<std::array<double, 3>, 3> rotation_matrix;
    std::array<double,6> pose_x;
    std::array<float, 3> calculated_origin_coord; 
    std::array<float, 3> calculated_origin_coord2; 
    std::array<float, 6> calculated_cam_coord;     
    std::array<float, 6> calculated_cam_coord2;   
    std::array<float, 12> calculated_tool_coord; 
    std::array<float, 6> target_coord; 
    std::array<float, 6> target_tool_coord; 

public:
    explicit pf_image_pipeline(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
    virtual ~pf_image_pipeline();
};


#endif //PF_IMAGE_PIPELINE_HPP