// #define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
#include "pf_image_pipeline/pf_image_pipeline.hpp"

// using std::placeholders::_1;

pf_image_pipeline::pf_image_pipeline(const rclcpp::NodeOptions& node_options)
: Node("image_pipeline",node_options),
  color_profile(""),
  image_w(0), image_h(0)
{
    auto client = this->create_client<rcl_interfaces::srv::GetParameters>("/camera/camera/get_parameters");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for Realsense. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for Realsense...");
        }
    //
    auto camera_info_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    camera_info_request->names.push_back("rgb_camera.color_profile");

    // 비동기 서비스 호출
    auto camera_info_result_future = client->async_send_request(camera_info_request);
    
    // 결과 기다리기
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), camera_info_result_future) == 
        rclcpp::FutureReturnCode::SUCCESS) {
        
        auto camera_info_response = camera_info_result_future.get();
        if (!camera_info_response->values.empty() && camera_info_response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
            color_profile = camera_info_response->values[0].string_value;
            std::cout << color_profile << endl;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Please check align_depth is true");
            throw "Please check align_depth is true";
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service get_parameters");
    }

    // 첫 번째 'x'의 위치 찾기
    size_t pos1 = color_profile.find('x');
    if (pos1 == std::string::npos) {
        std::cerr << "Format error: no 'x' found" << std::endl;
    }

    // 두 번째 'x'의 위치 찾기
    size_t pos2 = color_profile.find('x', pos1 + 1);
    if (pos2 == std::string::npos) {
        std::cerr << "Format error: only one 'x' found" << std::endl;
    }

    try {
        image_w = std::stoi(color_profile.substr(0, pos1));
        image_h = std::stoi(color_profile.substr(pos1 + 1, pos2 - pos1 - 1));
    } catch (const std::invalid_argument& e) {
        std::cerr << "Conversion error: invalid argument" << std::endl;
    } catch (const std::out_of_range& e) {
        std::cerr << "Conversion error: out of range" << std::endl;
    }

    // std::cout << image_w << image_h << endl;

    RCLCPP_INFO(this->get_logger(), "Image Pipeline node start!!!");

    // QoS 설정
    const auto QOS_YLNKPT = 
        rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();

    color_info_count = 0;

    rotation_matrix = {{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.}}};
    pose_x = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    calculated_cam_coord = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    calculated_cam_coord2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    calculated_tool_coord = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    AlignedDepth = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/camera/aligned_depth_to_color/image_raw", 10, std::bind(&pf_image_pipeline::depth_image_sub_cb, this, std::placeholders::_1));
    
    CamInfo = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera/camera/color/camera_info", 1, std::bind(&pf_image_pipeline::color_camera_info_sub_cb, this, std::placeholders::_1));
    
    YoloResultList = this->create_subscription<plantfarm_msgs::msg::YoloResultList>(
      "yolov7/result", QOS_YLNKPT, std::bind(&pf_image_pipeline::yolo_cb, this, std::placeholders::_1));
    
    YoloKPT = this->create_subscription<plantfarm_msgs::msg::YoloKPT>(
      "yolov8_kpt/result", QOS_YLNKPT, std::bind(&pf_image_pipeline::yolo_kpt_cb, this, std::placeholders::_1));

    DsrRotm = this->create_subscription<dsr_msgs2::msg::CurrentRotm>(
      "dsr01/msg/current_rotm", 10, std::bind(&pf_image_pipeline::dsr_rotm_cb, this, std::placeholders::_1));

    PosPub = this->create_publisher<plantfarm_msgs::msg::PosResults>("/image_pipeline/result", QOS_YLNKPT);
    // ImgPub = this->create_publisher<sensor_msgs::msg::Image>("/pipeline/image_topic", QOS_YLNKPT);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&pf_image_pipeline::pipeline_pub_cb, this));

    RotmClient = this->create_client<dsr_msgs2::srv::GetCurrentRotm>("/dsr01/aux_control/get_current_rotm");
    PosxClient = this->create_client<dsr_msgs2::srv::GetCurrentPosx>("/dsr01/aux_control/get_current_posx");
    PoseClient = this->create_client<dsr_msgs2::srv::GetCurrentPose>("/dsr01/system/get_current_pose");
    
    // if(image_w==-1 || image_h==-1)    // w==-1 , h==-1 이면 발생 & 뎁스 이미지와 컬러 해상도가 서로 안맞아도 발생
    // {
    //   RCLCPP_ERROR(this->get_logger(),"please check realsense in connected in USB3.0 mode");
    //   throw "please check realsense in connected in USB3.0 mode";
    // }
}

pf_image_pipeline::~pf_image_pipeline()
{
    RCLCPP_INFO(this->get_logger(), "Image Pipeline node end!!!");
}

/* Callback Functions */

void pf_image_pipeline::depth_image_sub_cb(const sensor_msgs::msg::Image::SharedPtr image_raw) 
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {    
        cv_ptr = cv_bridge::toCvCopy(image_raw, image_raw->encoding);
        // depth_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::TYPE_16UC1);
        // 16-bit의 회색조 이미지로 Depth이미지를 opencv형태로 받아옴
        // depth image 형식 => cv::Mat depth_image; 
        aligned_depth_image = cv_ptr->image;
    }
    catch(cv_bridge::Exception& e)
    {
        // e.what으로 예외에 관한 내용을 저장하는 문자열 필드 값을 들여다 볼 수 있음
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}

void pf_image_pipeline::color_camera_info_sub_cb(const sensor_msgs::msg::CameraInfo::SharedPtr depth_camera_info) 
{
    if(color_info_count>0) return;
    K = depth_camera_info->k;
    D = depth_camera_info->d;

    // std::cout << K[0] << K[2] << endl;

    color_info_count++;
}

void pf_image_pipeline::yolo_cb(const plantfarm_msgs::msg::YoloResultList::SharedPtr yolo)
{
    // yolo 결과 중 컨투어 영역에 해당하는 것들
    // yolo는 주소값만 가지고 있음. msg안의 YoloResultListPtr에 정의된 ret을 가져옴.
    auto yolo_returns = yolo->ret;
    abnormal_contours.clear();
    if(abnormal_contours.capacity() > 100)
      abnormal_contours.shrink_to_fit(); // shrink memory
    std::cout<<"yolo returns size : "<<yolo_returns.size()<<std::endl;
    for(auto yolo_ret : yolo_returns)
    {
      // cls
      // 0 : abnormal
      // 1 : plantcloud2
      int16_t cls = yolo_ret.cls;
      std::cout<<"class : "<<cls<<" size : "<<yolo_ret.x.size()<<std::endl;
      
      if(cls != 0) continue; // only abnormal
      if(yolo_ret.x.size() <= 2) continue; //ignore empty contour
      if(yolo_ret.x.size() != yolo_ret.y.size()) throw std::invalid_argument("the nuber of x and y point different");

      static std::vector<cv::Point> contour;
      contour.clear();
      for(int i=0; i<yolo_ret.x.size(); i++)
      {
        static cv::Point temp;

        temp.x = int(yolo_ret.x[i]*image_w);
        temp.y = int(yolo_ret.y[i]*image_h);
        contour.push_back(temp);
      }
      abnormal_contours.push_back(contour);
    //   std::cout<<"contour : "<<contour.size() << " " << abnormal_contours.size()<<std::endl;
    }
}

void pf_image_pipeline::yolo_kpt_cb(const plantfarm_msgs::msg::YoloKPT::SharedPtr yolo_kpt)
{
    // if(kptStop) return;
    auto& yolo_kpt_returns = yolo_kpt->data;
    keypoint.clear();
    keypoint2.clear();
    // Ensure that we have an even number of elements
    if(yolo_kpt_returns.size() % 2 != 0) {
        std::cerr << "yolo_kpt_returns does not contain pairs of coordinates." << std::endl;
        return;
    }
    // detect_leaves_num = 
    // RCLCPP_INFO(this->get_logger(), "RS: %zu",yolo_kpt_returns.size());

    for(std::size_t i = 0; i < yolo_kpt_returns.size(); i += 4)
    {
        std::cout <<"key point x: "<<yolo_kpt_returns[i]<<" y: "<<yolo_kpt_returns[i+1]<<"; x: "<< yolo_kpt_returns[i+2]<<" y: "<< yolo_kpt_returns[i+3] << endl;

        std::vector<cv::Point> contour2, contour4;

        cv::Point temp2, temp4;
        temp2.x = static_cast<int>(yolo_kpt_returns[i+2]);
        temp2.y = static_cast<int>(yolo_kpt_returns[i+3]);

        temp4.x = static_cast<int>(yolo_kpt_returns[i]);
        temp4.y = static_cast<int>(yolo_kpt_returns[i+1]);
        contour2.push_back(temp2);
        contour4.push_back(temp4);

        keypoint.push_back(contour2);
        keypoint2.push_back(contour4);
    } 
}

void pf_image_pipeline::dsr_rotm_cb(const dsr_msgs2::msg::CurrentRotm::SharedPtr dsr_rotm)
{
    // std::cout << current_rotm.size() << endl;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            auto& current_rotm = dsr_rotm->rotation_matrix[0];
            rotation_matrix[i][j] = current_rotm.data[i * 3 + j];
            // std::cout <<" | " <<rotation_matrix[i][j] ;
        }
        // std::cout << endl;
    }
    for (size_t i = 0; i < 6; ++i) pose_x[i] = dsr_rotm->posx[i]; // std::cout <<" | " <<pose_x[i] ;}
    // std::cout << endl;
}

void pf_image_pipeline::pipeline_pub_cb()
{
    if(aligned_depth_image.empty()) return;
    if(abnormal_contours.empty()) return;
    if(keypoint.empty()) return;
    // if(kpt2.empty()) return;
    
    if(abnormal_contours.size() != 1) return; // 일단은 객체가 한 개 일 때

    for(int i=0; i<abnormal_contours.size(); i++)
    {
        cv::Mat abnormal_depth = make_contour_mask(aligned_depth_image, abnormal_contours, keypoint, i);

        cv::Mat abnormal_depth2 = dot_kpt_mask(aligned_depth_image, abnormal_contours, keypoint, i);
        cv::Mat abnormal_depth4 = dot_kpt_mask(aligned_depth_image, abnormal_contours, keypoint2, i);

        /* for test */ 
        // std::cout << abnormal_depth.size() << endl;
        // cv::Mat normalizedDepthImage,normalizedDepthImage2,normalizedDepthImage4;
        // cv::normalize(abnormal_depth, normalizedDepthImage, 0, 255, cv::NORM_MINMAX, CV_8U);
        // cv::normalize(abnormal_depth2, normalizedDepthImage2, 0, 255, cv::NORM_MINMAX, CV_8U);
        // cv::normalize(abnormal_depth4, normalizedDepthImage4, 0, 255, cv::NORM_MINMAX, CV_8U);
        // cv::imshow("contour_mask", normalizedDepthImage);
        // cv::imshow("keypoint1", normalizedDepthImage2);
        // cv::imshow("keypoint2", normalizedDepthImage4);
        /* ~for test */

        pcl::PointCloud<pcl::PointXYZ> cloud_contour = depth_to_pointcloud(abnormal_depth);
        pcl::PointCloud<pcl::PointXYZ> cloud_keypoint1 = depth_to_pointcloud(abnormal_depth2);
        pcl::PointCloud<pcl::PointXYZ> cloud_keypoint2 = depth_to_pointcloud(abnormal_depth4);
        
        cv::Point pointFromKpt = keypoint[0][0];  // i와 j는 원하는 인덱스입니다.
        cv::Point pointFromKpt2 = keypoint2[0][0];  // 동일한 인덱스를 사용하거나 다른 인덱스를 사용할 수 있습니다.
        cv::Point direction = pointFromKpt2 - pointFromKpt;

        std::cout << direction.x << " "<< direction.y << endl;

        // publish_pointcloud(cloud_keypoint1);
        // // print_pc(cloud2);
        abnormal_pointcloud(cloud_contour, cloud_keypoint1, cloud_keypoint2, direction);

        plantfarm_msgs::msg::PosResults PosRmsg;

        bool all_non_zero = std::all_of(calculated_cam_coord.begin(), calculated_cam_coord.end(), [](float value) {
        return value != 0.0f;
        });

        if(all_non_zero)
        {
            for(int i = 0; i<6 ; i++) 
            {
                PosRmsg.cam_pos[i] = calculated_cam_coord[i];
                PosRmsg.cam_pos_cor[i] = calculated_cam_coord2[i];
                PosRmsg.tool_pos_out[i] = calculated_tool_coord[i];
                PosRmsg.tool_pos_in[i] = calculated_tool_coord[i + 6];
            }
        }
        PosPub -> publish(PosRmsg);
        // centroid_move(cloud2, cloud4);

        
        // cv::waitKey(1);

    }
}

/* Utility Functions */

cv::Mat pf_image_pipeline::make_contour_mask(cv::Mat &depth_image, std::vector<std::vector<cv::Point>> contour, std::vector<std::vector<cv::Point>> kpt, int idx)
{
    cv::Mat contour_mask = cv::Mat::zeros(depth_image.size(), CV_16UC1);

    cv::Mat kpt_mask = cv::Mat::zeros(depth_image.size(), CV_16UC1);
    try {
        drawContours(contour_mask, contour, idx, cv::Scalar(int(std::pow(2,16)-1)), -1);
    } catch (const cv::Exception& e) {
        // Handle or report error
        std::cerr << "OpenCV Error: " << e.what() << std::endl;
    }  
    try {
        drawContours(kpt_mask, kpt, idx, cv::Scalar(int(std::pow(2,16)-1)), -1);
    } catch (const cv::Exception& e) {
        // Handle or report error
        std::cerr << "OpenCV Error: " << e.what() << std::endl;
    }  
    // cv::Mat erode_mask = cv::Mat::zeros(depth_image.size(), CV_16UC1);;

    cv::erode(contour_mask, contour_mask, cv::Mat(), cv::Point(-1, -1), 8);

    cv::Mat contour_depth = cv::Mat::zeros(contour_mask.size(), CV_16UC1);
    cv::Mat contour_depth_kpt = cv::Mat::zeros(kpt_mask.size(), CV_16UC1);

    // cv::imshow("depth_image", depth_image);
    cv::bitwise_and(depth_image, contour_mask, contour_depth);
    cv::bitwise_and(depth_image, kpt_mask, contour_depth_kpt);

    // cv::Mat noCondepth;

    // cv::normalize(contour_depth, noCondepth, 0, 255, cv::NORM_MINMAX, CV_8U);
    // cv::imshow("noCondepth", noCondepth);
    
    // cv::arrowedLine(contour_mask, (kpt[2], kpt[3]),(kpt[0], kpt[1]), (1), 2)

    return contour_depth;
} 

cv::Mat pf_image_pipeline::dot_kpt_mask(cv::Mat &depth_image, std::vector<std::vector<cv::Point>> contour, std::vector<std::vector<cv::Point>> kpt, int idx)
{
    cv::Mat kpt_mask = cv::Mat::zeros(depth_image.size(), CV_16UC1);
    try {
        drawContours(kpt_mask, kpt, idx, cv::Scalar(int(std::pow(2,16)-1)), -1);
    } catch (const cv::Exception& e) {
        // Handle or report error
        std::cerr << "OpenCV Error: " << e.what() << std::endl;
    }    
    cv::Mat abnormal_depth_kpt = cv::Mat::zeros(kpt_mask.size(), CV_16UC1);
    cv::bitwise_and(depth_image, kpt_mask, abnormal_depth_kpt);

    cv::Mat contour_mask = cv::Mat::zeros(depth_image.size(), CV_16UC1);
    try {
        drawContours(contour_mask, contour, idx, cv::Scalar(int(std::pow(2,16)-1)), -1);
    } catch (const cv::Exception& e) {
        // Handle or report error
        std::cerr << "OpenCV Error: " << e.what() << std::endl;
    }    
    cv::erode(contour_mask, contour_mask, cv::Mat(), cv::Point(-1, -1), 4);
    cv::Mat abnormal_depth = cv::Mat::zeros(contour_mask.size(), CV_16UC1);
    cv::bitwise_and(depth_image, contour_mask, abnormal_depth);

    // Convert to binary images
    cv::Mat kpt_mask_bin = kpt_mask > 0;
    cv::Mat contour_mask_bin = contour_mask > 0;

    double minDist = std::numeric_limits<double>::max();
    cv::Point minLoc;
    for (int y = 0; y < contour_mask_bin.rows; ++y)
    {
        for (int x = 0; x < contour_mask_bin.cols; ++x)
        {
            if (contour_mask_bin.at<uint8_t>(y, x) > 0) // If the point is part of the contour
            {
                cv::Point pt(x, y);

                // Compute distances from the point on the contour to all keypoints
                for (const auto& keypoint : kpt)
                {
                    for (const auto& point : keypoint)
                    {
                        double dx = pt.x - point.x;
                        double dy = pt.y - point.y;
                        double distance = std::sqrt(dx*dx + dy*dy);

                        if (distance < minDist)
                        {
                            minDist = distance;
                            minLoc = pt;
                        }
                    }
                }
            }
        }
    }

    // Draw the closest point on a new mask
    cv::Mat newMask = cv::Mat::zeros(contour_mask.size(), CV_16UC1);
    newMask.at<uint16_t>(minLoc) = std::pow(2,16)-1;

    // cv::Mat kpt_mask = cv::Mat::zeros(depth_image.size(), CV_16UC1);
    // drawContours(kpt_mask, kpt, idx, cv::Scalar(int(std::pow(2,16)-1)), -1);
    cv::Mat abnormal_depth_kpt2 = cv::Mat::zeros(newMask.size(), CV_16UC1);
    cv::bitwise_and(depth_image, newMask, abnormal_depth_kpt2);

    // cv::imshow("erode_contour_mask", contour_mask);
    // cv::imshow("kpt_mask", kpt_mask);
    // cv::imshow("New_mask", newMask);
    // cv::waitKey(1);

    return abnormal_depth_kpt2;  // 여기 2로 바꾸기
}

pcl::PointCloud<pcl::PointXYZ> pf_image_pipeline::depth_to_pointcloud(cv::Mat depth_image)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    int width = depth_image.cols;
    int height = depth_image.rows;

    cloud.clear();
    cloud.is_dense = false;

    // Get the camera intrinsics
    double fx = K.at(0);  // 초점거리
    double fy = K.at(4);
    double cx = K.at(2);  // 주점
    double cy = K.at(5);

    // K = [fx 0 cx;
    //      0 fy cy;
    //      0  0  1]

    for (int v = 0; v < height; v++)
    {
      for (int u = 0; u < width; u++)
      {
        // https://github.com/IntelRealSense/librealsense/blob/5e73f7bb906a3cbec8ae43e888f182cc56c18692/include/librealsense2/rsutil.h#L46
        // get a data of an element from depth image

        // (u,v): 정규좌표계 (카메라 내부 파라미터의 영향을 제거한 이미지 좌표계)
        // 
        uint16_t depth = depth_image.at<uint16_t>(v, u);
        // Skip over pixels with a depth value of zero, which is used to indicate no data
        if(depth==0) continue;

        float x = (u - cx) / fx;
        float y = (v - cy) / fy;
 
        // // Apply distortion
        float r2 = x * x + y * y;
        float f = 1 + D.at(0) * r2 + D.at(1) * r2 * r2 + D.at(4) * r2 * r2 * r2;
        float ux = x * f + 2 * D.at(2) * x * y + D.at(3) * (r2 + 2 * x * x);
        float uy = y * f + 2 * D.at(3) * x * y + D.at(2) * (r2 + 2 * y * y);
        x = ux;
        y = uy;

        pcl::PointXYZ point;
        point.x = float(depth * x / 1000.0);
        point.y = float(depth * y / 1000.0);
        point.z = float(depth / 1000.0);

        // 22, 70, 424

        cloud.push_back(point);
        // if (v%100 == 0 & u%100 == 0){ 
        //   std::cout<<"u : "<<u<<" | v : "<<v<<" | x : "<<point.x<<" | y : "<<point.y<<" | z : "<<point.z<<""<<std::endl;
        // }
      }
    }

    return cloud;
}

void pf_image_pipeline::abnormal_pointcloud(pcl::PointCloud<pcl::PointXYZ> abnormal_depth, pcl::PointCloud<pcl::PointXYZ> cloud1, pcl::PointCloud<pcl::PointXYZ> cloud2, cv::Point direction)
{
    // // 서비스가 준비되었는지 확인
    // if (!PoseClient->wait_for_service(std::chrono::milliseconds(50))) {
    //     RCLCPP_ERROR(this->get_logger(), "Doosan robot launcher is not running now!!!");
    //     return;
    // }

    // int space_type = 1; // 0 = ROBOT_SPACE_JOINT, 1 = ROBOT_SPACE_TASK
    // // 요청 생성
    // auto requestPose = std::make_shared<dsr_msgs2::srv::GetCurrentPose::Request>();
    // requestPose->space_type = space_type;

    // // 비동기 요청
    // using ServiceResponseFuture = rclcpp::Client<dsr_msgs2::srv::GetCurrentPose>::SharedFuture;
    // auto response_received_pose_callback = [this](ServiceResponseFuture future) {
    //     auto responsePose = future.get();
    //     if (responsePose->success) {
    //         // for(int i = 0; i<6; i++) RCLCPP_INFO(this->get_logger(), "%d : %.2lf",i+1,responsePose->pos[i]);
    //     } else {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to service.");
    //     }
    // };

    // // 서비스 호출
    // auto Pose_future_result = PoseClient->async_send_request(requestPose, response_received_pose_callback);

    Eigen::Matrix4d camera2endeffector;
    Eigen::Matrix4d endeffector2base;

    camera2endeffector << 0.9995884550916401, -0.0286509350929972, -0.001429813206316577, -31.81668840797239,
                        0.02858327133778271, 0.9989768060104955, -0.03504764831909967,  -99.62247870764079,
                        0.002432498127190477, 0.03499235589904547, 0.9993846196442566, -2.546049086854508,
                        0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00;
    endeffector2base << rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], pose_x[0],
                        rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], pose_x[1],
                        rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], pose_x[2],
                        0, 0, 0, 1;            
    
    Eigen::Matrix4d camera2base = endeffector2base * camera2endeffector;
    camera2base.block<3, 1>(0, 3) = camera2base.block<3, 1>(0, 3) / 1000.0;
    // std::cout << "c2e" << std::endl
    //           << camera2endeffector << std::endl;
    // std::cout << "e2b" << std::endl
    //           << endeffector2base << std::endl;
    // std::cout << "c2b" << std::endl
    //           << camera2base << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix<float, 4, 1> centroid_point_re;
    pcl::compute3DCentroid(*cloud_base, centroid_point_re);

    pcl::transformPointCloud(abnormal_depth, *cloud_base, camera2base);
        // std::cout << "!!!!!!!!!cl_b: " << *cloud_base << std::endl;
    // 
    Eigen::Matrix<float, 4, 1> centroid_point2;
    pcl::compute3DCentroid(*cloud_base, centroid_point2);
    std::cout << "centroid" << std::endl
              << centroid_point2 << std::endl;

    // compute_normal(cloud_base, camera2endeffector, endeffector2base, centroid_point2,
    //           moved_point);


    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::transformPointCloud(cloud1, *cloud_base1, camera2base);
    pcl::transformPointCloud(cloud2, *cloud_base2, camera2base);


    Eigen::Vector4f centroid1;
    pcl::compute3DCentroid(*cloud_base1, centroid1);


    std::cout << "Origin Point: (" << centroid1[0] << ", " << centroid1[1] << ", " << centroid1[2] << ")" << std::endl;
    pcl::PointXYZ moved_point = move_point_towards(*cloud_base1, *cloud_base2, 10.0/1000);
    pcl::PointXYZ moved_point2 = move_point_towards(*cloud_base1, *cloud_base2, 1.0/1000);
    std::cout << "Moved Point: (" << moved_point.x << ", " << moved_point.y << ", " << moved_point.z << ")" << std::endl;

    Eigen::Vector4d origin_point(0.0, 0.0, 0.0, 1.0);  // homogeneous coordinates
    Eigen::Vector4d transformed_origin = camera2base*origin_point;
    
    // 이미 변환된 centroid1의 좌표 (이미 계산되었다고 가정)
    Eigen::Vector4d centroid1_transformed;  // 이 변수는 이미 초기화되었다고 가정합니다.

    // 3. 변환된 점과 centroid1 사이의 거리를 계산
    double distance = (centroid1.cast<double>().head<3>() - transformed_origin.head<3>()).norm();

    // 결과 출력
    // std::cout << "Distance between transformed (0,0,0) and centroid1: " << distance << std::endl;

    if(centroid_point2[0] == 0 && centroid_point2[1] == 0 && centroid_point2[2] == 0) return;

    compute_normal(cloud_base, camera2endeffector, endeffector2base, centroid_point2, centroid1, camera2base, moved_point, moved_point2, direction);

}

pcl::PointXYZ pf_image_pipeline::move_point_towards(const pcl::PointCloud<pcl::PointXYZ>& cloud1, const pcl::PointCloud<pcl::PointXYZ>& cloud2, double distance) {
    // Assuming you get the centroid of cloud1 and cloud2 using some method like compute3DCentroid
    Eigen::Vector4f centroid1;
    pcl::compute3DCentroid(cloud1, centroid1);

    Eigen::Vector4f centroid2;
    pcl::compute3DCentroid(cloud2, centroid2);

    // Compute the direction from centroid1 to centroid2
    Eigen::Vector4f direction = centroid2 - centroid1;
    direction.normalize();

    // Move the point by the specified distance in the direction
    Eigen::Vector4f movedCentroid = centroid1 - direction * distance;

    // Convert Eigen::Vector4f to pcl::PointXYZ to return
    pcl::PointXYZ movedPoint;
    movedPoint.x = movedCentroid[0];
    movedPoint.y = movedCentroid[1];
    movedPoint.z = movedCentroid[2];

    return movedPoint;
}

void pf_image_pipeline::compute_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base, Eigen::Matrix4d camera2endeffector, Eigen::Matrix4d endeffector2base, Eigen::Matrix<float, 4, 1> centroid_point2,
                  Eigen::Vector4f centroid1, Eigen::Matrix4d camera2base, pcl::PointXYZ moved_point, pcl::PointXYZ origin_point, cv::Point direction) {
    
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100000);
    seg.setDistanceThreshold(0.1);  // Adjust this value as per your data

    // Segment the largest planar component from the point cloud
    seg.setInputCloud(cloud_base);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
    }
    // Extract the plane
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    extract.setInputCloud(cloud_base);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);

    // The normal of the plane is given by the coefficients
    Eigen::Vector3d normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

    // Normalize the vector
    normal.normalize();

    // step 1 start

    // Convert Eigen Vector to OpenCV Vec
    cv::Vec3d normal_cv(normal[0], normal[1], normal[2]);

    // Change the sign of the normal
    normal_cv[0] = -normal_cv[0];
    normal_cv[1] = -normal_cv[1];
    normal_cv[2] = -normal_cv[2];

    std::cout << "normal vector: : " << std::endl
        << normal_cv[0]<< ", "<<normal_cv[1]<< ", "<<normal_cv[2]<< std::endl;

    cv::Vec3d up(0.0, 0.0, 1.0);
    cv::Vec3d axis = normal_cv.cross(up);
    double cosine = normal_cv.dot(up);
    double k = 1.0 / (1.0 + cosine);

    std::cout << "cosine: : " << std::endl
        << cosine<< std::endl;    
    cv::Matx33d rotation;  // Use Matx33d

    rotation(0, 0) = axis(0) * axis(0) * k + cosine; // cos(t) + (1- cos(t) * nx ^2)
    rotation(1, 0) = axis(1) * axis(0) * k - axis(2);
    rotation(2, 0) = axis(2) * axis(0) * k + axis(1);
    rotation(0, 1) = axis(0) * axis(1) * k + axis(2);
    rotation(1, 1) = axis(1) * axis(1) * k + cosine;
    rotation(2, 1) = axis(2) * axis(1) * k - axis(0);
    rotation(0, 2) = axis(0) * axis(2) * k - axis(1);
    rotation(1, 2) = axis(1) * axis(2) * k + axis(0);
    rotation(2, 2) = axis(2) * axis(2) * k + cosine;


    double normDirection = cv::norm(direction);

    // std::cout << "normDirection = " << normDirection  << std::endl;
    // std::cout << "Direction: x = " << direction.x << "y =" << direction.y << std::endl;
  
    double normalizedDirection[2];

    if(normDirection > 0.00001)  // 0으로 나누는 것을 방지
    {
        normalizedDirection[0] = direction.x / normDirection;
        normalizedDirection[1] = direction.y / normDirection;
    }
    else
    {
        normalizedDirection[0] = direction.x;
        normalizedDirection[1] = direction.y;
    }

    std::cout << "normalizedDirection: x = " << normalizedDirection[0] << "y =" << normalizedDirection[1] << std::endl;
    // cv::Point vect{0,1};
    double vect[2] = {0,1};
    // cv::Point direction{2,3};

    // 내적 계산
    double dotProduct = vect[0] * normalizedDirection[0] + vect[1] * normalizedDirection[1];

    // 두 벡터의 크기 계산
    // double normVect = cv::norm(vect);
    // double normNormalizedDirection = cv::norm(normalizedDirection);  // 이미 정규화 되어있기 때문에 항상 1일 것이다.

    // cos(theta) 계산
    double cosAngle = dotProduct;
    
    // 각도를 계산 (라디안에서 도로 변환)
    double angleRadian = std::acos(cosAngle);
    double angleDegree = angleRadian * (180.0 / CV_PI); // OpenCV에서는 CV_PI를 사용하여 π를 얻을 수 있다.

    // std::cout << "Dot Product: " << dotProduct << std::endl;
    // std::cout << "Cosine of angle: " << cosTheta << std::endl;
    // std::cout << "Angle in radians: " << thetaRadian << std::endl;
    // std::cout << "Angle in degrees: " << thetaDegree << std::endl;

    // double deltadegree = 180.0 - angleDegree;

    // float deltadegreeFloat = static_cast<float>(deltadegree);


    cv::Point3d targetPosition(centroid_point2[0] * 1000.0, centroid_point2[1] *1000.0, centroid_point2[2]*1000.0); 
    cv::Point3d endEffectorPosition = cv::Point3d(0.0, 0.0, 300.0);     //mm 10/05 200 -> 300으로 변경
    cv::Matx31d rotatedVector = rotation * cv::Matx31d(0.0, 0.0, -300.0); // 10/05 200 -> 300으로 변경
    cv::Point3d finalPosition(rotatedVector(0, 0) , rotatedVector(1, 0) , rotatedVector(2, 0) );

    cv::Matx44d T_cent(
      static_cast<double>(rotation(0, 0)), static_cast<double>(rotation(0, 1)), static_cast<double>(rotation(0, 2)), finalPosition.x + targetPosition.x,
      static_cast<double>(rotation(1, 0)), static_cast<double>(rotation(1, 1)), static_cast<double>(rotation(1, 2)), finalPosition.y + targetPosition.y,
      static_cast<double>(rotation(2, 0)), static_cast<double>(rotation(2, 1)), static_cast<double>(rotation(2, 2)), finalPosition.z + targetPosition.z,
      0.0, 0.0, 0.0, 1.0
    );

    // cv::Matx44d T_aruco2(
    //   static_cast<double>(rotation(0, 0)), static_cast<double>(rotation(0, 1)), static_cast<double>(rotation(0, 2)), finalPosition.x + centroid1(0)*1000,
    //   static_cast<double>(rotation(1, 0)), static_cast<double>(rotation(1, 1)), static_cast<double>(rotation(1, 2)), finalPosition.y + centroid1(1)*1000,
    //   static_cast<double>(rotation(2, 0)), static_cast<double>(rotation(2, 1)), static_cast<double>(rotation(2, 2)), finalPosition.z + centroid1(2)*1000,
    //   0.0, 0.0, 0.0, 1.0
    // );
    // cv::Matx44d T_leaf(
    //     static_cast<double>(rotation(0, 0)), static_cast<double>(rotation(0, 1)), static_cast<double>(rotation(0, 2)), finalPosition.x + moved_point.x*1000,
    //     static_cast<double>(rotation(1, 0)), static_cast<double>(rotation(1, 1)), static_cast<double>(rotation(1, 2)), finalPosition.y + moved_point.y*1000,
    //     static_cast<double>(rotation(2, 0)), static_cast<double>(rotation(2, 1)), static_cast<double>(rotation(2, 2)), finalPosition.z + moved_point.z*1000,
    //     0.0, 0.0, 0.0, 1.0
    // );

    float x1, y1, z1, r1, p1, w1;

    Eigen::Matrix4d eigenInverse = camera2endeffector.inverse();
    cv::Matx44d T_end2camera;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T_end2camera(i, j) = eigenInverse(i, j);

    // end effector의 위치 추출
    cv::Matx44d T_end2base = T_cent * T_end2camera;
    // cv::Matx44d T_end2base = T_leaf * T_end2camera;
    x1 = T_end2base(0, 3);
    y1 = T_end2base(1, 3);
    z1 = T_end2base(2, 3);// + 250;

    // end effector의 방향 추출
    cv::Matx33d R_end2base;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_end2base(i, j) = T_end2base(i, j);
        }
    }

    // 초기화
    double r_rad = 0;
    // float p_rad = 180;
    double yw_rad = 0;  

    double p_rad = std::acos(R_end2base(2, 2));


    if (R_end2base(1, 2) / std::sin(p_rad) > 0) {
        r_rad = std::acos(R_end2base(0, 2) / std::sin(p_rad));
    } else {
        r_rad = -std::acos(R_end2base(0, 2) / std::sin(p_rad));
    }
    if (R_end2base(2, 1) / std::sin(p_rad) > 0) {
        yw_rad = std::acos(-R_end2base(2, 0) / std::sin(p_rad));
    } else {
        yw_rad = -std::acos(-R_end2base(2, 0) / std::sin(p_rad));
    }

    r1 = r_rad * 180.0 / M_PI;
    p1 = p_rad * 180.0 / M_PI;
    w1 = yw_rad * 180.0 / M_PI;

    calculated_cam_coord[0] = x1; calculated_cam_coord[1] = y1; calculated_cam_coord[2] = z1;
    calculated_cam_coord[3] = r1; calculated_cam_coord[4] = p1; calculated_cam_coord[5] = w1;

    std::cout << "task1: x,y,z: " << std::endl
              << x1<< ", "<<y1<< ", "<<z1 << std::endl;
    std::cout << "task1: r,p,y: " << std::endl
              << r1<< ", "<<p1<< ", "<<w1 << std::endl;

    // step 1 end


    // step 2 
    // 1st calculate z - y - z' rotation form rotation matrix 
    // 2nd add delta theta and make rotation matrix again

    float x2, y2, z2, r2, p2, w2;
    // cv::Matx44d camera2baseCV;
    // for (int i = 0; i < 4; ++i)
    //     for (int j = 0; j < 4; ++j)
    //         camera2baseCV(i, j) = camera2base(i, j);


    // cv::Matx33d rotationC2B;
    // for (int i = 0; i < 3; ++i)
    //     for (int j = 0; j < 3; ++j)
    //         rotationC2B(i, j) = camera2baseCV(i, j);

    cv::Matx33d rotationC2B;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            rotationC2B(i, j) = camera2base(i, j);

    double zRot = 0;
    double zRot2 = 0;  
    double yRot = std::acos(rotationC2B(2, 2));


    if(std::sin(yRot) == 0.0)
    {
        zRot = 0.0; zRot2 = std::acos(rotationC2B(0,0)) - zRot;
    }
    else{
        if (rotationC2B(1, 2) / std::sin(yRot) > 0) {
            zRot = std::acos(rotationC2B(0, 2) / std::sin(yRot));
        } else {
            zRot = -std::acos(rotationC2B(0, 2) / std::sin(yRot));
        }
        if (rotationC2B(2, 1) / std::sin(yRot) > 0) {
            zRot2 = std::acos(-rotationC2B(2, 0) / std::sin(yRot));
        } else {
            zRot2 = -std::acos(-rotationC2B(2, 0) / std::sin(yRot));
        }
    }
    if (normalizedDirection[0] < 0) zRot2 = zRot2 - (M_PI - angleRadian);
    else zRot2 = zRot2 + (M_PI - angleRadian);

    // 회전 매트릭스 분해 끝

    double cosTheta = std::cos(yRot);
    double sinTheta = std::sin(yRot);
    double cosPsi = std::cos(zRot);
    double sinPsi = std::sin(zRot);
    double cosPhi = std::cos(zRot2);
    double sinPhi = std::sin(zRot2);

    rotationC2B(0, 0) = cosPsi * cosTheta * cosPhi - sinPsi * sinPhi;
    rotationC2B(0, 1) = -cosPsi * cosTheta * sinPhi - sinPsi * cosPhi;
    rotationC2B(0, 2) = cosPsi * sinTheta;
    rotationC2B(1, 0) = sinPsi * cosTheta * cosPhi + cosPsi * sinPhi;
    rotationC2B(1, 1) = -sinPsi * cosTheta * sinPhi + cosPsi * cosPhi;
    rotationC2B(1, 2) = sinPsi * sinTheta;
    rotationC2B(2, 0) = -sinTheta * cosPhi;
    rotationC2B(2, 1) = sinTheta * sinPhi;
    rotationC2B(2, 2) = cosTheta;

    // 회전 매트릭스 재조립 끝

    cv::Matx31d rotatedVector2 = rotationC2B * cv::Matx31d(0.0, 0.0, -300.0); // 10/05 200 -> 300으로 변경
    cv::Point3d finalPosition2(rotatedVector2(0, 0) , rotatedVector2(1, 0) , rotatedVector2(2, 0) );

    cv::Matx44d T_cent2(
      static_cast<double>(rotationC2B(0, 0)), static_cast<double>(rotationC2B(0, 1)), static_cast<double>(rotationC2B(0, 2)), finalPosition2.x + targetPosition.x,
      static_cast<double>(rotationC2B(1, 0)), static_cast<double>(rotationC2B(1, 1)), static_cast<double>(rotationC2B(1, 2)), finalPosition2.y + targetPosition.y,
      static_cast<double>(rotationC2B(2, 0)), static_cast<double>(rotationC2B(2, 1)), static_cast<double>(rotationC2B(2, 2)), finalPosition2.z + targetPosition.z,
      0.0, 0.0, 0.0, 1.0
    );

    //
    cv::Matx44d T_end2base2 = T_cent2 * T_end2camera;
    // cv::Matx44d T_end2base2 = T_leaf * T_end2camera;
    x2 = T_end2base2(0, 3);
    y2 = T_end2base2(1, 3);
    z2 = T_end2base2(2, 3);// + 250;

    // end effector의 방향 추출
    cv::Matx33d R_end2base2;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_end2base2(i, j) = T_end2base2(i, j);
        }
    }

    // 초기화
    double r_rad2 = 0;
    // float p_rad = 180;
    double yw_rad2 = 0;  

    double p_rad2 = std::acos(R_end2base2(2, 2));


    if (R_end2base2(1, 2) / std::sin(p_rad2) > 0) {
        r_rad2 = std::acos(R_end2base2(0, 2) / std::sin(p_rad2));
    } else {
        r_rad2 = -std::acos(R_end2base2(0, 2) / std::sin(p_rad2));
    }
    if (R_end2base2(2, 1) / std::sin(p_rad2) > 0) {
        yw_rad2 = std::acos(-R_end2base2(2, 0) / std::sin(p_rad2));
    } else {
        yw_rad2 = -std::acos(-R_end2base2(2, 0) / std::sin(p_rad2));
    }

    r2 = r_rad2 * 180.0 / M_PI;
    p2 = p_rad2 * 180.0 / M_PI;
    w2 = yw_rad2 * 180.0 / M_PI;

    calculated_cam_coord2[0] = x2; calculated_cam_coord2[1] = y2; calculated_cam_coord2[2] = z2;
    calculated_cam_coord2[3] = r2; calculated_cam_coord2[4] = p2; calculated_cam_coord2[5] = w2;

    std::cout << "task2: x,y,z: " << std::endl
              << x2<< ", "<<y2<< ", "<<z2 << std::endl;
    std::cout << "task2: r,p,y: " << std::endl
              << r2<< ", "<<p2<< ", "<<w2 << std::endl;

    // step2 end

    calculated_origin_coord2[0] = origin_point.x*1000; calculated_origin_coord2[1] = origin_point.y*1000; calculated_origin_coord2[2] = origin_point.z*1000; 

    calculated_origin_coord[0] = centroid1(0)*1000; calculated_origin_coord[1] = centroid1(1)*1000; calculated_origin_coord[2] = centroid1(2)*1000;



    // step3
    float x3, y3, z3, r3, p3, w3;

    cv::Matx44d T_leaf(
        static_cast<double>(rotationC2B(0, 0)), static_cast<double>(rotationC2B(0, 1)), static_cast<double>(rotationC2B(0, 2)), finalPosition2.x + moved_point.x*1000,
        static_cast<double>(rotationC2B(1, 0)), static_cast<double>(rotationC2B(1, 1)), static_cast<double>(rotationC2B(1, 2)), finalPosition2.y + moved_point.y*1000,
        static_cast<double>(rotationC2B(2, 0)), static_cast<double>(rotationC2B(2, 1)), static_cast<double>(rotationC2B(2, 2)), finalPosition2.z + moved_point.z*1000,
        0.0, 0.0, 0.0, 1.0
    );
    // cv::Matx44d c2t(1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
    //                 0.00000000e+00, 1.00000000e+00, 0.00000000e+00, 0.00000000e+00,
    //                 0.00000000e+00, 0.00000000e+00, 1.00000000e+00, -0.10500000e+00,
    //                 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00); 

    
    cv::Matx44d c2t(1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                    0.00000000e+00, 1.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                    0.00000000e+00, 0.00000000e+00, 1.00000000e+00, -0.0000000e+03,            /// 105에서 220으로 변경 // 300 - n = 180
                    0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00); 

    // Convert cv::Matx44d to Eigen::Matrix4d
    Eigen::Matrix4d eigenMatrix;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            eigenMatrix(i,j) = c2t(i,j);
        }
    }

    Eigen::Matrix4d eigenInverse2 = eigenMatrix.inverse();

    cv::Matx44d T_end2tool;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T_end2tool(i, j) = eigenInverse2(i, j);

    // cv::Matx44d T_end2base = camera2base22* T_end2tool;
    cv::Matx44d T_end2base3 = T_leaf * T_end2tool;

    std::cout << "R rotation: " << std::endl
            << camera2base(0,0)<< ", "<<camera2base(0,1)<< ", "<< camera2base(0,2) << std::endl
            << camera2base(1,0)<< ", "<<camera2base(1,1)<< ", "<< camera2base(1,2) << std::endl
            << camera2base(2,0)<< ", "<<camera2base(2,1)<< ", "<< camera2base(2,2) << std::endl;


    x3 = T_end2base3(0, 3);
    y3 = T_end2base3(1, 3);
    z3 = T_end2base3(2, 3);

    // end effector의 방향 추출
    cv::Matx33d R_end2base3;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_end2base3(i, j) = T_end2base3(i, j);
        }
    }
    
    double r_rad3 = 0;
    // float p_rad = 180;
    double yw_rad3 = 0; 
    
    double p_rad3 = std::acos(R_end2base3(2, 2));

    if (R_end2base3(1, 2) / std::sin(p_rad3) > 0) {
        r_rad3 = std::acos(R_end2base3(0, 2) / std::sin(p_rad3));
    } else {
        r_rad3 = -std::acos(R_end2base3(0, 2) / std::sin(p_rad3));
    }
    if (R_end2base3(2, 1) / std::sin(p_rad3) > 0) {
        yw_rad3 = std::acos(-R_end2base3(2, 0) / std::sin(p_rad3));
    } else {
        yw_rad3 = -std::acos(-R_end2base3(2, 0) / std::sin(p_rad3));
    }

    r3 = r_rad3 * 180.0 / M_PI;
    p3= p_rad3 * 180.0 / M_PI;
    w3 = yw_rad3 * 180.0 / M_PI;

    std::cout << "task3: x,y,z: " << std::endl
            << x3<< ", "<<y3<< ", "<<z3 << std::endl;
    std::cout << "task3: r,p,y: " << std::endl
            << r3<< ", "<<p3<< ", "<<w3 << std::endl;    

    //////////////////////////////////////////////////
    float x4, y4, z4, r4, p4, w4;

     cv::Matx44d c2t2(1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                    0.00000000e+00, 1.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                    0.00000000e+00, 0.00000000e+00, 1.00000000e+00, -0.13000000e+03,            /// 105에서 220으로 변경 // 300 - n = 180
                    0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00); 

    // Convert cv::Matx44d to Eigen::Matrix4d
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            eigenMatrix(i,j) = c2t2(i,j);
        }
    }

    eigenInverse2 = eigenMatrix.inverse();

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T_end2tool(i, j) = eigenInverse2(i, j);

    // cv::Matx44d T_end2base = camera2base22* T_end2tool;
    cv::Matx44d T_end2base4 = T_leaf * T_end2tool;


    x4 = T_end2base4(0, 3);
    y4 = T_end2base4(1, 3);
    z4 = T_end2base4(2, 3);

    // end effector의 방향 추출
    cv::Matx33d R_end2base4;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_end2base4(i, j) = T_end2base4(i, j);
        }
    }
    
    double r_rad4 = 0;
    // float p_rad = 180;
    double yw_rad4 = 0; 
    
    double p_rad4 = std::acos(R_end2base4(2, 2));

    if (R_end2base4(1, 2) / std::sin(p_rad4) > 0) {
        r_rad4 = std::acos(R_end2base4(0, 2) / std::sin(p_rad4));
    } else {
        r_rad4 = -std::acos(R_end2base4(0, 2) / std::sin(p_rad4));
    }
    if (R_end2base4(2, 1) / std::sin(p_rad4) > 0) {
        yw_rad4 = std::acos(-R_end2base4(2, 0) / std::sin(p_rad4));
    } else {
        yw_rad4 = -std::acos(-R_end2base4(2, 0) / std::sin(p_rad4));
    }

    r4 = r_rad4 * 180.0 / M_PI;
    p4= p_rad4 * 180.0 / M_PI;
    w4 = yw_rad4 * 180.0 / M_PI;
    ///////// 10.05 추가 내용 /////////////
    // cv::Point vect = (0,1);
    

    ////
    calculated_tool_coord[0] = x3; calculated_tool_coord[1] = y3; calculated_tool_coord[2] = z3;
    // calculated_tool_coord[0] = x * 1000; calculated_tool_coord[1] = y * 1000; calculated_tool_coord[2] = z * 1000;
    calculated_tool_coord[3] = r3; calculated_tool_coord[4] = p3; calculated_tool_coord[5] = w3;// - deltadegreeFloat;

    calculated_tool_coord[6] = x4; calculated_tool_coord[7] = y4; calculated_tool_coord[8] = z4;
    // calculated_tool_coord[0] = x * 1000; calculated_tool_coord[1] = y * 1000; calculated_tool_coord[2] = z * 1000;
    calculated_tool_coord[9] = r4; calculated_tool_coord[10] = p4; calculated_tool_coord[11] = w4;// - deltadegreeFloat;

} 