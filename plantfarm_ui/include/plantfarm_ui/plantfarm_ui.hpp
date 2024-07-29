#ifndef PLANTFARM_UI_H
#define PLANTFARM_UI_H

#include <vector>
#include <cmath>
#include <ctime>
#include <string>
#include <algorithm>
#include <thread>
#include <memory>
#include <chrono>
// QWidget
#include <QWidget>
#include <QFileDialog>
#include <QString>
#include <QTimer>
#include <QPixmap>
#include <QLabel>
#include <QMessageBox>
// ROS Headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "plantfarm_msgs/msg/pos_results.hpp"
#include "dsr_msgs2/srv/move_joint.hpp"
#include "dsr_msgs2/srv/move_line.hpp"
#include "dsr_msgs2/srv/set_ctrl_box_digital_output.hpp"
// OpenCV Headers
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Ui {
class plantfarm_ui;
}

class plantfarm_ui : public QWidget
{
    Q_OBJECT

public:
    explicit plantfarm_ui(QWidget *parent = 0);
    ~plantfarm_ui();

private slots:  
    void spinOnce();
   
    void on_pushButton_process_move_auto_clicked();

private:
    Ui::plantfarm_ui *ui;
    QTimer *rclcpp_timer;

    /* 콜백 함수 선언 */
    void yolo_image_sub_cb(const sensor_msgs::msg::Image::SharedPtr image_raw);
    void result_pos_cb(const plantfarm_msgs::msg::PosResults::SharedPtr  pos_result);

    /* 맴버 변수 선언 */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr YoloResultSub;
    rclcpp::Subscription<plantfarm_msgs::msg::PosResults>::SharedPtr PosSub;

    rclcpp::Client<dsr_msgs2::srv::MoveJoint>::SharedPtr MJClient;
    rclcpp::Client<dsr_msgs2::srv::MoveLine>::SharedPtr MLClient;

    rclcpp::Node::SharedPtr node_;

    /* 기타 함수 선언 */
    void movel(float fTargetPos[6], float fTargetVel[2], float fTargetAcc[2], float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSyncType);
    void movej(float fTargetPos[6], float fTargetVel, float fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveMode, int nBlendingType, int nSyncType);
    void wait(float time_);
    /* 전역 변수 선언 */
    cv::Mat yolo_image;    
    std::vector<std::array<float, 6>> cam_pos;     
    std::vector<std::array<float, 6>> cam_pos_cor;   
    std::vector<std::array<float, 6>> tool_pos_out; 
    std::vector<std::array<float, 6>> tool_pos_in; 
    
};



#endif //PLANTFARM_UI_H