#include "rclcpp/rclcpp.hpp"
#include "plantfarm_ui.hpp"
#include "ui_plantfarm_ui.h"



plantfarm_ui::plantfarm_ui(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::plantfarm_ui)
{
    ui->setupUi(this);

    // ROS 2
    int argc = 0; char **argv = NULL;
    rclcpp::init(argc, argv);
    
    node_ = rclcpp::Node::make_shared("plantfarm_ui");
    rclcpp_timer = new QTimer(this);
    connect(rclcpp_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
    rclcpp_timer->start(1);  // set the rate to 100ms  You can change this if you want to increase/decrease update rate

    YoloResultSub = node_->create_subscription<sensor_msgs::msg::Image>(
      "yolov7/image0", 10, std::bind(&plantfarm_ui::yolo_image_sub_cb, node_, std::placeholders::_1));
    PosSub = node_->create_subscription<plantfarm_msgs::msg::PosResults>(
      "image_pipeline/result", 10, std::bind(&plantfarm_ui::result_pos_cb, node_, std::placeholders::_1));

    ui->stackedWidget->setCurrentIndex(0);

    MJClient = node_->create_client<dsr_msgs2::srv::MoveJoint>("/dsr01/motion/move_joint");
    MLClient = node_->create_client<dsr_msgs2::srv::MoveLine>("/dsr01/motion/move_line");
}

plantfarm_ui::~plantfarm_ui()
{
    RCLCPP_INFO(node_->get_logger(), "UI node end!!!");
    delete ui;
}

void plantfarm_ui::spinOnce()
{
    if (rclcpp::ok())
    {
        rclcpp::spin_some(node_->get_node_base_interface());
    }
    else
    {
        // ROS 2 시스템이 종료된 경우, QApplication을 종료
        QApplication::quit();
    }
}

void plantfarm_ui::wait(float time_)
{
    double time_lf = static_cast<double>(time_);

    auto start_time = std::chrono::steady_clock::now();

    std::chrono::duration<double> loop_duration(time_lf);

    while(true){        
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - start_time >= loop_duration) {
            break; 
        }
        spinOnce();
    }
}

void plantfarm_ui::movej(float *fTargetPos, float fTargetVel, float fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveMode, int nBlendingType, int nSyncType)
{
    // 서비스가 준비되었는지 확인
    if (!MJClient->wait_for_service(std::chrono::milliseconds(50))) {
        RCLCPP_ERROR(node_->get_logger(), "Doosan robot launcher is not running now!!!");
        return;
    }
    // 요청 생성
    auto requestMJ = std::make_shared<dsr_msgs2::srv::MoveJoint::Request>();    

    for(int i=0; i<6; i++) requestMJ->pos[i] = fTargetPos[i];
    requestMJ->vel = fTargetVel;
    requestMJ->acc = fTargetAcc;
    requestMJ->time = fTargetTime;
    requestMJ->radius = fBlendingRadius;
    requestMJ->mode = nMoveMode;
    requestMJ->blend_type = nBlendingType;
    requestMJ->sync_type = nSyncType;

    // 비동기 요청
    using ServiceResponseFuture = rclcpp::Client<dsr_msgs2::srv::MoveJoint>::SharedFuture;
    auto response_received_mj_callback = [node_](ServiceResponseFuture future) {
        auto requestMJ = future.get();
        if (requestMJ->success) {
            // for(int i = 0; i<6; i++) RCLCPP_INFO(this->get_logger(), "%d : %.2lf",i+1,responsePose->pos[i]);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to service.");
        }
    };

    // 서비스 호출
    auto Pose_future_result = MJClient->async_send_request(requestMJ, response_received_mj_callback);
}

void plantfarm_ui::movel(float *fTargetPos, float *fTargetVel, float *fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSyncType)
{
    ui->textEdit_log->append("Move_line START!");
    if(fTargetPos[2] < 200.0) 
    {
        ui->textEdit_log->append("Unexpected Pose!");
        return;
    }

    // 서비스가 준비되었는지 확인
    if (!MLClient->wait_for_service(std::chrono::milliseconds(50))) {
        RCLCPP_ERROR(node_->get_logger(), "Doosan robot launcher is not running now!!!");
        return;
    }
    // 요청 생성
    auto requestML = std::make_shared<dsr_msgs2::srv::MoveLine::Request>();    

    for(int i=0; i<6; i++) requestML->pos[i] = fTargetPos[i];
    for(int i=0; i<2; i++) requestML->vel[i] = fTargetVel[i];
    for(int i=0; i<2; i++) requestML->acc[i] = fTargetAcc[i];
    requestML->time = fTargetTime;
    requestML->radius = fBlendingRadius;
    requestML->ref = nMoveReference;
    requestML->mode = nMoveMode;
    requestML->blend_type = nBlendingType;
    requestML->sync_type = nSyncType;

    QString text_for_append;

    // 비동기 요청
    using ServiceResponseFuture = rclcpp::Client<dsr_msgs2::srv::MoveLine>::SharedFuture;
    auto response_received_ml_callback = [node_](ServiceResponseFuture future) {
        auto requestML = future.get();
        if (requestML->success) {
            text_for_append.sprintf("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",requestML->pos[0],requestML->pos[1],requestML->pos[2],requestML->pos[3],requestML->pos[4],requestML->pos[5]);
            ui->textEdit_log->append(text_for_append);
            // for(int i = 0; i<6; i++) RCLCPP_INFO(this->get_logger(), "%d : %.2lf",i+1,responsePose->pos[i]);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to service.");
        }
    };

    // 서비스 호출
    auto Pose_future_result = MLClient->async_send_request(requestML, response_received_ml_callback);
}

void plantfarm_ui::yolo_image_sub_cb(const sensor_msgs::msg::Image::SharedPtr image_raw)
{
    cv_ptr = cv_bridge::toCvCopy(image_raw, image_raw->encoding);
    yolo_image = cv_ptr->image;
}

void plantfarm_ui::result_pos_cb(const plantfarm_msgs::msg::PosResults::SharedPtr  pos_result)
{
    size_t idx = msg->idx;

    std::array<float, 6> cam_pos_arr;
    std::array<float, 6> cam_pos_cor_arr;
    std::array<float, 6> tool_pos_out_arr;
    std::array<float, 6> tool_pos_in_arr;

    std::copy(std::begin(msg->cam_pos), std::end(msg->cam_pos), std::begin(cam_pos_arr));
    std::copy(std::begin(msg->cam_pos_cor), std::end(msg->cam_pos_cor), std::begin(cam_pos_cor_arr));
    std::copy(std::begin(msg->tool_pos_out), std::end(msg->tool_pos_out), std::begin(tool_pos_out_arr));
    std::copy(std::begin(msg->tool_pos_in), std::end(msg->tool_pos_in), std::begin(tool_pos_in_arr));

    cam_pos.push_back(cam_pos_arr);
    cam_pos_cor.push_back(cam_pos_cor_arr);
    tool_pos_out.push_back(tool_pos_out_arr);
    tool_pos_in.push_back(tool_pos_in_arr);

    RCLCPP_INFO(node_->get_logger(), "cam_pos = [%f, %f, %f, %f, %f, %f]",
                cam_pos[idx][0], cam_pos[idx][1], cam_pos[idx][2], cam_pos[idx][3], cam_pos[idx][4], cam_pos[idx][5]);

}

void plantfarm_ui::on_pushButton_process_move_auto_clicked()
{
    
    float joint_home[6] = {90.0, 0.0, 90.0, 0.0, 90.0, 0.0};
    float pos_home[6] = {650, 440, 665, 0.0, 180.0, 0.0};
    float velx[2] = {0,0};
    float accx[2] = {0,0};
    std::array<float, 6> target_coord; 

    QString text_for_append;
    
    text_for_append.sprintf("[INFO] Move to home position!!!");
    ui->textEdit_process_log->append(text_for_append);
    movej(joint_home,0,0,4.5,0,0,0,0); 
    wait(4.7);

    movel(pos_home,velx,accx,4.5,0,0,0,0,0);
    wait(4.7);

    cv::Mat showimage_bgr = yolo_image.clone();
    cv::Mat showimage;
    cv::cvtColor(showimage_bgr,showimage, cv::COLOR_BGR2RGB);
    cv::resize(showimage, showimage, cv::Size(640, 480));
    ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

    /////////////////////////////////////////
    for(int i=0; i< cam_pos.size(); i++) target_coord[i] = cam_pos[i];
    float target[6] = {target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]};

    QString text_for_append;
    text_for_append.sprintf("[INFO] X : %.5f, Y : %.5f, Z : %.5f \n      Z' : %.5f, Y' : %.5f, Z'' : %.5f", target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]);
    ui->textEdit_process_log->append(text_for_append);

    text_for_append.sprintf("[INFO] 소시야로 이동합니다!!!");
    ui->textEdit_process_log->append(text_for_append);

    movel(target,velx,accx,4.5,0,0,0,0,0);
    wait(4.7);

    showimage_bgr = yolo_image.clone();
    cv::cvtColor(showimage_bgr,showimage, cv::COLOR_BGR2RGB);
    cv::resize(showimage, showimage, cv::Size(640, 480));
    ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

    /////////////////////////////////////////

    for(int i=0; i< cam_pos_cor.size(); i++) target_coord[i] = cam_pos_cor[i];
    float target2[6] = {target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]};

    text_for_append.sprintf("[INFO] X : %.5f, Y : %.5f, Z : %.5f \n      Z' : %.5f, Y' : %.5f, Z'' : %.5f", target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]);
    ui->textEdit_process_log->append(text_for_append);

    text_for_append.sprintf("[INFO] 보정 소시야로 이동합니다!!!");
    ui->textEdit_process_log->append(text_for_append);

    movel(target2,velx,accx,4.5,0,0,0,0,0);
    wait(4.7);

    showimage_bgr = yolo_image.clone();
    cv::cvtColor(showimage_bgr,showimage, cv::COLOR_BGR2RGB);
    cv::resize(showimage, showimage, cv::Size(640, 480));
    ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

    for(int i=0; i< tool_pos_out.size(); i++) target_coord[i] = tool_pos_out[i];
    float target3[6] = {target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]};
    movel(target3,velx,accx,4.5,0,0,0,0,0);
    wait(4.7);

    text_for_append.sprintf("[INFO] 최종 위치로 이동합니다!!!");
    ui->textEdit_process_log->append(text_for_append);

    for(int i=0; i< tool_pos_in.size(); i++) target_coord[i] = tool_pos_in[i];
    float target4[6] = {target_coord[0], target_coord[1], target_coord[2], target_coord[3], target_coord[4], target_coord[5]};
    movel(target4,velx,accx,4.5,0,0,0,0,0);
    wait(4.7);

}