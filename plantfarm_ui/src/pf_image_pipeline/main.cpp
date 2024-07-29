#include "rclcpp/rclcpp.hpp"
#include "pf_image_pipeline/pf_image_pipeline.hpp"

int main(int argc, char * argv[])
{
    const rclcpp::NodeOptions options;
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<pf_image_pipeline>());

    rclcpp::shutdown();

    return 0;
}