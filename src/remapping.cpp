#include <mbplanner/MBRemapper.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto remapper = std::make_shared<MBP::MBRemapper>();
    rclcpp::spin(remapper);
    rclcpp::shutdown();
    return 0;
}
