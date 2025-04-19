#include"robot_camera_streamer/camera_stream.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto cameraStream = CameraStreamer::Create();
    rclcpp::spin(cameraStream);
    rclcpp::shutdown();
    return 0;
}

