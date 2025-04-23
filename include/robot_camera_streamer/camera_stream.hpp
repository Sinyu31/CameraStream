#ifndef LMP_PROJECT_CAMERA_STREAM_HPP
#define LMP_PROJECT_CAMERA_STREAM_HPP

#include"node_interruption_token.hpp"
#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<rclcpp/qos.hpp>

/**
 * @file camera_stream.hpp
 * @brief Declaration of CameraStreamer class for capturing and publishing camera frames in ROS2.
 * @author yuya
 * @date 2025-04-18
 */


/// @brief frame prop

constexpr int FrameWidth = 320;
constexpr int FrameHeight = 240;
constexpr int Fps = 30;

/// @class CameraStreamer
/// @brief Get camera stream from robot and publish ROS2 image message
///
/// This class captures video from a camera device using OpenCV
/// converts frames into sensor_msgs::msg::Image using cv_bridge,
/// and publishes them to a ROS2 topic.
/// If Suspend Message Required, capture is release
/// If Resume Message Required, capture is open

class CameraStreamer : public rclcpp::Node{
private:
    using ImageMessage = sensor_msgs::msg::Image;

public:
    CameraStreamer(
        const rclcpp::NodeOptions& nodeOptions = rclcpp::NodeOptions()
    );

    CameraStreamer(
        const std::string& nameSpace,
        const rclcpp::NodeOptions& nodeOptions = rclcpp::NodeOptions()
    );

    ~CameraStreamer();
        
    [[nodiscard]] static std::shared_ptr<CameraStreamer>  Create() noexcept;
    
    [[nodiscard]]  static std::shared_ptr<CameraStreamer>  Create(
        const rclcpp::NodeOptions& nodeOptions
    ) noexcept;

    [[nodiscard]] static std::shared_ptr<CameraStreamer>  Create(
        const std::string& nameSpace,
        const rclcpp::NodeOptions& nodeOptions
    ) noexcept;

private:
    void SetCaptureProp(int width, int height, int fps) noexcept;

    void InitializeComponents() noexcept;

    void OnUpdate();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ImageMessage>::SharedPtr imagePublisher_;

    cv::VideoCapture cap_;
    ImageMessage::SharedPtr cameraImage_; 
    cv_bridge::CvImage cvImage_;

    NodeInterruptionToken token_;
};

#endif //LMP_PROJECT_CAMERA_STREAM_HPP 

