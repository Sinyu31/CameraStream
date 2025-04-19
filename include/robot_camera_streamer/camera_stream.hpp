#ifndef LMP_PROJECT_CAMERA_STREAM_HPP
#define LMP_PROJECT_CAMERA_STREAM_HPP

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


/// @class CameraStreamer
/// @brief Get camera stream from robot and publish ROS2 image message
///
/// This class captures video from a camera device using OpenCV
/// converts frames into sensor_msgs::msg::Image using cv_bridge,
/// and publishes them to a ROS2 topic.

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
    
    /// @brief Create a shared CameraStreamer instance with default options.
    /// @return Shared pointer to CameraStreamer instance
    
    static std::shared_ptr<CameraStreamer>  Create() noexcept;

    /// @brief Create a shared CameraStreamer instance with custom node options.
    /// @param nodeOptions NodeOptions to configure the node.
    /// @return Shared pointer to CameraStreamer instance
    
    static std::shared_ptr<CameraStreamer>  Create(
        const rclcpp::NodeOptions& nodeOptions
    ) noexcept;

    /// @brief Create a shared CameraStreamer instance with namespace and custom options.
    /// @param nameSpace   namespace for the  node
    /// @param nodeOptions NodeOptions to configure the node
    /// @return Shared pointer to cameraStreamer instance
    
    
    static std::shared_ptr<CameraStreamer>  Create(
        const std::string& nameSpace,
        const rclcpp::NodeOptions& nodeOptions
    ) noexcept;

private:

    /// @brief Initialize camera and ROS publisher components.
    /// 
    /// Opens the video capture, sets camera parameters, and prepares the image publisher and timer.
    /// If Camera is not opened, Node is shutdwon
    
    inline void InitializeComponents() noexcept;

    /// @brief Callback Function

    void OnUpdate();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ImageMessage>::SharedPtr imagePublisher_;

    cv::VideoCapture cap_;
    ImageMessage::SharedPtr cameraImage_; 
    cv_bridge::CvImage cvImage_;

};

#endif //LMP_PROJECT_CAMERA_STREAM_HPP 

