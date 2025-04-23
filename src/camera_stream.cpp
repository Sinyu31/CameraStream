#include"robot_camera_streamer/camera_stream.hpp"

CameraStreamer::CameraStreamer(
    const rclcpp::NodeOptions& nodeOptions
) 
: rclcpp::Node("camera_streamer", nodeOptions), token_(this, "/image/interruption"){

    InitializeComponents();
}

CameraStreamer::CameraStreamer(
    const std::string& nameSpace, 
    const rclcpp::NodeOptions& nodeOptions
)
: rclcpp::Node("camera_streamer", nameSpace, nodeOptions), token_(this, "/image/interruption"){

    InitializeComponents();
}

CameraStreamer::~CameraStreamer() = default;

std::shared_ptr<CameraStreamer> CameraStreamer::Create() noexcept{
    return std::make_shared<CameraStreamer>();
}

std::shared_ptr<CameraStreamer> CameraStreamer::Create(
    const rclcpp::NodeOptions& nodeOptions
) noexcept{
    return std::make_shared<CameraStreamer>(nodeOptions);
}

std::shared_ptr<CameraStreamer> CameraStreamer::Create(
    const std::string& nameSpace,
    const rclcpp::NodeOptions& nodeOptions
) noexcept{
    return std::make_shared<CameraStreamer>(nameSpace, nodeOptions);
}

void CameraStreamer::InitializeComponents() noexcept{    
    cap_.open(0);
    
    if(cap_.isOpened()){
        SetCaptureProp(FrameWidth, FrameHeight, Fps);
    }
    else{
        RCLCPP_ERROR(
            this -> get_logger(), "Failed to open the camera. shutdown this node" 
        );
        rclcpp::shutdown();
        return;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(30));  //sleep
    
    imagePublisher_ = this->create_publisher<ImageMessage>(
        "/image_raw",                         //topic name
        rclcpp::SensorDataQoS()               //qos
    );
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000/30),           //30fps   
        std::bind(&CameraStreamer::OnUpdate, this)    //callback
    );

    token_.Register([this](){
        if(token_.IsSuspensionRequested() && cap_.isOpened()){
            
            // publish black image before cap release
            auto black = cv::Mat::zeros(FrameHeight, FrameWidth, CV_8UC3);
            cvImage_.image = black;
            cvImage_.header.stamp = this->now();
            cameraImage_ = cvImage_.toImageMsg();
            imagePublisher_->publish(std::move(*cameraImage_));
            cap_.release();
        }
        if(token_.IsResumptionRequested() && !cap_.isOpened()){
            
            //cap reopen and set prop, this code is may be overhead
            cap_.open(0);
            SetCaptureProp(FrameWidth, FrameHeight, Fps);
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }   
    });

}

void CameraStreamer::SetCaptureProp(int width, int height, int fps) noexcept{
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap_.set(cv::CAP_PROP_FPS, fps);
    cvImage_.encoding = "bgr8";
}


void CameraStreamer::OnUpdate(){
    if(token_.IsSuspensionRequested()) return;

    cap_ >> cvImage_.image;
    if(cvImage_.image.empty()){
        RCLCPP_WARN(this->get_logger(), "Empty frame captured");
        return;
    }
    
    try{
        cvImage_.header.stamp = this->now();
        cameraImage_ = cvImage_.toImageMsg();
        imagePublisher_->publish(std::move(*cameraImage_));
    }
    catch(const cv_bridge::Exception& ex){
        RCLCPP_ERROR(
            this->get_logger(), ex.what()
        );
    }
    catch(const std::exception& ex){
        RCLCPP_ERROR(
            this->get_logger(), ex.what()
        );
    }
}
