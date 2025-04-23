#include"robot_camera_streamer/node_interruption_token.hpp"

NodeInterruptionToken::NodeInterruptionToken(
    rclcpp::Node* host,
    const std::string& topicName
)
: operationState_(Status::NoOperated), registerState_(Status::NoOperated){
    operationSubscriber_ = host->create_subscription<OperationMessage>(
        topicName,
        rclcpp::SystemDefaultsQoS(),
        std::bind(
            &NodeInterruptionToken::OnOperationMessageRecieved,
            this,
            std::placeholders::_1
        )
    );
}

NodeInterruptionToken::~NodeInterruptionToken() = default;


bool NodeInterruptionToken::IsSuspensionRequested() const noexcept{
    return operationState_ == Status::Suspend;
}

bool NodeInterruptionToken::IsResumptionRequested() const noexcept{
    return operationState_ == Status::Resumed;
}

void NodeInterruptionToken::Register(Action action) noexcept{
    callback_ = action;
}

void NodeInterruptionToken::Register(Action action, Status state) noexcept{
    callback_ = action;
    registerState_ = state;
}


void NodeInterruptionToken::OnOperationMessageRecieved(
    OperationMessage::SharedPtr message
){
    Status operationCode = static_cast<Status>(message->data);

    switch (operationCode){
    case Status::Suspend:
        operationState_ = Status::Suspend;
        break;

    case Status::Resumed:
        operationState_ = Status::Resumed;
        break;

    default:
        break;
    }

    if(callback_){
        if(registerState_ == Status::NoOperated || registerState_ == operationCode)
            callback_();
    }

}
