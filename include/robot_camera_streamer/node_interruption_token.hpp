#ifndef LMP_PROJECT_NODE_INTERRUPTION_TOKEN_HPP
#define LMP_PROJECT_NODE_INTERRUPTION_TOKEN_HPP

#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/int32.hpp>
#include<functional>

namespace Internal{

    enum class NodeOperationStatus : int{
        NoOperated = 0,
        Suspend    = 1,
        Resumed    = 2
    };

}//namespace Internal

using Status = Internal::NodeOperationStatus;

class NodeInterruptionToken{

    using OperationMessage = std_msgs::msg::Int32;
    using Action = std::function<void()>;

public:
    NodeInterruptionToken(
        rclcpp::Node* host,
        const std::string& topicName
    );

    ~NodeInterruptionToken();

    bool IsSuspensionRequested() const noexcept;

    bool IsResumptionRequested() const noexcept;

    void Register(Action action) noexcept;

    void Register(Action action, Status state) noexcept;

private:
    void OnOperationMessageRecieved(
        OperationMessage::SharedPtr message
    );

    Action callback_;
    Status operationState_, registerState_;
    rclcpp::Subscription<OperationMessage>::SharedPtr operationSubscriber_;
};


#endif //LMP_PROJECT_NODE_INTERRUPTION_TOKEN_HPP