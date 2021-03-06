#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/int32.hpp"

#include "high_freq_pub/visibility_control.h"

#include <cmath>


using namespace std::chrono_literals;

namespace high_freq_pub
{

class Listener : public rclcpp::Node
{
public:
    HIGH_FREQ_PUB_PUBLIC
    explicit Listener(const rclcpp::NodeOptions& options)
        : Node("listener_high_freq", options),
          m_messageID(0),
          m_numReceivedMsgs(0)
    {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        auto callback =
            [this](const std_msgs::msg::Int32::SharedPtr msg) -> void
            {
                ++m_numReceivedMsgs;
                int32_t msgDiff = msg->data - m_messageID;
                m_messageID = msg->data;
                if (msgDiff >= 2)
                {
                    //RCLCPP_ERROR(this->get_logger(), "Lost %d msgs, curr: %d", msgDiff - 1, msg->data);
                }
                else
                {
                    //RCLCPP_INFO(this->get_logger(), "Received msg %d", msg->data);
                }
            };

        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable();
        sub_ = create_subscription<std_msgs::msg::Int32>("chatter", qos, callback);
        RCLCPP_INFO(this->get_logger(), "Ready to receive messages");
    }

    ~Listener()
    {
        RCLCPP_INFO(this->get_logger(), "Received %d messages", m_numReceivedMsgs);
        RCLCPP_INFO(this->get_logger(), "Lost %d messages", 100000 - m_numReceivedMsgs);
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    int32_t m_messageID;
    int32_t m_numReceivedMsgs;
};

}  // namespace high_freq_pub

RCLCPP_COMPONENTS_REGISTER_NODE(high_freq_pub::Listener)
