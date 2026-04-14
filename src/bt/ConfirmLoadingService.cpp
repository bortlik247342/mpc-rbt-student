#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "std_srvs/srv/trigger.hpp"

// Tento soubor implementuje BT krok, který blokuje Behavior Tree, dokud warehouse manager nepotvrdí dokončení nakládky pomocí ROS2 Trigger služby.

using Trigger = std_srvs::srv::Trigger;

class ConfirmLoadingService : public BT::RosServiceNode<Trigger> {
public:
    ConfirmLoadingService(const std::string& name, const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params)
        : RosServiceNode<Trigger>(name, conf, params) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({});
    }

    bool setRequest(Request::SharedPtr& request) override
    {
        // Trigger nemá žádné pole v requestu
        (void)request;
        return true;
    }

    BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override
{
    if (!response->success) {
        RCLCPP_WARN(logger(), "ConfirmLoadingService: failed");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(logger(), "Loading confirmed");
    return BT::NodeStatus::SUCCESS;
}

    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "ConfirmLoadingService failed: %d", static_cast<int>(error));
        return BT::NodeStatus::FAILURE;
    }
};

CreateRosNodePlugin(ConfirmLoadingService, "ConfirmLoadingService");
