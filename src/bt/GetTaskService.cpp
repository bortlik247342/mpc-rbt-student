#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "std_srvs/srv/trigger.hpp"

// získá manipulátor
// Tento Behavior Tree uzel volá ROS2 službu /get_pickup_task, která přidělí robotovi konkrétního manipulátora (1–3) a vrátí jeho ID do stromu.

using Trigger = std_srvs::srv::Trigger;

class GetTaskService : public BT::RosServiceNode<Trigger> {
public:
    GetTaskService(const std::string& name, const BT::NodeConfig& conf,
                   const BT::RosNodeParams& params)
        : RosServiceNode<Trigger>(name, conf, params) {}

// Tento node vrátí ID manipulátoru do Behavior Tree
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::OutputPort<std::string>("manipulator_id", "Assigned manipulator ID (1-3)")
        });
    }

// Pošli prázdný požadavek na službu
    bool setRequest(Request::SharedPtr& request) override
    {
        // Trigger nemá žádné pole v requestu
        (void)request;
        return true;
    }

// Dostal jsem ID manipulátoru → ulož ho a pokračuj v BT
    BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override
{
    if (!response->success) {
        RCLCPP_WARN(logger(), "GetTaskService: request failed");
        return BT::NodeStatus::FAILURE;
    }

    setOutput("manipulator_id", response->message);

    RCLCPP_INFO(logger(), "Assigned manipulator: %s", response->message.c_str());

    return BT::NodeStatus::SUCCESS;
}

// Služba neodpověděla správně → uzel selhal
    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "GetTaskService failed: %d", static_cast<int>(error));
        return BT::NodeStatus::FAILURE;
    }
};

CreateRosNodePlugin(GetTaskService, "GetTaskService");
