#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "std_srvs/srv/trigger.hpp"

// zjistí sklad
// Tento node je Behavior Tree uzel, který volá ROS2 službu /get_dropoff_location.
// Úkolem je získat ID skladu (např. "C1") a předat ho dál do stromu přes output port.

using Trigger = std_srvs::srv::Trigger;

class GetDropoffService : public BT::RosServiceNode<Trigger> {
public:
    GetDropoffService(const std::string& name, const BT::NodeConfig& conf,
                      const BT::RosNodeParams& params)
        : RosServiceNode<Trigger>(name, conf, params) {}

// Tento node vrací hodnotu storage_id ven do stromu
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::OutputPort<std::string>("storage_id", "Assigned storage location (A1-D2)")
        });
    }
    
// Pošli prázdný request na ROS service
    bool setRequest(Request::SharedPtr& request) override
    {
        // Trigger nemá žádné pole v requestu
        (void)request;
        return true;
    }

// Dostal jsem odpověď → pokud je validní, ulož sklad a pokračuj dál
    BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override
{
    if (!response->success) {
        RCLCPP_ERROR(logger(), "GetDropoffService: service failed");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(logger(), "GetDropoffService: storage_id = %s",
                response->message.c_str());

    setOutput("storage_id", response->message);
    return BT::NodeStatus::SUCCESS;
}
// Služba se nepodařila vůbec zavolat → selhání uzlu
    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "GetDropoffService failed: %d", static_cast<int>(error));
        return BT::NodeStatus::FAILURE;
    }
};

CreateRosNodePlugin(GetDropoffService, "GetDropoffService");
