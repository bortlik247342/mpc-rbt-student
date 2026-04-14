#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

// posílá cíl do navigace (/go_to_goal)

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class NavigateToPoseAction : public BT::RosActionNode<NavigateToPose> {
public:
    NavigateToPoseAction(const std::string& name, const BT::NodeConfig& conf,
                         const BT::RosNodeParams& params)
        : RosActionNode<NavigateToPose>(name, conf, params) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<double>("x", "Target position X"),
            BT::InputPort<double>("y", "Target position Y")
        });
    }



// vezme {x}, {y} z blackboardu a vytvoří goal
    bool setGoal(Goal& goal) override
{
    auto node = node_.lock();

    double x, y;
    if (!getInput<double>("x", x) || !getInput<double>("y", y)) {
        RCLCPP_ERROR(node->get_logger(), "Missing input ports x or y");
        return false;
    }

    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node->now();

    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.w = 1.0;
    
    RCLCPP_INFO(node->get_logger(),
            "NavigateToPose: goal x=%f y=%f", x, y);

    return true;
}


// vyhodnotí výsledek navigace
    BT::NodeStatus onResultReceived(const WrappedResult& wr) override
{
    auto node = node_.lock();

    if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node->get_logger(), "Navigation succeeded");
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(node->get_logger(), "Navigation failed");
    return BT::NodeStatus::FAILURE;
}



    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
{
    auto node = node_.lock();

    RCLCPP_ERROR(node->get_logger(), "Action call failed, error code: %d", error);
    return BT::NodeStatus::FAILURE;
}


// vrací RUNNING (robot se pohybuje)
    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> /*feedback*/) override
    {
        // TODO: Vraťte RUNNING (akce stále probíhá).
        return BT::NodeStatus::RUNNING;
    }
};



CreateRosNodePlugin(NavigateToPoseAction, "NavigateToPoseAction");
