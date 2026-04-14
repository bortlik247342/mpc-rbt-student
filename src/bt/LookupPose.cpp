#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"

#include "rclcpp/rclcpp.hpp"

#include <map>
#include <string>
#include <stdexcept>


// Tento node převádí textové ID lokace na reálné souřadnice a předává je dál v Behavior Tree.

struct Pose2D {
    double x;
    double y;
};

class LookupPose : public BT::SyncActionNode {
public:
// Předpřipravím databázi souřadnic, aby se podle ID dalo rychle hledat
    LookupPose(const std::string& name, const BT::NodeConfig& config)
        : SyncActionNode(name, config)
    {
        // TODO: Naplňte tabulku souřadnic pose_table_.
        // Manipulátory: ID "1", "2", "3" (viz tabulka v zadání).
        // Sklady: ID "A1", "A2", "B1", "B2", "C1", "C2", "D1", "D2" (viz tabulka v zadání).
        // Příklad: pose_table_["1"] = { 4.5, 1.5 };
        
        // Manipulátory
    pose_table_["1"] = {4.5, 1.5};
    pose_table_["2"] = {4.5, -0.5};
    pose_table_["3"] = {4.5, -2.5};

    // Sklady
    pose_table_["A1"] = {1.5, 0.5};
    pose_table_["A2"] = {1.5, -1.5};
    pose_table_["B1"] = {-0.5, 0.5};
    pose_table_["B2"] = {-0.5, -1.5};
    pose_table_["C1"] = {-2.5, 0.5};
    pose_table_["C2"] = {-2.5, -1.5};
    pose_table_["D1"] = {-4.5, 0.5};
    pose_table_["D2"] = {-4.5, -1.5};
    
    pose_table_["START"] = {-0.5, 0.0};
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("location_id", "Location ID to look up (e.g. '1', 'A1')"),
            BT::OutputPort<double>("x", "Looked up X coordinate"),
            BT::OutputPort<double>("y", "Looked up Y coordinate")
        };
    }

    BT::NodeStatus tick() override
{
// vezme location_id z BT stromu
    auto id = getInput<std::string>("location_id");
    if (!id) {
        std::cout << "LookupPose: missing location_id" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "LookupPose: looking for ID = " << id.value() << std::endl;

// hledá ID v pose_table_
    auto it = pose_table_.find(id.value());
    if (it == pose_table_.end()) {
        std::cout << "LookupPose: ID NOT FOUND!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "LookupPose: found x=" << it->second.x
              << " y=" << it->second.y << std::endl;

    setOutput("x", it->second.x);
    setOutput("y", it->second.y);
    return BT::NodeStatus::SUCCESS;
}

private:
    std::map<std::string, Pose2D> pose_table_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<LookupPose>("LookupPose");
}
