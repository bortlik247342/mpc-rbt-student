#include "behaviortree_ros2/tree_execution_server.hpp"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

#include "rclcpp/rclcpp.hpp"
#include <memory>

// TODO: Vytvořte třídu BTServer odvozenou z BT::TreeExecutionServer.
// Konstruktor přijímá const rclcpp::NodeOptions& a předá je rodičovské třídě.
// Volitelně přepište metodu onTreeCreated() pro přidání loggeru (BT::StdCoutLogger).

class BTServer : public BT::TreeExecutionServer {
public:
    BTServer(const rclcpp::NodeOptions& options)
        : TreeExecutionServer(options)
    {
    }

    void onTreeCreated(BT::Tree& tree) override
    {
        // Volitelné: logování stromu do konzole
        static BT::StdCoutLogger logger(tree);
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // TODO: Vytvořte instanci BTServer s výchozími NodeOptions.
    // Vytvořte MultiThreadedExecutor, přidejte do něj server->node() a zavolejte spin().

    rclcpp::NodeOptions options;

    auto server = std::make_shared<BTServer>(options);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(server->node());

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
