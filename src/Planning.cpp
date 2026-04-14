#include "Planning.hpp"
#include <algorithm>
#include <cmath>

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

    // Client pro mapu
    map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

    // Služba pro plánování cesty
    plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
        "/plan_path",
        std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Publisher pro cestu
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    RCLCPP_INFO(get_logger(), "Planning node started.");

    // Čekáme na dostupnost mapy
    while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for map service...");
    }
    RCLCPP_INFO(get_logger(), "Map service available!");

    // Požadavek na mapu
    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    map_client_->async_send_request(
        request,
        std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Trying to fetch map...");
}

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    auto response = future.get();
    if (response) {
        map_ = response->map;
        RCLCPP_INFO(this->get_logger(), "Map received: width=%d, height=%d, resolution=%.3f",
            map_.info.width, map_.info.height, map_.info.resolution);

        // Hned můžeme mapu dilatovat
        dilateMap();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive map!");
    }
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                            std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received plan request");

    if (map_.data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Map not available! Cannot plan path.");
        return;
    }

    // Vypočítáme cestu A*
    aStar(request->start, request->goal);

    // Vyhladíme cestu
    smoothPath();
    
    // Nastavení správného headeru celé cesty
    path_.header.frame_id = "map";
    path_.header.stamp = this->now();

    // Publikujeme cestu
    path_pub_->publish(path_);

    // Odpověď služby
    response->plan = path_;
    RCLCPP_INFO(this->get_logger(), "Plan response sent with %zu points", path_.poses.size());
}

void PlanningNode::dilateMap() {
    if (map_.data.empty()) return;

    double safe_distance = 0.4;
    int radius = static_cast<int>(safe_distance / map_.info.resolution);

    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    int width = map_.info.width;
    int height = map_.info.height;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            if (map_.data[idx] > 50) { // překážka
                for (int dy = -radius; dy <= radius; dy++) {
                    for (int dx = -radius; dx <= radius; dx++) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            int nidx = ny * width + nx;
                            dilatedMap.data[nidx] = 100;
                        }
                    }
                }
            }
        }
    }

    map_ = dilatedMap;
    RCLCPP_INFO(this->get_logger(), "Map dilated with radius %d cells.", radius);
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start,
                         const geometry_msgs::msg::PoseStamped &goal) {
    int start_x = static_cast<int>((start.pose.position.x - map_.info.origin.position.x) / map_.info.resolution);
    int start_y = static_cast<int>((start.pose.position.y - map_.info.origin.position.y) / map_.info.resolution);
    int goal_x  = static_cast<int>((goal.pose.position.x - map_.info.origin.position.x) / map_.info.resolution);
    int goal_y  = static_cast<int>((goal.pose.position.y - map_.info.origin.position.y) / map_.info.resolution);

    auto cStart = std::make_shared<Cell>(start_x, start_y);
    auto cGoal  = std::make_shared<Cell>(goal_x, goal_y);

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);
    openList.push_back(cStart);

    while(!openList.empty() && rclcpp::ok()) {
        auto current_it = std::min_element(openList.begin(), openList.end(),
            [](const std::shared_ptr<Cell> &a, const std::shared_ptr<Cell> &b){ return a->f < b->f; });
        auto current = *current_it;
        openList.erase(current_it);

        if(current->x == goal_x && current->y == goal_y) {
            path_.poses.clear();
            while(current) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.position.x = current->x * map_.info.resolution + map_.info.origin.position.x;
                pose.pose.position.y = current->y * map_.info.resolution + map_.info.origin.position.y;
                pose.pose.orientation.w = 1.0;
                path_.poses.push_back(pose);
                current = current->parent;
            }
            std::reverse(path_.poses.begin(), path_.poses.end());
            path_pub_->publish(path_);
            RCLCPP_INFO(get_logger(), "Path found!");
            return;
        }

        closedList[current->y * map_.info.width + current->x] = true;

        for(int dx = -1; dx <= 1; ++dx) {
            for(int dy = -1; dy <= 1; ++dy) {
                if(dx == 0 && dy == 0) continue;
                int nx = current->x + dx;
                int ny = current->y + dy;

                if(nx < 0 || nx >= static_cast<int>(map_.info.width) || ny < 0 || ny >= static_cast<int>(map_.info.height))	// mimo mapu
                    continue;
                if(map_.data[ny * map_.info.width + nx] > 50) continue;		// překážka
                if(closedList[ny * map_.info.width + nx]) continue;		// už zpracováno

                float g_new = current->g + std::hypot(dx, dy);
                float h_new = std::hypot(goal_x - nx, goal_y - ny);
                float f_new = g_new + h_new;

                auto it = std::find_if(openList.begin(), openList.end(),
                    [nx, ny](const std::shared_ptr<Cell> &c){ return c->x == nx && c->y == ny; });
                if(it != openList.end()) {
                    if(g_new < (*it)->g) {
                        (*it)->g = g_new;
                        (*it)->f = f_new;
                        (*it)->parent = current;
                    }
                } else {
                    auto neighbor = std::make_shared<Cell>(nx, ny);
                    neighbor->g = g_new;
                    neighbor->h = h_new;
                    neighbor->f = f_new;
                    neighbor->parent = current;
                    openList.push_back(neighbor);
                }
            }
        }
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
}

void PlanningNode::smoothPath() {
    if (path_.poses.size() < 3) return;

    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;

    double alpha = 0.1;  
    double beta  = 0.3;
    double tolerance = 1e-4;

    double change = tolerance;

    while (change >= tolerance) {
        change = 0.0;

        for (size_t i = 1; i < path_.poses.size() - 1; i++) {
            double old_x = newPath[i].pose.position.x;
            double old_y = newPath[i].pose.position.y;

            double xi = path_.poses[i].pose.position.x;
            double yi = path_.poses[i].pose.position.y;

            double xim1 = newPath[i - 1].pose.position.x;
            double yim1 = newPath[i - 1].pose.position.y;

            double xip1 = newPath[i + 1].pose.position.x;
            double yip1 = newPath[i + 1].pose.position.y;

            // podle vzorce
            newPath[i].pose.position.x +=
                alpha * (xi - newPath[i].pose.position.x) +
                beta * (xim1 + xip1 - 2 * newPath[i].pose.position.x);

            newPath[i].pose.position.y +=
                alpha * (yi - newPath[i].pose.position.y) +
                beta * (yim1 + yip1 - 2 * newPath[i].pose.position.y);

            change += std::abs(old_x - newPath[i].pose.position.x);
            change += std::abs(old_y - newPath[i].pose.position.y);
        }
    }

    path_.poses = newPath;

    RCLCPP_INFO(get_logger(), "Path smoothed (gradient): %zu points", path_.poses.size());
}
Cell::Cell(int c, int r) : x(c), y(r), f(0), g(0), h(0), parent(nullptr) {}
