#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"


MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

        // Subscribers for odometry - získávám pozici robota
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 10,
        std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1)
    );
    
    	// Subscribers laser scans - získávám dala z lidaru (detekce překážek)
    	lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/tiago_base/Hokuyo_URG_04LX_UG01", 10,
        std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1)
    );

    
        // Publisher for robot control - ovládání robota (rychlost a otočení)
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10
    );

        // Client for path planning - žádost o trajektorii
        plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");


        // Action server - přijímá cíle
        nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(this, "/go_to_goal",
    		std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
    		std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
    		std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Motion control node started.");

        // Connect to path planning service server - čeká než se spustí plánování cesty (minulé cvičení)
        while (!plan_client_->wait_for_service(std::chrono::seconds(1))) {
    		RCLCPP_WARN(this->get_logger(), "Waiting for planning service...");
    	}
	RCLCPP_INFO(this->get_logger(), "Connected to planning service.");
    }

// kontroluje kolizi s překážkami
void MotionControlNode::checkCollision() {
    if (laser_scan_.ranges.empty())
        return;

    double thresh = 0.3;	// práh v metrech
    bool collision = false;

    for (size_t i = 0; i < laser_scan_.ranges.size(); ++i) {
        double angle = laser_scan_.angle_min + i * laser_scan_.angle_increment;

        // pouze před robotem 30°
        if (std::abs(angle) < M_PI / 6) {
            double r = laser_scan_.ranges[i];

            if (std::isfinite(r) && r < thresh) {
                collision = true;
                break;
            }
        }
    }

    if (collision) {
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);

        if (goal_handle_) {
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_->abort(result);
            goal_handle_.reset();
        }

        RCLCPP_WARN(get_logger(), "Collision detected!");
    }
}

// implementace Pure Pursuit
void MotionControlNode::updateTwist()
{
    // 🔴 DŮLEŽITÉ kontroly
    if (!goal_active_ || !goal_handle_ || path_.poses.empty())
        return;

    double Ld = 0.5;
    double v_max = 0.2;

    // === Najdi nejbližší bod ===
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;

    for (size_t i = 0; i < path_.poses.size(); ++i) {
        double dx = path_.poses[i].pose.position.x - current_pose_.pose.position.x;
        double dy = path_.poses[i].pose.position.y - current_pose_.pose.position.y;
        double dist = std::hypot(dx, dy);

        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // === Lookahead bod ===
    geometry_msgs::msg::Pose target;
    bool found = false;

    for (size_t i = closest_idx; i < path_.poses.size(); ++i) {
        double dx = path_.poses[i].pose.position.x - current_pose_.pose.position.x;
        double dy = path_.poses[i].pose.position.y - current_pose_.pose.position.y;
        double dist = std::hypot(dx, dy);

        if (dist >= Ld) {
            target = path_.poses[i].pose;
            found = true;
            break;
        }
    }

    if (!found) {
        target = path_.poses.back().pose;
    }

    // === Transformace ===
    double dx = target.position.x - current_pose_.pose.position.x;
    double dy = target.position.y - current_pose_.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(current_pose_.pose.orientation, q);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double x_r =  cos(yaw) * dx + sin(yaw) * dy;
    double y_r = -sin(yaw) * dx + cos(yaw) * dy;

    double L = std::hypot(x_r, y_r);

    double v = v_max;
    double w = 0.0;

    if (L > 1e-3) {
        double curvature = (2.0 * y_r) / (L * L);
        w = curvature * v;
    }

    // otočení pokud je cíl za robotem
    if (x_r < 0) {
        v = 0.0;
        w = (y_r > 0) ? 0.5 : -0.5;
    }

    // === Kontrola cíle (FIX) ===
    double goal_dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
    double goal_dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
    double goal_dist = std::hypot(goal_dx, goal_dy);

    if (goal_dist < 0.1) {   // 🔴 mírně větší tolerance
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);

        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        goal_handle_->succeed(result);

        RCLCPP_INFO(get_logger(), "Goal reached!");

        goal_handle_.reset();
        goal_active_ = false;
        path_.poses.clear();

        return;
    }

    // === publish ===
    geometry_msgs::msg::Twist twist;
    twist.linear.x = v;
    twist.angular.z = w;

    twist_publisher_->publish(twist);
}

// přijme nový cíl
rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    (void)uuid;
    (void)goal;

    RCLCPP_INFO(this->get_logger(), "Goal received");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// když uživatel zruší goal
rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    (void)goal_handle;

    RCLCPP_WARN(this->get_logger(), "Goal canceled");

    return rclcpp_action::CancelResponse::ACCEPT;
}

// uloží goal, zkontroluje odometrii, pošle request na plánování
void MotionControlNode::navHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
{
    goal_handle_ = goal_handle;
    goal_pose_ = goal_handle_->get_goal()->pose;

    goal_active_ = true;

    // 🔴 KRITICKÉ – smažeme starou trajektorii
    path_.poses.clear();

    if (current_pose_.header.frame_id.empty()) {
        RCLCPP_WARN(get_logger(), "Waiting for first odometry...");
        return;
    }

    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = current_pose_;
    request->goal  = goal_pose_;
    request->tolerance = 0.0;

    RCLCPP_INFO(get_logger(), "Requesting path: start=(%.2f, %.2f), goal=(%.2f, %.2f)",
        current_pose_.pose.position.x, current_pose_.pose.position.y,
        goal_pose_.pose.position.x, goal_pose_.pose.position.y);

    plan_client_->async_send_request(
        request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1)
    );
}

// běžící smyčka akce -> kontroluje zrušení, posílá feedback (pozice robota)
void MotionControlNode::execute()
{
    rclcpp::Rate loop_rate(5); // rychlejší než 1 Hz

    while (rclcpp::ok() && goal_active_) {

        if (!goal_handle_) break;

        if (goal_handle_->is_canceling()) {
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_->canceled(result);

            RCLCPP_WARN(get_logger(), "Goal canceled");

            goal_handle_.reset();
            goal_active_ = false;
            return;
        }

        auto feedback_msg = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
        feedback_msg->current_pose = current_pose_;
        goal_handle_->publish_feedback(feedback_msg);

        loop_rate.sleep();
    }
}

// odpověď plánovače, plán neexistuje → abort, jinak uloží trajektorii a spustí execute
void MotionControlNode::pathCallback(
    rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future)
{
    auto response = future.get();

    if (!response || response->plan.poses.empty()) {
        RCLCPP_ERROR(get_logger(), "Path planning failed!");
        if (goal_handle_) {
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_->abort(result);
            goal_handle_.reset();
        }
        goal_active_ = false;
        return;
    }

    path_ = response->plan;

    RCLCPP_INFO(get_logger(), "Path received with %zu points", path_.poses.size());

    std::thread(&MotionControlNode::execute, this).detach();
}

// ukládá odometrii
void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    	current_pose_.pose = msg.pose.pose;
    	current_pose_.header = msg.header;

    	checkCollision();
    	updateTwist();
}

// ukládá lidar data
void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    laser_scan_ = msg;       // uložíme data
    checkCollision();        // kontrola kolize
}
