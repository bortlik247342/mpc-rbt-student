#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Localization.hpp"


LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    odometry_.pose.pose.orientation.w = 1.0;
    odometry_.pose.pose.position.x = -0.5;

    // Subscriber for joint_states
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 
        10, 
        std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1)
    );

    // Publisher for odometry
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    //dt
    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0.0) return;

    double left_vel = msg.velocity[1];
    double right_vel = msg.velocity[0];

    updateOdometry(left_vel, right_vel, dt);
    publishOdometry();
    publishTransform();
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // rychlost kol
    double v_l = left_wheel_vel * robot_config::WHEEL_RADIUS;
    double v_r = right_wheel_vel * robot_config::WHEEL_RADIUS;

    // lineární a úhlová rychlost celého robota
    double linear = (v_r + v_l) / 2.0;
    double angular = (v_r - v_l) / (2.0 * robot_config::HALF_DISTANCE_BETWEEN_WHEELS);

    // theta z kvaternionu
    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    // Integrace polohy
    odometry_.pose.pose.position.x += linear * std::cos(theta) * dt;
    odometry_.pose.pose.position.y += linear * std::sin(theta) * dt;
    theta += angular * dt;

    // normalizace theta
    theta = std::atan2(std::sin(theta), std::cos(theta));

    // Uložení jako kvaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odometry_.pose.pose.orientation = tf2::toMsg(q);

    odometry_.twist.twist.linear.x = linear;
    odometry_.twist.twist.angular.z = angular;
}


void LocalizationNode::publishOdometry() {
    odometry_.header.stamp = this->get_clock()->now();
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";

    t.transform.translation.x = odometry_.pose.pose.position.x;
    t.transform.translation.y = odometry_.pose.pose.position.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odometry_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
}

