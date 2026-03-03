#include <mpc-rbt-solution/Receiver.hpp>

void Receiver::Node::run()
{
  while (errno != EINTR) {
    RCLCPP_INFO(logger, "Waiting for data ...");
    Socket::IPFrame frame{};
    if (receive(frame)) {
      RCLCPP_INFO(logger, "Received data from host: '%s:%d'", frame.address.c_str(), frame.port);

      callback(frame);

    } else {
      RCLCPP_WARN(logger, "Failed to receive data.");
    }
  }
}

void Receiver::Node::onDataReceived(const Socket::IPFrame & frame)
{
  if (Utils::Message::deserialize(frame, data)) {
      RCLCPP_INFO(logger, "--- DATA PRIJATA ---");
      //RCLCPP_INFO(logger, "X: %.2f, Y: %.2f, Z: %.2f", data.frame.c_str(), data.x, data.y, data.z);
      RCLCPP_INFO(logger, "\n\tstamp: %ld, X: %.2f, Y: %.2f, Z: %.2f", data.timestamp,data.frame.c_str(), data.x, data.y, data.z);
  } else {
      RCLCPP_ERROR(logger, "Chyba deserializace!");
  }

  RCLCPP_INFO(logger, "\n\tstamp: %ld", data.timestamp);
}
