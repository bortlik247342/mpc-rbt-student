#include <mpc-rbt-solution/Sender.hpp>

void Sender::Node::run()
{
  while (errno != EINTR) {
    if ((std::chrono::steady_clock::now() - timer_tick) < timer_period) continue;
    timer_tick = std::chrono::steady_clock::now();

    callback();
  }
}

void Sender::Node::onDataTimerTick()
{
  data.x += 0.5;
  data.y += 6.7;
  data.z += 10.2;

  data.timestamp =
    static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());

  Socket::IPFrame frame{
    .port = config.remotePort,
    .address = config.remoteAddress,
  };
  if (Utils::Message::serialize(frame, data)) {
    if(!send(frame))
       RCLCPP_ERROR(logger, "Sending failed!");
    RCLCPP_INFO(logger, "Sending data to host: '%s:%d'", frame.address.c_str(), frame.port);
    RCLCPP_INFO(logger, "\n\tstamp: %ld, X: %2f, Y: %2f, Z: %2f", data.timestamp, data.x, data.y, data.z);
  } else {
    RCLCPP_ERROR(logger, "Serialization failed!");
  }

  //RCLCPP_INFO(logger, "Sending data to host: '%s:%d'", frame.address.c_str(), frame.port);

  //RCLCPP_INFO(logger, "\n\tstamp: %ld", data.timestamp);
}
