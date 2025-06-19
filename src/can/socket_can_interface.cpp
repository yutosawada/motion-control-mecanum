#include "can/socket_can_interface.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

namespace can_control {

SocketCanInterface::SocketCanInterface(const std::string &interface_name)
    : socket_fd_(-1),
      interface_name_(interface_name),
      logger_(rclcpp::get_logger("SocketCanInterface")) {
  if (!InitializeSocketCAN()) {
    RCLCPP_ERROR(logger_, "Failed to initialize SocketCAN interface: %s",
                 interface_name_.c_str());
  }
}

SocketCanInterface::~SocketCanInterface() { CloseSocketCAN(); }

bool SocketCanInterface::Send(const CanFrame &frame) {
  struct can_frame can_frame;
  can_frame.can_id = frame.arbitration_id;
  can_frame.can_dlc = frame.dlc;
  std::memcpy(can_frame.data, frame.data.data(), frame.dlc);

  if (write(socket_fd_, &can_frame, sizeof(can_frame)) != sizeof(can_frame)) {
    RCLCPP_ERROR(logger_, "Failed to send CAN frame");
    return false;
  }

  // RCLCPP_INFO(logger_, "Sent CAN frame with ID: 0x%X", frame.arbitration_id);
  return true;
}

bool SocketCanInterface::Receive(CanFrame *frame, int timeout_ms) {
  // RCLCPP_INFO(logger_, "Received CAN frame ");
  struct can_frame can_frame;
  fd_set read_fds;
  struct timeval timeout;

  FD_ZERO(&read_fds);
  FD_SET(socket_fd_, &read_fds);

  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;

  int ret = select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
  if (ret <= 0) {
    return false;
  }

  if (read(socket_fd_, &can_frame, sizeof(can_frame)) != sizeof(can_frame)) {
    RCLCPP_ERROR(logger_, "Failed to receive CAN frame");
    return false;
  }

  frame->arbitration_id = can_frame.can_id;
  frame->dlc = can_frame.can_dlc;
  frame->data.assign(can_frame.data, can_frame.data + can_frame.can_dlc);

  // RCLCPP_INFO(logger_, "Received CAN frame with ID: 0x%X",
  // frame->arbitration_id);
  return true;
}

bool SocketCanInterface::InitializeSocketCAN() {
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(logger_, "Failed to create CAN socket");
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(logger_, "Failed to get interface index for %s",
                 interface_name_.c_str());
    CloseSocketCAN();
    return false;
  }

  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    RCLCPP_ERROR(logger_, "Failed to bind CAN socket to interface %s",
                 interface_name_.c_str());
    CloseSocketCAN();
    return false;
  }

  RCLCPP_INFO(logger_, "SocketCAN interface %s initialized",
              interface_name_.c_str());
  return true;
}

void SocketCanInterface::CloseSocketCAN() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
    RCLCPP_INFO(logger_, "SocketCAN interface closed");
  }
}

}  // namespace can_control