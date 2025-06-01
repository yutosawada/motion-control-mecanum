#include "motion-control-mecanum/motor_controller.hpp"

namespace motion_control_mecanum {

MotorController::MotorController(std::shared_ptr<can_control::CanInterface> can)
: can_(std::move(can))
{
}

bool MotorController::writeSpeeds(const std::array<double, 4> & speeds)
{
  for (size_t i = 0; i < speeds.size(); ++i) {
    can_control::CanFrame frame;
    frame.arbitration_id = 0x200 + static_cast<uint32_t>(i);
    frame.dlc = 4;
    frame.data.resize(4);
    int32_t value = static_cast<int32_t>(speeds[i] * 1000.0);
    frame.data[0] = (value >> 24) & 0xFF;
    frame.data[1] = (value >> 16) & 0xFF;
    frame.data[2] = (value >> 8) & 0xFF;
    frame.data[3] = value & 0xFF;
    if (!can_->Send(frame)) {
      return false;
    }
  }
  return true;
}

}  // namespace motion_control_mecanum
