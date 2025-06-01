#ifndef SOCKET_CAN_INTERFACE_H_
#define SOCKET_CAN_INTERFACE_H_

#include <string>
#include <vector>
#include "can/can_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace can_control
{

// SocketCAN を利用した CAN インターフェースの実装クラス
class SocketCanInterface : public CanInterface
{
public:
  // コンストラクタ。interface_name には "can0" などのインターフェース名を指定
  explicit SocketCanInterface(const std::string & interface_name);

  ~SocketCanInterface() override;

  // CANフレームの送信。成功すれば true、失敗すれば false を返す
  bool Send(const CanFrame & frame) override;

  // CANフレームの受信。timeout_ms（ミリ秒）以内に受信できれば true、タイムアウトやエラーなら false
  bool Receive(CanFrame * frame, int timeout_ms) override;

private:
  int socket_fd_;
  std::string interface_name_;
  rclcpp::Logger logger_;

  bool InitializeSocketCAN();
  void CloseSocketCAN();
};

}  // namespace can_control

#endif  // SOCKET_CAN_INTERFACE_H_