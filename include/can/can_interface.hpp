#ifndef CAN_INTERFACE_H_
#define CAN_INTERFACE_H_

#include <cstdint>
#include <vector>

namespace can_control {

// CANフレームの構造体
struct CanFrame {
  uint32_t arbitration_id;    // 11ビットまたは29ビットのCAN ID
  uint8_t dlc;                // Data Length Code (0〜8)
  std::vector<uint8_t> data;  // 可変長のデータ（サイズはdlcに依存）
};

// CANインターフェースの抽象クラス
class CanInterface {
 public:
  virtual ~CanInterface() = default;

  // CANフレームを送信する
  // 戻り値がtrueなら送信成功、falseなら失敗
  virtual bool Send(const CanFrame& frame) = 0;

  // CANフレームを受信する
  // timeout_ms: タイムアウト時間（ミリ秒）
  // 戻り値がtrueならフレームを受信、falseならタイムアウトまたはエラー
  virtual bool Receive(CanFrame* frame, int timeout_ms) = 0;
};

}  // namespace can_control

#endif  // CAN_INTERFACE_H_