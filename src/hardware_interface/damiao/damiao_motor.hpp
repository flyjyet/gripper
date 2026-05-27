#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <memory>
#include <thread>
#include <vector>

#include "common/units.hpp"
#include "hardware_interface/adapter_types.hpp"
#include "hardware_interface/damiao/damiao_protocol.hpp"
#include "hardware_interface/motor_interface.hpp"
#include "hardware_interface/transport_interface.hpp"
#include "hardware_interface/virtual_encoder.hpp"

namespace gripper::hardware_interface::damiao {

enum class EncoderUnwrapSource {
  ProtocolPosition,
  RawPositionCounts,
};

struct DamiaoMotorConfig {
  MotorId motor_id{};
  MotorId host_id{};
  AdapterConfig adapter{};
  DamiaoMotorLimits limits{};
  common::A max_phase_current{20.0};
  common::Ratio torque_per_amp{0.625};
  int position_command_sign{1};
  bool auto_switch_mode{true};
  common::S feedback_poll_period{0.02};
  common::S feedback_stale_timeout{0.2};
  bool continuous_encoder_enabled{true};
  EncoderUnwrapSource encoder_unwrap_source{EncoderUnwrapSource::ProtocolPosition};
  common::Rad encoder_wrap_range{0.0};
  std::uint32_t encoder_raw_count_range{65536};
  DamiaoFrameOptions frame_options{};
};

// SDK-backed motor implementation. Vendor SDK handles stay inside the
// TransportInterface implementation; this class only maps generic motor
// commands to Damiao protocol frames.
class DamiaoMotor final : public MotorInterface {
 public:
  DamiaoMotor(DamiaoMotorConfig config,
              std::unique_ptr<TransportInterface> transport);
  ~DamiaoMotor() override;

  [[nodiscard]] common::Result connect() override;
  [[nodiscard]] common::Result disconnect() override;
  [[nodiscard]] common::Result enable() override;
  [[nodiscard]] common::Result disable() override;
  [[nodiscard]] common::Result sendCommand(const MotorCommand& command) override;
  [[nodiscard]] common::Result readFeedback(MotorFeedback* feedback) override;
  [[nodiscard]] common::Result runCommunicationProbe(
      std::vector<std::string>* lines) override;
  [[nodiscard]] bool isConnected() const noexcept override;
  [[nodiscard]] bool isEnabled() const noexcept override;

 private:
  DamiaoMotorConfig config_{};
  std::unique_ptr<TransportInterface> transport_{};
  std::atomic_bool connected_{false};
  std::atomic_bool enabled_{false};
  std::atomic_bool feedback_thread_running_{false};
  DamiaoControlMode active_mode_{DamiaoControlMode::PositionForce};
  MotorFeedback last_feedback_{};
  bool last_feedback_valid_{false};
  common::Result last_feedback_result_{
      common::ErrorCode::FeedbackTimedOut, "no motor feedback received yet"};
  std::chrono::steady_clock::time_point last_feedback_steady_time_{};
  MultiTurnVirtualEncoder virtual_encoder_{};
  mutable std::mutex command_mutex_{};
  mutable std::mutex transport_mutex_{};
  mutable std::mutex feedback_mutex_{};
  std::thread feedback_thread_{};

  [[nodiscard]] DamiaoIds ids() const noexcept;
  [[nodiscard]] common::Result requireConnected() const;
  [[nodiscard]] common::Result write(const CanFrame& frame);
  [[nodiscard]] common::Result writeUnlocked(const CanFrame& frame);
  [[nodiscard]] common::Result switchMode(DamiaoControlMode mode);
  [[nodiscard]] common::Result refreshFeedback();
  // Reads one DM register while transport_mutex_ is already held. This is used
  // during connect/probe before the feedback thread can safely consume frames.
  [[nodiscard]] common::Result readRegisterUnlocked(
      std::uint8_t register_id, DamiaoRegisterValue* value,
      std::chrono::milliseconds timeout);
  [[nodiscard]] common::Result updateRuntimeLimitsFromRegisters();
  [[nodiscard]] common::Result readLatestFeedback(MotorFeedback* feedback) const;
  [[nodiscard]] common::Result waitForFreshFeedback(
      std::chrono::steady_clock::time_point min_time,
      std::chrono::milliseconds timeout, MotorFeedback* feedback);
  void startFeedbackThread();
  void stopFeedbackThread();
  void feedbackLoop();
  void resetContinuousEncoder() noexcept;
  [[nodiscard]] common::Rad encoderWrappedPosition(
      const DamiaoFeedback& feedback) const noexcept;
  [[nodiscard]] common::Result sendProbeFrame(
      const char* name, const CanFrame& frame, std::vector<std::string>* lines);
  [[nodiscard]] common::Result sendProbeRegisterRead(
      const char* name, std::uint8_t register_id,
      std::vector<std::string>* lines);
  [[nodiscard]] common::A torqueToCurrent(common::Nm torque) const noexcept;
};

}  // namespace gripper::hardware_interface::damiao
