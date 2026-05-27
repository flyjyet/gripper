#include "hardware_interface/damiao/damiao_motor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "common/error_code.hpp"
#include "hardware_interface/damiao/dm_usb2fdcan_transport.hpp"

namespace gripper::hardware_interface::damiao {
namespace {

[[nodiscard]] common::Result connectionNotOpen() {
  return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                               "Damiao motor is not connected");
}

[[nodiscard]] DamiaoControlMode toDamiaoMode(MotorControlMode mode) {
  switch (mode) {
    case MotorControlMode::Velocity:
      return DamiaoControlMode::Velocity;
    case MotorControlMode::Current:
    case MotorControlMode::Torque:
      return DamiaoControlMode::Mit;
    case MotorControlMode::Position:
      return DamiaoControlMode::PositionVelocity;
    case MotorControlMode::PositionVelocityTorque:
      return DamiaoControlMode::PositionForce;
    case MotorControlMode::Disabled:
      return DamiaoControlMode::PositionForce;
  }
  return DamiaoControlMode::PositionForce;
}

[[nodiscard]] double positiveOrDefault(double value, double fallback) {
  return value > 0.0 ? value : fallback;
}

[[nodiscard]] common::Rad commandPosition(common::Rad controller_position,
                                          int position_command_sign) {
  const double sign = position_command_sign < 0 ? -1.0 : 1.0;
  return common::Rad{controller_position.value * sign};
}

[[nodiscard]] common::Rad encoderWrapRange(const DamiaoMotorConfig& config) {
  if (config.encoder_wrap_range.value > 0.0) {
    return config.encoder_wrap_range;
  }
  if (config.encoder_unwrap_source == EncoderUnwrapSource::RawPositionCounts) {
    constexpr double two_pi = 6.28318530717958647692;
    return common::Rad{two_pi};
  }
  return common::Rad{2.0 * positiveOrDefault(config.limits.position.value, 12.5)};
}

[[nodiscard]] std::chrono::milliseconds secondsToMilliseconds(
    common::S seconds, double fallback_seconds) {
  const double value = positiveOrDefault(seconds.value, fallback_seconds);
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>{value});
}

[[nodiscard]] common::Timestamp steadyTimestamp(
    std::chrono::steady_clock::time_point time_point) {
  return common::Timestamp::fromNanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          time_point.time_since_epoch())
          .count());
}

[[nodiscard]] std::string frameModeName(CanPayloadMode mode) {
  return mode == CanPayloadMode::CanFd ? "FD" : "CAN";
}

[[nodiscard]] std::string frameDataHex(const CanFrame& frame) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (std::uint8_t index = 0; index < frame.data_length; ++index) {
    if (index != 0U) {
      stream << ' ';
    }
    stream << std::setw(2) << static_cast<int>(frame.data[index]);
  }
  return stream.str();
}

void appendTxLine(const char* name, const CanFrame& frame,
                  std::vector<std::string>* lines) {
  std::ostringstream stream;
  stream << "TX " << name << ' ' << frameModeName(frame.payload_mode)
         << " id=0x" << std::hex << std::uppercase << frame.id
         << std::nouppercase << std::dec
         << " dlc=" << static_cast<int>(frame.data_length)
         << " brs=" << (frame.bitrate_switch ? 1 : 0)
         << " data=" << frameDataHex(frame);
  lines->push_back(stream.str());
}

void appendRxLine(const CanFrame& frame, std::vector<std::string>* lines) {
  std::ostringstream stream;
  stream << "RX " << frameModeName(frame.payload_mode)
         << " id=0x" << std::hex << std::uppercase << frame.id
         << std::nouppercase << std::dec
         << " dlc=" << static_cast<int>(frame.data_length)
         << " brs=" << (frame.bitrate_switch ? 1 : 0)
         << " data=" << frameDataHex(frame);
  lines->push_back(stream.str());
}

[[nodiscard]] std::string rawFrameSummary(const CanFrame& frame) {
  std::ostringstream stream;
  stream << frameModeName(frame.payload_mode)
         << " id=0x" << std::hex << std::uppercase << frame.id
         << std::nouppercase << std::dec
         << " dlc=" << static_cast<int>(frame.data_length)
         << " brs=" << (frame.bitrate_switch ? 1 : 0)
         << " data=" << frameDataHex(frame);
  return stream.str();
}

}  // namespace

DamiaoMotor::DamiaoMotor(DamiaoMotorConfig config,
                         std::unique_ptr<TransportInterface> transport)
    : config_(config), transport_(std::move(transport)) {
  virtual_encoder_.setConfig(VirtualEncoderConfig{
      encoderWrapRange(config_),
      config_.continuous_encoder_enabled});
}

DamiaoMotor::~DamiaoMotor() {
  (void)disconnect();
}

common::Result DamiaoMotor::connect() {
  connected_ = false;
  enabled_ = false;
  stopFeedbackThread();
  resetContinuousEncoder();
  if (!transport_) {
    return common::Result::error(common::ErrorCode::ControlNotReady,
                                 "Damiao motor has no transport");
  }
  if (config_.adapter.adapter_type == AdapterType::Unknown) {
    config_.adapter.adapter_type = AdapterType::UsbCanFd;
  }
  common::Result result = common::Ok();
  {
    std::lock_guard<std::mutex> transport_lock{transport_mutex_};
    result = transport_->open(config_.adapter);
    if (result.isError()) {
      return result;
    }

    transport_->clearRxQueue();
    const auto limit_result = updateRuntimeLimitsFromRegisters();
    if (limit_result.isError()) {
      (void)transport_->close();
      std::ostringstream message;
      message << "failed to read Damiao runtime feedback mapping limits"
              << " before feedback parsing";
      if (limit_result.hasMessage()) {
        message << ": " << limit_result.message();
      }
      return common::Result::error(limit_result.code(), message.str());
    }
  }
  connected_ = true;
  active_mode_ = DamiaoControlMode::Mit;
  {
    std::lock_guard<std::mutex> lock{feedback_mutex_};
    last_feedback_ = {};
    last_feedback_valid_ = false;
    last_feedback_result_ = common::Result::error(
        common::ErrorCode::FeedbackTimedOut, "no motor feedback received yet");
    last_feedback_steady_time_ = {};
  }
  startFeedbackThread();
  MotorFeedback ignored{};
  (void)waitForFreshFeedback(std::chrono::steady_clock::now(),
                             std::chrono::milliseconds{250}, &ignored);
  return common::Ok();
}

common::Result DamiaoMotor::disconnect() {
  if (connected_ && enabled_) {
    (void)disable();
  }
  stopFeedbackThread();
  connected_ = false;
  enabled_ = false;
  resetContinuousEncoder();
  if (transport_) {
    std::lock_guard<std::mutex> transport_lock{transport_mutex_};
    return transport_->close();
  }
  return common::Ok();
}

common::Result DamiaoMotor::enable() {
  const auto ready = requireConnected();
  if (ready.isError()) {
    return ready;
  }
  std::lock_guard<std::mutex> command_lock{command_mutex_};
  if (!feedback_thread_running_.load()) {
    startFeedbackThread();
  }
  const auto mode_result = switchMode(DamiaoControlMode::PositionForce);
  if (mode_result.isError()) {
    enabled_ = false;
    return mode_result;
  }

  for (int attempt = 0; attempt < 3; ++attempt) {
    const auto result = write(DamiaoProtocol::makeEnableCommand(
        ids(), active_mode_, config_.frame_options));
    if (result.isError()) {
      enabled_ = false;
      return result;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
  }
  enabled_ = true;
  {
    std::lock_guard<std::mutex> lock{feedback_mutex_};
    last_feedback_.enabled = true;
  }
  const auto feedback_request_time = std::chrono::steady_clock::now();
  MotorFeedback feedback{};
  const auto feedback_result = waitForFreshFeedback(
      feedback_request_time, std::chrono::milliseconds{350}, &feedback);
  if (feedback_result.isError()) {
    enabled_ = false;
    std::lock_guard<std::mutex> lock{feedback_mutex_};
    last_feedback_.enabled = false;
    return feedback_result;
  }
  const auto hold_result = write(DamiaoProtocol::makePositionForceCommand(
      ids(), commandPosition(feedback.position, config_.position_command_sign),
      common::RadPerS{},
      common::A{}, config_.max_phase_current, config_.frame_options));
  if (hold_result.isError()) {
    enabled_ = false;
    std::lock_guard<std::mutex> lock{feedback_mutex_};
    last_feedback_.enabled = false;
    return hold_result;
  }
  return common::Ok();
}

common::Result DamiaoMotor::disable() {
  if (!connected_) {
    enabled_ = false;
    return connectionNotOpen();
  }
  std::lock_guard<std::mutex> command_lock{command_mutex_};
  MotorFeedback feedback{};
  common::Rad hold_position{};
  if (readLatestFeedback(&feedback).isOk()) {
    hold_position = feedback.position;
  } else {
    std::lock_guard<std::mutex> feedback_lock{feedback_mutex_};
    hold_position = last_feedback_.position;
  }
  const auto stop_frame =
      active_mode_ == DamiaoControlMode::Mit
          ? DamiaoProtocol::makeMitTorqueCommand(
                ids(), common::Nm{}, config_.limits, config_.frame_options)
          : DamiaoProtocol::makePositionForceCommand(
                ids(),
                commandPosition(hold_position, config_.position_command_sign),
                common::RadPerS{}, common::A{},
                config_.max_phase_current, config_.frame_options);
  (void)write(stop_frame);
  for (int attempt = 0; attempt < 3; ++attempt) {
    const auto result = write(DamiaoProtocol::makeDisabledCommand(
        ids(), active_mode_, config_.frame_options));
    if (result.isError()) {
      enabled_ = false;
      return result;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
  }
  if (config_.frame_options.command_id_includes_mode_offset &&
      active_mode_ != DamiaoControlMode::Mit) {
    auto base_id_options = config_.frame_options;
    base_id_options.command_id_includes_mode_offset = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
      const auto result = write(DamiaoProtocol::makeDisabledCommand(
          ids(), active_mode_, base_id_options));
      if (result.isError()) {
        enabled_ = false;
        return result;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
  }
  enabled_ = false;
  {
    std::lock_guard<std::mutex> lock{feedback_mutex_};
    last_feedback_.enabled = false;
  }
  return common::Ok();
}

common::Result DamiaoMotor::sendCommand(const MotorCommand& command) {
  const auto ready = requireConnected();
  if (ready.isError()) {
    return ready;
  }
  if (command.clear_fault) {
    std::lock_guard<std::mutex> command_lock{command_mutex_};
    const auto clear_result = write(DamiaoProtocol::makeClearFaultCommand(
        ids(), active_mode_, config_.frame_options));
    if (clear_result.isError()) {
      return clear_result;
    }
  }
  if (!command.enable ||
      command.control_mode == MotorControlMode::Disabled) {
    return disable();
  }
  std::lock_guard<std::mutex> command_lock{command_mutex_};
  if (!enabled_) {
    return common::Result::error(common::ErrorCode::ControlNotReady,
                                 "Damiao motor is not enabled");
  }

  const DamiaoControlMode requested_mode = toDamiaoMode(command.control_mode);
  const auto mode_result = switchMode(requested_mode);
  if (mode_result.isError()) {
    return mode_result;
  }

  CanFrame frame{};
  switch (command.control_mode) {
    case MotorControlMode::Velocity:
      frame = DamiaoProtocol::makeVelocityCommand(
          ids(), command.target_velocity, config_.frame_options);
      break;
    case MotorControlMode::Position:
      frame = DamiaoProtocol::makePositionVelocityCommand(
          ids(),
          commandPosition(command.target_position,
                          config_.position_command_sign),
          command.target_velocity,
          config_.frame_options);
      break;
    case MotorControlMode::Current: {
      const common::Nm torque{
          command.target_current.value * config_.torque_per_amp.value};
      frame = DamiaoProtocol::makeMitTorqueCommand(
          ids(), torque, config_.limits, config_.frame_options);
      break;
    }
    case MotorControlMode::Torque:
      frame = DamiaoProtocol::makeMitTorqueCommand(
          ids(), command.target_torque, config_.limits, config_.frame_options);
      break;
    case MotorControlMode::PositionVelocityTorque:
      frame = DamiaoProtocol::makePositionForceCommand(
          ids(),
          commandPosition(command.target_position,
                          config_.position_command_sign),
          command.target_velocity,
          command.target_current, config_.max_phase_current,
          config_.frame_options);
      break;
    case MotorControlMode::Disabled:
      return disable();
  }

  const auto result = write(frame);
  if (result.isError()) {
    return result;
  }
  return common::Ok();
}

common::Result DamiaoMotor::readFeedback(MotorFeedback* feedback) {
  if (feedback == nullptr) {
    return common::Result::error(common::ErrorCode::InvalidArgument,
                                 "feedback output pointer is null");
  }
  const auto ready = requireConnected();
  if (ready.isError()) {
    return ready;
  }
  return readLatestFeedback(feedback);
}

common::Result DamiaoMotor::runCommunicationProbe(
    std::vector<std::string>* lines) {
  if (lines == nullptr) {
    return common::Result::error(common::ErrorCode::InvalidArgument,
                                 "communication probe output is null");
  }
  lines->clear();
  const auto ready = requireConnected();
  if (ready.isError()) {
    return ready;
  }
  stopFeedbackThread();
  std::lock_guard<std::mutex> command_lock{command_mutex_};
  {
    std::lock_guard<std::mutex> transport_lock{transport_mutex_};
    transport_->clearRxQueue();
  }

  std::ostringstream header;
  header << "DM_DeviceSDK probe motor_id=0x" << std::hex << std::uppercase
         << config_.motor_id.value << " host_id=0x" << config_.host_id.value
         << std::nouppercase << std::dec
         << " channel=" << config_.adapter.channel_index
         << " canfd=" << (config_.adapter.can_fd_enabled ? 1 : 0)
         << " brs=" << (config_.adapter.bitrate_switch_enabled ? 1 : 0)
         << " nominal_bps=" << config_.adapter.nominal_bitrate
         << " data_bps=" << config_.adapter.data_bitrate;
  if (const auto* dm_transport =
          dynamic_cast<const DmUsb2FdcanTransport*>(transport_.get())) {
    header << ' ' << dm_transport->diagnostics();
  }
  lines->push_back(header.str());

  // Keep this probe passive: no enable, no jog, no zeroing. These are the same
  // low-energy requests used by the verified Python UI to distinguish no-bus
  // response from feedback-ID/protocol parsing mismatches.
  common::Result first_error = common::Ok();
  const auto refresh_result = sendProbeFrame(
      "refresh",
      DamiaoProtocol::makeRefreshCommand(ids(), config_.frame_options), lines);
  if (refresh_result.isError() && first_error.isOk()) {
    first_error = refresh_result;
  }
  constexpr std::uint8_t kMasterIdRegister = 0x08;
  const auto master_id_result =
      sendProbeRegisterRead("query_master_id", kMasterIdRegister, lines);
  if (master_id_result.isError() && first_error.isOk()) {
    first_error = master_id_result;
  }
  constexpr std::uint8_t kPmaxRegister = 0x15;
  constexpr std::uint8_t kVmaxRegister = 0x16;
  constexpr std::uint8_t kTmaxRegister = 0x17;
  constexpr std::uint8_t kMotorPositionRegister = 0x50;
  constexpr std::uint8_t kOutputPositionRegister = 0x51;
  const std::pair<const char*, std::uint8_t> registers[] = {
      {"query_pmax", kPmaxRegister},
      {"query_vmax", kVmaxRegister},
      {"query_tmax", kTmaxRegister},
      {"query_p_m", kMotorPositionRegister},
      {"query_xout", kOutputPositionRegister},
  };
  for (const auto& entry : registers) {
    const auto result = sendProbeRegisterRead(entry.first, entry.second, lines);
    if (result.isError() && first_error.isOk()) {
      first_error = result;
    }
  }
  if (connected_) {
    startFeedbackThread();
  }
  if (first_error.isOk()) {
    MotorFeedback ignored{};
    (void)waitForFreshFeedback(std::chrono::steady_clock::now(),
                               std::chrono::milliseconds{250}, &ignored);
  }
  return first_error;
}

bool DamiaoMotor::isConnected() const noexcept {
  return connected_.load();
}

bool DamiaoMotor::isEnabled() const noexcept {
  return enabled_.load();
}

DamiaoIds DamiaoMotor::ids() const noexcept {
  return DamiaoIds{config_.motor_id.value, config_.host_id.value};
}

common::Result DamiaoMotor::requireConnected() const {
  if (!connected_.load() || !transport_ || !transport_->isOpen()) {
    return connectionNotOpen();
  }
  return common::Ok();
}

common::Result DamiaoMotor::write(const CanFrame& frame) {
  std::lock_guard<std::mutex> lock{transport_mutex_};
  return writeUnlocked(frame);
}

common::Result DamiaoMotor::writeUnlocked(const CanFrame& frame) {
  if (!DamiaoProtocol::isValidCanFdFrame(frame)) {
    return common::Result::error(common::ErrorCode::ProtocolError,
                                 "invalid Damiao CAN frame length");
  }
  if (!transport_) {
    return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                 "Damiao motor has no transport");
  }
  return transport_->writeFrame(frame);
}

common::Result DamiaoMotor::switchMode(DamiaoControlMode mode) {
  if (!config_.auto_switch_mode) {
    active_mode_ = mode;
    return common::Ok();
  }
  if (active_mode_ == mode) {
    return common::Ok();
  }
  const auto result = write(
      DamiaoProtocol::makeSetControlModeCommand(ids(), mode,
                                                config_.frame_options));
  if (result.isError()) {
    return result;
  }
  active_mode_ = mode;
  std::this_thread::sleep_for(std::chrono::milliseconds{2});
  return common::Ok();
}

common::Result DamiaoMotor::readRegisterUnlocked(
    std::uint8_t register_id, DamiaoRegisterValue* value,
    std::chrono::milliseconds timeout) {
  if (value == nullptr) {
    return common::Result::error(common::ErrorCode::InvalidArgument,
                                 "register value output pointer is null");
  }
  const auto write_result = writeUnlocked(DamiaoProtocol::makeReadRegisterCommand(
      ids(), register_id, config_.frame_options));
  if (write_result.isError()) {
    return write_result;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::vector<std::string> ignored_frames{};
  while (std::chrono::steady_clock::now() < deadline) {
    CanFrame frame{};
    const auto read_result =
        transport_->readFrameFor(&frame, std::chrono::milliseconds{5});
    if (read_result.isError()) {
      if (read_result.code() == common::ErrorCode::FeedbackTimedOut) {
        continue;
      }
      return read_result;
    }

    DamiaoRegisterValue parsed{};
    if (DamiaoProtocol::parseRegisterResponse(frame, ids(), &parsed) &&
        parsed.register_id == register_id) {
      *value = parsed;
      return common::Ok();
    }
    if (ignored_frames.size() < 4U) {
      ignored_frames.push_back(rawFrameSummary(frame));
    }
  }

  std::ostringstream message;
  message << "Damiao register read timed out"
          << " rid=0x" << std::hex << std::uppercase
          << static_cast<int>(register_id) << std::nouppercase << std::dec;
  for (const auto& line : ignored_frames) {
    message << " | ignored " << line;
  }
  return common::Result::error(common::ErrorCode::FeedbackTimedOut,
                               message.str());
}

common::Result DamiaoMotor::updateRuntimeLimitsFromRegisters() {
  constexpr std::uint8_t kPmaxRegister = 0x15;
  constexpr std::uint8_t kVmaxRegister = 0x16;
  constexpr std::uint8_t kTmaxRegister = 0x17;
  DamiaoRegisterValue pmax{};
  DamiaoRegisterValue vmax{};
  DamiaoRegisterValue tmax{};
  auto result = readRegisterUnlocked(kPmaxRegister, &pmax,
                                     std::chrono::milliseconds{250});
  if (result.isError()) {
    return result;
  }
  result = readRegisterUnlocked(kVmaxRegister, &vmax,
                                std::chrono::milliseconds{250});
  if (result.isError()) {
    return result;
  }
  result = readRegisterUnlocked(kTmaxRegister, &tmax,
                                std::chrono::milliseconds{250});
  if (result.isError()) {
    return result;
  }

  if (pmax.value <= 0.0 || vmax.value <= 0.0 || tmax.value <= 0.0) {
    std::ostringstream message;
    message << "invalid Damiao runtime mapping limits"
            << " pmax=" << pmax.value << " vmax=" << vmax.value
            << " tmax=" << tmax.value;
    return common::Result::error(common::ErrorCode::HardwareFeedbackInvalid,
                                 message.str());
  }

  config_.limits.position = common::Rad{pmax.value};
  config_.limits.velocity = common::RadPerS{vmax.value};
  config_.limits.torque = common::Nm{tmax.value};
  resetContinuousEncoder();
  return common::Ok();
}

common::Result DamiaoMotor::refreshFeedback() {
  std::lock_guard<std::mutex> transport_lock{transport_mutex_};
  const auto refresh_result =
      writeUnlocked(DamiaoProtocol::makeRefreshCommand(ids(), config_.frame_options));
  if (refresh_result.isError()) {
    return refresh_result;
  }

  DamiaoFeedback parsed{};
  MotorFeedback newest_feedback{};
  common::Result newest_result = common::Result::error(
      common::ErrorCode::FeedbackTimedOut, "no matching Damiao motor feedback");
  std::chrono::steady_clock::time_point newest_time{};
  std::vector<std::string> ignored_frames{};
  const auto deadline = std::chrono::steady_clock::now() +
                        std::max(std::chrono::milliseconds{5},
                                 secondsToMilliseconds(
                                     config_.feedback_poll_period, 0.02));
  bool received_any = false;
  common::Result last_read_result = common::Result::error(
      common::ErrorCode::FeedbackTimedOut, "no feedback read attempted");
  while (std::chrono::steady_clock::now() < deadline) {
    CanFrame frame{};
    last_read_result =
        transport_->readFrameFor(&frame, std::chrono::milliseconds{2});
    if (last_read_result.isError()) {
      if (last_read_result.code() == common::ErrorCode::FeedbackTimedOut) {
        if (newest_result.isOk()) {
          break;
        }
        continue;
      }
      return last_read_result;
    }
    received_any = true;
    if (DamiaoProtocol::parseFeedback(frame, ids(), config_.limits, &parsed)) {
      const common::Rad continuous_position =
          virtual_encoder_.update(encoderWrappedPosition(parsed),
                                  parsed.position)
              .continuous_position;
      if (parsed.fault) {
        enabled_ = false;
      }
      const auto now = std::chrono::steady_clock::now();
      newest_feedback.position = continuous_position;
      newest_feedback.wrapped_position = parsed.position;
      newest_feedback.wrapped_position_valid = true;
      newest_feedback.velocity = parsed.velocity;
      newest_feedback.torque = parsed.torque;
      newest_feedback.current = torqueToCurrent(parsed.torque);
      newest_feedback.raw_position_counts = parsed.raw_position_counts;
      newest_feedback.raw_position_counts_valid = true;
      newest_feedback.raw_feedback_frame_id = frame.id;
      newest_feedback.raw_feedback_frame_length = frame.data_length;
      newest_feedback.raw_feedback_frame_data = frame.data;
      newest_feedback.raw_feedback_frame_valid = true;
      newest_feedback.runtime_position_limit = config_.limits.position;
      newest_feedback.runtime_velocity_limit = config_.limits.velocity;
      newest_feedback.runtime_torque_limit = config_.limits.torque;
      newest_feedback.runtime_limits_valid =
          config_.limits.position.value > 0.0 &&
          config_.limits.velocity.value > 0.0 &&
          config_.limits.torque.value > 0.0;
      newest_feedback.temperature =
          common::DegC{std::max(parsed.mos_temperature.value,
                                parsed.rotor_temperature.value)};
      newest_feedback.enabled = parsed.enabled;
      newest_feedback.fault = parsed.fault;
      newest_feedback.timestamp = steadyTimestamp(now);
      newest_time = now;
      newest_result = common::Ok();
      continue;
    }
    if (ignored_frames.size() < 6U) {
      ignored_frames.push_back(rawFrameSummary(frame));
    }
  }

  if (newest_result.isOk()) {
    std::lock_guard<std::mutex> lock{feedback_mutex_};
    last_feedback_ = newest_feedback;
    last_feedback_valid_ = true;
    last_feedback_result_ = common::Ok();
    last_feedback_steady_time_ = newest_time;
    return common::Ok();
  }

  if (!received_any) {
    return last_read_result.isError()
               ? last_read_result
               : common::Result::error(
                     common::ErrorCode::FeedbackTimedOut,
                     "no Damiao CAN frame received before timeout");
  }

  std::ostringstream message;
  message << "received CAN frames but no matching Damiao motor feedback";
  for (const auto& line : ignored_frames) {
    message << " | ignored " << line;
  }
  return common::Result::error(common::ErrorCode::HardwareFeedbackInvalid,
                               message.str());
}

common::Result DamiaoMotor::readLatestFeedback(MotorFeedback* feedback) const {
  if (feedback == nullptr) {
    return common::Result::error(common::ErrorCode::InvalidArgument,
                                 "feedback output pointer is null");
  }
  std::lock_guard<std::mutex> lock{feedback_mutex_};
  if (!last_feedback_valid_) {
    return last_feedback_result_;
  }

  const auto now = std::chrono::steady_clock::now();
  const double age_s =
      std::chrono::duration<double>{now - last_feedback_steady_time_}.count();
  const double stale_timeout_s =
      positiveOrDefault(config_.feedback_stale_timeout.value, 0.2);
  if (age_s > stale_timeout_s) {
    std::ostringstream message;
    message << "motor feedback is stale"
            << " age_s=" << age_s
            << " stale_timeout_s=" << stale_timeout_s;
    if (last_feedback_result_.isError() &&
        last_feedback_result_.hasMessage()) {
      message << " last_error=" << last_feedback_result_.message();
    }
    return common::Result::error(common::ErrorCode::FeedbackTimedOut,
                                 message.str());
  }

  *feedback = last_feedback_;
  return common::Ok();
}

common::Result DamiaoMotor::waitForFreshFeedback(
    std::chrono::steady_clock::time_point min_time,
    std::chrono::milliseconds timeout, MotorFeedback* feedback) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  common::Result last_error = common::Result::error(
      common::ErrorCode::FeedbackTimedOut, "fresh motor feedback not received");
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock{feedback_mutex_};
      if (last_feedback_valid_ && last_feedback_steady_time_ >= min_time) {
        if (feedback != nullptr) {
          *feedback = last_feedback_;
        }
        return common::Ok();
      }
      last_error = last_feedback_result_;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
  }

  std::ostringstream message;
  message << "fresh motor feedback not received before timeout";
  if (last_error.isError() && last_error.hasMessage()) {
    message << " last_error=" << last_error.message();
  }
  return common::Result::error(common::ErrorCode::FeedbackTimedOut,
                               message.str());
}

void DamiaoMotor::startFeedbackThread() {
  if (feedback_thread_running_.exchange(true)) {
    return;
  }
  feedback_thread_ = std::thread{[this]() { feedbackLoop(); }};
}

void DamiaoMotor::stopFeedbackThread() {
  feedback_thread_running_ = false;
  if (feedback_thread_.joinable()) {
    feedback_thread_.join();
  }
}

void DamiaoMotor::feedbackLoop() {
  const auto poll_period =
      std::max(std::chrono::milliseconds{5},
               secondsToMilliseconds(config_.feedback_poll_period, 0.02));
  while (feedback_thread_running_.load()) {
    const auto cycle_start = std::chrono::steady_clock::now();
    if (!connected_.load() || !transport_ || !transport_->isOpen()) {
      std::lock_guard<std::mutex> lock{feedback_mutex_};
      last_feedback_result_ = connectionNotOpen();
    } else {
      const auto result = refreshFeedback();
      if (result.isError()) {
        std::lock_guard<std::mutex> lock{feedback_mutex_};
        last_feedback_result_ = result;
      }
    }
    const auto wake_time = cycle_start + poll_period;
    while (feedback_thread_running_.load() &&
           std::chrono::steady_clock::now() < wake_time) {
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
  }
}

void DamiaoMotor::resetContinuousEncoder() noexcept {
  virtual_encoder_.setConfig(VirtualEncoderConfig{
      encoderWrapRange(config_),
      config_.continuous_encoder_enabled});
}

common::Rad DamiaoMotor::encoderWrappedPosition(
    const DamiaoFeedback& feedback) const noexcept {
  if (config_.encoder_unwrap_source != EncoderUnwrapSource::RawPositionCounts) {
    return feedback.position;
  }

  const std::uint32_t count_range =
      config_.encoder_raw_count_range > 0U ? config_.encoder_raw_count_range
                                           : 65536U;
  const double normalized =
      static_cast<double>(feedback.raw_position_counts) /
      static_cast<double>(count_range);
  return common::Rad{normalized * encoderWrapRange(config_).value};
}

common::Result DamiaoMotor::sendProbeFrame(
    const char* name, const CanFrame& frame, std::vector<std::string>* lines) {
  appendTxLine(name, frame, lines);
  std::lock_guard<std::mutex> transport_lock{transport_mutex_};
  const auto write_result = writeUnlocked(frame);
  if (write_result.isError()) {
    lines->push_back(std::string{"TX failed: "} + write_result.message());
    return write_result;
  }

  bool received = false;
  common::Result last_read_result = common::Ok();
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds{200};
  while (std::chrono::steady_clock::now() < deadline) {
    CanFrame rx{};
    last_read_result =
        transport_->readFrameFor(&rx, std::chrono::milliseconds{20});
    if (last_read_result.isError()) {
      if (last_read_result.code() == common::ErrorCode::FeedbackTimedOut) {
        continue;
      }
      lines->push_back(std::string{"RX failed: "} + last_read_result.message());
      return last_read_result;
    }
    received = true;
    appendRxLine(rx, lines);
  }

  if (!received) {
    lines->push_back("RX none");
    return common::Result::error(common::ErrorCode::FeedbackTimedOut,
                                 "communication probe received no CAN frames");
  }
  return common::Ok();
}

common::Result DamiaoMotor::sendProbeRegisterRead(
    const char* name, std::uint8_t register_id, std::vector<std::string>* lines) {
  const CanFrame frame =
      DamiaoProtocol::makeReadRegisterCommand(ids(), register_id,
                                              config_.frame_options);
  appendTxLine(name, frame, lines);
  std::lock_guard<std::mutex> transport_lock{transport_mutex_};
  const auto write_result = writeUnlocked(frame);
  if (write_result.isError()) {
    lines->push_back(std::string{"TX failed: "} + write_result.message());
    return write_result;
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds{250};
  while (std::chrono::steady_clock::now() < deadline) {
    CanFrame rx{};
    const auto read_result =
        transport_->readFrameFor(&rx, std::chrono::milliseconds{20});
    if (read_result.isError()) {
      if (read_result.code() == common::ErrorCode::FeedbackTimedOut) {
        continue;
      }
      lines->push_back(std::string{"RX failed: "} + read_result.message());
      return read_result;
    }
    appendRxLine(rx, lines);
    DamiaoRegisterValue parsed{};
    if (DamiaoProtocol::parseRegisterResponse(rx, ids(), &parsed) &&
        parsed.register_id == register_id) {
      std::ostringstream decoded;
      decoded << "REG " << name << " rid=0x" << std::hex << std::uppercase
              << static_cast<int>(register_id) << std::nouppercase << std::dec
              << " value=" << parsed.value
              << " type=" << (parsed.is_integer ? "uint32" : "float");
      lines->push_back(decoded.str());
      return common::Ok();
    }
  }

  std::ostringstream message;
  message << "register probe timed out rid=0x" << std::hex << std::uppercase
          << static_cast<int>(register_id) << std::nouppercase << std::dec;
  lines->push_back(message.str());
  return common::Result::error(common::ErrorCode::FeedbackTimedOut,
                               message.str());
}

common::A DamiaoMotor::torqueToCurrent(common::Nm torque) const noexcept {
  const double torque_per_amp =
      positiveOrDefault(config_.torque_per_amp.value, 0.625);
  return common::A{torque.value / torque_per_amp};
}

}  // namespace gripper::hardware_interface::damiao
