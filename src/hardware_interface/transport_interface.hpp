#pragma once

#include <chrono>

#include "common/result.hpp"
#include "hardware_interface/adapter_types.hpp"
#include "hardware_interface/can_frame.hpp"

namespace gripper::hardware_interface {

// Transport boundary for CAN/CAN-FD adapters. Implementations own adapter SDK
// handles internally and expose only project-level result and frame types.
class TransportInterface {
 public:
  virtual ~TransportInterface() = default;

  // Opens the adapter using config. Fails if the adapter, channel, or bit-rate
  // settings are unavailable or unsupported.
  [[nodiscard]] virtual common::Result open(
      const AdapterConfig& config) = 0;

  // Closes the adapter. Fails if the adapter reports a shutdown error.
  [[nodiscard]] virtual common::Result close() = 0;

  // Writes one frame. Fails when the adapter is closed or the frame length is
  // invalid for its payload mode.
  [[nodiscard]] virtual common::Result writeFrame(
      const CanFrame& frame) = 0;

  // Reads one frame into frame. Fails when the adapter is closed, frame is null,
  // or no frame arrives before the implementation-defined timeout.
  [[nodiscard]] virtual common::Result readFrame(CanFrame* frame) = 0;

  // Reads one frame with a caller-selected timeout. Implementations may
  // override this for short raw probes; the default keeps existing transports
  // compatible by delegating to readFrame().
  [[nodiscard]] virtual common::Result readFrameFor(
      CanFrame* frame, std::chrono::milliseconds timeout) {
    (void)timeout;
    return readFrame(frame);
  }

  // Drops queued receive frames that may belong to an earlier command. Real
  // hardware implementations should override this when callbacks buffer frames.
  virtual void clearRxQueue() {}

  [[nodiscard]] virtual bool isOpen() const noexcept = 0;
};

}  // namespace gripper::hardware_interface
