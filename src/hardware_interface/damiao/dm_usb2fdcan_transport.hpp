#pragma once

#include <chrono>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <cstdint>

#include "hardware_interface/transport_interface.hpp"
#include "third_party/damiao/include/dmcan.h"
#include "third_party/damiao/include/dmcan_dynamic.hpp"

namespace gripper::hardware_interface::damiao {

// Transport for DM-USB2FDCAN_Dual.
//
// The implementation dynamically loads libdm_device.dll so the project can
// still build without a vendor import library. SDK handles and callbacks remain
// hidden behind the project-level TransportInterface.
class DmUsb2FdcanTransport final : public TransportInterface {
 public:
  DmUsb2FdcanTransport();
  ~DmUsb2FdcanTransport() override;

  [[nodiscard]] common::Result open(const AdapterConfig& config) override;
  [[nodiscard]] common::Result close() override;
  [[nodiscard]] common::Result writeFrame(const CanFrame& frame) override;
  [[nodiscard]] common::Result readFrame(CanFrame* frame) override;
  [[nodiscard]] common::Result readFrameFor(
      CanFrame* frame, std::chrono::milliseconds timeout) override;
  void clearRxQueue() override;
  [[nodiscard]] bool isOpen() const noexcept override;
  [[nodiscard]] std::string diagnostics() const;

 private:
  struct LibraryHandle;

  [[nodiscard]] common::Result loadSdk();
  [[nodiscard]] common::Result configureChannel();
  [[nodiscard]] common::Result popFrame(CanFrame* frame,
                                        std::chrono::milliseconds timeout);
  void pushReceivedFrame(const usb_rx_frame_t& frame);

  static void onReceive(dmcan_device_handle* handle, usb_rx_frame_t* frame);
  static void onSent(dmcan_device_handle* handle, usb_rx_frame_t* frame);
  static void onError(dmcan_device_handle* handle, usb_rx_frame_t* frame);

  AdapterConfig config_{};
  third_party::damiao::DmcanApi api_{};
  std::unique_ptr<LibraryHandle> library_{};
  dmcan_context* context_{nullptr};
  dmcan_device_handle* device_{nullptr};
  std::deque<CanFrame> rx_queue_{};
  mutable std::mutex rx_mutex_{};
  std::uint64_t tx_count_{0};
  std::uint64_t rx_count_{0};
  std::uint32_t last_tx_id_{0};
  std::uint32_t last_rx_id_{0};
  std::uint8_t last_rx_length_{0};
  int last_device_count_{0};
  bool used_type_filtered_discovery_{false};
  std::uint32_t sdk_version_{0};
  std::string device_version_{};
  dmcan_channel_can_info_t channel_baudrate_{};
  dmcan_ch_can_config_t channel_baudrate_details_{};
  bool channel_baudrate_valid_{false};
  bool channel_baudrate_details_valid_{false};
  std::chrono::milliseconds read_timeout_{100};
  bool open_{false};
};

}  // namespace gripper::hardware_interface::damiao
