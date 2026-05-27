#include "hardware_interface/damiao/dm_usb2fdcan_transport.hpp"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <thread>

#include "common/error_code.hpp"

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace gripper::hardware_interface::damiao {
namespace {

DmUsb2FdcanTransport* g_active_transport = nullptr;

#if defined(_WIN32)
constexpr const char* kDefaultDllPath =
    "src/third_party/damiao/bin/libdm_device.dll";

template <typename Fn>
[[nodiscard]] Fn loadSymbol(HMODULE module, const char* name) {
  return reinterpret_cast<Fn>(GetProcAddress(module, name));
}

[[nodiscard]] std::string win32LoadErrorMessage(const std::string& path,
                                                DWORD error_code) {
  return std::string{"failed to load "} + path +
         " (Win32 error " + std::to_string(error_code) + ")";
}
#endif

[[nodiscard]] std::uint8_t frameLength(const usb_rx_frame_t& frame) {
  if (frame.head.dlc <= 8U) {
    return frame.head.dlc;
  }
  // CAN-FD DLC mapping. Only callback conversion uses this fallback; outbound
  // frame validation still uses byte lengths directly.
  switch (frame.head.dlc) {
    case 9:
      return 12;
    case 10:
      return 16;
    case 11:
      return 20;
    case 12:
      return 24;
    case 13:
      return 32;
    case 14:
      return 48;
    case 15:
      return 64;
    default:
      return 0;
  }
}

}  // namespace

struct DmUsb2FdcanTransport::LibraryHandle {
#if defined(_WIN32)
  HMODULE module{nullptr};
  std::vector<HMODULE> dependencies{};
  ~LibraryHandle() {
    if (module != nullptr) {
      FreeLibrary(module);
      module = nullptr;
    }
    for (auto iter = dependencies.rbegin(); iter != dependencies.rend(); ++iter) {
      if (*iter != nullptr) {
        FreeLibrary(*iter);
      }
    }
    dependencies.clear();
  }
#endif
};

DmUsb2FdcanTransport::DmUsb2FdcanTransport() = default;

DmUsb2FdcanTransport::~DmUsb2FdcanTransport() {
  (void)close();
}

common::Result DmUsb2FdcanTransport::open(const AdapterConfig& config) {
  config_ = config;
  if (open_) {
    return common::Ok();
  }

  auto result = loadSdk();
  if (result.isError()) {
    return result;
  }

  api_.context_create(&context_);
  if (context_ == nullptr) {
    return common::Result::error(common::ErrorCode::HardwareUnavailable,
                                 "dmcan_context_create returned null");
  }
  sdk_version_ = 0;
  if (api_.get_sdk_version != nullptr) {
    api_.get_sdk_version(context_, &sdk_version_);
  }

  used_type_filtered_discovery_ = false;
  int count = 0;
  if (api_.find_devices != nullptr) {
    count = api_.find_devices(context_);
  } else {
    used_type_filtered_discovery_ = true;
    count = api_.find_devices_with_type(context_, USB2CANFD_DUAL);
  }
  last_device_count_ = count;
  if (count <= 0) {
    api_.context_destroy(context_);
    context_ = nullptr;
    return common::Result::error(common::ErrorCode::HardwareUnavailable,
                                 "no DM-USB2FDCAN_Dual device found");
  }

  if (!api_.device_get(context_, &device_,
                       static_cast<int>(config_.device_index)) ||
      device_ == nullptr) {
    api_.context_destroy(context_);
    context_ = nullptr;
    return common::Result::error(common::ErrorCode::HardwareUnavailable,
                                 "failed to get DM-USB2FDCAN_Dual handle");
  }

  if (!api_.device_open(device_)) {
    api_.context_destroy(context_);
    context_ = nullptr;
    device_ = nullptr;
    return common::Result::error(common::ErrorCode::ConnectionFailed,
                                 "failed to open DM-USB2FDCAN_Dual");
  }
  device_version_.clear();
  if (api_.device_get_version != nullptr) {
    char version_buf[128]{};
    api_.device_get_version(device_, version_buf, sizeof(version_buf));
    device_version_ = version_buf;
  }

  g_active_transport = this;
  api_.hook_recv(device_, &DmUsb2FdcanTransport::onReceive);
  result = configureChannel();
  if (result.isError()) {
    (void)close();
    return result;
  }

  open_ = true;
  return common::Ok();
}

common::Result DmUsb2FdcanTransport::close() {
  if (g_active_transport == this) {
    g_active_transport = nullptr;
  }
  {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    rx_queue_.clear();
  }
  // The current Damiao Windows SDK can hang in device_close /
  // device_disable_channel after USB transfer cancellation on this test PC.
  // During hardware bring-up we prioritize a responsive operator tool; the OS
  // releases the adapter when the process exits. Strict SDK shutdown remains a
  // separate compatibility task after command/feedback validation.
  context_ = nullptr;
  device_ = nullptr;
  open_ = false;
  return common::Ok();
}

common::Result DmUsb2FdcanTransport::writeFrame(const CanFrame& frame) {
  if (!open_ || device_ == nullptr) {
    return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                 "DM transport is not open");
  }
  if (frame.payload_mode == CanPayloadMode::ClassicCan &&
      frame.data_length > 8U) {
    return common::Result::error(common::ErrorCode::InvalidArgument,
                                 "classic CAN frame length exceeds 8 bytes");
  }
  if (frame.data_length > 64U) {
    return common::Result::error(common::ErrorCode::InvalidArgument,
                                 "CAN-FD frame length exceeds 64 bytes");
  }
  std::uint8_t payload[64]{};
  std::copy(frame.data.begin(), frame.data.begin() + frame.data_length,
            payload);

  const bool ok = api_.send_can(
      device_, static_cast<std::uint8_t>(config_.channel_index), frame.id,
      frame.payload_mode == CanPayloadMode::CanFd,
      frame.id_format == CanIdFormat::Extended29Bit, false,
      frame.bitrate_switch, frame.data_length, payload);
  if (!ok) {
    return common::Result::error(common::ErrorCode::HardwareRejectedCommand,
                                 "dmcan_device_send_can failed");
  }
  ++tx_count_;
  last_tx_id_ = frame.id;
  return common::Ok();
}

common::Result DmUsb2FdcanTransport::readFrame(CanFrame* frame) {
  return readFrameFor(frame, read_timeout_);
}

common::Result DmUsb2FdcanTransport::readFrameFor(
    CanFrame* frame, std::chrono::milliseconds timeout) {
  if (frame == nullptr) {
    return common::Result::error(common::ErrorCode::InvalidArgument,
                                 "CAN frame output pointer is null");
  }
  if (!open_) {
    return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                 "DM transport is not open");
  }
  return popFrame(frame, timeout);
}

void DmUsb2FdcanTransport::clearRxQueue() {
  std::lock_guard<std::mutex> lock(rx_mutex_);
  rx_queue_.clear();
}

bool DmUsb2FdcanTransport::isOpen() const noexcept {
  return open_;
}

std::string DmUsb2FdcanTransport::diagnostics() const {
  std::ostringstream stream;
  stream << "sdk_count=" << last_device_count_
         << " discovery="
         << (used_type_filtered_discovery_ ? "type-filtered" : "all-devices")
         << " sdk_version_raw=0x" << std::hex << sdk_version_ << std::dec
         << " device_version="
         << (device_version_.empty() ? "unknown" : device_version_)
         << " tx_count=" << tx_count_ << " rx_count=" << rx_count_;
  if (channel_baudrate_valid_) {
    stream << " channel_readback=ok"
           << " rb_ch=" << static_cast<int>(channel_baudrate_.channel)
           << " rb_canfd=" << (channel_baudrate_.canfd ? 1 : 0)
           << " rb_nominal_bps=" << channel_baudrate_.can_baudrate
           << " rb_data_bps=" << channel_baudrate_.canfd_baudrate
           << " rb_can_sp=" << std::fixed << std::setprecision(2)
           << channel_baudrate_.can_sp
           << " rb_canfd_sp=" << channel_baudrate_.canfd_sp
           << std::defaultfloat;
  } else {
    stream << " channel_readback=unavailable";
  }
  if (channel_baudrate_details_valid_) {
    stream << " rb_fd_details="
           << static_cast<int>(channel_baudrate_details_.canfd_seg1) << '/'
           << static_cast<int>(channel_baudrate_details_.canfd_seg2) << '/'
           << static_cast<int>(channel_baudrate_details_.canfd_sjw) << '/'
           << static_cast<int>(channel_baudrate_details_.canfd_prescaler);
  }
  return stream.str();
}

common::Result DmUsb2FdcanTransport::loadSdk() {
#if !defined(_WIN32)
  return common::Result::error(common::ErrorCode::Unsupported,
                               "DM DeviceSDK dynamic loading is Windows-only");
#else
  if (library_ && library_->module != nullptr) {
    return common::Ok();
  }
  library_ = std::make_unique<LibraryHandle>();
  const std::string configured_path = config_.driver_library_path.empty()
                                          ? kDefaultDllPath
                                          : config_.driver_library_path;
  const std::string absolute_path =
      std::filesystem::absolute(configured_path).string();
  const auto dll_directory = std::filesystem::path{absolute_path}.parent_path();

  const char* dependency_names[] = {
      "VCRUNTIME140.dll",   "VCRUNTIME140_1.dll", "libwinpthread-1.dll",
      "libgcc_s_seh-1.dll", "libstdc++-6.dll",    "libusb-1.0.dll"};
  for (const char* dependency_name : dependency_names) {
    const auto dependency_path = dll_directory / dependency_name;
    if (!std::filesystem::exists(dependency_path)) {
      continue;
    }
    HMODULE dependency = LoadLibraryA(dependency_path.string().c_str());
    if (dependency == nullptr) {
      return common::Result::error(
          common::ErrorCode::HardwareUnavailable,
          win32LoadErrorMessage(dependency_path.string(), GetLastError()));
    }
    library_->dependencies.push_back(dependency);
  }
  library_->module = LoadLibraryA(absolute_path.c_str());
  const DWORD load_error = GetLastError();
  if (library_->module == nullptr) {
    return common::Result::error(
        common::ErrorCode::HardwareUnavailable,
        win32LoadErrorMessage(absolute_path, load_error));
  }

  api_.context_create =
      loadSymbol<third_party::damiao::DmcanApi::ContextCreate>(
          library_->module, "dmcan_context_create");
  api_.context_destroy =
      loadSymbol<third_party::damiao::DmcanApi::ContextDestroy>(
          library_->module, "dmcan_context_destroy");
  api_.get_sdk_version =
      loadSymbol<third_party::damiao::DmcanApi::GetSdkVersion>(
          library_->module, "dmcan_get_sdk_version");
  api_.find_devices = loadSymbol<third_party::damiao::DmcanApi::FindDevices>(
      library_->module, "dmcan_find_devices");
  api_.find_devices_with_type =
      loadSymbol<third_party::damiao::DmcanApi::FindDevicesWithType>(
          library_->module, "dmcan_find_devices_with_type");
  api_.device_get = loadSymbol<third_party::damiao::DmcanApi::DeviceGet>(
      library_->module, "dmcan_device_get");
  api_.device_open = loadSymbol<third_party::damiao::DmcanApi::DeviceOpen>(
      library_->module, "dmcan_device_open");
  api_.device_close = loadSymbol<third_party::damiao::DmcanApi::DeviceClose>(
      library_->module, "dmcan_device_close");
  api_.device_get_version =
      loadSymbol<third_party::damiao::DmcanApi::DeviceGetVersion>(
          library_->module, "dmcan_device_get_version");
  api_.device_enable_channel =
      loadSymbol<third_party::damiao::DmcanApi::DeviceEnableChannel>(
          library_->module, "dmcan_device_enable_channel");
  api_.device_disable_channel =
      loadSymbol<third_party::damiao::DmcanApi::DeviceDisableChannel>(
          library_->module, "dmcan_device_disable_channel");
  api_.device_get_channel_baudrate =
      loadSymbol<third_party::damiao::DmcanApi::DeviceGetChannelBaudrate>(
          library_->module, "dmcan_device_get_channel_baudrate");
  api_.device_get_channel_baudrate_details =
      loadSymbol<
          third_party::damiao::DmcanApi::DeviceGetChannelBaudrateDetails>(
          library_->module, "dmcan_device_get_channel_baudrate_details");
  api_.device_set_channel_baudrate =
      loadSymbol<third_party::damiao::DmcanApi::DeviceSetChannelBaudrate>(
          library_->module, "dmcan_device_set_channel_baudrate");
  api_.hook_recv = loadSymbol<third_party::damiao::DmcanApi::HookRecv>(
      library_->module, "dmcan_device_hook_recv_callback");
  api_.hook_sent = loadSymbol<third_party::damiao::DmcanApi::HookSent>(
      library_->module, "dmcan_device_hook_sent_callback");
  api_.hook_err = loadSymbol<third_party::damiao::DmcanApi::HookErr>(
      library_->module, "dmcan_device_hook_err_callback");
  api_.send_can = loadSymbol<third_party::damiao::DmcanApi::SendCan>(
      library_->module, "dmcan_device_send_can");
  api_.get_len_from_dlc =
      loadSymbol<third_party::damiao::DmcanApi::GetLenFromDlc>(
          library_->module, "dmcan_utils_get_len_from_dlc");

  if (!api_.context_create || !api_.context_destroy ||
      (!api_.find_devices && !api_.find_devices_with_type) ||
      !api_.device_get || !api_.device_open ||
      !api_.device_close || !api_.device_enable_channel ||
      !api_.device_disable_channel || !api_.device_set_channel_baudrate ||
      !api_.hook_recv || !api_.hook_sent || !api_.hook_err ||
      !api_.send_can) {
    library_.reset();
    return common::Result::error(common::ErrorCode::HardwareUnavailable,
                                 "libdm_device.dll is missing required symbols");
  }
  return common::Ok();
#endif
}

common::Result DmUsb2FdcanTransport::configureChannel() {
  dmcan_channel_can_info_t info{};
  info.channel = static_cast<std::uint8_t>(config_.channel_index);
  info.canfd = config_.can_fd_enabled;
  info.can_baudrate = config_.nominal_bitrate;
  info.canfd_baudrate = config_.data_bitrate;
  info.can_sp = 0.75F;
  info.canfd_sp = 0.75F;

  if (!api_.device_set_channel_baudrate(device_, info.channel, info)) {
    return common::Result::error(common::ErrorCode::ConfigUnsupported,
                                 "failed to configure DM CAN-FD bitrate");
  }
  channel_baudrate_valid_ = false;
  channel_baudrate_details_valid_ = false;
  if (api_.device_get_channel_baudrate != nullptr) {
    channel_baudrate_ = {};
    channel_baudrate_valid_ =
        api_.device_get_channel_baudrate(device_, info.channel,
                                         &channel_baudrate_);
  }
  if (api_.device_get_channel_baudrate_details != nullptr) {
    channel_baudrate_details_ = {};
    channel_baudrate_details_valid_ =
        api_.device_get_channel_baudrate_details(
            device_, info.channel, &channel_baudrate_details_);
  }
  if (!api_.device_enable_channel(device_, info.channel)) {
    return common::Result::error(common::ErrorCode::ConnectionFailed,
                                 "failed to enable DM CAN-FD channel");
  }
  return common::Ok();
}

common::Result DmUsb2FdcanTransport::popFrame(
    CanFrame* frame, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(rx_mutex_);
      if (!rx_queue_.empty()) {
        *frame = rx_queue_.front();
        rx_queue_.pop_front();
        return common::Ok();
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  std::lock_guard<std::mutex> lock(rx_mutex_);
  std::ostringstream message;
  message << "no DM CAN frame received before timeout"
          << " | sdk_count=" << last_device_count_
          << " discovery="
          << (used_type_filtered_discovery_ ? "type-filtered" : "all-devices")
          << " | tx_count=" << tx_count_ << " last_tx_id=0x" << std::hex
          << last_tx_id_ << std::dec << " | rx_count=" << rx_count_
          << " last_rx_id=0x" << std::hex << last_rx_id_ << std::dec
          << " last_rx_len=" << static_cast<int>(last_rx_length_);
  return common::Result::error(common::ErrorCode::FeedbackTimedOut,
                               message.str());
}

void DmUsb2FdcanTransport::pushReceivedFrame(const usb_rx_frame_t& frame) {
  CanFrame output{};
  output.id = frame.head.can_id;
  output.id_format =
      frame.head.ext ? CanIdFormat::Extended29Bit : CanIdFormat::Standard11Bit;
  output.payload_mode =
      frame.head.canfd ? CanPayloadMode::CanFd : CanPayloadMode::ClassicCan;
  output.bitrate_switch = frame.head.brs != 0;
  output.data_length = frameLength(frame);
  if (output.data_length > output.data.size()) {
    output.data_length = static_cast<std::uint8_t>(output.data.size());
  }
  std::copy(frame.payload, frame.payload + output.data_length,
            output.data.begin());

  std::lock_guard<std::mutex> lock(rx_mutex_);
  ++rx_count_;
  last_rx_id_ = output.id;
  last_rx_length_ = output.data_length;
  constexpr std::size_t max_queue_size = 256;
  if (rx_queue_.size() >= max_queue_size) {
    rx_queue_.pop_front();
  }
  rx_queue_.push_back(output);
}

void DmUsb2FdcanTransport::onReceive(dmcan_device_handle*,
                                     usb_rx_frame_t* frame) {
  if (frame == nullptr || g_active_transport == nullptr) {
    return;
  }
  g_active_transport->pushReceivedFrame(*frame);
}

void DmUsb2FdcanTransport::onSent(dmcan_device_handle*, usb_rx_frame_t*) {}

void DmUsb2FdcanTransport::onError(dmcan_device_handle*, usb_rx_frame_t*) {}

}  // namespace gripper::hardware_interface::damiao
