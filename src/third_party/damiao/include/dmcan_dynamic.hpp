#pragma once

#include "dmcan.h"

namespace gripper::third_party::damiao {

struct DmcanApi {
  using ContextCreate = void (*)(dmcan_context** ctx);
  using ContextDestroy = void (*)(dmcan_context* ctx);
  using GetSdkVersion = void (*)(dmcan_context* ctx, std::uint32_t* version);
  using FindDevices = int (*)(dmcan_context* ctx);
  using FindDevicesWithType =
      int (*)(dmcan_context* ctx, dmcan_device_type_t type);
  using DeviceGet =
      bool (*)(dmcan_context* ctx, dmcan_device_handle** dev_handle, int index);
  using DeviceOpen = bool (*)(dmcan_device_handle* dev_handle);
  using DeviceClose = void (*)(dmcan_device_handle* dev_handle);
  using DeviceGetVersion =
      void (*)(dmcan_device_handle* dev_handle, char* version_buf,
               std::size_t buf_size);
  using DeviceEnableChannel =
      bool (*)(dmcan_device_handle* dev_handle, std::uint8_t channel);
  using DeviceDisableChannel =
      bool (*)(dmcan_device_handle* dev_handle, std::uint8_t channel);
  using DeviceGetChannelBaudrate =
      bool (*)(dmcan_device_handle* dev_handle, std::uint8_t channel,
               dmcan_channel_can_info_t* baud_info);
  using DeviceGetChannelBaudrateDetails =
      bool (*)(dmcan_device_handle* dev_handle, std::uint8_t channel,
               dmcan_ch_can_config_t* config);
  using DeviceSetChannelBaudrate =
      bool (*)(dmcan_device_handle* dev_handle, std::uint8_t channel,
               dmcan_channel_can_info_t baud_info);
  using HookRecv =
      void (*)(dmcan_device_handle* dev_handle, dev_recv_callback callback);
  using HookSent =
      void (*)(dmcan_device_handle* dev_handle, dev_sent_callback callback);
  using HookErr =
      void (*)(dmcan_device_handle* dev_handle, dev_err_callback callback);
  using SendCan = bool (*)(dmcan_device_handle* dev_handle, std::uint8_t ch,
                           std::uint32_t can_id, bool canfd, bool ext, bool rtr,
                           bool brs, std::uint8_t dlen, std::uint8_t* payload);
  using GetLenFromDlc = int (*)(int dlc);

  ContextCreate context_create{};
  ContextDestroy context_destroy{};
  GetSdkVersion get_sdk_version{};
  FindDevices find_devices{};
  FindDevicesWithType find_devices_with_type{};
  DeviceGet device_get{};
  DeviceOpen device_open{};
  DeviceClose device_close{};
  DeviceGetVersion device_get_version{};
  DeviceEnableChannel device_enable_channel{};
  DeviceDisableChannel device_disable_channel{};
  DeviceGetChannelBaudrate device_get_channel_baudrate{};
  DeviceGetChannelBaudrateDetails device_get_channel_baudrate_details{};
  DeviceSetChannelBaudrate device_set_channel_baudrate{};
  HookRecv hook_recv{};
  HookSent hook_sent{};
  HookErr hook_err{};
  SendCan send_can{};
  GetLenFromDlc get_len_from_dlc{};
};

}  // namespace gripper::third_party::damiao
