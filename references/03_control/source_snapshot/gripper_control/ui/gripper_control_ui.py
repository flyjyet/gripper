import csv
import ctypes
import math
import os
import struct
import subprocess
import sys
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from threading import Lock
from tkinter import filedialog, messagebox, ttk
from typing import Any

try:
    import can  # type: ignore
except ImportError:  # pragma: no cover - optional real hardware dependency.
    can = None


STROKE_MIN_MM = 0.0
STROKE_MAX_MM = 16.0
SCREW_LEAD_MM = 2.0
TWO_PI = 2.0 * math.pi


class GripperState:
    DISABLED = "Disabled"
    READY = "Ready"
    HOMING = "HomingOpenStop"
    FRICTION_ID = "FrictionIdentify"
    GUARDED_CLOSING = "GuardedClosing"
    FORCE_BUILD = "ForceBuild"
    UNLOAD = "UnloadBeforeDisable"
    CLAMP_DONE_DISABLED = "ClampDoneDisabled"
    UNLOCK = "UnlockBeforeOpen"
    OPENING = "Opening"
    ANTI_JAM = "AntiJamRelease"
    FAULT = "FaultStop"


@dataclass
class MotorFeedback:
    timestamp_s: float = 0.0
    stroke_mm: float = 0.0
    velocity_mm_s: float = 0.0
    current_A: float = 0.0
    torque_Nm: float = 0.0
    temperature_C: float = 28.0
    enabled: bool = False
    fault: str = ""


@dataclass
class UiLimits:
    close_speed_mm_s: float = 0.30
    open_speed_mm_s: float = 0.80
    homing_speed_mm_s: float = 0.20
    max_current_A: float = 2.00
    homing_current_A: float = 0.35
    unlock_current_A: float = 0.40
    target_force_N: float = 150.0
    contact_current_delta_A: float = 0.18
    stall_current_delta_A: float = 0.35
    stall_speed_threshold_mm_s: float = 0.03
    contact_debounce_s: float = 0.10
    unload_mm: float = 0.05


@dataclass
class SimulatedObject:
    contact_stroke_mm: float = 8.0
    stiffness_A_per_mm: float = 0.80
    name: str = "object"


class MotorBackend:
    def enable(self) -> None:
        raise NotImplementedError

    def disable(self) -> None:
        raise NotImplementedError

    def update(self, dt_s: float) -> MotorFeedback:
        raise NotImplementedError

    def command_velocity(self, velocity_mm_s: float, current_limit_A: float) -> None:
        raise NotImplementedError

    def command_current(self, current_A: float) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def reset_zero(self) -> None:
        raise NotImplementedError

    def probe_can(self) -> list[str]:
        return ["probe not supported by this backend"]

    def drain_trace(self) -> list[str]:
        return []

    def is_zero_homed(self) -> bool:
        return True

    def close(self) -> None:
        self.disable()


class SimulatedMotorBackend(MotorBackend):
    def __init__(self) -> None:
        self.feedback = MotorFeedback(timestamp_s=time.monotonic(), stroke_mm=0.0)
        self._velocity_cmd = 0.0
        self._current_limit = 0.0
        self._current_cmd = 0.0
        self._mode = "disabled"
        self.object = SimulatedObject()
        self.friction_current_close_A = 0.10
        self.friction_current_open_A = 0.08

    def set_object(self, obj: SimulatedObject) -> None:
        self.object = obj

    def enable(self) -> None:
        self.feedback.enabled = True
        self.feedback.fault = ""
        self._mode = "velocity"
        self._velocity_cmd = 0.0

    def disable(self) -> None:
        self._mode = "disabled"
        self._velocity_cmd = 0.0
        self._current_cmd = 0.0
        self.feedback.enabled = False
        self.feedback.velocity_mm_s = 0.0
        self.feedback.current_A = 0.0
        self.feedback.torque_Nm = 0.0

    def command_velocity(self, velocity_mm_s: float, current_limit_A: float) -> None:
        if not self.feedback.enabled:
            return
        self._mode = "velocity"
        self._velocity_cmd = velocity_mm_s
        self._current_limit = max(0.0, abs(current_limit_A))

    def command_current(self, current_A: float) -> None:
        if not self.feedback.enabled:
            return
        self._mode = "current"
        self._current_cmd = current_A

    def stop(self) -> None:
        self._velocity_cmd = 0.0
        self._current_cmd = 0.0

    def reset_zero(self) -> None:
        self.feedback.stroke_mm = 0.0

    def is_zero_homed(self) -> bool:
        return True

    def update(self, dt_s: float) -> MotorFeedback:
        now = time.monotonic()
        fb = self.feedback
        fb.timestamp_s = now

        if not fb.enabled:
            return fb

        contact_depth = max(0.0, fb.stroke_mm - self.object.contact_stroke_mm)
        contact_current = contact_depth * self.object.stiffness_A_per_mm
        friction = self.friction_current_close_A if self._velocity_cmd >= 0 else self.friction_current_open_A

        if self._mode == "velocity":
            demanded_current = friction + contact_current
            if demanded_current > self._current_limit and self._current_limit > 0:
                actual_velocity = 0.0
                current = self._current_limit
            else:
                actual_velocity = self._velocity_cmd
                current = min(self._current_limit, demanded_current) if self._current_limit > 0 else demanded_current
        elif self._mode == "current":
            current = abs(self._current_cmd)
            force_margin = current - friction - contact_current
            actual_velocity = 0.08 * math.copysign(max(0.0, force_margin), self._current_cmd)
        else:
            actual_velocity = 0.0
            current = 0.0

        next_stroke = fb.stroke_mm + actual_velocity * dt_s
        if next_stroke <= STROKE_MIN_MM:
            next_stroke = STROKE_MIN_MM
            actual_velocity = 0.0
        if next_stroke >= STROKE_MAX_MM:
            next_stroke = STROKE_MAX_MM
            actual_velocity = 0.0

        fb.stroke_mm = next_stroke
        fb.velocity_mm_s = actual_velocity
        fb.current_A = current
        fb.torque_Nm = current * 0.55
        fb.temperature_C += max(0.0, current * current * 0.002 * dt_s - 0.01 * dt_s)
        return fb


class DamiaoProtocol:
    MODE_MIT = 1
    MODE_POSITION_VELOCITY = 2
    MODE_VELOCITY = 3
    MODE_POSITION_FORCE = 4
    REG_CTRL_MODE = 0x0A
    # OpenArm CAN maps DM4310 feedback/MIT ranges to p=12.5 rad, v=30 rad/s, t=10 Nm.
    P_MAX_RAD = 12.5
    V_MAX_RAD_S = 30.0
    T_MAX_NM = 10.0

    @staticmethod
    def enable_frame(control_id: int) -> tuple[int, bytes]:
        return control_id, b"\xff\xff\xff\xff\xff\xff\xff\xfc"

    @staticmethod
    def disable_frame(control_id: int) -> tuple[int, bytes]:
        return control_id, b"\xff\xff\xff\xff\xff\xff\xff\xfd"

    @staticmethod
    def clear_error_frame(control_id: int) -> tuple[int, bytes]:
        return control_id, b"\xff\xff\xff\xff\xff\xff\xff\xfb"

    @staticmethod
    def velocity_frame(motor_id: int, velocity_rad_s: float) -> tuple[int, bytes]:
        return 0x200 + motor_id, struct.pack("<f", velocity_rad_s)

    @staticmethod
    def pos_force_frame(motor_id: int, position_rad: float, velocity_limit_rad_s: float, current_norm: float) -> tuple[int, bytes]:
        v_u = int(max(0, min(10000, round(abs(velocity_limit_rad_s) * 100))))
        i_u = int(max(0, min(10000, round(max(0.0, min(1.0, current_norm)) * 10000))))
        return 0x300 + motor_id, struct.pack("<fHH", position_rad, v_u, i_u)

    @staticmethod
    def mit_torque_frame(motor_id: int, torque_Nm: float) -> tuple[int, bytes]:
        p = DamiaoProtocol.float_to_uint(0.0, -DamiaoProtocol.P_MAX_RAD, DamiaoProtocol.P_MAX_RAD, 16)
        v = DamiaoProtocol.float_to_uint(0.0, -DamiaoProtocol.V_MAX_RAD_S, DamiaoProtocol.V_MAX_RAD_S, 12)
        kp = DamiaoProtocol.float_to_uint(0.0, 0.0, 500.0, 12)
        kd = DamiaoProtocol.float_to_uint(0.0, 0.0, 5.0, 12)
        t = DamiaoProtocol.float_to_uint(torque_Nm, -DamiaoProtocol.T_MAX_NM, DamiaoProtocol.T_MAX_NM, 12)
        data = bytes(
            [
                (p >> 8) & 0xFF,
                p & 0xFF,
                (v >> 4) & 0xFF,
                ((v & 0x0F) << 4) | ((kp >> 8) & 0x0F),
                kp & 0xFF,
                (kd >> 4) & 0xFF,
                ((kd & 0x0F) << 4) | ((t >> 8) & 0x0F),
                t & 0xFF,
            ]
        )
        return motor_id, data

    @staticmethod
    def register_write_u32(motor_id: int, register: int, value: int) -> tuple[int, bytes]:
        return 0x7FF, bytes([motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0x55, register]) + struct.pack("<I", value)

    @staticmethod
    def register_read(motor_id: int, register: int) -> tuple[int, bytes]:
        return 0x7FF, bytes([motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0x33, register, 0, 0, 0, 0])

    @staticmethod
    def refresh_frame(motor_id: int) -> tuple[int, bytes]:
        return 0x7FF, bytes([motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])

    @staticmethod
    def float_to_uint(value: float, low: float, high: float, bits: int) -> int:
        span = high - low
        max_int = (1 << bits) - 1
        value = max(low, min(high, value))
        return int(round((value - low) * max_int / span))

    @staticmethod
    def uint_to_float(value: int, low: float, high: float, bits: int) -> float:
        return float(value) * (high - low) / float((1 << bits) - 1) + low

    @staticmethod
    def parse_feedback(arbitration_id: int, data: bytes, master_id: int) -> dict[str, float | int | bool] | None:
        if arbitration_id != master_id or len(data) < 8:
            return None
        motor_id = data[0] & 0x0F
        err = (data[0] >> 4) & 0x0F
        p_u = (data[1] << 8) | data[2]
        v_u = (data[3] << 4) | (data[4] >> 4)
        t_u = ((data[4] & 0x0F) << 8) | data[5]
        return {
            "motor_id": motor_id,
            "err": err,
            "enabled": err == 1,
            "fault_bits": 0 if err in (0, 1) else 1 << err,
            "position_rad": DamiaoProtocol.uint_to_float(p_u, -DamiaoProtocol.P_MAX_RAD, DamiaoProtocol.P_MAX_RAD, 16),
            "velocity_rad_s": DamiaoProtocol.uint_to_float(v_u, -DamiaoProtocol.V_MAX_RAD_S, DamiaoProtocol.V_MAX_RAD_S, 12),
            "torque_Nm": DamiaoProtocol.uint_to_float(t_u, -DamiaoProtocol.T_MAX_NM, DamiaoProtocol.T_MAX_NM, 12),
            "mos_temp_C": data[6],
            "rotor_temp_C": data[7],
        }


def _append_prefixed(lines: list[str], prefix: str, text: str) -> None:
    for line in text.splitlines():
        if line.strip():
            lines.append(f"{prefix}{line}")


def _has_dll_symbol(dll: ctypes.CDLL, name: str) -> bool:
    try:
        getattr(dll, name)
        return True
    except AttributeError:
        return False


def _diagnose_dmcan_v11(dll: ctypes.CDLL) -> list[str]:
    lines: list[str] = ["sdk: detected DM_DeviceSDK v1.1 dmcan API"]
    ctx = ctypes.POINTER(_DamiaoDmCanContext)()

    dll.dmcan_context_create.argtypes = [ctypes.POINTER(ctypes.POINTER(_DamiaoDmCanContext))]
    dll.dmcan_context_create.restype = None
    dll.dmcan_context_destroy.argtypes = [ctypes.POINTER(_DamiaoDmCanContext)]
    dll.dmcan_context_destroy.restype = None
    dll.dmcan_get_sdk_version.argtypes = [ctypes.POINTER(_DamiaoDmCanContext), ctypes.POINTER(ctypes.c_uint32)]
    dll.dmcan_get_sdk_version.restype = None
    dll.dmcan_find_devices.argtypes = [ctypes.POINTER(_DamiaoDmCanContext)]
    dll.dmcan_find_devices.restype = ctypes.c_int
    dll.dmcan_device_get.argtypes = [
        ctypes.POINTER(_DamiaoDmCanContext),
        ctypes.POINTER(ctypes.POINTER(_DamiaoDmCanDeviceHandle)),
        ctypes.c_int,
    ]
    dll.dmcan_device_get.restype = ctypes.c_bool
    dll.dmcan_device_open.argtypes = [ctypes.POINTER(_DamiaoDmCanDeviceHandle)]
    dll.dmcan_device_open.restype = ctypes.c_bool
    dll.dmcan_device_close.argtypes = [ctypes.POINTER(_DamiaoDmCanDeviceHandle)]
    dll.dmcan_device_close.restype = None
    dll.dmcan_device_get_version.argtypes = [ctypes.POINTER(_DamiaoDmCanDeviceHandle), ctypes.c_char_p, ctypes.c_size_t]
    dll.dmcan_device_get_version.restype = None

    dll.dmcan_context_create(ctypes.byref(ctx))
    lines.append(f"sdk: dmcan context={bool(ctx)}")
    if not ctx:
        return lines

    try:
        version = ctypes.c_uint32()
        try:
            dll.dmcan_get_sdk_version(ctx, ctypes.byref(version))
            lines.append(f"sdk: version_raw=0x{version.value:08X}")
        except Exception as exc:
            lines.append(f"sdk: version failed {type(exc).__name__}: {exc}")

        all_count = int(dll.dmcan_find_devices(ctx))
        lines.append(f"sdk: all device count={all_count}")
        lines.append("sdk: type-filtered discovery skipped because dmcan_find_devices_with_type() is unstable in this DLL")
        for i in range(max(0, all_count)):
            dev = ctypes.POINTER(_DamiaoDmCanDeviceHandle)()
            got = bool(dll.dmcan_device_get(ctx, ctypes.byref(dev), i))
            if not got or not dev:
                lines.append(f"sdk: dev[{i}] get=False")
                continue
            opened = bool(dll.dmcan_device_open(dev))
            version_buf = ctypes.create_string_buffer(128)
            if opened:
                try:
                    dll.dmcan_device_get_version(dev, version_buf, len(version_buf))
                except Exception:
                    pass
            lines.append(
                f"sdk: dev[{i}] open={opened} "
                f"version={version_buf.value.decode(errors='replace')}"
            )
            if opened:
                try:
                    dll.dmcan_device_close(dev)
                except Exception:
                    pass
    finally:
        # The v1.1 Windows DLL can raise an access violation in dmcan_context_destroy()
        # after successful enumeration. Closing opened devices is enough here; the UI
        # process lifetime owns the remaining SDK context allocation.
        pass

    return lines


def _diagnose_dmcan_legacy(dll: ctypes.CDLL) -> list[str]:
    lines: list[str] = ["sdk: detected legacy DM_DeviceSDK damiao_handle API"]
    dll.damiao_handle_create.argtypes = [ctypes.c_int]
    dll.damiao_handle_create.restype = ctypes.c_void_p
    dll.damiao_get_sdk_version.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t]
    dll.damiao_get_sdk_version.restype = None
    dll.damiao_handle_find_devices.argtypes = [ctypes.c_void_p]
    dll.damiao_handle_find_devices.restype = ctypes.c_int
    dll.damiao_handle_get_devices.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_void_p),
        ctypes.POINTER(ctypes.c_int),
    ]
    dll.damiao_handle_get_devices.restype = None
    dll.device_get_pid_vid.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
    ]
    dll.device_get_pid_vid.restype = None
    dll.device_get_serial_number.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t]
    dll.device_get_serial_number.restype = None
    dll.device_open.argtypes = [ctypes.c_void_p]
    dll.device_open.restype = ctypes.c_bool
    dll.device_close.argtypes = [ctypes.c_void_p]
    dll.device_close.restype = ctypes.c_bool

    sdk_types = [
        (DamiaoUsb2FdCanBackend.DEV_USB2CANFD, "DEV_USB2CANFD"),
        (DamiaoUsb2FdCanBackend.DEV_USB2CANFD_DUAL, "DEV_USB2CANFD_DUAL"),
    ]
    for sdk_type, sdk_type_name in sdk_types:
        handle = ctypes.c_void_p(dll.damiao_handle_create(sdk_type))
        lines.append(f"sdk: {sdk_type_name} handle={bool(handle)}")
        if not handle:
            continue
        version = ctypes.create_string_buffer(128)
        try:
            dll.damiao_get_sdk_version(handle, version, len(version))
            lines.append(f"sdk: version={version.value.decode(errors='replace')}")
        except Exception as exc:
            lines.append(f"sdk: version failed {type(exc).__name__}: {exc}")

        count = int(dll.damiao_handle_find_devices(handle))
        lines.append(f"sdk: {sdk_type_name} count={count}")
        devices = (ctypes.c_void_p * max(16, count))()
        device_count = ctypes.c_int(0)
        dll.damiao_handle_get_devices(handle, devices, ctypes.byref(device_count))
        lines.append(f"sdk: {sdk_type_name} get_devices count={device_count.value}")
        for i in range(device_count.value):
            pid = ctypes.c_int()
            vid = ctypes.c_int()
            sn = ctypes.create_string_buffer(128)
            dll.device_get_pid_vid(devices[i], ctypes.byref(pid), ctypes.byref(vid))
            try:
                dll.device_get_serial_number(devices[i], sn, len(sn))
            except Exception:
                pass
            opened = bool(dll.device_open(devices[i]))
            lines.append(
                f"sdk: {sdk_type_name} dev[{i}] vid=0x{vid.value:04X} pid=0x{pid.value:04X} "
                f"sn={sn.value.decode(errors='replace')} open={opened}"
            )
            if opened:
                try:
                    dll.device_close(devices[i])
                except Exception:
                    pass
    return lines


def diagnose_damiao_adapter(port: str, tty_baudrate: int) -> list[str]:
    lines: list[str] = []
    lines.append("DM-USB2FDCAN local diagnosis")

    try:
        from serial.tools import list_ports  # type: ignore

        ports = list(list_ports.comports())
        lines.append(f"serial: {len(ports)} port(s) found")
        for item in ports:
            lines.append(f"serial: {item.device} desc={item.description} hwid={item.hwid}")
    except Exception as exc:
        lines.append(f"serial list failed: {type(exc).__name__}: {exc}")

    try:
        pnp = subprocess.run(
            ["pnputil", "/enum-devices", "/connected"],
            capture_output=True,
            text=True,
            timeout=8,
            check=False,
        )
        selected: list[str] = []
        context: list[str] = []
        for raw in pnp.stdout.splitlines():
            line = raw.strip()
            if not line:
                context = []
                continue
            context.append(line)
            context = context[-8:]
            upper = line.upper()
            if "34B7" in upper or "6877" in upper or "DM-USB2FDCAN" in upper or port.upper() in upper:
                selected.extend(context)
        for item in selected[:40]:
            lines.append(f"pnp: {item}")
        if not selected:
            lines.append("pnp: no VID_34B7/PID_6877 lines found")
    except Exception as exc:
        lines.append(f"pnputil failed: {type(exc).__name__}: {exc}")

    try:
        dll = DamiaoUsb2FdCanBackend._load_sdk_dll()
        if _has_dll_symbol(dll, "dmcan_context_create"):
            lines.extend(_diagnose_dmcan_v11(dll))
        else:
            lines.extend(_diagnose_dmcan_legacy(dll))
    except Exception as exc:
        lines.append(f"sdk diagnosis failed: {type(exc).__name__}: {exc}")

    try:
        libusb_path = DamiaoUsb2FdCanBackend._dll_dir() / "libusb-1.0.dll"
        if libusb_path.exists():
            libusb = ctypes.CDLL(str(libusb_path))
            libusb.libusb_init.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
            libusb.libusb_init.restype = ctypes.c_int
            libusb.libusb_exit.argtypes = [ctypes.c_void_p]
            libusb.libusb_exit.restype = None
            libusb.libusb_open_device_with_vid_pid.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16]
            libusb.libusb_open_device_with_vid_pid.restype = ctypes.c_void_p
            libusb.libusb_close.argtypes = [ctypes.c_void_p]
            libusb.libusb_close.restype = None
            libusb.libusb_claim_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
            libusb.libusb_claim_interface.restype = ctypes.c_int
            libusb.libusb_release_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
            libusb.libusb_release_interface.restype = ctypes.c_int
            ctx = ctypes.c_void_p()
            init_result = int(libusb.libusb_init(ctypes.byref(ctx)))
            lines.append(f"libusb: init={init_result}")
            if init_result == 0:
                dev = libusb.libusb_open_device_with_vid_pid(ctx, 0x34B7, 0x6877)
                lines.append(f"libusb: open VID_34B7/PID_6877={bool(dev)}")
                if dev:
                    claim_result = int(libusb.libusb_claim_interface(dev, 0))
                    lines.append(f"libusb: claim interface 0 result={claim_result}")
                    if claim_result == 0:
                        libusb.libusb_release_interface(dev, 0)
                    libusb.libusb_close(dev)
                libusb.libusb_exit(ctx)
        else:
            lines.append(f"libusb: missing {libusb_path}")
    except Exception as exc:
        lines.append(f"libusb diagnosis failed: {type(exc).__name__}: {exc}")

    try:
        import serial  # type: ignore

        ser = serial.Serial(port=port, baudrate=tty_baudrate, timeout=0.2, write_timeout=0.2, exclusive=True)
        try:
            lines.append(f"serial open: {port} {tty_baudrate} ok")
            for cmd in (b"V\r", b"v\r", b"N\r"):
                ser.reset_input_buffer()
                ser.write(cmd)
                time.sleep(0.08)
                reply = ser.read(128)
                lines.append(f"slcan cmd {cmd!r} rx={reply!r}")
        finally:
            ser.close()
    except Exception as exc:
        lines.append(f"serial/slcan test failed: {type(exc).__name__}: {exc}")

    lines.append(
        "diagnosis hint: if sdk count>0 but open=False while WinUSB is present, the adapter "
        "access layer is failing before any motor CAN command is sent."
    )
    lines.append(
        "diagnosis hint: the program supports both legacy damiao_handle API and new v1.1 dmcan API. "
        "If official DMTool opens the adapter but this SDK path fails, verify that dlls\\libdm_device.dll "
        "matches the adapter firmware."
    )
    return lines


class DamiaoCanBackend(MotorBackend):
    def __init__(
        self,
        interface: str,
        channel: str,
        bitrate: int,
        tty_baudrate: int,
        data_bitrate: int,
        motor_id: int,
        master_id: int,
        max_phase_current_A: float,
        torque_per_amp_Nm: float,
        stroke_sign: float,
        auto_switch_mode: bool,
        motor_frames_canfd: bool,
        mode_offset_command_id: bool,
    ) -> None:
        if can is None:
            raise RuntimeError("python-can is not installed. Run: .\\.venv\\Scripts\\python -m pip install python-can")
        interface_name = interface.strip().lower()
        bus_kwargs: dict[str, Any] = {"interface": interface, "channel": channel, "bitrate": bitrate}
        if interface_name == "slcan":
            if data_bitrate > 0:
                timing = can.BitTimingFd.from_sample_point(
                    f_clock=80_000_000,
                    nom_bitrate=bitrate,
                    nom_sample_point=75.0,
                    data_bitrate=data_bitrate,
                    data_sample_point=75.0,
                )
                bus_kwargs.pop("bitrate", None)
                bus_kwargs["timing"] = timing
            bus_kwargs["tty_baudrate"] = tty_baudrate
        self.bus = can.Bus(**bus_kwargs)
        self.use_canfd = interface_name == "slcan" and data_bitrate > 0
        self.motor_id = motor_id
        self.master_id = master_id
        self.max_phase_current_A = max(0.1, max_phase_current_A)
        self.torque_per_amp_Nm = max(0.001, torque_per_amp_Nm)
        self.stroke_sign = 1.0 if stroke_sign >= 0 else -1.0
        self.auto_switch_mode = auto_switch_mode
        self.motor_frames_canfd = motor_frames_canfd
        self.mode_offset_command_id = mode_offset_command_id
        self.feedback = MotorFeedback(timestamp_s=time.monotonic(), stroke_mm=0.0)
        self.zero_output_rad: float | None = None
        self.last_output_rad = 0.0
        self.current_mode: int | None = None
        self._sent_motor_command = False
        self._trace_lines: deque[str] = deque(maxlen=256)

    def _command_id(self, mode: int | None = None) -> int:
        if not self.mode_offset_command_id:
            return self.motor_id
        active_mode = self.current_mode if mode is None else mode
        if active_mode == DamiaoProtocol.MODE_POSITION_FORCE:
            return 0x300 + self.motor_id
        if active_mode == DamiaoProtocol.MODE_VELOCITY:
            return 0x200 + self.motor_id
        if active_mode == DamiaoProtocol.MODE_POSITION_VELOCITY:
            return 0x100 + self.motor_id
        return self.motor_id

    def _trace(self, text: str) -> None:
        self._trace_lines.append(text)

    def drain_trace(self) -> list[str]:
        lines = list(self._trace_lines)
        self._trace_lines.clear()
        return lines

    def _send(self, arbitration_id: int, data: bytes, *, motor_frame: bool = True) -> bool:
        is_fd = self.motor_frames_canfd if motor_frame else self.use_canfd
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False,
            is_fd=is_fd,
            bitrate_switch=is_fd,
        )
        self.bus.send(msg, timeout=0.05)
        self._trace(
            f"TX {'FD' if is_fd else 'CAN'} id=0x{arbitration_id:X} dlc={len(data)} data={data.hex(' ')} ok=True"
        )
        if arbitration_id != 0x7FF:
            self._sent_motor_command = True
        return True

    def _recv_raw(self, timeout_s: float) -> can.Message | None:
        return self.bus.recv(timeout=timeout_s)

    def _control_id(self) -> int:
        if self.current_mode == DamiaoProtocol.MODE_POSITION_FORCE:
            return 0x300 + self.motor_id
        if self.current_mode == DamiaoProtocol.MODE_VELOCITY:
            return 0x200 + self.motor_id
        return self.motor_id

    def _switch_mode(self, mode: int) -> None:
        if not self.auto_switch_mode or self.current_mode == mode:
            self.current_mode = mode
            return
        arbitration_id, data = DamiaoProtocol.register_write_u32(self.motor_id, DamiaoProtocol.REG_CTRL_MODE, mode)
        self._send(arbitration_id, data, motor_frame=False)
        self.current_mode = mode

    def _stroke_to_rad(self, stroke_mm: float) -> float:
        zero = self.zero_output_rad if self.zero_output_rad is not None else self.last_output_rad
        return zero + self.stroke_sign * stroke_mm * TWO_PI / SCREW_LEAD_MM

    def _rad_to_stroke(self, output_rad: float) -> float:
        if self.zero_output_rad is None:
            return 0.0
        return self.stroke_sign * (output_rad - self.zero_output_rad) * SCREW_LEAD_MM / TWO_PI

    def _mm_s_to_rad_s(self, velocity_mm_s: float) -> float:
        return self.stroke_sign * velocity_mm_s * TWO_PI / SCREW_LEAD_MM

    def _rad_s_to_mm_s(self, velocity_rad_s: float) -> float:
        return self.stroke_sign * velocity_rad_s * SCREW_LEAD_MM / TWO_PI

    def _velocity_target_rad(self, velocity_mm_s: float) -> float:
        if abs(velocity_mm_s) < 1e-9:
            return self.last_output_rad
        direction = 1.0 if velocity_mm_s >= 0.0 else -1.0
        lead_mm = max(0.20, min(2.0, abs(velocity_mm_s) * 2.0))
        if self.zero_output_rad is None:
            return self.last_output_rad + self.stroke_sign * direction * lead_mm * TWO_PI / SCREW_LEAD_MM
        target_stroke = max(STROKE_MIN_MM, min(STROKE_MAX_MM, self.feedback.stroke_mm + direction * lead_mm))
        return self._stroke_to_rad(target_stroke)

    def enable(self) -> None:
        if self.auto_switch_mode:
            self._switch_mode(DamiaoProtocol.MODE_POSITION_FORCE)
        arbitration_id, data = DamiaoProtocol.enable_frame(self._command_id(DamiaoProtocol.MODE_POSITION_FORCE))
        self._send(arbitration_id, data)
        self.feedback.enabled = True

    def disable(self) -> None:
        self.stop()
        arbitration_id, data = DamiaoProtocol.disable_frame(self._command_id())
        self._send(arbitration_id, data)
        self.feedback.enabled = False

    def command_velocity(self, velocity_mm_s: float, current_limit_A: float) -> None:
        self._switch_mode(DamiaoProtocol.MODE_POSITION_FORCE)
        current_norm = abs(current_limit_A) / self.max_phase_current_A
        target_rad = self._velocity_target_rad(velocity_mm_s)
        velocity_rad_s = abs(self._mm_s_to_rad_s(velocity_mm_s))
        self._trace(
            f"CMD posforce velocity_mm_s={velocity_mm_s:.4f} target_rad={target_rad:.4f} "
            f"speed_rad_s={velocity_rad_s:.4f} current_norm={current_norm:.4f} homed={self.zero_output_rad is not None}"
        )
        arbitration_id, data = DamiaoProtocol.pos_force_frame(
            self.motor_id,
            target_rad,
            velocity_rad_s,
            current_norm,
        )
        self._send(arbitration_id, data)

    def command_current(self, current_A: float) -> None:
        self._switch_mode(DamiaoProtocol.MODE_MIT)
        torque = current_A * self.torque_per_amp_Nm
        arbitration_id, data = DamiaoProtocol.mit_torque_frame(self.motor_id, torque)
        self._send(arbitration_id, data)

    def stop(self) -> None:
        if self.current_mode == DamiaoProtocol.MODE_MIT:
            arbitration_id, data = DamiaoProtocol.mit_torque_frame(self.motor_id, 0.0)
        else:
            arbitration_id, data = DamiaoProtocol.pos_force_frame(self.motor_id, self._stroke_to_rad(self.feedback.stroke_mm), 0.0, 0.0)
        self._send(arbitration_id, data)
        self.feedback.velocity_mm_s = 0.0

    def reset_zero(self) -> None:
        self.zero_output_rad = self.last_output_rad
        self.feedback.stroke_mm = 0.0

    def is_zero_homed(self) -> bool:
        return self.zero_output_rad is not None

    def update(self, dt_s: float) -> MotorFeedback:
        del dt_s
        deadline = time.monotonic() + 0.02
        while time.monotonic() < deadline:
            msg = self.bus.recv(timeout=0.002)
            if msg is None:
                break
            self._trace(
                f"RX {'FD' if getattr(msg, 'is_fd', False) else 'CAN'} id=0x{msg.arbitration_id:X} "
                f"dlc={len(bytes(msg.data))} data={bytes(msg.data).hex(' ')}"
            )
            parsed = DamiaoProtocol.parse_feedback(msg.arbitration_id, bytes(msg.data), self.master_id)
            if parsed is None or parsed["motor_id"] != self.motor_id:
                continue
            self.last_output_rad = float(parsed["position_rad"])
            self.feedback.timestamp_s = time.monotonic()
            self.feedback.stroke_mm = self._rad_to_stroke(self.last_output_rad)
            self.feedback.velocity_mm_s = self._rad_s_to_mm_s(float(parsed["velocity_rad_s"]))
            self.feedback.torque_Nm = float(parsed["torque_Nm"])
            self.feedback.current_A = self.feedback.torque_Nm / self.torque_per_amp_Nm
            self.feedback.temperature_C = max(float(parsed["mos_temp_C"]), float(parsed["rotor_temp_C"]))
            self.feedback.enabled = bool(parsed["enabled"])
            fault_bits = int(parsed["fault_bits"])
            self.feedback.fault = "" if fault_bits == 0 else f"drive ERR=0x{int(parsed['err']):X}"
            break
        return self.feedback

    def probe_can(self) -> list[str]:
        lines: list[str] = []
        while self._recv_raw(0.001) is not None:
            pass

        commands = [
            ("refresh", DamiaoProtocol.refresh_frame),
            ("query_master_id", lambda motor_id: DamiaoProtocol.register_read(motor_id, 0x08)),
        ]
        for name, pack in commands:
            arbitration_id, data = pack(self.motor_id)
            self._send(arbitration_id, data, motor_frame=False)
            lines.append(
                f"TX {name} {'FD' if self.use_canfd else 'CAN'} id=0x{arbitration_id:X} data={data.hex(' ')}"
            )
            deadline = time.monotonic() + 0.20
            received = False
            while time.monotonic() < deadline:
                msg = self._recv_raw(0.02)
                if msg is None:
                    continue
                received = True
                payload = bytes(msg.data)
                lines.append(f"RX id=0x{msg.arbitration_id:X} dlc={len(payload)} data={payload.hex(' ')}")
            if not received:
                lines.append("RX none")

        return lines

    def close(self) -> None:
        try:
            if self._sent_motor_command:
                self.disable()
        finally:
            self.bus.shutdown()


class _DamiaoUsbRxFrameHead(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("can_id", ctypes.c_uint32, 29),
        ("esi", ctypes.c_uint32, 1),
        ("ext", ctypes.c_uint32, 1),
        ("rtr", ctypes.c_uint32, 1),
        ("time_stamp", ctypes.c_uint64),
        ("channel", ctypes.c_uint8),
        ("canfd", ctypes.c_uint8, 1),
        ("dir", ctypes.c_uint8, 1),
        ("brs", ctypes.c_uint8, 1),
        ("ack", ctypes.c_uint8, 1),
        ("dlc", ctypes.c_uint8, 4),
        ("reserved", ctypes.c_uint16),
    ]


class _DamiaoUsbRxFrame(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("head", _DamiaoUsbRxFrameHead),
        ("payload", ctypes.c_uint8 * 64),
    ]


class _DamiaoDeviceBaud(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("can_baudrate", ctypes.c_int),
        ("canfd_baudrate", ctypes.c_int),
        ("can_sp", ctypes.c_float),
        ("canfd_sp", ctypes.c_float),
    ]


class _DamiaoDmCanChannelInfo(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("channel", ctypes.c_uint8),
        ("canfd", ctypes.c_bool),
        ("can_baudrate", ctypes.c_uint32),
        ("canfd_baudrate", ctypes.c_uint32),
        ("can_sp", ctypes.c_float),
        ("canfd_sp", ctypes.c_float),
    ]


class _DamiaoDmCanContext(ctypes.Structure):
    pass


class _DamiaoDmCanDeviceHandle(ctypes.Structure):
    pass


_DAMIAO_LEGACY_RECV_CALLBACK = ctypes.CFUNCTYPE(None, ctypes.POINTER(_DamiaoUsbRxFrame))
_DAMIAO_DMCAN_RECV_CALLBACK = ctypes.CFUNCTYPE(
    None,
    ctypes.POINTER(_DamiaoDmCanDeviceHandle),
    ctypes.POINTER(_DamiaoUsbRxFrame),
)


class DamiaoUsb2FdCanBackend(MotorBackend):
    DEV_USB2CANFD = 0
    DEV_USB2CANFD_DUAL = 1
    _dll_directory_handles: list[Any] = []

    def __init__(
        self,
        device_index: int,
        device_type: int,
        can_channel: int,
        bitrate: int,
        data_bitrate: int,
        use_canfd: bool,
        motor_id: int,
        master_id: int,
        max_phase_current_A: float,
        torque_per_amp_Nm: float,
        stroke_sign: float,
        auto_switch_mode: bool,
        motor_frames_canfd: bool,
        mode_offset_command_id: bool,
    ) -> None:
        if can_channel not in (0, 1):
            raise RuntimeError("DM-USB2FDCAN CAN channel must be 0 or 1.")
        if device_type not in (self.DEV_USB2CANFD, self.DEV_USB2CANFD_DUAL):
            raise RuntimeError("DM device type must be 0 for USB2CANFD or 1 for USB2CANFD_Dual.")

        self.dll: ctypes.CDLL | None = None
        self.handle: Any | None = None
        self.device: Any | None = None
        self._sdk_api = "unknown"
        self._recv_cb: Any | None = None
        self.device_index = max(0, device_index)
        self.device_type = device_type
        self.can_channel = can_channel
        self.use_canfd = use_canfd
        self.data_bitrate = max(0, data_bitrate)
        self.motor_id = motor_id
        self.master_id = master_id
        self.max_phase_current_A = max(0.1, max_phase_current_A)
        self.torque_per_amp_Nm = max(0.001, torque_per_amp_Nm)
        self.stroke_sign = 1.0 if stroke_sign >= 0 else -1.0
        self.auto_switch_mode = auto_switch_mode
        self.motor_frames_canfd = motor_frames_canfd
        self.mode_offset_command_id = mode_offset_command_id
        self.feedback = MotorFeedback(timestamp_s=time.monotonic(), stroke_mm=0.0)
        self.zero_output_rad: float | None = None
        self.last_output_rad = 0.0
        self.current_mode: int | None = None
        self._rx_frames: deque[tuple[int, bytes]] = deque(maxlen=512)
        self._rx_lock = Lock()
        self._closed = False
        self._device_opened = False
        self._channel_opened = False
        self._sent_motor_command = False
        self._trace_lines: deque[str] = deque(maxlen=256)

        try:
            self.dll = self._load_sdk_dll()
        except OSError as exc:
            raise RuntimeError(
                "Failed to load Damiao DeviceSDK DLL. Ensure dlls\\libdm_device.dll exists and the "
                "libusb/MinGW runtime DLLs are available in the same dlls folder."
            ) from exc

        try:
            if _has_dll_symbol(self.dll, "dmcan_context_create"):
                self._open_dmcan_v11(bitrate)
            else:
                self._open_legacy(bitrate)
        except Exception:
            self.close()
            raise

    @staticmethod
    def _dll_dir() -> Path:
        if getattr(sys, "frozen", False):
            bundled = Path(getattr(sys, "_MEIPASS", Path(sys.executable).resolve().parent)) / "dlls"
            if bundled.exists():
                return bundled
            return Path(sys.executable).resolve().parent / "dlls"
        return Path(__file__).resolve().parents[3] / "dlls"

    @classmethod
    def _load_sdk_dll(cls) -> ctypes.CDLL:
        dll_dir = cls._dll_dir()
        dll_path = dll_dir / "libdm_device.dll"
        if not dll_path.exists():
            raise OSError(f"Missing {dll_path}")
        if hasattr(os, "add_dll_directory"):
            cls._dll_directory_handles.append(os.add_dll_directory(str(dll_dir)))
        return ctypes.CDLL(str(dll_path))

    def _open_dmcan_v11(self, bitrate: int) -> None:
        assert self.dll is not None
        self._sdk_api = "dmcan_v1.1"
        self._init_dmcan_v11_functions()

        ctx = ctypes.POINTER(_DamiaoDmCanContext)()
        self.dll.dmcan_context_create(ctypes.byref(ctx))
        self.handle = ctx
        if not self.handle:
            raise RuntimeError("Failed to create DM-USB2FDCAN dmcan context.")

        count = int(self.dll.dmcan_find_devices(self.handle))
        if count <= self.device_index:
            raise RuntimeError(f"Found {count} DM-USB2FDCAN device(s), device index {self.device_index} is unavailable.")

        dev = ctypes.POINTER(_DamiaoDmCanDeviceHandle)()
        if not self.dll.dmcan_device_get(self.handle, ctypes.byref(dev), self.device_index) or not dev:
            raise RuntimeError(f"Failed to get DM-USB2FDCAN device index {self.device_index}.")
        self.device = dev

        if not self.dll.dmcan_device_open(self.device):
            self.device = None
            raise RuntimeError(
                "Found DM-USB2FDCAN, but dmcan_device_open() failed. Close DMTool and verify the "
                "WinUSB driver and SDK DLL match the adapter firmware."
            )
        self._device_opened = True

        baud = _DamiaoDmCanChannelInfo(
            ctypes.c_uint8(self.can_channel),
            ctypes.c_bool(bool(self.use_canfd)),
            ctypes.c_uint32(int(bitrate)),
            ctypes.c_uint32(int(self.data_bitrate)),
            ctypes.c_float(0.75),
            ctypes.c_float(0.75),
        )
        if not self.dll.dmcan_device_set_channel_baudrate(self.device, ctypes.c_uint8(self.can_channel), baud):
            raise RuntimeError(
                f"Failed to set CAN channel {self.can_channel} bitrate to {bitrate}/{self.data_bitrate}."
            )

        self._recv_cb = _DAMIAO_DMCAN_RECV_CALLBACK(self._on_recv_dmcan)
        self.dll.dmcan_device_hook_recv_callback(self.device, self._recv_cb)
        if not self.dll.dmcan_device_enable_channel(self.device, ctypes.c_uint8(self.can_channel)):
            raise RuntimeError(f"Failed to enable DM-USB2FDCAN CAN channel {self.can_channel}.")
        self._channel_opened = True

    def _open_legacy(self, bitrate: int) -> None:
        assert self.dll is not None
        self._sdk_api = "legacy"
        self._init_legacy_functions()

        self.handle = ctypes.c_void_p(self.dll.damiao_handle_create(self.device_type))
        if not self.handle:
            raise RuntimeError("Failed to create DM-USB2FDCAN SDK handle.")

        count = int(self.dll.damiao_handle_find_devices(self.handle))
        if count <= self.device_index:
            raise RuntimeError(f"Found {count} DM-USB2FDCAN device(s), device index {self.device_index} is unavailable.")

        devices = (ctypes.c_void_p * max(16, count))()
        device_count = ctypes.c_int(0)
        self.dll.damiao_handle_get_devices(self.handle, devices, ctypes.byref(device_count))
        if device_count.value <= self.device_index or not devices[self.device_index]:
            raise RuntimeError(f"Failed to get DM-USB2FDCAN device index {self.device_index}.")

        self.device = ctypes.c_void_p(devices[self.device_index])
        if not self.dll.device_open(self.device):
            self.device = None
            raise RuntimeError(
                "Found DM-USB2FDCAN, but failed to open it. Check the Damiao/WinUSB driver "
                "and close DMTool or any other process using the adapter."
            )
        self._device_opened = True

        if not self.dll.device_channel_set_baud_with_sp(
            self.device,
            ctypes.c_uint8(self.can_channel),
            bool(self.use_canfd),
            int(bitrate),
            int(self.data_bitrate),
            ctypes.c_float(0.75),
            ctypes.c_float(0.75),
        ):
            raise RuntimeError(
                f"Failed to set CAN channel {self.can_channel} bitrate to {bitrate}/{self.data_bitrate}."
            )

        self._recv_cb = _DAMIAO_LEGACY_RECV_CALLBACK(self._on_recv_legacy)
        self.dll.device_hook_to_rec(self.device, self._recv_cb)
        if not self.dll.device_open_channel(self.device, ctypes.c_uint8(self.can_channel)):
            raise RuntimeError(f"Failed to open DM-USB2FDCAN CAN channel {self.can_channel}.")
        self._channel_opened = True

    def _init_dmcan_v11_functions(self) -> None:
        assert self.dll is not None
        self.dll.dmcan_context_create.argtypes = [ctypes.POINTER(ctypes.POINTER(_DamiaoDmCanContext))]
        self.dll.dmcan_context_create.restype = None
        self.dll.dmcan_context_destroy.argtypes = [ctypes.POINTER(_DamiaoDmCanContext)]
        self.dll.dmcan_context_destroy.restype = None
        self.dll.dmcan_find_devices.argtypes = [ctypes.POINTER(_DamiaoDmCanContext)]
        self.dll.dmcan_find_devices.restype = ctypes.c_int
        self.dll.dmcan_device_get.argtypes = [
            ctypes.POINTER(_DamiaoDmCanContext),
            ctypes.POINTER(ctypes.POINTER(_DamiaoDmCanDeviceHandle)),
            ctypes.c_int,
        ]
        self.dll.dmcan_device_get.restype = ctypes.c_bool
        self.dll.dmcan_device_open.argtypes = [ctypes.POINTER(_DamiaoDmCanDeviceHandle)]
        self.dll.dmcan_device_open.restype = ctypes.c_bool
        self.dll.dmcan_device_close.argtypes = [ctypes.POINTER(_DamiaoDmCanDeviceHandle)]
        self.dll.dmcan_device_close.restype = None
        self.dll.dmcan_device_enable_channel.argtypes = [ctypes.POINTER(_DamiaoDmCanDeviceHandle), ctypes.c_uint8]
        self.dll.dmcan_device_enable_channel.restype = ctypes.c_bool
        self.dll.dmcan_device_disable_channel.argtypes = [ctypes.POINTER(_DamiaoDmCanDeviceHandle), ctypes.c_uint8]
        self.dll.dmcan_device_disable_channel.restype = ctypes.c_bool
        self.dll.dmcan_device_set_channel_baudrate.argtypes = [
            ctypes.POINTER(_DamiaoDmCanDeviceHandle),
            ctypes.c_uint8,
            _DamiaoDmCanChannelInfo,
        ]
        self.dll.dmcan_device_set_channel_baudrate.restype = ctypes.c_bool
        self.dll.dmcan_device_hook_recv_callback.argtypes = [
            ctypes.POINTER(_DamiaoDmCanDeviceHandle),
            _DAMIAO_DMCAN_RECV_CALLBACK,
        ]
        self.dll.dmcan_device_hook_recv_callback.restype = None
        self.dll.dmcan_device_send_can.argtypes = [
            ctypes.POINTER(_DamiaoDmCanDeviceHandle),
            ctypes.c_uint8,
            ctypes.c_uint32,
            ctypes.c_bool,
            ctypes.c_bool,
            ctypes.c_bool,
            ctypes.c_bool,
            ctypes.c_uint8,
            ctypes.POINTER(ctypes.c_uint8),
        ]
        self.dll.dmcan_device_send_can.restype = ctypes.c_bool

    def _init_legacy_functions(self) -> None:
        assert self.dll is not None
        self.dll.damiao_handle_create.argtypes = [ctypes.c_int]
        self.dll.damiao_handle_create.restype = ctypes.c_void_p
        self.dll.damiao_handle_destroy.argtypes = [ctypes.c_void_p]
        self.dll.damiao_handle_destroy.restype = None
        self.dll.damiao_handle_find_devices.argtypes = [ctypes.c_void_p]
        self.dll.damiao_handle_find_devices.restype = ctypes.c_int
        self.dll.damiao_handle_get_devices.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_int)]
        self.dll.damiao_handle_get_devices.restype = None
        self.dll.device_open.argtypes = [ctypes.c_void_p]
        self.dll.device_open.restype = ctypes.c_bool
        self.dll.device_close.argtypes = [ctypes.c_void_p]
        self.dll.device_close.restype = ctypes.c_bool
        self.dll.device_open_channel.argtypes = [ctypes.c_void_p, ctypes.c_uint8]
        self.dll.device_open_channel.restype = ctypes.c_bool
        self.dll.device_close_channel.argtypes = [ctypes.c_void_p, ctypes.c_uint8]
        self.dll.device_close_channel.restype = ctypes.c_bool
        self.dll.device_channel_set_baud_with_sp.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint8,
            ctypes.c_bool,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_float,
            ctypes.c_float,
        ]
        self.dll.device_channel_set_baud_with_sp.restype = ctypes.c_bool
        self.dll.device_hook_to_rec.argtypes = [ctypes.c_void_p, _DAMIAO_LEGACY_RECV_CALLBACK]
        self.dll.device_hook_to_rec.restype = None
        self.dll.device_channel_send_fast.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint8,
            ctypes.c_uint32,
            ctypes.c_int32,
            ctypes.c_bool,
            ctypes.c_bool,
            ctypes.c_bool,
            ctypes.c_uint8,
            ctypes.POINTER(ctypes.c_uint8),
        ]
        self.dll.device_channel_send_fast.restype = None

    @staticmethod
    def _dlc_to_len(dlc: int) -> int:
        if dlc <= 8:
            return max(0, dlc)
        return {9: 12, 10: 16, 11: 20, 12: 24, 13: 32, 14: 48, 15: 64}.get(dlc, 64)

    def _on_recv_dmcan(self, _handle: Any, frame_ptr: Any) -> None:
        self._queue_rx_frame(frame_ptr)

    def _on_recv_legacy(self, frame_ptr: Any) -> None:
        self._queue_rx_frame(frame_ptr)

    def _queue_rx_frame(self, frame_ptr: Any) -> None:
        try:
            if not frame_ptr:
                return
            frame = frame_ptr.contents
            head = frame.head
            if int(head.channel) != self.can_channel:
                return
            if bool(head.ext) or bool(head.rtr):
                return
            length = self._dlc_to_len(int(head.dlc))
            data = bytes(int(frame.payload[i]) & 0xFF for i in range(length))
            with self._rx_lock:
                self._rx_frames.append((int(head.can_id), data))
            self._trace(
                f"RX {'FD' if bool(head.canfd) else 'CAN'} id=0x{int(head.can_id):X} dlc={length} data={data.hex(' ')}"
            )
        except Exception:
            return

    def _command_id(self, mode: int | None = None) -> int:
        if not self.mode_offset_command_id:
            return self.motor_id
        active_mode = self.current_mode if mode is None else mode
        if active_mode == DamiaoProtocol.MODE_POSITION_FORCE:
            return 0x300 + self.motor_id
        if active_mode == DamiaoProtocol.MODE_VELOCITY:
            return 0x200 + self.motor_id
        if active_mode == DamiaoProtocol.MODE_POSITION_VELOCITY:
            return 0x100 + self.motor_id
        return self.motor_id

    def _trace(self, text: str) -> None:
        self._trace_lines.append(text)

    def drain_trace(self) -> list[str]:
        lines = list(self._trace_lines)
        self._trace_lines.clear()
        return lines

    def _send(self, arbitration_id: int, data: bytes, *, motor_frame: bool = True) -> bool:
        if self._closed or self.device is None or self.dll is None:
            return False
        is_fd = self.motor_frames_canfd if motor_frame else self.use_canfd
        payload = (ctypes.c_uint8 * len(data))(*data)
        if self._sdk_api == "dmcan_v1.1":
            ok = bool(self.dll.dmcan_device_send_can(
                self.device,
                self.can_channel,
                arbitration_id,
                is_fd,
                False,
                False,
                is_fd,
                len(data),
                payload,
            ))
            if arbitration_id != 0x7FF:
                self._sent_motor_command = True
        else:
            self.dll.device_channel_send_fast(
                self.device,
                self.can_channel,
                arbitration_id,
                1,
                False,
                is_fd,
                is_fd,
                len(data),
                payload,
            )
            ok = True
            if arbitration_id != 0x7FF:
                self._sent_motor_command = True
        self._trace(
            f"TX {'FD' if is_fd else 'CAN'} id=0x{arbitration_id:X} dlc={len(data)} data={data.hex(' ')} ok={ok}"
        )
        return ok

    def _control_id(self) -> int:
        if self.current_mode == DamiaoProtocol.MODE_POSITION_FORCE:
            return 0x300 + self.motor_id
        if self.current_mode == DamiaoProtocol.MODE_VELOCITY:
            return 0x200 + self.motor_id
        return self.motor_id

    def _switch_mode(self, mode: int) -> None:
        if not self.auto_switch_mode or self.current_mode == mode:
            self.current_mode = mode
            return
        arbitration_id, data = DamiaoProtocol.register_write_u32(self.motor_id, DamiaoProtocol.REG_CTRL_MODE, mode)
        self._send(arbitration_id, data, motor_frame=False)
        self.current_mode = mode

    def _stroke_to_rad(self, stroke_mm: float) -> float:
        zero = self.zero_output_rad if self.zero_output_rad is not None else self.last_output_rad
        return zero + self.stroke_sign * stroke_mm * TWO_PI / SCREW_LEAD_MM

    def _rad_to_stroke(self, output_rad: float) -> float:
        if self.zero_output_rad is None:
            return 0.0
        return self.stroke_sign * (output_rad - self.zero_output_rad) * SCREW_LEAD_MM / TWO_PI

    def _mm_s_to_rad_s(self, velocity_mm_s: float) -> float:
        return self.stroke_sign * velocity_mm_s * TWO_PI / SCREW_LEAD_MM

    def _rad_s_to_mm_s(self, velocity_rad_s: float) -> float:
        return self.stroke_sign * velocity_rad_s * SCREW_LEAD_MM / TWO_PI

    def _velocity_target_rad(self, velocity_mm_s: float) -> float:
        if abs(velocity_mm_s) < 1e-9:
            return self.last_output_rad
        direction = 1.0 if velocity_mm_s >= 0.0 else -1.0
        lead_mm = max(0.20, min(2.0, abs(velocity_mm_s) * 2.0))
        if self.zero_output_rad is None:
            return self.last_output_rad + self.stroke_sign * direction * lead_mm * TWO_PI / SCREW_LEAD_MM
        target_stroke = max(STROKE_MIN_MM, min(STROKE_MAX_MM, self.feedback.stroke_mm + direction * lead_mm))
        return self._stroke_to_rad(target_stroke)

    def enable(self) -> None:
        if self.auto_switch_mode:
            self._switch_mode(DamiaoProtocol.MODE_POSITION_FORCE)
        arbitration_id, data = DamiaoProtocol.enable_frame(self._command_id(DamiaoProtocol.MODE_POSITION_FORCE))
        self._send(arbitration_id, data)
        self.feedback.enabled = True

    def disable(self) -> None:
        self.stop()
        arbitration_id, data = DamiaoProtocol.disable_frame(self._command_id())
        self._send(arbitration_id, data)
        self.feedback.enabled = False

    def command_velocity(self, velocity_mm_s: float, current_limit_A: float) -> None:
        self._switch_mode(DamiaoProtocol.MODE_POSITION_FORCE)
        current_norm = abs(current_limit_A) / self.max_phase_current_A
        target_rad = self._velocity_target_rad(velocity_mm_s)
        velocity_rad_s = abs(self._mm_s_to_rad_s(velocity_mm_s))
        self._trace(
            f"CMD posforce velocity_mm_s={velocity_mm_s:.4f} target_rad={target_rad:.4f} "
            f"speed_rad_s={velocity_rad_s:.4f} current_norm={current_norm:.4f} homed={self.zero_output_rad is not None}"
        )
        arbitration_id, data = DamiaoProtocol.pos_force_frame(
            self.motor_id,
            target_rad,
            velocity_rad_s,
            current_norm,
        )
        self._send(arbitration_id, data)

    def command_current(self, current_A: float) -> None:
        self._switch_mode(DamiaoProtocol.MODE_MIT)
        torque = current_A * self.torque_per_amp_Nm
        arbitration_id, data = DamiaoProtocol.mit_torque_frame(self.motor_id, torque)
        self._send(arbitration_id, data)

    def stop(self) -> None:
        if self.current_mode == DamiaoProtocol.MODE_MIT:
            arbitration_id, data = DamiaoProtocol.mit_torque_frame(self.motor_id, 0.0)
        else:
            arbitration_id, data = DamiaoProtocol.pos_force_frame(self.motor_id, self._stroke_to_rad(self.feedback.stroke_mm), 0.0, 0.0)
        self._send(arbitration_id, data)
        self.feedback.velocity_mm_s = 0.0

    def reset_zero(self) -> None:
        self.zero_output_rad = self.last_output_rad
        self.feedback.stroke_mm = 0.0

    def is_zero_homed(self) -> bool:
        return self.zero_output_rad is not None

    def update(self, dt_s: float) -> MotorFeedback:
        del dt_s
        deadline = time.monotonic() + 0.02
        while time.monotonic() < deadline:
            with self._rx_lock:
                frame = self._rx_frames.popleft() if self._rx_frames else None
            if frame is None:
                time.sleep(0.001)
                continue

            arbitration_id, data = frame
            parsed = DamiaoProtocol.parse_feedback(arbitration_id, data, self.master_id)
            if parsed is None or parsed["motor_id"] != self.motor_id:
                continue
            self.last_output_rad = float(parsed["position_rad"])
            self.feedback.timestamp_s = time.monotonic()
            self.feedback.stroke_mm = self._rad_to_stroke(self.last_output_rad)
            self.feedback.velocity_mm_s = self._rad_s_to_mm_s(float(parsed["velocity_rad_s"]))
            self.feedback.torque_Nm = float(parsed["torque_Nm"])
            self.feedback.current_A = self.feedback.torque_Nm / self.torque_per_amp_Nm
            self.feedback.temperature_C = max(float(parsed["mos_temp_C"]), float(parsed["rotor_temp_C"]))
            self.feedback.enabled = bool(parsed["enabled"])
            fault_bits = int(parsed["fault_bits"])
            self.feedback.fault = "" if fault_bits == 0 else f"drive ERR=0x{int(parsed['err']):X}"
            break
        return self.feedback

    def probe_can(self) -> list[str]:
        lines: list[str] = [
            f"DM_DeviceSDK type={self.device_type} channel={self.can_channel} "
            f"canfd={int(self.use_canfd)} bitrate={self.data_bitrate}"
        ]
        with self._rx_lock:
            self._rx_frames.clear()

        commands = [
            ("refresh", DamiaoProtocol.refresh_frame),
            ("query_master_id", lambda motor_id: DamiaoProtocol.register_read(motor_id, 0x08)),
        ]
        for name, pack in commands:
            arbitration_id, data = pack(self.motor_id)
            self._send(arbitration_id, data, motor_frame=False)
            lines.append(
                f"TX {name} {'FD' if self.use_canfd else 'CAN'} id=0x{arbitration_id:X} data={data.hex(' ')}"
            )
            deadline = time.monotonic() + 0.20
            received = False
            while time.monotonic() < deadline:
                with self._rx_lock:
                    frame = self._rx_frames.popleft() if self._rx_frames else None
                if frame is None:
                    time.sleep(0.002)
                    continue
                received = True
                rx_id, payload = frame
                lines.append(f"RX id=0x{rx_id:X} dlc={len(payload)} data={payload.hex(' ')}")
            if not received:
                lines.append("RX none")

        return lines

    def close(self) -> None:
        if self._closed:
            return
        try:
            if self.device is not None and self.dll is not None and self._device_opened:
                if self._channel_opened:
                    try:
                        if self._sent_motor_command:
                            self.disable()
                    except Exception:
                        pass
                    if self._sdk_api == "dmcan_v1.1":
                        try:
                            self.dll.dmcan_device_disable_channel(self.device, ctypes.c_uint8(self.can_channel))
                        except Exception:
                            pass
                    else:
                        try:
                            self.dll.device_close_channel(self.device, ctypes.c_uint8(self.can_channel))
                        except Exception:
                            pass
                if self._sdk_api == "dmcan_v1.1":
                    try:
                        self.dll.dmcan_device_close(self.device)
                    except Exception:
                        pass
                else:
                    try:
                        self.dll.device_close(self.device)
                    except Exception:
                        pass
        finally:
            self._closed = True
            if self.handle is not None and self.dll is not None:
                # Current Damiao Windows SDK DLLs can corrupt the Python process heap
                # when context/handle destroy functions are called through ctypes.
                # Closing the channel/device is enough for this UI process.
                self.handle = None


@dataclass
class GripperControllerSim:
    backend: MotorBackend
    limits: UiLimits = field(default_factory=UiLimits)
    state: str = GripperState.DISABLED
    fault: str = ""
    friction_close_A: float = 0.10
    friction_open_A: float = 0.08
    _state_started_s: float = field(default_factory=time.monotonic)
    _contact_started_s: float | None = None
    _stall_started_s: float | None = None
    _target_current_A: float = 0.0
    _unload_target_mm: float = 0.0

    def set_state(self, state: str) -> None:
        self.state = state
        self._state_started_s = time.monotonic()
        self._contact_started_s = None
        self._stall_started_s = None

    def enable_ready(self) -> None:
        self.fault = ""
        self.backend.enable()
        self.set_state(GripperState.READY)

    def disable(self) -> None:
        self.backend.disable()
        self.set_state(GripperState.DISABLED)

    def stop(self) -> None:
        self.backend.stop()
        self.backend.disable()
        self.set_state(GripperState.DISABLED)

    def home(self) -> None:
        self.fault = ""
        self.backend.enable()
        self.set_state(GripperState.HOMING)

    def friction_identify(self) -> None:
        self.fault = ""
        self.backend.enable()
        self.set_state(GripperState.FRICTION_ID)

    def clamp(self) -> None:
        self.fault = ""
        self.backend.enable()
        self._target_current_A = min(self.limits.max_current_A, 0.30 + self.limits.target_force_N / 200.0 * 0.90)
        self.set_state(GripperState.GUARDED_CLOSING)

    def open(self) -> None:
        self.fault = ""
        self.backend.enable()
        self.set_state(GripperState.UNLOCK)

    def update(self, dt_s: float) -> MotorFeedback:
        fb = self.backend.update(dt_s)
        now = time.monotonic()

        if self.state == GripperState.HOMING:
            self.backend.command_velocity(-abs(self.limits.homing_speed_mm_s), self.limits.homing_current_A)
            homed = self.backend.is_zero_homed()
            stalled = abs(fb.velocity_mm_s) <= self.limits.stall_speed_threshold_mm_s
            loaded = abs(fb.current_A) >= max(0.05, 0.4 * self.limits.homing_current_A)
            stroke_zero_reached = homed and fb.stroke_mm <= STROKE_MIN_MM + 0.001 and now - self._state_started_s > 0.5
            open_limit_reached = (not homed) and stalled and loaded and now - self._state_started_s > 0.35
            if stroke_zero_reached or open_limit_reached:
                if self._stall_started_s is None:
                    self._stall_started_s = now
                elif now - self._stall_started_s >= max(0.08, self.limits.contact_debounce_s):
                    self.backend.reset_zero()
                    self.backend.stop()
                    self.set_state(GripperState.READY)
            else:
                self._stall_started_s = None

            no_response = (not homed) and stalled and abs(fb.current_A) < 0.03 and now - self._state_started_s > 1.2
            if no_response:
                self.fault = "Homing command has no motion/current; check CANFD motor frames, mode switch, stroke sign, and current limit"
                self.backend.stop()
                self.set_state(GripperState.FAULT)

        elif self.state == GripperState.FRICTION_ID:
            elapsed = now - self._state_started_s
            if elapsed < 0.8:
                self.backend.command_velocity(abs(self.limits.homing_speed_mm_s), self.limits.homing_current_A)
            elif elapsed < 1.6:
                self.backend.command_velocity(-abs(self.limits.homing_speed_mm_s), self.limits.homing_current_A)
            else:
                self.friction_close_A = max(0.05, min(0.5, abs(fb.current_A) + 0.02))
                self.friction_open_A = max(0.05, min(0.5, abs(fb.current_A) + 0.01))
                self.backend.reset_zero()
                self.backend.stop()
                self.set_state(GripperState.READY)

        elif self.state == GripperState.GUARDED_CLOSING:
            v = min(abs(self.limits.close_speed_mm_s), 2.0)
            self.backend.command_velocity(v, self.limits.max_current_A)
            current_over_friction = abs(fb.current_A) - self.friction_close_A
            stalled = abs(fb.velocity_mm_s) <= self.limits.stall_speed_threshold_mm_s
            contact_signal = current_over_friction >= self.limits.contact_current_delta_A and stalled
            if contact_signal:
                if self._contact_started_s is None:
                    self._contact_started_s = now
                elif now - self._contact_started_s >= self.limits.contact_debounce_s:
                    self.backend.stop()
                    self.set_state(GripperState.FORCE_BUILD)
            else:
                self._contact_started_s = None
            if fb.stroke_mm >= STROKE_MAX_MM:
                self.fault = "Contact not found before close soft limit"
                self.backend.stop()
                self.set_state(GripperState.FAULT)

        elif self.state == GripperState.FORCE_BUILD:
            elapsed = now - self._state_started_s
            ramp = min(1.0, elapsed / 0.8)
            self.backend.command_current(self._target_current_A * ramp)
            if elapsed > 1.0:
                self._unload_target_mm = max(STROKE_MIN_MM, fb.stroke_mm - self.limits.unload_mm)
                self.set_state(GripperState.UNLOAD)

        elif self.state == GripperState.UNLOAD:
            self.backend.command_velocity(-0.05, max(self.limits.unlock_current_A, 0.1))
            if fb.stroke_mm <= self._unload_target_mm or now - self._state_started_s > 0.5:
                self.backend.disable()
                self.set_state(GripperState.CLAMP_DONE_DISABLED)

        elif self.state == GripperState.UNLOCK:
            self.backend.command_velocity(-0.08, self.limits.unlock_current_A)
            if abs(fb.velocity_mm_s) > 0.01 or now - self._state_started_s > 0.3:
                self.set_state(GripperState.OPENING)

        elif self.state == GripperState.OPENING:
            self.backend.command_velocity(-abs(self.limits.open_speed_mm_s), self.limits.max_current_A)
            homed = self.backend.is_zero_homed()
            stalled = abs(fb.velocity_mm_s) <= self.limits.stall_speed_threshold_mm_s
            loaded = abs(fb.current_A) >= max(0.05, 0.25 * self.limits.max_current_A)
            if homed and fb.stroke_mm <= STROKE_MIN_MM + 0.01:
                self.backend.stop()
                self.backend.reset_zero()
                self.set_state(GripperState.READY)
            elif (not homed) and stalled and loaded and now - self._state_started_s > 0.35:
                if self._stall_started_s is None:
                    self._stall_started_s = now
                elif now - self._stall_started_s >= max(0.08, self.limits.contact_debounce_s):
                    self.backend.stop()
                    self.backend.reset_zero()
                    self.set_state(GripperState.READY)
            else:
                self._stall_started_s = None

        elif self.state == GripperState.ANTI_JAM:
            self.backend.command_velocity(-0.05, self.limits.unlock_current_A)
            if now - self._state_started_s > 1.0:
                self.fault = "Anti-jam release failed in simulator"
                self.backend.disable()
                self.set_state(GripperState.FAULT)

        return fb


class GripperControlUi(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Gripper Control Test UI")
        self.geometry("1120x760")
        self.minsize(980, 680)

        self.backend = SimulatedMotorBackend()
        self.controller = GripperControllerSim(self.backend)
        self.last_update_s = time.monotonic()
        self.log_rows: list[dict[str, str]] = []
        self.log_item_rows: dict[str, dict[str, str]] = {}

        self.vars: dict[str, tk.StringVar] = {}
        self._build_ui()
        self._apply_object_preset()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(50, self._tick)

    def _var(self, name: str, value: str) -> tk.StringVar:
        var = tk.StringVar(value=value)
        self.vars[name] = var
        return var

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        left = ttk.Frame(self, padding=10)
        left.grid(row=0, column=0, sticky="ns")

        right = ttk.Frame(self, padding=10)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)

        self._build_command_panel(left)
        self._build_backend_panel(left)
        self._build_parameter_panel(left)
        self._build_status_panel(right)
        self._build_log_panel(right)

    def _build_command_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Commands", padding=8)
        frame.pack(fill="x", pady=(0, 10))

        buttons = [
            ("Enable", self._cmd_enable),
            ("CAN probe", self._cmd_can_probe),
            ("HW diagnose", self._cmd_hw_diagnose),
            ("Home low-current", self._cmd_home),
            ("Friction ID", self._cmd_friction_id),
            ("Clamp force", self._cmd_clamp),
            ("Open", self._cmd_open),
            ("Stop + Disable", self._cmd_stop),
            ("Disable", self._cmd_disable),
        ]
        for text, command in buttons:
            ttk.Button(frame, text=text, command=command).pack(fill="x", pady=3)

    def _build_backend_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Backend", padding=8)
        frame.pack(fill="x", pady=(0, 10))

        ttk.Label(frame, text="Mode").grid(row=0, column=0, sticky="w", pady=2)
        backend_var = self._var("backend", "Simulation")
        backend = ttk.Combobox(
            frame,
            textvariable=backend_var,
            values=("Simulation", "Damiao CAN", "Damiao USB2FDCAN"),
            state="readonly",
            width=14,
        )
        backend.grid(row=0, column=1, sticky="ew", pady=2)
        backend.bind("<<ComboboxSelected>>", self._on_backend_mode_changed)

        fields = [
            ("dm_device_index", "DM device index", "0"),
            ("dm_device_type", "DM device type", "1"),
            ("can_interface", "CAN interface", "slcan"),
            ("can_channel", "CAN channel", "COM4"),
            ("can_bitrate", "Bitrate", "1000000"),
            ("canfd_bitrate", "FDCAN data bitrate", "5000000"),
            ("slcan_tty_baud", "SLCAN tty baud", "921600"),
            ("motor_id", "Motor ID", "0x08"),
            ("master_id", "Master ID", "0x18"),
            ("max_phase_current", "Max phase A", "20.0"),
            ("torque_per_amp", "Torque/A Nm", "0.625"),
            ("stroke_sign", "Stroke sign", "1"),
        ]
        for row, (key, label, value) in enumerate(fields, start=1):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w", pady=2)
            ttk.Entry(frame, textvariable=self._var(key, value), width=14).grid(row=row, column=1, sticky="ew", pady=2)

        self.vars["auto_switch_mode"] = tk.StringVar(value="1")
        ttk.Checkbutton(frame, text="Auto mode switch", variable=self.vars["auto_switch_mode"]).grid(
            row=len(fields) + 1, column=0, columnspan=2, sticky="w", pady=(4, 2)
        )
        self.vars["use_canfd"] = tk.StringVar(value="1")
        ttk.Checkbutton(frame, text="Adapter FDCAN/BRS", variable=self.vars["use_canfd"]).grid(
            row=len(fields) + 2, column=0, columnspan=2, sticky="w", pady=(2, 2)
        )
        self.vars["motor_frames_canfd"] = tk.StringVar(value="1")
        ttk.Checkbutton(frame, text="Motor frames CANFD", variable=self.vars["motor_frames_canfd"]).grid(
            row=len(fields) + 3, column=0, columnspan=2, sticky="w", pady=(2, 2)
        )
        self.vars["mode_offset_command_id"] = tk.StringVar(value="1")
        ttk.Checkbutton(frame, text="Command ID + mode", variable=self.vars["mode_offset_command_id"]).grid(
            row=len(fields) + 4, column=0, columnspan=2, sticky="w", pady=(2, 2)
        )

        ttk.Button(frame, text="Connect backend", command=self._connect_backend).grid(
            row=len(fields) + 5, column=0, columnspan=2, sticky="ew", pady=(8, 2)
        )

    def _on_backend_mode_changed(self, _event: object | None = None) -> None:
        mode = self.vars["backend"].get()
        if mode == "Damiao USB2FDCAN":
            self.vars["can_channel"].set("0")
            self.vars["can_interface"].set("slcan")
            self.vars["dm_device_type"].set("1")
            self.vars["use_canfd"].set("1")
            self.vars["motor_frames_canfd"].set("1")
            self.vars["mode_offset_command_id"].set("1")
        elif mode == "Damiao CAN":
            if self.vars["can_channel"].get().strip() in ("", "0", "1"):
                self.vars["can_channel"].set("COM4")
            if self.vars["can_interface"].get().strip().lower() in ("", "pcan"):
                self.vars["can_interface"].set("slcan")

    def _build_parameter_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Limits and Test Object", padding=8)
        frame.pack(fill="x")

        fields = [
            ("target_force_N", "Target force/side (N)", "150"),
            ("close_speed", "Close speed (mm/s)", "0.30"),
            ("open_speed", "Open speed (mm/s)", "0.80"),
            ("max_current", "Max current (A)", "2.00"),
            ("homing_current", "Homing current (A)", "0.35"),
            ("unlock_current", "Unlock current (A)", "0.40"),
            ("contact_delta", "Contact delta (A)", "0.18"),
            ("unload_mm", "Unload stroke (mm)", "0.05"),
            ("object_stroke", "Object/contact stroke (mm)", "8.0"),
            ("object_stiffness", "Object stiffness (A/mm)", "0.80"),
        ]
        for row, (key, label, value) in enumerate(fields):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w", pady=2)
            ttk.Entry(frame, textvariable=self._var(key, value), width=12).grid(row=row, column=1, sticky="e", pady=2)

        ttk.Button(frame, text="Apply parameters", command=self._apply_parameters).grid(
            row=len(fields), column=0, columnspan=2, sticky="ew", pady=(8, 2)
        )
        ttk.Button(frame, text="Apply object", command=self._apply_object_preset).grid(
            row=len(fields) + 1, column=0, columnspan=2, sticky="ew", pady=2
        )

    def _build_status_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Status", padding=8)
        frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        for i in range(6):
            frame.columnconfigure(i, weight=1)

        status_fields = [
            ("state", "State"),
            ("stroke", "Stroke mm"),
            ("velocity", "Velocity mm/s"),
            ("current", "Current A"),
            ("torque", "Torque Nm"),
            ("temp", "Temp C"),
            ("enabled", "Enabled"),
            ("fault", "Fault"),
            ("friction", "Friction A"),
        ]
        self.status_labels: dict[str, ttk.Label] = {}
        for idx, (key, label) in enumerate(status_fields):
            row = idx // 3
            col = (idx % 3) * 2
            ttk.Label(frame, text=label + ":").grid(row=row, column=col, sticky="w", padx=(0, 6), pady=4)
            value = ttk.Label(frame, text="-", width=22)
            value.grid(row=row, column=col + 1, sticky="w", pady=4)
            self.status_labels[key] = value

    def _build_log_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Runtime Log", padding=8)
        frame.grid(row=1, column=0, sticky="nsew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        columns = ("t", "state", "stroke", "velocity", "current", "torque", "enabled", "fault")
        self.tree = ttk.Treeview(frame, columns=columns, show="headings", height=18)
        for col in columns:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=105 if col != "fault" else 220, anchor="center")
        self.tree.grid(row=0, column=0, sticky="nsew")
        self.tree.bind("<<TreeviewSelect>>", self._on_log_selected)

        scroll = ttk.Scrollbar(frame, orient="vertical", command=self.tree.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.tree.configure(yscrollcommand=scroll.set)

        detail_frame = ttk.Frame(frame)
        detail_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        detail_frame.columnconfigure(0, weight=1)
        self.log_detail = tk.Text(detail_frame, height=4, wrap="word")
        self.log_detail.grid(row=0, column=0, sticky="ew")
        self.log_detail.configure(state="disabled")
        detail_scroll = ttk.Scrollbar(detail_frame, orient="vertical", command=self.log_detail.yview)
        detail_scroll.grid(row=0, column=1, sticky="ns")
        self.log_detail.configure(yscrollcommand=detail_scroll.set)

        buttons = ttk.Frame(frame)
        buttons.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        self.vars["log_auto_scroll"] = tk.StringVar(value="1")
        ttk.Checkbutton(buttons, text="Auto-scroll", variable=self.vars["log_auto_scroll"]).pack(side="left")
        ttk.Button(buttons, text="Jump bottom", command=self._jump_log_bottom).pack(side="left", padx=(8, 0))
        ttk.Button(buttons, text="Clear log", command=self._clear_log).pack(side="left", padx=(8, 0))
        ttk.Button(buttons, text="Save CSV", command=self._save_csv).pack(side="left", padx=6)

    def _parse_float(self, key: str, fallback: float) -> float:
        try:
            return float(self.vars[key].get())
        except (KeyError, ValueError):
            return fallback

    def _parse_int(self, key: str, fallback: int) -> int:
        try:
            text = self.vars[key].get().strip()
            return int(text, 0)
        except (KeyError, ValueError):
            return fallback

    def _append_probe_lines(self, label: str, lines: list[str]) -> None:
        now = time.monotonic()
        for line in lines:
            row = {
                "t": f"{now:.3f}",
                "state": label,
                "stroke": "-",
                "velocity": "-",
                "current": "-",
                "torque": "-",
                "enabled": "-",
                "fault": line,
            }
            self.log_rows.append(row)
            item = self.tree.insert("", "end", values=tuple(row.values()))
            self.log_item_rows[item] = row
        self._maybe_autoscroll_log()

    def _diagnose_current_adapter(self) -> list[str]:
        return diagnose_damiao_adapter(
            self.vars["can_channel"].get().strip() or "COM4",
            self._parse_int("slcan_tty_baud", 921_600),
        )

    def _connect_backend(self) -> None:
        mode = self.vars["backend"].get()
        try:
            self.controller.stop()
        except Exception:
            pass
        try:
            self.backend.close()
        except Exception:
            pass

        if mode == "Simulation":
            self.backend = SimulatedMotorBackend()
            self.controller = GripperControllerSim(self.backend)
            self._apply_parameters()
            messagebox.showinfo("Backend", "Simulation backend connected.")
            return

        try:
            if mode == "Damiao USB2FDCAN":
                self.backend = DamiaoUsb2FdCanBackend(
                    device_index=self._parse_int("dm_device_index", 0),
                    device_type=self._parse_int("dm_device_type", 0),
                    can_channel=self._parse_int("can_channel", 0),
                    bitrate=self._parse_int("can_bitrate", 1_000_000),
                    data_bitrate=self._parse_int("canfd_bitrate", 5_000_000),
                    use_canfd=self.vars["use_canfd"].get() == "1",
                    motor_id=self._parse_int("motor_id", 0x08),
                    master_id=self._parse_int("master_id", 0x18),
                    max_phase_current_A=self._parse_float("max_phase_current", 20.0),
                    torque_per_amp_Nm=self._parse_float("torque_per_amp", 0.625),
                    stroke_sign=self._parse_float("stroke_sign", 1.0),
                    auto_switch_mode=self.vars["auto_switch_mode"].get() == "1",
                    motor_frames_canfd=self.vars["motor_frames_canfd"].get() == "1",
                    mode_offset_command_id=self.vars["mode_offset_command_id"].get() == "1",
                )
                info = "Damiao USB2FDCAN backend connected. Start with low-current homing."
            else:
                self.backend = DamiaoCanBackend(
                    interface=self.vars["can_interface"].get().strip(),
                    channel=self.vars["can_channel"].get().strip(),
                    bitrate=self._parse_int("can_bitrate", 1_000_000),
                    tty_baudrate=self._parse_int("slcan_tty_baud", 921_600),
                    data_bitrate=self._parse_int("canfd_bitrate", 5_000_000) if self.vars["use_canfd"].get() == "1" else 0,
                    motor_id=self._parse_int("motor_id", 0x08),
                    master_id=self._parse_int("master_id", 0x18),
                    max_phase_current_A=self._parse_float("max_phase_current", 20.0),
                    torque_per_amp_Nm=self._parse_float("torque_per_amp", 0.625),
                    stroke_sign=self._parse_float("stroke_sign", 1.0),
                    auto_switch_mode=self.vars["auto_switch_mode"].get() == "1",
                    motor_frames_canfd=self.vars["motor_frames_canfd"].get() == "1",
                    mode_offset_command_id=self.vars["mode_offset_command_id"].get() == "1",
                )
                info = "Damiao CAN backend connected. Start with low-current homing."
            self.controller = GripperControllerSim(self.backend)
            self._apply_parameters()
            messagebox.showinfo("Backend", info)
        except Exception as exc:
            self.backend = SimulatedMotorBackend()
            self.controller = GripperControllerSim(self.backend)
            self._apply_parameters()
            self.vars["backend"].set("Simulation")
            details = [f"connect failed: {type(exc).__name__}: {exc}"]
            try:
                details.extend(self._diagnose_current_adapter())
            except Exception as diag_exc:
                details.append(f"diagnosis failed: {type(diag_exc).__name__}: {diag_exc}")
            self._append_probe_lines("HWDiag", details)
            sdk_open_failed = any("open=False" in line for line in details)
            if mode == "Damiao USB2FDCAN" and sdk_open_failed:
                message = (
                    f"{exc}\n\n"
                    "The adapter is enumerated, but DM_DeviceSDK device_open() failed. "
                    "Because official DMTool can open this same adapter, this points to an SDK/firmware "
                    "version mismatch rather than CAN bitrate, motor ID, or enable-command settings.\n\n"
                    "Hardware diagnosis was appended to Runtime Log."
                )
            else:
                message = f"{exc}\n\nHardware diagnosis was appended to Runtime Log."
            messagebox.showerror("CAN connection failed", message)

    def _apply_parameters(self) -> None:
        limits = self.controller.limits
        limits.target_force_N = self._parse_float("target_force_N", limits.target_force_N)
        limits.close_speed_mm_s = max(0.0, self._parse_float("close_speed", limits.close_speed_mm_s))
        limits.open_speed_mm_s = max(0.0, self._parse_float("open_speed", limits.open_speed_mm_s))
        limits.max_current_A = max(0.0, self._parse_float("max_current", limits.max_current_A))
        limits.homing_current_A = max(0.0, self._parse_float("homing_current", limits.homing_current_A))
        limits.unlock_current_A = max(0.0, self._parse_float("unlock_current", limits.unlock_current_A))
        limits.contact_current_delta_A = max(0.0, self._parse_float("contact_delta", limits.contact_current_delta_A))
        limits.unload_mm = max(0.0, self._parse_float("unload_mm", limits.unload_mm))
        self._apply_object_preset()

    def _apply_object_preset(self) -> None:
        contact = max(STROKE_MIN_MM, min(STROKE_MAX_MM, self._parse_float("object_stroke", 8.0)))
        stiffness = max(0.05, self._parse_float("object_stiffness", 0.80))
        if isinstance(self.backend, SimulatedMotorBackend):
            self.backend.set_object(SimulatedObject(contact_stroke_mm=contact, stiffness_A_per_mm=stiffness))

    def _flush_backend_trace(self) -> None:
        try:
            lines = self.backend.drain_trace()
        except Exception:
            return
        if lines:
            self._append_probe_lines("CANTrace", lines)

    def _cmd_enable(self) -> None:
        try:
            self.controller.enable_ready()
            self._flush_backend_trace()
        except Exception as exc:
            messagebox.showerror("Enable failed", str(exc))

    def _cmd_home(self) -> None:
        try:
            self.controller.home()
            self._flush_backend_trace()
        except Exception as exc:
            messagebox.showerror("Home failed", str(exc))

    def _cmd_friction_id(self) -> None:
        try:
            self.controller.friction_identify()
            self._flush_backend_trace()
        except Exception as exc:
            messagebox.showerror("Friction ID failed", str(exc))

    def _cmd_open(self) -> None:
        try:
            self.controller.open()
            self._flush_backend_trace()
        except Exception as exc:
            messagebox.showerror("Open failed", str(exc))

    def _cmd_stop(self) -> None:
        try:
            self.controller.stop()
            self._flush_backend_trace()
        except Exception as exc:
            messagebox.showerror("Stop failed", str(exc))

    def _cmd_disable(self) -> None:
        try:
            self.controller.disable()
            self._flush_backend_trace()
        except Exception as exc:
            messagebox.showerror("Disable failed", str(exc))

    def _cmd_clamp(self) -> None:
        self._apply_parameters()
        if self.controller.limits.target_force_N > 200:
            if not messagebox.askyesno("High force", "Target force exceeds 200 N per side. Continue?"):
                return
        real_backend = isinstance(self.backend, (DamiaoCanBackend, DamiaoUsb2FdCanBackend))
        if real_backend and self.controller.limits.max_current_A > 2.0:
            if not messagebox.askyesno("High current", "Max current exceeds 2.0 A. Continue on real CAN backend?"):
                return
        self.controller.clamp()
        self._flush_backend_trace()

    def _cmd_can_probe(self) -> None:
        try:
            lines = self.backend.probe_can()
        except Exception as exc:
            messagebox.showerror("CAN probe failed", str(exc))
            return

        self._append_probe_lines("CANProbe", lines)
        rx_count = sum(1 for line in lines if line.startswith("RX id="))
        messagebox.showinfo("CAN probe", f"Probe complete. RX frames: {rx_count}. See Runtime Log.")

    def _cmd_hw_diagnose(self) -> None:
        try:
            lines = self._diagnose_current_adapter()
        except Exception as exc:
            messagebox.showerror("HW diagnose failed", str(exc))
            return

        self._append_probe_lines("HWDiag", lines)
        sdk_open_failed = any("open=False" in line for line in lines)
        slcan_silent = any("slcan cmd" in line and "rx=b''" in line for line in lines)
        if sdk_open_failed:
            summary = (
                "Adapter was enumerated, but DeviceSDK device_open() failed. If official DMTool opens it, "
                "use a matching SDK DLL or SLCAN/gsusb firmware. See Runtime Log."
            )
        elif slcan_silent:
            summary = "Serial port opened, but SLCAN commands got no response. See Runtime Log."
        else:
            summary = "Hardware diagnosis complete. See Runtime Log."
        messagebox.showinfo("HW diagnose", summary)

    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(0.2, max(0.001, now - self.last_update_s))
        self.last_update_s = now

        try:
            fb = self.controller.update(dt)
        except Exception as exc:
            self.controller.fault = str(exc)
            self.controller.set_state(GripperState.FAULT)
            try:
                self.backend.disable()
            except Exception:
                pass
            fb = self.backend.feedback if hasattr(self.backend, "feedback") else MotorFeedback(timestamp_s=now, fault=str(exc))
        trace_lines = self.backend.drain_trace()
        if trace_lines:
            self._append_probe_lines("CANTrace", trace_lines)
        self._update_status(fb)
        self._append_log(fb)
        self.after(50, self._tick)

    def _update_status(self, fb: MotorFeedback) -> None:
        self.status_labels["state"].configure(text=self.controller.state)
        self.status_labels["stroke"].configure(text=f"{fb.stroke_mm:.3f}")
        self.status_labels["velocity"].configure(text=f"{fb.velocity_mm_s:.3f}")
        self.status_labels["current"].configure(text=f"{fb.current_A:.3f}")
        self.status_labels["torque"].configure(text=f"{fb.torque_Nm:.3f}")
        self.status_labels["temp"].configure(text=f"{fb.temperature_C:.1f}")
        self.status_labels["enabled"].configure(text=str(fb.enabled))
        self.status_labels["fault"].configure(text=self.controller.fault or fb.fault or "-")
        self.status_labels["friction"].configure(
            text=f"close {self.controller.friction_close_A:.2f}, open {self.controller.friction_open_A:.2f}"
        )

    def _append_log(self, fb: MotorFeedback) -> None:
        if len(self.log_rows) > 3000:
            return
        row = {
            "t": f"{fb.timestamp_s:.3f}",
            "state": self.controller.state,
            "stroke": f"{fb.stroke_mm:.3f}",
            "velocity": f"{fb.velocity_mm_s:.3f}",
            "current": f"{fb.current_A:.3f}",
            "torque": f"{fb.torque_Nm:.3f}",
            "enabled": str(fb.enabled),
            "fault": self.controller.fault or fb.fault,
        }
        self.log_rows.append(row)
        if len(self.log_rows) % 2 == 0:
            item = self.tree.insert("", "end", values=tuple(row.values()))
            self.log_item_rows[item] = row
            children = self.tree.get_children()
            if len(children) > 400:
                self.log_item_rows.pop(children[0], None)
                self.tree.delete(children[0])
            self._maybe_autoscroll_log()

    def _maybe_autoscroll_log(self) -> None:
        if self.vars.get("log_auto_scroll") is None:
            return
        if self.vars["log_auto_scroll"].get() != "1":
            return
        if self.tree.selection():
            return
        self.tree.yview_moveto(1.0)

    def _jump_log_bottom(self) -> None:
        self.tree.selection_remove(self.tree.selection())
        self.tree.yview_moveto(1.0)

    def _set_log_detail(self, text: str) -> None:
        self.log_detail.configure(state="normal")
        self.log_detail.delete("1.0", "end")
        self.log_detail.insert("1.0", text)
        self.log_detail.configure(state="disabled")

    def _on_log_selected(self, _event: object | None = None) -> None:
        selected = self.tree.selection()
        if not selected:
            self._set_log_detail("")
            return
        row = self.log_item_rows.get(selected[0])
        if row is None:
            values = self.tree.item(selected[0], "values")
            self._set_log_detail(" | ".join(str(value) for value in values))
            return
        parts = [f"{key}={value}" for key, value in row.items() if value]
        self._set_log_detail(" | ".join(parts))

    def _clear_log(self) -> None:
        self.log_rows.clear()
        self.log_item_rows.clear()
        for item in self.tree.get_children():
            self.tree.delete(item)
        self._set_log_detail("")

    def _save_csv(self) -> None:
        if not self.log_rows:
            messagebox.showinfo("No data", "No log rows to save.")
            return
        default_path = Path(__file__).with_name("gripper_ui_log.csv")
        path = filedialog.asksaveasfilename(
            title="Save log CSV",
            initialfile=default_path.name,
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.log_rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.log_rows)
        messagebox.showinfo("Saved", f"Saved log:\n{path}")

    def _on_close(self) -> None:
        try:
            self.controller.stop()
        except Exception:
            pass
        try:
            self.backend.close()
        except Exception:
            pass
        self.destroy()


def main() -> None:
    app = GripperControlUi()
    app.mainloop()


if __name__ == "__main__":
    main()
