#!/usr/bin/env python3
from ctypes import *
import ctypes
import json
import os
import time
import numpy as np

# ------------------ ctypes structures ------------------
class VCI_INIT_CONFIG(Structure):
    _fields_ = [
        ("AccCode", c_uint),
        ("AccMask", c_uint),
        ("Reserved", c_uint),
        ("Filter", c_ubyte),
        ("Timing0", c_ubyte),
        ("Timing1", c_ubyte),
        ("Mode", c_ubyte),
    ]


class VCI_CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint),
        ("TimeStamp", c_uint),
        ("TimeFlag", c_ubyte),
        ("SendType", c_ubyte),
        ("RemoteFlag", c_ubyte),
        ("ExternFlag", c_ubyte),
        ("DataLen", c_ubyte),
        ("Data", c_ubyte * 8),
        ("Reserved", c_ubyte * 3),
    ]


class VCI_CAN_OBJ_ARRAY(Structure):
    _fields_ = [
        ("SIZE", ctypes.c_uint16),
        ("STRUCT_ARRAY", ctypes.POINTER(VCI_CAN_OBJ)),
    ]

    def __init__(self, num_of_structs):
        self.STRUCT_ARRAY = ctypes.cast(
            (VCI_CAN_OBJ * num_of_structs)(), ctypes.POINTER(VCI_CAN_OBJ)
        )
        self.SIZE = num_of_structs
        self.ADDR = self.STRUCT_ARRAY[0]


# ------------------ CAN interface ------------------
class CANInterface:
    def __init__(self):
        self.device_type = 4  # VCI_USBCAN2
        self.status_ok = 1    # success code
        self.canDLL = None
        self.initialize()

    def initialize(self):
        can_dll_name = "/home/saurabh/Downloads/joint_control/lib/libcontrolcan.so"
        self.canDLL = cdll.LoadLibrary(can_dll_name)
        print(f"Loading CAN library: {can_dll_name}")

        ret = self.canDLL.VCI_OpenDevice(self.device_type, 0, 0)
        if ret != self.status_ok:
            raise RuntimeError("VCI_OpenDevice failed")

        vci_initconfig = VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0, 1, 0x00, 0x14, 0)

        for channel in range(2):
            ret = self.canDLL.VCI_InitCAN(self.device_type, 0, channel, byref(vci_initconfig))
            if ret != self.status_ok:
                raise RuntimeError(f"VCI_InitCAN channel {channel} failed")
            ret = self.canDLL.VCI_StartCAN(self.device_type, 0, channel)
            if ret != self.status_ok:
                raise RuntimeError(f"VCI_StartCAN channel {channel} failed")

    def convert_hex_array_to_decimal(self, hex_array):
        result = 0
        for i in range(4):
            result = (result << 8) | hex_array[i]
        if result > 0x7FFFFFFF:
            result -= 0x100000000
        return result

    def to_int_array(self, number, size):
        unsigned_number = number if number >= 0 else (1 << (size * 8)) + number
        return [(unsigned_number >> (8 * i)) & 0xFF for i in range(size)]

    def write(self, can_id, command, parameter, data_len=5, period=None):
        """Send a single CAN frame with 1B command + 4B parameter. Optionally set a period byte at index 5."""
        send = VCI_CAN_OBJ()
        send.SendType = 0
        send.RemoteFlag = 0
        send.ExternFlag = 0
        send.DataLen = data_len

        send.ID = can_id
        send.Data[0] = command

        res = self.to_int_array(parameter, 4)
        for i in range(4):
            send.Data[i + 1] = res[i]

        if period is not None and data_len >= 6:
            send.Data[5] = period

        ret = self.canDLL.VCI_Transmit(self.device_type, 0, 0, byref(send), 1)
        if ret != self.status_ok:
            print(f"Failed to send CAN message to actuator {send.ID} (ret: {ret})")

    def read(self, can_id, command):
        """Send a 1B command and read back responses; returns list of signed 32-bit ints or None."""
        send = VCI_CAN_OBJ()
        send.SendType = 0
        send.RemoteFlag = 0
        send.ExternFlag = 0
        send.DataLen = 1

        send.ID = can_id
        send.Data[0] = command

        if self.canDLL.VCI_Transmit(self.device_type, 0, 0, byref(send), 1) == 1:
            rec = VCI_CAN_OBJ_ARRAY(3000)
            cnt = 5

            reclen = self.canDLL.VCI_Receive(self.device_type, 0, 0, byref(rec.ADDR), 3000, 0)
            while reclen <= 0 and cnt > 0:
                reclen = self.canDLL.VCI_Receive(self.device_type, 0, 0, byref(rec.ADDR), 3000, 0)
                cnt -= 1
                if cnt == 0:
                    print(f"Ops! ID {send.ID} failed after 5 tries!")
                    return None

            if reclen > 0:
                res = []
                for j in range(reclen):
                    data_array = [
                        rec.STRUCT_ARRAY[j].Data[4],
                        rec.STRUCT_ARRAY[j].Data[3],
                        rec.STRUCT_ARRAY[j].Data[2],
                        rec.STRUCT_ARRAY[j].Data[1],
                    ]
                    decimal = self.convert_hex_array_to_decimal(data_array)
                    res.append(decimal)
                return res
        else:
            print(f"Failed to transmit command to ID {can_id}")
            return None


# ------------------ Motor control ------------------
class MotorControl:
    def __init__(self, can: CANInterface, motor_id: int):
        self.can = can
        self.motor_id = int(motor_id)
        self.reduction_ratio = 101
        # ensure joint_limits exists (prevents AttributeError)
        self.joint_limits = {"min": float("-inf"), "max": float("inf")}
        # optional: load from file if present
        try:
            jl = self.read_joint_limits()
            if jl:
                self.joint_limits = jl
        except Exception:
            pass

    def read_joint_limits(self):
        try:
            with open(os.path.join("spaceopro_sdk", "positions", "joint_limits.json"), "r") as f:
                joint_limits = json.load(f)
            return joint_limits.get(f"{self.motor_id}", {"max": 1000, "min": -1000})
        except Exception as e:
            # file not present -> ignore
            return None

    def is_within_max_limit(self):
        pose = self.get_pose()
        if pose is None:
            return True
        return pose < self.joint_limits["max"]

    def is_within_min_limit(self):
        pose = self.get_pose()
        if pose is None:
            return True
        return pose > self.joint_limits["min"]

    def stop(self):
        cmd_register = 29
        self.can.write(self.motor_id, cmd_register, 0)
        print(f"Motor {self.motor_id} stopped.")

    def set_position(self, target_angle, ramp_time=2.0, steps=1000):
        """
        Smoothly move motor to target_angle in degrees.
        - ramp_time: total time in seconds to reach position
        - steps: number of intermediate position steps
        """
        cmd_register = 30

        # Get current position
        current_angle = self.get_pose()
        if current_angle is None:
            current_angle = target_angle  # fallback if cannot read

        # Compute increments
        diff = target_angle - current_angle
        step_size = diff / steps
        delay_per_step = ramp_time / steps

        # Ramp loop
        for i in range(steps):
            intermediate_angle = current_angle + step_size * (i + 1)
            data = int((intermediate_angle / 360.0) * self.reduction_ratio * 65536)
            self.can.write(self.motor_id, cmd_register, data)
            time.sleep(delay_per_step)

        print(f"[Motor {self.motor_id}] Smooth move to {target_angle}° completed.")

    # Fix: make this set speed mode first (many drivers require mode set)
    def set_speed_mode(self):
        """Set drive to speed mode (sends register 29 with parameter 0)."""
        cmd = 29
        # write register 29 with parameter 0 to select speed mode (as in old code pattern)
        self.can.write(self.motor_id, cmd, 0)
        print(f"Motor {self.motor_id} set to speed mode (reg {cmd}=0).")

    def start_motor(self, target_speed, direction, ramp_time=2.0, steps=20):
        """
        Smoothly accelerate motor to target_speed (RPM) in given direction.
        - direction: 1 for forward, -1 for reverse
        - ramp_time: total ramp-up time in seconds
        - steps: number of intermediate speed steps
        """
        direction = 1 if direction >= 0 else -1
        target_speed = float(abs(target_speed))

        # If zero -> stop motor smoothly
        if target_speed == 0:
            self.stop()
            return

        self.set_speed_mode()
        time.sleep(0.02)

        # Get current speed from motor, fallback to 0 if unknown
        current_speed = self.get_speed() or 0.0
        current_speed = abs(current_speed)

        # Create step increments
        step_size = (target_speed - current_speed) / steps
        delay_per_step = ramp_time / steps

        # Ramp loop
        for i in range(steps):
            speed_cmd = current_speed + step_size * (i + 1)
            raw = int(speed_cmd * direction * self.reduction_ratio * 100 / 360)
            self.can.write(self.motor_id, 29, raw)
            time.sleep(delay_per_step)

        # Final exact speed
        raw_final = int(target_speed * direction * self.reduction_ratio * 100 / 360)
        self.can.write(self.motor_id, 29, raw_final)
        print(f"[Motor {self.motor_id}] Smooth start to {target_speed} RPM (dir {direction}) done.")

    def set_speed_limit(self, speed_limit):
        """Keep same register usage as old file (36 & 37)."""
        scaled = int((speed_limit * self.reduction_ratio * 100) / 360)
        self.can.write(self.motor_id, 36, scaled)
        # Older firmware sometimes expects signed min; preserve original behavior by sending negative
        self.can.write(self.motor_id, 37, -scaled)
        print(f"[Motor {self.motor_id}] Speed limit ±{speed_limit} RPM (scaled {scaled}).")

    def get_pose(self):
        cmd = 8
        try:
            res = self.can.read(self.motor_id, cmd)
            if not res:
                return None
            return (res[0] / 65536.0 / self.reduction_ratio) * 360.0
        except Exception:
            return None

    def set_joint_limits(self, min_angle, max_angle):
        min_cmd = 38
        max_cmd = 39
        min_data = int((min_angle / 360.0) * self.reduction_ratio * 65536)
        max_data = int((max_angle / 360.0) * self.reduction_ratio * 65536)
        try:
            self.can.write(self.motor_id, min_cmd, min_data)
            self.can.write(self.motor_id, max_cmd, max_data)
            self.joint_limits = {"min": float(min_angle), "max": float(max_angle)}
            print(f"Motor {self.motor_id} limits set to {min_angle} and {max_angle}")
        except Exception as e:
            print("Err:", e)

    # keep other helpers from working file
    def get_speed(self):
        cmd = 6
        res = self.can.read(self.motor_id, cmd)
        if not res:
            return None
        return (res[0] / 100.0 / self.reduction_ratio) * 360.0

    def get_error(self):
        cmd = 10
        return self.can.read(self.motor_id, cmd)

    def get_maxSpeed(self):
        cmd = 24
        res = self.can.read(self.motor_id, cmd)
        if not res:
            return None
        return (res[0] / 100.0 / self.reduction_ratio) * 360.0

    def set_canID(self, new_id):
        cmd = 46
        self.can.write(self.motor_id, cmd, int(new_id))
        self.motor_id = int(new_id)

    def disable_torque(self):
        cmd = 28
        self.can.write(self.motor_id, cmd, 0)

    def clear_error(self):
        send = VCI_CAN_OBJ()
        send.ID = self.motor_id
        send.SendType = 0
        send.RemoteFlag = 0
        send.ExternFlag = 0
        send.DataLen = 1
        send.Data[0] = 0x0B  # Clear error
        self.can.canDLL.VCI_Transmit(self.can.device_type, 0, 0, byref(send), 1)


# ------------------ Main (example) ------------------
if __name__ == "__main__":
    can = CANInterface()
    elbow_motor_ID = 20
    Elbow = MotorControl(can, elbow_motor_ID)

    # Keep behavior from old working example:
    # set speed limit and send a position command (old working pattern)
    Elbow.set_speed_limit(30)
    Elbow.set_position(0)   # old working command that previously moved motor

    # Wait a bit and then try a small velocity command test
    time.sleep(1.0)

    # Try velocity: small RPM so it is safe
    # NOTE: use positive RPM for forward, negative for reverse.
    # Elbow.start_motor(90, -1)    # try 10 RPM forward
    # time.sleep(10)
    # Elbow.start_motor(0, -1)
    # time.sleep(10)

    # print telemetry
    while True:
        pose = Elbow.get_pose()
        print("Elbow pose (deg):", pose)
        time.sleep(0.5)
        # Elbow.start_motor(90, -1)    # try 90 RPM reverse
        # time.sleep(10)
        # Elbow.start_motor(0, 1)    # try 0 RPM forward
        # time.sleep(10)
        # Elbow.start_motor(90, 1)   # try 90 RPM forward
        # time.sleep(10)
        # Elbow.start_motor(0, -1)    # try 0 RPM reverse
        # time.sleep(10)
