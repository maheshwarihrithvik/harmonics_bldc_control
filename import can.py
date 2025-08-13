from ctypes import *
#from spaceopro_sdk.constants import MOTOR_NAMES
import ctypes
import json
import os
import time
import numpy as np


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


class CANInterface:
    def __init__(self):
        self.device_type = 4  # Assuming 4 is the device type for VCI_USBCAN2
        self.status_ok = 1  # Assuming 1 is the status code for success
        self.canDLL = None
        self.initialize()

    def initialize(self):
        can_dll_name = "/home/hrithvik/lib/libcontrolcan.so"

        self.canDLL = cdll.LoadLibrary(can_dll_name)
        print(f"Loading CAN library: {can_dll_name}")

        ret = self.canDLL.VCI_OpenDevice(self.device_type, 0, 0)
        if ret != self.status_ok:
            raise RuntimeError("VCI_OpenDevice failed")

        vci_initconfig = VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0, 1, 0x00, 0x14, 0)

        for channel in range(2):
            ret = self.canDLL.VCI_InitCAN(
                self.device_type, 0, channel, byref(vci_initconfig)
            )
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
        # Create and configure the CAN object
        send = VCI_CAN_OBJ()
        send.SendType = 0
        send.RemoteFlag = 0
        send.ExternFlag = 0
        send.DataLen = data_len

        # Set CAN ID and command
        send.ID = can_id
        send.Data[0] = command

        # Convert parameter to 4 bytes and add them to the data field
        res = self.to_int_array(parameter, 4)
        for i in range(4):
            send.Data[i + 1] = res[i]

        if period is not None:
            print("Setting period:", period)
            send.Data[5] = period

        # print(send.Data[:data_len])

        # Transmit the message
        ret = self.canDLL.VCI_Transmit(self.device_type, 0, 0, byref(send), 1)
        # print("Ret:", ret)
        if ret != self.status_ok:
            print(f"Failed to send CAN message to actuator {send.ID} (ret: {ret})")

    def read(self, can_id, command):

        send = VCI_CAN_OBJ()
        send.SendType = 0
        send.RemoteFlag = 0
        send.ExternFlag = 0
        send.DataLen = 1

        # Set CAN ID and command for the single actuator
        send.ID = can_id
        send.Data[0] = command

        if self.canDLL.VCI_Transmit(self.device_type, 0, 0, byref(send), 1) == 1:
            rec = VCI_CAN_OBJ_ARRAY(3000)
            cnt = 5

            # Try to receive the response up to 5 times
            reclen = self.canDLL.VCI_Receive(
                self.device_type, 0, 0, byref(rec.ADDR), 3000, 0
            )
            while reclen <= 0 and cnt > 0:
                reclen = self.canDLL.VCI_Receive(
                    self.device_type, 0, 0, byref(rec.ADDR), 3000, 0
                )
                cnt -= 1
                if cnt == 0:
                    print(f"Ops! ID {send.ID} failed after 5 tries!")
                    return None  # Return None if the actuator failed to respond

            # Process the received data
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
                    # print(f"ID: {send.ID}\tdata: {decimal}")
                return res
        else:
            print(f"Failed to transmit command to ID {can_id}")
            return None


class MotorControl:
    def __init__(self, can, motor_id):
        self.can: CANInterface = can
        self.motor_id = motor_id
        self.reduction_ratio = 101
        # self.joint_limits = self.read_joint_limits()
        # with open("spaceopro_sdk/positions/limits.json", "r") as f:
        #     self.interp_data = json.load(f)

    def read_joint_limits(self):
        try:
            with open(
                os.path.join("spaceopro_sdk", "positions", "joint_limits.json"), "r"
            ) as f:
                joint_limits = json.load(f)
            return joint_limits[f"{self.motor_id}"]
        except Exception as e:
            print("Err:", e)
            return {"max": 1000, "min": -1000}

    def is_within_max_limit(self):
        if self.get_pose() < self.joint_limits["max"]:
            return True
        return False

    def is_within_min_limit(self):
        if self.get_pose() > self.joint_limits["min"]:
            return True
        return False

    def stop(self):
        cmd_register = 29
        self.can.write(self.motor_id, cmd_register, 0)
        # print(f"Motor {self.motor_id} stopped.")

    def set_position(self, target_angle):
        cmd_register = 30
        data = int((target_angle / 360) * self.reduction_ratio * 65536)
        # print("Sending:", data)
        # print("Target :", target_angle)
        self.can.write(self.motor_id, cmd_register, data)
        print(f"Motor {self.motor_id} set to position {data}")

    def start_motor(self, target_speed, directrion):
        # print(self.motor_id)
        cmd_register = 29
        data = int(abs(target_speed) * directrion * self.reduction_ratio * 100 / 360)
        if directrion == 1 and self.is_within_max_limit():
            print(f"Motor {self.motor_id} is within max limit at {self.get_pose()}.")
            # print(data)
            self.can.write(self.motor_id, cmd_register, data)
            print(
                f"Motor {self.motor_id} speed set to {target_speed} in direction {directrion}"
            )
        elif directrion == -1 and self.is_within_min_limit():
            print(f"Motor {self.motor_id} is within min limit at {self.get_pose()}.")
            self.can.write(self.motor_id, cmd_register, data)
            print(
                f"Motor {self.motor_id} speed set to {target_speed} in direction {directrion}"
            )
        else:
            print(f"Motor {self.motor_id} is not within limits.")
            self.stop()
            return

    def set_maxCurrent(self, target_current):
        # print(self.motor_id)
        cmd_register = 32
        data = target_current
        # print(data)
        self.can.write(self.motor_id, cmd_register, data)
        print(f"Motor {self.motor_id} speed set to {data}")

    def set_minCurrent(self, target_current):
        print("MID:", self.motor_id)
        cmd_register = 33
        if not target_current < 0:
            target_current = -target_current
        data = target_current
        print("Data:", data)
        self.can.write(self.motor_id, cmd_register, data)
        print(f"Motor {self.motor_id} speed set to {data}")

    def set_current_limit(self, current_limit):
        # speed_limit speed_limit * self.reduction_ratio * 100) / 360)
        set_max_speed_register = 32
        set_min_speed_register = 33
        current_min_limit = -current_limit
        for register, data in zip(
            [set_max_speed_register, set_min_speed_register],
            [current_limit, current_min_limit],
        ):
            self.can.write(self.motor_id, register, data)

    def set_speed_limit(self, speed_limit):
        speed_limit = int((speed_limit * self.reduction_ratio * 100) / 360)
        set_max_speed_register = 36
        set_min_speed_register = 37
        speed_min_limit = -speed_limit
        for register, data in zip(
            [set_max_speed_register, set_min_speed_register],
            [speed_limit, speed_min_limit],
        ):
            self.can.write(self.motor_id, register, data)

    def get_pose(self):
        cmd = 8
        try:
            res = self.can.read(self.motor_id, cmd)
            # if self.motor_id == 12:
            #     print(res)
            return (res[0] / 65536 / self.reduction_ratio) * 360
        except Exception as e:
            pass
        # print((res[0] / 65536 / self.reduction_ratio) * 360)

    def set_joint_limits(self, min_angle, max_angle):
        min_cmd = 38
        max_cmd = 39
        min_data = int((min_angle / 360) * self.reduction_ratio * 65536)
        max_data = int((max_angle / 360) * self.reduction_ratio * 65536)
        try:
            self.can.write(self.motor_id, min_cmd, min_data)
            self.can.write(self.motor_id, max_cmd, max_data)
            print(f"Motor {self.motor_id} limits set to {min_angle} and {max_angle}")
        except Exception as e:
            print("Err:", e)

    def factory_reset(self):
        cmd = 15
        try:
            if self.motor_id == 12:
                raise Exception("Factory reset of waist motor not allowed!!!")
            self.can.read(self.motor_id, cmd)
            print(f"Motor {self.motor_id} reset to factory settings.")
        except Exception as e:
            print("Err:", e)
        # print((res[0] / 65536 / self.reduction_ratio) * 360)

    def get_mode(self):
        cmd = 3
        res = self.can.read(self.motor_id, cmd)
        print("Getmode:", res)
        return res[0]

    def set_speed_mode(self):
        cmd = 29
        res = self.can.write(self.motor_id, cmd)
        # print(res)
        return res

    # def restore_to_factory(self):
    #     cmd=15
    #     res=self.can.read(self.motor_id,cmd)

    # def save_flash(self):

    def get_speed(self):
        cmd = 6
        res = self.can.read(self.motor_id, cmd)
        return (res[0] / 100 / self.reduction_ratio) * 360

    def get_error(self):
        cmd = 10
        res = self.can.read(self.motor_id, cmd)
        return res

    def get_maxSpeed(self):
        cmd = 24
        res = self.can.read(self.motor_id, cmd)
        print(res)
        return (res[0] / 100 / self.reduction_ratio) * 360

    def get_maxCurrent(self):
        cmd = 53
        res = self.can.read(self.motor_id, cmd)
        return res[0]

    def get_minCurrent(self):
        cmd = 54
        res = self.can.read(self.motor_id, cmd)
        return res[0]

    def get_motorTemp(self):
        cmd = 49
        res = self.can.read(self.motor_id, cmd)
        return res[0]

    def get_cktTemp(self):
        # pass
        cmd = 50
        res = self.can.read(self.motor_id, cmd)
        return res[0]

    def set_canID(self, new_id):
        cmd = 46
        self.can.write(self.motor_id, cmd, new_id)
        self.motor_id = new_id

    def diable_torque(self):
        cmd = 28
        data = 0
        self.can.write(self.motor_id, cmd, data)

    def set_contour_position(
        self, target_angle, contour_speed=60, immediate_update=False
    ):
        """
        Send target position using contour position mode for smooth teleoperation.

        :param target_angle: desired angle in degrees
        :param contour_speed: maximum speed in degrees/sec
        :param immediate_update: if True, immediately update position; otherwise wait until motor reaches previous target
        """

        cmd_register = 87
        target_angle = np.interp(
            target_angle,
            [
                self.interp_data[MOTOR_NAMES[self.motor_id]]["required"]["start"],
                self.interp_data[MOTOR_NAMES[self.motor_id]]["required"]["end"],
            ],
            [
                self.interp_data[MOTOR_NAMES[self.motor_id]]["actual"]["start"],
                self.interp_data[MOTOR_NAMES[self.motor_id]]["actual"]["end"],
            ],
        )
        target_position = int((target_angle / 360) * self.reduction_ratio * 65536)
        contour_speed_converted = int(
            (contour_speed * self.reduction_ratio * 100) / 360
        )
        contour_mode = 0x3F if immediate_update else 0x1F

        send = VCI_CAN_OBJ()
        send.ID = self.motor_id
        send.SendType = 0
        send.RemoteFlag = 0
        send.ExternFlag = 0
        send.DataLen = 8

        target_pos_bytes = self.can.to_int_array(target_position, 4)
        speed_bytes = self.can.to_int_array(contour_speed_converted, 2)

        send.Data[0] = cmd_register
        send.Data[1] = target_pos_bytes[0]
        send.Data[2] = target_pos_bytes[1]
        send.Data[3] = target_pos_bytes[2]
        send.Data[4] = target_pos_bytes[3]
        send.Data[5] = speed_bytes[0]
        send.Data[6] = speed_bytes[1]
        send.Data[7] = contour_mode

        ret = self.can.canDLL.VCI_Transmit(self.can.device_type, 0, 0, byref(send), 1)
        if ret != self.can.status_ok:
            print(f"Failed to send CSP command to motor {self.motor_id}")
        else:
            print(
                f"CSP sent to motor {self.motor_id}: Angle={target_angle} deg, Speed={contour_speed} deg/s"
            )
            
    def set_interpolated_position(self, target_angle, interpolation_period_ms=5):
        """
        Send target position using interpolated position mode for smoother continuous teleoperation.

        :param target_angle: desired angle in degrees
        :param interpolation_period_ms: interpolation cycle in milliseconds (multiple of 5 ms)
        """
        cmd_register = 86
        target_angle = np.interp(
            target_angle,
            [
                self.interp_data[MOTOR_NAMES[self.motor_id]]["required"]["start"],
                self.interp_data[MOTOR_NAMES[self.motor_id]]["required"]["end"],
            ],
            [
                self.interp_data[MOTOR_NAMES[self.motor_id]]["actual"]["start"],
                self.interp_data[MOTOR_NAMES[self.motor_id]]["actual"]["end"],
            ],
        )
        target_position = int((target_angle / 360) * self.reduction_ratio * 65536)
        interpolation_cycle = max(1, min(255, interpolation_period_ms // 5))

        send = VCI_CAN_OBJ()
        send.ID = self.motor_id
        send.SendType = 0
        send.RemoteFlag = 0
        send.ExternFlag = 0
        send.DataLen = 6

        target_pos_bytes = self.can.to_int_array(target_position, 4)

        send.Data[0] = cmd_register
        send.Data[1] = target_pos_bytes[0]
        send.Data[2] = target_pos_bytes[1]
        send.Data[3] = target_pos_bytes[2]
        send.Data[4] = target_pos_bytes[3]
        send.Data[5] = interpolation_cycle

        ret = self.can.canDLL.VCI_Transmit(self.can.device_type, 0, 0, byref(send), 1)
        if ret != self.can.status_ok:
            print(f"❌ Failed to send IPM command to motor {self.motor_id}")
        else:
            print(
                f"✅ IPM sent to motor {self.motor_id}: Angle={target_angle:.2f}°, Cycle={interpolation_period_ms}ms"
            )
            print(f"Sending angle={target_angle:.2f}°, raw_pos={target_position}")


    def set_ipm_timing(self, cycle_ms=20, timeout_ms=200):
        cmd = 89
        cycle = cycle_ms
        timeout = timeout_ms

        send = VCI_CAN_OBJ()
        send.ID = self.motor_id
        send.SendType = 0
        send.RemoteFlag = 0
        send.ExternFlag = 0
        send.DataLen = 6

        send.Data[0] = cmd
        send.Data[1] = cycle & 0xFF
        send.Data[2] = (cycle >> 8) & 0xFF
        send.Data[3] = timeout & 0xFF
        send.Data[4] = (timeout >> 8) & 0xFF
        send.Data[5] = 0

        self.can.canDLL.VCI_Transmit(self.can.device_type, 0, 0, byref(send), 1)


    def set_angle(self, target_angle):
        cmd_register = 30

        angle = np.interp(
            target_angle,
            [
                self.interp_data[MOTOR_NAMES[self.motor_id]]["required"]["start"],
                self.interp_data[MOTOR_NAMES[self.motor_id]]["required"]["end"],
            ],
            [
                self.interp_data[MOTOR_NAMES[self.motor_id]]["actual"]["start"],
                self.interp_data[MOTOR_NAMES[self.motor_id]]["actual"]["end"],
            ],
        )

        # print("Angle:", angle)

        data = int((angle / 360) * self.reduction_ratio * 65536)
        # print("Sending:", data)
        # print("Target :", angle)
        self.can.write(self.motor_id, cmd_register, data)
        # print(f"Motor {self.motor_id} set to position {data}")

    def clear_error(self):
        send = VCI_CAN_OBJ()
        send.ID = self.motor_id
        send.SendType = 0
        send.RemoteFlag = 0
        send.ExternFlag = 0
        send.DataLen = 1
        send.Data[0] = 0x0B  # Clear error
        self.can.canDLL.VCI_Transmit(self.can.device_type, 0, 0, byref(send), 1)


if __name__ == "__main__":

    can = CANInterface()
    elbow_motor_ID = 20
    Elbow = MotorControl(can, elbow_motor_ID)

    # Shoulder.set_speed_limit(5)
    # Shoulder.set_position(90)
    # # Shoulder.set_speed(10)
    # time.sleep(2)
    Elbow.set_speed_limit(50)
    Elbow.set_position(180)
    #Elbow.set_speed(500)
    # Elbow.set_angle(180)

    while(1):
    #     print(Shoulder.get_pose())
        print(Elbow.get_pose())

        time.sleep(0.5)
