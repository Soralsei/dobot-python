from enum import IntEnum
from typing import Optional
import serial
import threading

from .message import Message, IODirection


class PTPMode(IntEnum):
    JUMP_XYZ = 0x00
    MOVJ_XYZ = 0x01
    MOVL_XYZ = 0x02
    JUMP_ANGLE = 0x03
    MOVJ_ANGLE = 0x04
    MOVL_ANGLE = 0x05
    MOVJ_INC = 0x06
    MOVL_INC = 0x07
    MOVJ_XYZ_INC = 0x08
    JUMP_MOVL_XYZ = 0x09


class GPIO(IntEnum):
    PORT_GP1 = 0x00
    PORT_GP2 = 0x01
    PORT_GP4 = 0x02
    PORT_GP5 = 0x03


class Interface:
    def __init__(self, port: Optional[str]):
        self.lock = threading.RLock()

        self.serial = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=2,
        )

    def send(self, message):
        with self.lock:
            self.serial.write(message.package())
            self.serial.flush()
            response = Message.read(self.serial)

        if response is None:
            return None

        return response.params

    def close(self, force: bool = False):
        """Close the serial connection properly."""
        self.stop_queue(force=force)
        self.clear_queue()
        self.serial.close()

    def connected(self):
        return self.serial.is_open

    def get_device_serial_number(self):
        request = Message(
            [0xAA, 0xAA], 2, 0, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_device_serial_number(self, serial_number):
        request = Message(
            [0xAA, 0xAA], 2, 0, True, False, [serial_number], direction=IODirection.OUT
        )
        return self.send(request)

    def get_device_name(self):
        request = Message(
            [0xAA, 0xAA], 2, 1, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_device_name(self, device_name):
        request = Message(
            [0xAA, 0xAA], 2, 1, True, False, [device_name], direction=IODirection.OUT
        )
        return self.send(request)

    def get_device_version(self):
        request = Message(
            [0xAA, 0xAA], 2, 2, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_sliding_rail_status(self, enable, version):
        request = Message(
            [0xAA, 0xAA], 2, 3, True, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    # Time in milliseconds since start
    def get_device_time(self):
        request = Message(
            [0xAA, 0xAA], 2, 4, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def get_device_id(self):
        request = Message(
            [0xAA, 0xAA], 2, 5, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def get_pose(self):
        request = Message(
            [0xAA, 0xAA], 2, 10, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def reset_pose(self, manual, rear_arm_angle, front_arm_angle):
        request = Message(
            [0xAA, 0xAA],
            2,
            11,
            True,
            False,
            [manual, rear_arm_angle, front_arm_angle],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_sliding_rail_pose(self):
        request = Message(
            [0xAA, 0xAA], 2, 13, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def get_alarms(self):
        request = Message(
            [0xAA, 0xAA], 2, 20, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def clear_alarms(self):
        request = Message(
            [0xAA, 0xAA], 2, 21, True, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def get_homing_parameters(self):
        request = Message(
            [0xAA, 0xAA], 2, 30, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_homing_parameters(self, x, y, z, r, queue=True):
        request = Message(
            [0xAA, 0xAA], 2, 30, True, queue, [x, y, z, r], direction=IODirection.OUT
        )
        return self.send(request)

    def set_homing_command(self, command, queue=True):
        request = Message(
            [0xAA, 0xAA], 2, 31, True, queue, [command], direction=IODirection.OUT
        )
        return self.send(request)

    # TODO: Reference is wrong here, arm does not send the said value
    def get_auto_leveling(self):
        request = Message(
            [0xAA, 0xAA], 2, 32, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_auto_leveling(self, enable, accuracy, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            32,
            True,
            queue,
            [enable, accuracy],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_handheld_teaching_mode(self):
        request = Message(
            [0xAA, 0xAA], 2, 40, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_handheld_teaching_mode(self, mode):
        request = Message(
            [0xAA, 0xAA], 2, 40, True, False, [mode], direction=IODirection.OUT
        )
        return self.send(request)

    def get_handheld_teaching_state(self):
        request = Message(
            [0xAA, 0xAA], 2, 41, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_handheld_teaching_state(self, enable):
        request = Message(
            [0xAA, 0xAA], 2, 41, True, False, [enable], direction=IODirection.OUT
        )
        return self.send(request)

    def get_handheld_teaching_trigger(self):
        request = Message(
            [0xAA, 0xAA], 2, 42, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def get_end_effector_params(self):
        request = Message(
            [0xAA, 0xAA], 2, 60, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_end_effector_params(self, bias_x, bias_y, bias_z):
        request = Message(
            [0xAA, 0xAA],
            2,
            60,
            True,
            False,
            [bias_x, bias_y, bias_z],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_end_effector_laser(self):
        request = Message(
            [0xAA, 0xAA], 2, 61, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_end_effector_laser(self, enable_control, enable_laser, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            61,
            True,
            queue,
            [enable_control, enable_laser],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_end_effector_suction_cup(self):
        request = Message(
            [0xAA, 0xAA], 2, 62, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_end_effector_suction_cup(self, enable_control, enable_suction, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            62,
            True,
            queue,
            [enable_control, enable_suction],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_end_effector_gripper(self):
        request = Message(
            [0xAA, 0xAA], 2, 63, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_end_effector_gripper(self, enable_control, enable_grip, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            63,
            True,
            queue,
            [enable_control, enable_grip],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_jog_joint_params(self):
        request = Message(
            [0xAA, 0xAA], 2, 70, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    # TODO: Does not work - but is implemented according to spec. Bad documentation?
    def set_jog_joint_params(self, velocity, acceleration, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            70,
            True,
            queue,
            velocity + acceleration,
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_jog_coordinate_params(self):
        request = Message(
            [0xAA, 0xAA], 2, 71, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    # TODO: Does not work - but is implemented according to spec. Bad documentation?
    def set_jog_coordinate_params(self, velocity, acceleration, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            71,
            True,
            queue,
            velocity + acceleration,
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_jog_common_params(self):
        request = Message(
            [0xAA, 0xAA], 2, 72, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    # TODO: Does not work - but is implemented according to spec. Bad documentation?
    def set_jog_common_params(self, velocity_ratio, acceleration_ratio, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            72,
            True,
            queue,
            [velocity_ratio, acceleration_ratio],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def set_jog_command(self, jog_type, command, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            73,
            True,
            queue,
            [jog_type, command],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_sliding_rail_jog_params(self):
        request = Message(
            [0xAA, 0xAA], 2, 74, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_sliding_rail_jog_params(self, velocity, acceleration, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            74,
            True,
            queue,
            [velocity, acceleration],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_point_to_point_joint_params(self):
        request = Message(
            [0xAA, 0xAA], 2, 80, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_point_to_point_joint_params(self, velocity, acceleration, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            80,
            True,
            queue,
            velocity + acceleration,
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_point_to_point_coordinate_params(self):
        request = Message(
            [0xAA, 0xAA], 2, 81, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_point_to_point_coordinate_params(
        self,
        coordinate_velocity,
        effector_velocity,
        coordinate_acceleration,
        effector_acceleration,
        queue=True,
    ):
        request = Message(
            [0xAA, 0xAA],
            2,
            81,
            True,
            queue,
            [
                coordinate_velocity,
                effector_velocity,
                coordinate_acceleration,
                effector_acceleration,
            ],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_point_to_point_jump_params(self):
        request = Message(
            [0xAA, 0xAA], 2, 82, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_point_to_point_jump_params(self, jump_height, z_limit, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            82,
            True,
            queue,
            [jump_height, z_limit],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_point_to_point_common_params(self):
        request = Message(
            [0xAA, 0xAA], 2, 83, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_point_to_point_common_params(
        self, velocity_ratio, acceleration_ratio, queue=True
    ):
        request = Message(
            [0xAA, 0xAA],
            2,
            83,
            True,
            queue,
            [velocity_ratio, acceleration_ratio],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def set_point_to_point_command(
        self, mode: int, x: float, y: float, z: float, r: float, queue: bool = True
    ):
        request = Message(
            [0xAA, 0xAA],
            2,
            84,
            True,
            queue,
            [mode, x, y, z, r],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_point_to_point_sliding_rail_params(self):
        request = Message(
            [0xAA, 0xAA], 2, 85, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_point_to_point_sliding_rail_params(
        self, velocity, acceleration, queue=True
    ):
        request = Message(
            [0xAA, 0xAA],
            2,
            85,
            True,
            queue,
            [velocity, acceleration],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def set_point_to_point_sliding_rail_command(self, mode, x, y, z, r, l, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            86,
            True,
            queue,
            [mode, x, y, z, r, l],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_point_to_point_jump2_params(self):
        request = Message(
            [0xAA, 0xAA], 2, 87, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_point_to_point_jump2_params(
        self, start_height, end_height, z_limit, queue=True
    ):
        request = Message(
            [0xAA, 0xAA],
            2,
            87,
            True,
            queue,
            [start_height, end_height, z_limit],
            direction=IODirection.OUT,
        )
        return self.send(request)

    # TODO: Reference is ambigious here - needs testing
    def set_point_to_point_po_command(self, mode, x, y, z, r, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            88,
            True,
            queue,
            [mode, x, y, z, r],
            direction=IODirection.OUT,
        )
        return self.send(request)

    # TODO: Reference is ambigious here - needs testing
    def set_point_to_point_sliding_rail_po_command(
        self, ratio, address, level, queue=True
    ):
        request = Message(
            [0xAA, 0xAA],
            2,
            89,
            True,
            queue,
            [ratio, address, level],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_continous_trajectory_params(self):
        request = Message(
            [0xAA, 0xAA], 2, 90, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_continous_trajectory_params(
        self, max_planned_acceleration, max_junction_velocity, acceleration, queue=True
    ):
        request = Message(
            [0xAA, 0xAA],
            2,
            90,
            True,
            queue,
            [max_planned_acceleration, max_junction_velocity, acceleration, 0],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def set_continous_trajectory_real_time_params(
        self, max_planned_acceleration, max_junction_velocity, period, queue=True
    ):
        request = Message(
            [0xAA, 0xAA],
            2,
            90,
            True,
            queue,
            [max_planned_acceleration, max_junction_velocity, period, 1],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def set_continous_trajectory_command(self, mode, x, y, z, velocity, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            91,
            True,
            queue,
            [mode, x, y, z, velocity],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def set_continous_trajectory_laser_engraver_command(
        self, mode, x, y, z, power, queue=True
    ):
        request = Message(
            [0xAA, 0xAA],
            2,
            92,
            True,
            queue,
            [mode, x, y, z, power],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_arc_params(self):
        request = Message(
            [0xAA, 0xAA], 2, 100, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_arc_params(
        self,
        coordinate_velocity,
        effector_velocity,
        coordinate_acceleration,
        effector_acceleration,
        queue=True,
    ):
        request = Message(
            [0xAA, 0xAA],
            2,
            100,
            True,
            queue,
            [
                coordinate_velocity,
                effector_velocity,
                coordinate_acceleration,
                effector_acceleration,
            ],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def set_arc_command(self, circumference_point, ending_point, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            101,
            True,
            queue,
            circumference_point + ending_point,
            direction=IODirection.OUT,
        )
        return self.send(request)

    def wait(self, milliseconds, queue=True):
        request = Message(
            [0xAA, 0xAA], 2, 110, True, queue, [milliseconds], direction=IODirection.OUT
        )
        return self.send(request)

    def set_trigger_command(self, address, mode, condition, threshold, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            120,
            True,
            queue,
            [address, mode, condition, threshold],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_io_multiplexing(self):
        request = Message(
            [0xAA, 0xAA], 2, 130, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_io_multiplexing(self, address, multiplex, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            130,
            True,
            queue,
            [address, multiplex],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_io_do(self):
        request = Message(
            [0xAA, 0xAA], 2, 131, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_io_do(self, address, level, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            131,
            True,
            queue,
            [address, level],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_io_pwm(self):
        request = Message(
            [0xAA, 0xAA], 2, 132, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_io_pwm(self, address, frequency, duty_cycle, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            132,
            True,
            queue,
            [address, frequency, duty_cycle],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_io_di(self):
        request = Message(
            [0xAA, 0xAA], 2, 133, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def get_io_adc(self):
        request = Message(
            [0xAA, 0xAA], 2, 134, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_extended_motor_velocity(self, index, enable, speed, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            135,
            True,
            queue,
            [index, enable, speed],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_color_sensor(self, index):
        request = Message(
            [0xAA, 0xAA], 2, 137, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_color_sensor(self, index, enable, port, version, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            137,
            True,
            queue,
            [enable, port, version],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_ir_switch(self, index):
        request = Message(
            [0xAA, 0xAA], 2, 138, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_ir_switch(self, index, enable, port, version, queue=True):
        request = Message(
            [0xAA, 0xAA],
            2,
            138,
            True,
            queue,
            [enable, port, version],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_angle_sensor_static_error(self, index):
        request = Message(
            [0xAA, 0xAA], 2, 140, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_angle_sensor_static_error(
        self, index, rear_arm_angle_error, front_arm_angle_error
    ):
        request = Message(
            [0xAA, 0xAA],
            2,
            140,
            True,
            False,
            [rear_arm_angle_error, front_arm_angle_error],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_wifi_status(self):
        request = Message(
            [0xAA, 0xAA], 2, 150, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_wifi_status(self, index, enable):
        request = Message(
            [0xAA, 0xAA], 2, 150, True, False, [enable], direction=IODirection.OUT
        )
        return self.send(request)

    def get_wifi_ssid(self):
        request = Message(
            [0xAA, 0xAA], 2, 151, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_wifi_ssid(self, index, ssid):
        request = Message(
            [0xAA, 0xAA], 2, 151, True, False, [ssid], direction=IODirection.OUT
        )
        return self.send(request)

    def get_wifi_password(self):
        request = Message(
            [0xAA, 0xAA], 2, 152, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_wifi_password(self, index, ssid):
        request = Message(
            [0xAA, 0xAA], 2, 152, True, False, [ssid], direction=IODirection.OUT
        )
        return self.send(request)

    def get_wifi_address(self):
        request = Message(
            [0xAA, 0xAA], 2, 153, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    # 192.168.1.1 = a.b.c.d
    def set_wifi_address(self, index, use_dhcp, a, b, c, d):
        request = Message(
            [0xAA, 0xAA],
            2,
            153,
            True,
            False,
            [use_dhcp, a, b, c, d],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_wifi_netmask(self):
        request = Message(
            [0xAA, 0xAA], 2, 154, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    # 255.255.255.0 = a.b.c.d
    def set_wifi_netmask(self, index, a, b, c, d):
        request = Message(
            [0xAA, 0xAA], 2, 154, True, False, [a, b, c, d], direction=IODirection.OUT
        )
        return self.send(request)

    def get_wifi_gateway(self):
        request = Message(
            [0xAA, 0xAA], 2, 155, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    # 192.168.1.1 = a.b.c.d
    def set_wifi_gateway(self, index, use_dhcp, a, b, c, d):
        request = Message(
            [0xAA, 0xAA],
            2,
            155,
            True,
            False,
            [use_dhcp, a, b, c, d],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_wifi_dns(self):
        request = Message(
            [0xAA, 0xAA], 2, 156, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    # 192.168.1.1 = a.b.c.d
    def set_wifi_dns(self, index, use_dhcp, a, b, c, d):
        request = Message(
            [0xAA, 0xAA],
            2,
            156,
            True,
            False,
            [use_dhcp, a, b, c, d],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def get_wifi_connect_status(self):
        request = Message(
            [0xAA, 0xAA], 2, 157, False, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def set_lost_step_params(self, param):
        request = Message(
            [0xAA, 0xAA], 2, 170, True, False, [param], direction=IODirection.OUT
        )
        return self.send(request)

    def set_lost_step_command(self):
        request = Message(
            [0xAA, 0xAA], 2, 171, True, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def start_queue(self):
        request = Message(
            [0xAA, 0xAA], 2, 240, True, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def stop_queue(self, force=False):
        request = Message(
            [0xAA, 0xAA],
            2,
            242 if force else 241,
            True,
            False,
            [],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def start_queue_download(self, total_loop, line_per_loop):
        request = Message(
            [0xAA, 0xAA],
            2,
            243,
            True,
            False,
            [total_loop, line_per_loop],
            direction=IODirection.OUT,
        )
        return self.send(request)

    def stop_queue_download(self):
        request = Message(
            [0xAA, 0xAA], 2, 244, True, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def clear_queue(self):
        request = Message(
            [0xAA, 0xAA], 2, 245, True, False, [], direction=IODirection.OUT
        )
        return self.send(request)

    def get_current_queue_index(self):
        request = Message(
            [0xAA, 0xAA], 2, 246, True, False, [], direction=IODirection.OUT
        )
        return self.send(request)
