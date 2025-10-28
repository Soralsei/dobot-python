from time import sleep

from .interface import Interface

ALARM_LABELS = {
    0x00: "Reset occurred",
    0x01: "Undefined instruction",
    0x02: "File system error",
    0x03: "Communications error between MCU and FPGA",
    0x04: "Angle sensor error",
    0x10: "Plan: pose is abnormal",
    0x11: "Plan: pose is out of workspace",
    0x12: "Plan: joint limit",
    0x13: "Plan: repetitive points",
    0x14: "Plan: arc input parameter",
    0x15: "Plan: jump parameter",
    0x20: "Motion: kinematic singularity",
    0x21: "Motion: out of workspace",
    0x22: "Motion: inverse limit",
    0x30: "Axis 1 over speed limit",
    0x31: "Axis 2 over speed limit",
    0x32: "Axis 3 over speed limit",
    0x33: "Axis 4 over speed limit",
    0x40: "Axis 1 positive limit",
    0x41: "Axis 1 negative limit",
    0x42: "Axis 2 positive limit",
    0x43: "Axis 2 negative limit",
    0x44: "Axis 3 positive limit",
    0x45: "Axis 3 negative limit",
    0x46: "Axis 4 positive limit",
    0x47: "Axis 4 negative limit",
    0x50: "Axis 1 lost steps",
    0x51: "Axis 2 lost steps",
    0x52: "Axis 3 lost steps",
    0x53: "Axis 4 lost steps",
}


class Dobot:
    def __init__(self, port):
        self.interface = Interface(port)

        self.interface.stop_queue(True)
        self.interface.clear_queue()
        self.interface.start_queue()

        self.interface.set_point_to_point_jump_params(10, 10)
        self.interface.set_point_to_point_joint_params(
            [50, 50, 50, 50], [50, 50, 50, 50]
        )
        self.interface.set_point_to_point_coordinate_params(50, 50, 50, 50)

        # velocity and acceleration ratio
        self.interface.set_point_to_point_common_params(50, 50)
        self.interface.set_point_to_point_jump2_params(5, 5, 5)

        self.interface.set_jog_joint_params([50, 50, 50, 50], [50, 50, 50, 50])
        self.interface.set_jog_coordinate_params([50, 50, 50, 50], [50, 50, 50, 50])
        self.interface.set_jog_common_params(50, 50)

        self.interface.set_continous_trajectory_params(50, 50, 50)

    def close(self):
        self.interface.close()

    def connected(self):
        return self.interface.connected()

    def get_pose(self):
        return self.interface.get_pose()

    def printq(self):
        pose = self.get_pose()
        if pose is None:
            print("Unable to get pose")
            return
        print("q:   ", " ".join([f"{q:8.1f}" for q in pose[4:]]))

    def printx(self):
        pose = self.get_pose()
        if pose is None:
            print("Unable to get x")
            return
        print("x:   ", " ".join([f"{q:8.1f}" for q in pose[:4]]))

    def print_alarms(self, a):
        alarms = []
        for i, x in enumerate(a):
            for j in range(8):
                if x & (1 << j) > 0:
                    alarms.append(8 * i + j)
        for alarm in alarms:
            print("ALARM:", ALARM_LABELS[alarm])

    def home(self, wait=True):
        self.interface.set_homing_command(0)
        if wait:
            self.wait()

    # Move to the absolute coordinate, one axis at a time
    def move_to(self, x, y, z, r, wait=True):
        self.interface.set_point_to_point_command(3, x, y, z, r)
        if wait:
            self.wait()

    # Slide to the absolute coordinate, shortest possible path
    def slide_to(self, x, y, z, r, wait=True):
        self.interface.set_point_to_point_command(4, x, y, z, r)
        if wait:
            self.wait()

    # Move to the absolute coordinate, one axis at a time
    def move_to_relative(self, x, y, z, r, wait=True):
        self.interface.set_point_to_point_command(7, x, y, z, r)
        if wait:
            self.wait()

    # Slide to the relative coordinate, one axis at a time
    def slide_to_relative(self, x, y, z, r, wait=True):
        self.interface.set_point_to_point_command(6, x, y, z, r)
        if wait:
            self.wait()

    # Wait until the instruction finishes
    def wait(self, queue_index=None):
        # If there are no more instructions in the queue, it will end up
        # always returning the last instruction - even if it has finished.
        # Use a zero wait as a non-operation to bypass this limitation
        self.interface.wait(0)

        if queue_index is None:
            queue_index = self.interface.get_current_queue_index()
        while True:
            index = self.interface.get_current_queue_index()
            if index is not None and index > queue_index:
                break

            sleep(0.5)

    # Move according to the given path
    def follow_path(self, path, wait=True):
        self.interface.stop_queue()
        queue_index = None
        for point in path:
            queue_index = self.interface.set_continous_trajectory_command(
                1, point[0], point[1], point[2], 50
            )
        self.interface.start_queue()
        if wait:
            self.wait(queue_index)

    # Move according to the given path
    def follow_path_relative(self, path, wait=True):
        self.interface.stop_queue()
        queue_index = None
        for point in path:
            queue_index = self.interface.set_continous_trajectory_command(
                0, point[0], point[1], point[2], 50
            )
        self.interface.start_queue()
        if wait:
            self.wait(queue_index)
