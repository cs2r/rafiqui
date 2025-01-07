import time

from dynamixel_sdk import *
import xarm
import os
import json
import logging
import types
import numpy as np

logger = logging.getLogger(__name__)

if os.name == 'nt':
    import msvcrt


    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios

    def getch():
        if sys.stdin.isatty():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch
        else:
            # Fallback for non-terminal environments
            return input("Press a key: ")

# Control table address
TORQUE_ENABLE_ADD = 64  # Control table address is different in Dynamixel model
GOAL_POSITION_ADD = 116
PRESENT_POSITION_ADD = 132
PRESENT_LOAD_ADD = 126

# Protocol version
PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque


def load_config(path):
    try:
        with open(path, 'r') as file:
            config_data = json.load(file)
            print("config data loaded successfully:")
            return config_data
    except FileNotFoundError:
        print(f"Error: File not found in path {path}")
    except json.JSONDecodeError:
        print(f"Error: Failed to read JSON file in {path}")


def set_torque(self, torque):
    if self.name in ["r_elbow_forearm", "l_elbow_forearm"]:
        self.elbow_1.set_torque(torque)
        self.elbow_2.set_torque(torque)
    elif self.name in ["r_hand", "l_hand", "neck"]:
        for att_name in dir(self):
            motor = getattr(self, att_name)
            if isinstance(motor, Robot.HWMotor):
                motor.servo_off()


def set_position(self, ELBOW_GOAL, FOREARM_GOAL):
    if self.name in ["r_elbow_forearm", "l_elbow_forearm"]:
        self.elbow = ELBOW_GOAL
        self.forearm = -FOREARM_GOAL
        self.elbow_1.set_position(-self.elbow + self.forearm)
        self.elbow_2.set_position(self.elbow + self.forearm)
    elif self.name in ["r_hand", "l_hand", "neck"]:
        print("This joint does not has set_position() callback")


def get_position(self):
    if self.name in ["r_elbow_forearm", "l_elbow_forearm"]:
        self.elbow = (self.elbow_2.get_position() - self.elbow_1.get_position()) / 2
        self.forearm = (self.elbow_1.get_position() + self.elbow_2.get_position()) / 2
        return self.elbow, self.forearm
    elif self.name in ["r_hand", "l_hand", "neck"]:
        print("This joint does not has get_position() callback")


def reboot(self):
    self.elbow_1.reboot()
    self.elbow_2.reboot()


def make_alias(robot, config):
    alias = config['motorgroups']
    # Create the alias for the motorgroups
    for alias_name in alias:
        motors = _motor_extractor(robot, alias, alias_name)
        if alias_name in ["r_elbow_forearm", "l_elbow_forearm"]:
            motors.set_torque = types.MethodType(set_torque, motors)
            motors.reboot = types.MethodType(reboot, motors)
            motors.set_position = types.MethodType(set_position, motors)
            motors.get_position = types.MethodType(get_position, motors)
            motors.get_position()
        elif alias_name in ["r_hand", "l_hand"]:
            motors.set_torque = types.MethodType(set_torque, motors)
            motors.set_position = types.MethodType(set_position, motors)
            motors.get_position = types.MethodType(get_position, motors)
            motors.get_position()
        setattr(robot, alias_name, motors)


def _motor_extractor(robot, alias, name):
    joint = Robot.Joint(name)

    if name not in alias:
        return [name]

    for key in alias[name]:
        setattr(joint, key if name == "neck" else key[2:], getattr(robot, key))
    return joint


position_range = {
    'MX': (0, 4096, 0, 360.0),
    'ME': (-12288, 12288, -1080, 1080.0),
    'XM': (0, 4096, 0, 360.0)
}


def dxl_to_degree(value, model):
    determined_model = '*'
    if model.startswith('MX'):
        determined_model = 'MX'
    elif model.startswith('ME'):
        determined_model = 'ME'
    elif model.startswith('XM'):
        determined_model = 'XM'
    elif model.startswith('SR'):
        determined_model = 'SR'
    elif model.startswith('EX'):
        determined_model = 'EX'
    min_pos, max_pos, min_deg, max_deg = position_range[determined_model]

    return round(((max_deg * float(value)) / (max_pos - 1)) - (max_deg / 2), 2)


def degree_to_dxl(value, model):
    determined_model = '*'
    if model.startswith('MX'):
        determined_model = 'MX'
    elif model.startswith('ME'):
        determined_model = 'ME'
    elif model.startswith('XM'):
        determined_model = 'XM'
    elif model.startswith('SR'):
        determined_model = 'SR'
    elif model.startswith('EX'):
        determined_model = 'EX'
    min_pos, max_pos, min_deg, max_deg = position_range[determined_model]

    pos = int(round((max_pos - 1) * ((max_deg / 2 + float(value)) / max_deg), 0))
    pos = min(max(pos, min_pos), max_pos - 1)

    return pos


class Robot:
    class Joint:
        def __init__(self, name):
            self.name = name
            if name in ["r_elbow_forearm", "l_elbow_forearm"]:
                self.elbow = 0
                self.forearm = 0

    class Controller:
        def __init__(self, port, protocol_version, baudrate):
            self.port = PortHandler(port)
            self.packet = PacketHandler(protocol_version)
            self.baudrate = baudrate

    def check_motor(self, motor_id):
        dxl_model_number, dxl_comm_result, dxl_error = self.DXcontroller.packet.ping(self.DXcontroller.port, motor_id)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.DXcontroller.packet.getTxRxResult(dxl_comm_result).replace("[TxRxResult] ", ""),
                  "ID: %d" % motor_id)
        elif dxl_error != 0:
            print("%s" % self.DXcontroller.packet.getRxPacketError(dxl_error).replace("[RxPacketError] ", ""),
                  "ID: %d" % motor_id)
        else:
            # print("Successful connect to motor: %d" % motor_id)
            return 1

    def __init__(self, config):
        self.motors = {}
        self.missing = {}
        #self.alias = []

        config_data = load_config(config)
        if config_data:
            DEVICENAME = config_data["controller"]["port"]
            PROTOCOL_VERSION = config_data["controller"]["protocol"]
            BAUDRATE = config_data["controller"]["baudrate"]
            alias = config_data['motorgroups']

            self.DXcontroller = self.Controller(DEVICENAME, PROTOCOL_VERSION, BAUDRATE)
            self.HWcontroller = xarm.Controller('USB')

            if self.open_port():
                for motor, motor_param in config_data["motors"].items():
                    if motor_param["model"].startswith("MX") or motor_param["model"].startswith("XM") or motor_param["model"].startswith("ME"):
                        if self.check_motor(motor_param["id"]):
                            self.add_motor(motor, motor_param["id"], motor_param["model"], motor_param["offset"],
                                           motor_param["orientation"], motor_param["gear_ratio"])
                        else:
                            self.add_motor(motor, motor_param["id"], motor_param["model"], motor_param["offset"],
                                           motor_param["orientation"], motor_param["gear_ratio"], missing=True)
                    elif motor_param["model"].startswith("HX"):
                        self.add_motor(motor, motor_param["id"], motor_param["model"], motor_param["offset"],
                                       motor_param["orientation"])

                make_alias(self, config_data)
            print("Robot is READY")

    def open_port(self):
        if self.DXcontroller.port.openPort():
            print("Succeeded to open the port")
            return 1
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

    def add_motor(self, name, motor_id, model, offset, orientation, gear_ratio=1.0, missing=False):
        if model.startswith("MX") or model.startswith("XM") or model.startswith("ME"):
            motor = self.DXMotor(self.DXcontroller, motor_id, model, offset, orientation, gear_ratio, name)
        elif model.startswith("HX"):
            motor = self.HWMotor(self.HWcontroller, motor_id, model, offset, orientation, name)
        else:
            motor = None
        if not missing:
            self.motors[name] = motor
            setattr(self, name, motor)
        else:
            self.missing[name] = motor
        return motor

    def release_all(self):
        for motor in self.motors:
            motor.set_torque(TORQUE_DISABLE)
        print("all motors released")

    def reset_all(self):
        current_pos = []
        goal_pos = []
        for motor in self.motors:
            current_pos.append(motor.get_position())
            goal_pos.append(0)

        steps = np.linspace(current_pos, goal_pos, 50)
        for step in steps:
            ndx = 0
            for motor in self.motors:
                motor.set_position(step[ndx])
                ndx += 1
            time.sleep(0.01)
        print("all motors reseated")
        time.sleep(0.5)
        self.release_all()

    class DXMotor:
        def __init__(self, controller, motor_id, model, offset, orientation, gear_ratio, name=None):
            self.name = name if name is not None else "m" + str(motor_id)
            self.id = motor_id
            self.model = model
            self.offset = offset
            self.orientation = 1 if orientation == "direct" else -1
            self.gear_ratio = gear_ratio
            self.controller = controller
            self.initial = 0.0

        def get_position(self):
            dxl_present_position, dxl_comm_result, dxl_error = self.controller.packet.read4ByteTxRx(
                self.controller.port,
                self.id,
                PRESENT_POSITION_ADD)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.controller.packet.getTxRxResult(dxl_comm_result))
                return None
            elif dxl_error != 0:
                print("%s" % self.controller.packet.getRxPacketError(dxl_error))
                return None
            else:
                return (dxl_to_degree(dxl_present_position,
                                      self.model[0:2]) - self.offset) * self.orientation / self.gear_ratio

        def set_torque(self, torque):
            dxl_comm_result, dxl_error = self.controller.packet.write1ByteTxRx(self.controller.port, self.id,
                                                                               TORQUE_ENABLE_ADD,
                                                                               torque)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.controller.packet.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.controller.packet.getRxPacketError(dxl_error))
            else:
                return 1

        def set_position(self, GOAL):
            # Enable Dynamixel Torque
            # print("Motor %d pos: %d " % (self.id, (GOAL * self.gear_ratio + self.offset) * self.orientation))
            # print("motor %d cmd: %d " % (self.id, degree_to_dxl((GOAL * self.gear_ratio + self.offset) * self.orientation, self.model[0:2])))
            if self.set_torque(TORQUE_ENABLE):
                dxl_comm_result, dxl_error = self.controller.packet.write4ByteTxRx(self.controller.port, self.id,
                                                                                   GOAL_POSITION_ADD,
                                                                                   degree_to_dxl((
                                                                                                         GOAL * self.gear_ratio + self.offset) * self.orientation,
                                                                                                 self.model[0:2]))
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.controller.packet.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.controller.packet.getRxPacketError(dxl_error))
                else:
                    # print("Dynamixel has been successfully moved to %d" % (GOAL + self.offset))
                    return 1
            else:
                return 0

        def get_load(self):
            dxl_present_load, dxl_comm_result, dxl_error = self.controller.packet.read2ByteTxRx(
                self.controller.port,
                self.id,
                PRESENT_LOAD_ADD)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.controller.packet.getTxRxResult(dxl_comm_result))
                return None
            elif dxl_error != 0:
                print("%s" % self.controller.packet.getRxPacketError(dxl_error))
                return None
            else:
                if dxl_present_load > 60000:
                    dxl_present_load = dxl_present_load - 65536
                return dxl_present_load / 10

        def go_to_pos(self, pos, nbr_steps):
            steps = np.linspace(self.get_position(), pos, nbr_steps)
            for step in steps:
                self.set_position(step)
                time.sleep(0.01)

        def reset(self):
            self.go_to_pos(self.initial, 50)
            print("motor %d reseated" % self.id)

        def reboot(self):
            dxl_comm_result, dxl_error = self.controller.packet.reboot(self.controller.port, self.id)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.controller.packet.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.controller.packet.getRxPacketError(dxl_error))
            print("[ID:%03d] reboot Succeeded" % self.id)

    class HWMotor:
        def __init__(self, controller, motor_id, model, offset, orientation, name=None):
            self.name = name if name is not None else "s" + str(motor_id)
            self.id = motor_id
            self.model = model
            self.offset = offset
            self.orientation = 1 if orientation == "direct" else -1
            self.initial = 200
            self.controller = controller

        def get_position(self):
            return self.controller.getPosition(self.id)

        def set_position(self, GOAL, duration=10):
            self.controller.setPosition(self.id, GOAL, duration)

        def servo_off(self):
            self.controller.servoOff(self.id)


if __name__ == "__main__":
    myRobot = Robot("config.json")
    print("DONE")