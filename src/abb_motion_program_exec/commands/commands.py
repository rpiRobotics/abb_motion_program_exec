from .command_base import CommandBase
from dataclasses import dataclass
from .rapid_types import *
import io
from . import util

@dataclass
class MoveAbsJCommand(CommandBase):
    command_opcode = 0x1

    to_joint_pos: jointtarget
    speed: speeddata
    zone: zonedata

    def write_params(self, f: io.IOBase):
        to_joint_pos_b = util.jointtarget_to_bin(self.to_joint_pos)
        speed_b = util.speeddata_to_bin(self.speed)
        zone_b = util.zonedata_to_bin(self.zone)
        f.write(to_joint_pos_b)
        f.write(speed_b)
        f.write(zone_b)

    def write_rapid(self, f: io.TextIOBase):
        pass

    _append_method_doc = ""

@dataclass
class MoveJCommand(CommandBase):
    command_opcode = 0x2

    to_point: robtarget
    speed: speeddata
    zone: zonedata

    def write_params(self, f: io.IOBase):
        to_point_b = util.robtarget_to_bin(self.to_point)
        speed_b = util.speeddata_to_bin(self.speed)
        zone_b = util.zonedata_to_bin(self.zone)
        f.write(to_point_b)
        f.write(speed_b)
        f.write(zone_b)

    _append_method_doc = ""

@dataclass
class MoveLCommand(CommandBase):
    command_opcode = 0x3

    to_point: robtarget
    speed: speeddata
    zone: zonedata

    def write_params(self, f: io.IOBase):
        to_point_b = util.robtarget_to_bin(self.to_point)
        speed_b = util.speeddata_to_bin(self.speed)
        zone_b = util.zonedata_to_bin(self.zone)

        f.write(to_point_b)
        f.write(speed_b)
        f.write(zone_b)

    _append_method_doc = ""

@dataclass
class MoveCCommand(CommandBase):
    command_opcode = 0x4

    cir_point: robtarget
    to_point: robtarget
    speed: speeddata
    zone: zonedata

    def write_params(self, f: io.IOBase):
        cir_point_b = util.robtarget_to_bin(self.cir_point)
        to_point_b = util.robtarget_to_bin(self.to_point)
        speed_b = util.speeddata_to_bin(self.speed)
        zone_b = util.zonedata_to_bin(self.zone)

        f.write(cir_point_b)
        f.write(to_point_b)
        f.write(speed_b)
        f.write(zone_b)

    _append_method_doc = ""

@dataclass
class WaitTimeCommand(CommandBase):
    command_opcode=0x5

    t: float

    def write_params(self, f: io.IOBase):
        f.write(util.num_to_bin(self.t))

    _append_method_doc = ""

@dataclass
class CirPathModeCommand(CommandBase):
    command_opcode = 0x5

    switch: CirPathModeSwitch

    def write_params(self, f: io.IOBase):
        val = self.switch.value
        assert val >=1 and val <= 6, "Invalid CirPathMode switch"
        f.write(util.num_to_bin(val))

    _append_method_doc = ""

@dataclass
class SyncMoveOnCommand(CommandBase):
    command_opcode = 0x6

    def write_params(self, f: io.IOBase):
        pass

    _append_method_doc = ""

class SyncMoveOffCommand(CommandBase):
    command_opcode = 0x7

    def write_params(self, f: io.IOBase):
        pass

    _append_method_doc = ""
