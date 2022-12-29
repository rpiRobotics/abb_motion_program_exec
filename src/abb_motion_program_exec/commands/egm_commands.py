# Copyright 2022 Wason Technology LLC, Rensselaer Polytechnic Institute
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from .command_base import CommandBase
from dataclasses import dataclass
from .rapid_types import *
import io
from . import util
from typing import Union

@dataclass
class EGMRunJointCommand(CommandBase):
    command_opcode = 50001

    cond_time: float
    ramp_in_time: float
    ramp_out_time: float

    def write_params(self, f: io.IOBase):
        f.write(util.num_to_bin(self.cond_time))
        f.write(util.num_to_bin(self.ramp_in_time))
        f.write(util.num_to_bin(self.ramp_out_time))

    def to_rapid(self, **kwargs):
        raise NotImplementedError("EGM not supported for RAPID generation")

    _append_method_doc = ""

@dataclass
class EGMRunPoseCommand(CommandBase):
    command_opcode = 50002

    cond_time: float
    ramp_in_time: float
    ramp_out_time: float
    offset: pose

    def write_params(self, f: io.IOBase):
        f.write(util.num_to_bin(self.cond_time))
        f.write(util.num_to_bin(self.ramp_in_time))
        f.write(util.num_to_bin(self.ramp_out_time))
        f.write(util.pose_to_bin(self.offset))

    def to_rapid(self, **kwargs):
        raise NotImplementedError("EGM not supported for RAPID generation")

    _append_method_doc = ""

@dataclass
class EGMMoveLCommand(CommandBase):
    command_opcode = 50003

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

    def to_rapid(self, **kwargs):
        raise NotImplementedError("EGM not supported for RAPID generation")

    _append_method_doc = ""

@dataclass
class EGMMoveCCommand(CommandBase):
    command_opcode = 50004

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

    def to_rapid(self, **kwargs):
        raise NotImplementedError("EGM not supported for RAPID generation")

    _append_method_doc = ""


class egm_minmax(NamedTuple):
    """egm_minmax structure"""
    min: float
    """min value"""
    max: float
    """max value"""

def _egm_minmax_to_bin(e: egm_minmax):
    return util.num_to_bin(e.min) + util.num_to_bin(e.max)

class EGMStreamConfig(NamedTuple):
    """
    Configure EGM to stream feedback data only
    """
    pass

class EGMJointTargetConfig(NamedTuple):
    """
    Activate EGM for joint target control
    """
    J1: egm_minmax
    """J1 convergence criteria"""
    J2: egm_minmax
    """J2 convergence criteria"""
    J3: egm_minmax
    """J3 convergence criteria"""
    J4: egm_minmax
    """J4 convergence criteria"""
    J5: egm_minmax
    """J5 convergence criteria"""
    J6: egm_minmax
    """J6 convergence criteria"""
    max_pos_deviation: float
    """Max joint deviation in degrees"""
    max_speed_deviation: float
    """Max joint speed deviation in degrees/second"""

class egmframetype(IntEnum):
    """Frame types for corrections and sensor measurements"""
    EGM_FRAME_BASE = 0
    """Base frame"""
    EGM_FRAME_TOOL = 1
    """Tool frame"""
    EGM_FRAME_WOBJ = 2
    """Wobj frame"""
    EGM_FRAME_WORLD = 3
    """World frame"""
    EGM_FRAME_JOINT = 4
    """Joint frame"""

class EGMPoseTargetConfig(NamedTuple):
    """
    Activate EGM for pose target control
    """
    corr_frame: pose
    """The correction frame"""
    corr_fr_type: egmframetype
    """The correction frame type"""
    sensor_frame: pose
    """The sensor frame"""
    sensor_fr_type: egmframetype
    """The sensor frame type"""
    x: egm_minmax
    """x convergence criteria"""
    y: egm_minmax
    """y convergence criteria"""
    z: egm_minmax
    """z convergence criteria"""
    rx: egm_minmax
    """rx convergence criteria"""
    ry: egm_minmax
    """ry convergence criteria"""
    rz: egm_minmax
    """rz convergence criteria"""
    max_pos_deviation: float
    """Max joint deviation in degrees"""
    max_speed_deviation: float
    """Max joint speed deviation in degrees/second"""

class EGMPathCorrectionConfig(NamedTuple):
    """
    Activate EGM for path correction (``EGMMoveL``, ``EGMMoveC``)
    """
    sensor_frame: pose
    """The sensor frame"""


def _egm_joint_target_config_to_bin(c: EGMJointTargetConfig):
    return _egm_minmax_to_bin(c.J1) \
        + _egm_minmax_to_bin(c.J2) \
        + _egm_minmax_to_bin(c.J3) \
        + _egm_minmax_to_bin(c.J4) \
        + _egm_minmax_to_bin(c.J5) \
        + _egm_minmax_to_bin(c.J6) \
        + util.num_to_bin(c.max_pos_deviation) \
        + util.num_to_bin(c.max_speed_deviation)

def _egm_pose_target_config_to_bin(c: EGMPoseTargetConfig):
    return \
        util.pose_to_bin(c.corr_frame) \
        + util.num_to_bin(c.corr_fr_type.value) \
        + util.pose_to_bin(c.sensor_frame) \
        + util.num_to_bin(c.sensor_fr_type.value) \
        + _egm_minmax_to_bin(c.x) \
        + _egm_minmax_to_bin(c.y) \
        + _egm_minmax_to_bin(c.z) \
        + _egm_minmax_to_bin(c.rx) \
        + _egm_minmax_to_bin(c.ry) \
        + _egm_minmax_to_bin(c.rz) \
        + util.num_to_bin(c.max_pos_deviation) \
        + util.num_to_bin(c.max_speed_deviation)

def _egm_path_correction_config_to_bin(c: EGMPathCorrectionConfig):
    return util.pose_to_bin(c.sensor_frame)

def write_egm_config(f: io.IOBase, 
    egm_config: Union[EGMStreamConfig,EGMJointTargetConfig,EGMPoseTargetConfig,EGMPathCorrectionConfig]):
    if egm_config is None or isinstance(egm_config,EGMStreamConfig):
        f.write(util.num_to_bin(0))
    elif isinstance(egm_config,EGMJointTargetConfig):
        f.write(util.num_to_bin(1))
        f.write(_egm_joint_target_config_to_bin(egm_config))
    elif isinstance(egm_config,EGMPoseTargetConfig):
        f.write(util.num_to_bin(2))
        f.write(_egm_pose_target_config_to_bin(egm_config))
    elif isinstance(egm_config,EGMPathCorrectionConfig):
        f.write(util.num_to_bin(3))
        f.write(_egm_path_correction_config_to_bin(egm_config))
    else:
        assert False, "Invalid EGM configuration"