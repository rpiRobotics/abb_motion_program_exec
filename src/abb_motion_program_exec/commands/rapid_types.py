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

from typing import Callable, NamedTuple, Any, List
from enum import IntEnum
import numpy as np
from . import util

"""
This module contains types found on the ABB Robot Controller. Documentation is taken from the
ABB Robotics manual "Technical reference manual RAPID Instructions, Functions and Data types".
"""


class speeddata(NamedTuple):
    """
    ABB RAPID ``speeddata`` structure.

    The ``abb_motion_program_exec`` module contains constants for ``v5`` through ``v5000`` and ``vmax`` as found
    in RAPID.
    """
    v_tcp: float
    """Velocity of the tool center point (TCP) in mm/s"""
    v_ori: float
    """The reorientation velocity of the TCP expressed in degrees/s."""
    v_leax: float
    """The velocity of linear external axes in mm/s."""
    v_reax: float
    """The velocity of rotating external axes in degrees/s."""

    def to_rapid(self):
        return util.nums_to_rapid_array([
            self.v_tcp, 
            self.v_ori, 
            self.v_leax, 
            self.v_reax
            ])

v5 = speeddata(5,500,5000,1000)
"""5 mm/s speeddata"""
v10 = speeddata(10,500,5000,1000)
"""10 mm/s speeddata"""
v20 = speeddata(20,500,5000,1000)
"""20 mm/s speeddata"""
v30 = speeddata(30,500,5000,1000)
"""30 mm/s speeddata"""
v40 = speeddata(40,500,5000,1000)
"""40 mm/s speeddata"""
v50 = speeddata(50,500,5000,1000)
"""50 mm/s speeddata"""
v60 = speeddata(60,500,5000,1000)
"""60 mm/s speeddata"""
v80 = speeddata(80,500,5000,1000)
"""80 mm/s speeddata"""
v100 = speeddata(100,500,5000,1000)
"""100 mm/s speeddata"""
v150 = speeddata(150,500,5000,1000)
"""150 mm/s speeddata"""
v200 = speeddata(200,500,5000,1000)
"""200 mm/s speeddata"""
v300 = speeddata(300,500,5000,1000)
"""300 mm/s speeddata"""
v400 = speeddata(400,500,5000,1000)
"""400 mm/s speeddata"""
v500 = speeddata(500,500,5000,1000)
"""500 mm/s speeddata"""
v600 = speeddata(600,500,5000,1000)
"""600 mm/s speeddata"""
v800 = speeddata(800,500,5000,1000)
"""800 mm/s speeddata"""
v1000 = speeddata(1000,500,5000,1000)
"""1000 mm/s speeddata"""
v1500 = speeddata(1500,500,5000,1000)
"""1500 mm/s speeddata"""
v2000 = speeddata(2000,500,5000,1000)
"""2000 mm/s speeddata"""
v2500 = speeddata(2500,500,5000,1000)
"""2500 mm/s speeddata"""
v3000 = speeddata(3000,500,5000,1000)
"""3000 mm/s speeddata"""
v4000 = speeddata(4000,500,5000,1000)
"""4000 mm/s speeddata"""
v5000 = speeddata(5000,500,5000,1000)
"""5000 mm/s speeddata"""
v6000 = speeddata(6000,500,5000,1000)
"""6000 mm/s speeddata"""
v7000 = speeddata(7000,500,5000,1000)
"""7000 mm/s speeddata"""
vmax = speeddata(10000,500,5000,1000)
"""max speeddata"""

class zonedata(NamedTuple):
    """
    ABB RAPID ``zonedata`` structure.
    
    The ``abb_motion_program_exec`` module contains constants for ``fine`` and ``z0`` through ``z200`` as found
    in RAPID.
    """
    finep: bool
    """Defines whether the movement is to terminate as a stop point (fine point) or as a fly-by point."""
    pzone_tcp: float
    """The size (the radius) of the TCP zone in mm."""
    pzone_ori: float
    """The zone size (the radius) for the tool reorientation."""
    pzone_eax: float
    """The zone size (the radius) for external axes."""
    zone_ori: float
    """The zone size for the tool reorientation in degrees."""
    zone_leax: float
    """The zone size for linear external axes in mm."""
    zone_reax: float
    """The zone size for rotating external axes in degrees."""

    def to_rapid(self):
        return util.nums_to_rapid_array([
            util.bool_to_rapid(self.finep),
            self.pzone_tcp,
            self.pzone_ori,
            self.pzone_eax,
            self.zone_ori,
            self.zone_leax,
            self.zone_reax
        ])

fine = zonedata(True,0,0,0,0,0,0)
"""Stop point, no bleding"""
z0 = zonedata(False,0.3,0.3,0.3,0.03,0.3,0.03)
"""0.3 mm zone"""
z1 = zonedata(False,1,1,1,0.1,1,0.1)
"""1 mm zone"""
z5 = zonedata(False,5,8,8,0.8,8,0.8)
"""5 mm zone"""
z10 = zonedata(False,10,15,15,1.5,15,1.5)
"""10 mm zone"""
z15 = zonedata(False,15,23,23,2.3,23,2.3)
"""15 mm zone"""
z20 = zonedata(False,20,30,30,3.0,30,3.0)
"""20 mm zone"""
z30 = zonedata(False,30,45,45,4.5,45,4.5)
"""30 mm zone"""
z40 = zonedata(False,40,60,60,6.0,60,6.0)
"""40 mm zone"""
z50 = zonedata(False,50,75,75,7.5,75,7.5)
"""50 mm zone"""
z60 = zonedata(False,60,90,90,9.0,90,9.0)
"""60 mm zone"""
z80 = zonedata(False,80,120,120,12,120,12)
"""80 mm zone"""
z100 = zonedata(False,100,150,150,15,150,15)
"""100 mm zone"""
z150 = zonedata(False,150,225,225,23,225,23)
"""150 mm zone"""
z200 = zonedata(False,200,300,300,30,300,30)
"""200 mm zone"""

class jointtarget(NamedTuple):
    """ABB RAPID ``jointtarget`` structure"""
    robax: np.ndarray # shape=(6,)
    """Axis positions of the robot axes in degrees. Must have 6 elements"""
    extax: np.ndarray # shape=(6,)
    """The position of external axes. Must have 6 elements"""

    def to_rapid(self):
        return f"[{util.nums_to_rapid_array(self.robax)},{util.nums_to_rapid_array(self.extax)}]"

class pose(NamedTuple):
    """ABB RAPID ``pose`` structure"""
    trans: np.ndarray # [x,y,z]
    """Translation in mm. Must have 3 elements"""
    rot: np.ndarray # [qw,qx,qy,qz]
    """Rotation in quaternions. [w,x,y,z] format"""

    def to_rapid(self):
        return f"[{util.nums_to_rapid_array(self.trans)},{util.nums_to_rapid_array(self.rot)}]"

class confdata(NamedTuple):
    """ABB RAPID ``confdata`` structure. This structure has a very peculiar meaning. See the reference manual
    for details."""
    cf1: float
    """cf1"""
    cf4: float
    """cf4"""
    cf6: float
    """cf6"""
    cfx: float
    """cfx"""

    def to_rapid(self):
        return util.nums_to_rapid_array([
            self.cf1,
            self.cf4,
            self.cf6,
            self.cfx
        ])

class robtarget(NamedTuple):
    """ABB RAPID ``robtarget`` structure"""
    trans: np.ndarray # [x,y,z]
    """Translation [x,y,z] in mm"""
    rot: np.ndarray # [qw,qx,qy,qz]
    """Rotation in quaternions. [w,x,y,z] format"""
    robconf: confdata # 
    """Robot configuration"""
    extax: np.ndarray # shape=(6,)
    """External axes positions. Must have 6 elements"""

    def to_rapid(self):
        trans_str = util.nums_to_rapid_array(self.trans)
        rot_str = util.nums_to_rapid_array(self.rot)
        robconf_str = self.robconf.to_rapid()
        extax_str = util.nums_to_rapid_array(self.extax)
        return f"[{trans_str},{rot_str},{robconf_str},{extax_str}]"

class loaddata(NamedTuple):
    """ABB RAPID ``loaddata`` structure"""
    mass: float
    """Mass in kg"""
    cog: np.ndarray # shape=(3,)
    """Center of gravity [x,y,z] in mm"""
    aom: np.ndarray # shape=(4,)
    """Axes of moment (principal axis) transform [w,x,y,z]"""
    ix: float
    """x-axis moment in kgm^2"""
    iy: float
    """y-axis moment in kgm^2"""
    iz: float
    """z-axis moment in kgm^2"""

    def to_rapid(self):
        return f"[{self.mass},{util.nums_to_rapid_array(self.cog)}," \
            f"{util.nums_to_rapid_array(self.aom)},{self.ix},{self.iy},{self.iz}]"

class CirPathModeSwitch(IntEnum):
    """ABB RAPID Options for CirPathMode command"""
    PathFrame = 1
    """Path Frame"""
    ObjectFrame = 2
    """Object Frame"""
    CirPointOri = 3
    """Pass through ToPoint during MoveC"""
    Wrist45 = 4
    """Wrist45"""
    Wrist46 = 5
    """Wrist46"""
    Wrist56 = 6
    """Wrist56"""

class tooldata(NamedTuple):
    """ABB RAPID ``tooldata`` structure"""
    robhold: bool
    """Defines whether or not the robot is holding the tool"""
    tframe: pose
    """The tool coordinate system"""
    tload : loaddata
    """The load of the tool"""

    def to_rapid(self):
        return f"[{util.bool_to_rapid(self.robhold)},{self.tframe.to_rapid()},{self.tload.to_rapid()}]"

class wobjdata(NamedTuple):
    """ABB RAPID ``wobjdata`` structure"""
    robhold: bool
    """True if the robot is holding the work object. Typically False"""
    ufprog: bool
    """Defines whether or not a fixed user coordinate system is used"""
    ufmec: str
    """The mechanical unit which the robot movements are coordinated"""
    uframe: pose
    """User coordinate system"""
    oframe: pose
    """Object coordinate system"""

    def to_rapid(self):
        return f"[{util.bool_to_rapid(self.robhold)},{util.bool_to_rapid(self.ufprog)}," \
            f"\"{self.ufmec}\",{self.uframe.to_rapid()},{self.oframe.to_rapid()}]"