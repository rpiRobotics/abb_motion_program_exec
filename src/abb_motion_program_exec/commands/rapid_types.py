
from typing import Callable, NamedTuple, Any, List
from enum import IntEnum
import numpy as np
from . import util

class speeddata(NamedTuple):
    v_tcp: float
    v_ori: float
    v_leax: float
    v_reax: float

    def to_rapid(self):
        return util.nums_to_rapid_array([
            self.v_tcp, 
            self.v_ori, 
            self.v_leax, 
            self.v_reax
            ])

v5 = speeddata(5,500,5000,1000)
v10 = speeddata(10,500,5000,1000)
v20 = speeddata(20,500,5000,1000)
v30 = speeddata(30,500,5000,1000)
v40 = speeddata(40,500,5000,1000)
v50 = speeddata(50,500,5000,1000)
v60 = speeddata(60,500,5000,1000)
v80 = speeddata(80,500,5000,1000)
v100 = speeddata(100,500,5000,1000)
v150 = speeddata(150,500,5000,1000)
v200 = speeddata(200,500,5000,1000)
v300 = speeddata(300,500,5000,1000)
v400 = speeddata(400,500,5000,1000)
v500 = speeddata(500,500,5000,1000)
v600 = speeddata(600,500,5000,1000)
v800 = speeddata(800,500,5000,1000)
v1000 = speeddata(1000,500,5000,1000)
v1500 = speeddata(1500,500,5000,1000)
v2000 = speeddata(2000,500,5000,1000)
v2500 = speeddata(2500,500,5000,1000)
v3000 = speeddata(3000,500,5000,1000)
v4000 = speeddata(4000,500,5000,1000)
v5000 = speeddata(5000,500,5000,1000)
v6000 = speeddata(6000,500,5000,1000)
v7000 = speeddata(7000,500,5000,1000)
vmax = speeddata(10000,500,5000,1000)

class zonedata(NamedTuple):
    finep: bool
    pzone_tcp: float
    pzone_ori: float
    pzone_eax: float
    zone_ori: float
    zone_leax: float
    zone_reax: float

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
z0 = zonedata(False,0.3,0.3,0.3,0.03,0.3,0.03)
z1 = zonedata(False,1,1,1,0.1,1,0.1)
z5 = zonedata(False,5,8,8,0.8,8,0.8)
z10 = zonedata(False,10,15,15,1.5,15,1.5)
z15 = zonedata(False,15,23,23,2.3,23,2.3)
z20 = zonedata(False,20,30,30,3.0,30,3.0)
z30 = zonedata(False,30,45,45,4.5,45,4.5)
z40 = zonedata(False,40,60,60,6.0,60,6.0)
z50 = zonedata(False,50,75,75,7.5,75,7.5)
z60 = zonedata(False,60,90,90,9.0,90,9.0)
z80 = zonedata(False,80,120,120,12,120,12)
z100 = zonedata(False,100,150,150,15,150,15)
z150 = zonedata(False,150,225,225,23,225,23)
z200 = zonedata(False,200,300,300,30,300,30)

class jointtarget(NamedTuple):
    robax: np.ndarray # shape=(6,)
    extax: np.ndarray # shape=(6,)

    def to_rapid(self):
        return f"[{util.nums_to_rapid_array(self.robax)},{util.nums_to_rapid_array(self.extax)}]"

class pose(NamedTuple):
    trans: np.ndarray # [x,y,z]
    rot: np.ndarray # [qw,qx,qy,qz]

    def to_rapid(self):
        return f"[{util.nums_to_rapid_array(self.trans)},{util.nums_to_rapid_array(self.rot)}]"

class confdata(NamedTuple):
    cf1: float
    cf4: float
    cf6: float
    cfx: float

    def to_rapid(self):
        return util.nums_to_rapid_array([
            self.cf1,
            self.cf4,
            self.cf6,
            self.cfx
        ])

class robtarget(NamedTuple):
    trans: np.ndarray # [x,y,z]
    rot: np.ndarray # [qw,qx,qy,qz]
    robconf: confdata # 
    extax: np.ndarray # shape=(6,)

    def to_rapid(self):
        trans_str = util.nums_to_rapid_array(self.trans)
        rot_str = util.nums_to_rapid_array(self.rot)
        robconf_str = self.robconf.to_rapid()
        extax_str = util.nums_to_rapid_array(self.extax)
        return f"[{trans_str},{rot_str},{robconf_str},{extax_str}]"

class loaddata(NamedTuple):
    mass: float
    cog: np.ndarray # shape=(3,)
    aom: np.ndarray # shape=(4,)
    ix: float
    iy: float
    iz: float

    def to_rapid(self):
        return f"[{self.mass},{util.nums_to_rapid_array(self.cog)}," \
            f"{util.nums_to_rapid_array(self.aom)},{self.ix},{self.iy},{self.iz}]"

class CirPathModeSwitch(IntEnum):
    PathFrame = 1
    ObjectFrame = 2
    CirPointOri = 3
    Wrist45 = 4
    Wrist46 = 5
    Wrist56 = 6

class tooldata(NamedTuple):
    robhold: bool
    tframe: pose
    tload : loaddata

    def to_rapid(self):
        return f"[{util.bool_to_rapid(self.robhold)},{self.tframe.to_rapid()},{self.tload.to_rapid()}]"

class wobjdata(NamedTuple):
    robhold: bool
    ufprog: bool
    ufmec: str
    uframe: pose
    oframe: pose

    def to_rapid(self):
        return f"[{util.bool_to_rapid(self.robhold)},{util.bool_to_rapid(self.ufprog)}," \
            f"\"{self.ufmec}\",{self.uframe.to_rapid()},{self.oframe.to_rapid()}]"