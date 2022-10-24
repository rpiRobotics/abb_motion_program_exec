# Copyright 2022 Wason Technology, LLC
#                     Rensselaer Polytechnic Institute
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


import re
from typing import Callable, NamedTuple, Any, List
import struct
import numpy as np
import io
import requests
from bs4 import BeautifulSoup
import time
import datetime
from enum import IntEnum

class speeddata(NamedTuple):
    v_tcp: float
    v_ori: float
    v_leax: float
    v_reax: float

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

class pose(NamedTuple):
    trans: np.ndarray # [x,y,z]
    rot: np.ndarray # [qw,qx,qy,qz]

class confdata(NamedTuple):
    cf1: float
    cf4: float
    cf6: float
    cfx: float

class robtarget(NamedTuple):
    trans: np.ndarray # [x,y,z]
    rot: np.ndarray # [qw,qx,qy,qz]
    robconf: confdata # 
    extax: np.ndarray # shape=(6,)

class loaddata(NamedTuple):
    mass: float
    cog: np.ndarray # shape=(3,)
    aom: np.ndarray # shape=(4,)
    ix: float
    iy: float
    iz: float

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

class wobjdata(NamedTuple):
    robhold: bool
    ufprog: bool
    ufmec: str
    uframe: pose
    oframe: pose

_speeddata_struct_fmt = struct.Struct("<4f")
def _speeddata_to_bin(z: speeddata):
    return _speeddata_struct_fmt.pack(
        z.v_tcp, z.v_ori, z.v_leax, z.v_reax
    )

_zonedata_struct_fmt = struct.Struct("<7f")
def _zonedata_to_bin(z: zonedata):
    return _zonedata_struct_fmt.pack(
        0.0 if not z.finep else 1.0,
        z.pzone_tcp, z.pzone_ori, z.pzone_eax, z.zone_ori, z.zone_leax, z.zone_reax
    )

def _fix_array(arr, l):
    if isinstance(arr,list):
        assert len(arr) == l, f"Invalid array, expected array length {l}"
        return np.array(arr,dtype=np.float64)
    if arr.shape == (l,):
        return arr
    if arr.shape == (l,1) or arr.shape == (1,l):
        return arr.flatten()
    assert False, f"Invalid array, expected array length {l}"
        

_jointtarget_struct_fmt = struct.Struct("<12f")
def _jointtarget_to_bin(j: jointtarget):
    r = _fix_array(j.robax,6).tolist()
    e = _fix_array(j.extax,6).tolist()
    return _jointtarget_struct_fmt.pack(*r, *e)

_pose_struct_fmt = struct.Struct("<7f")
def _pose_to_bin(p: pose):
    p1 = _fix_array(p.trans,3).tolist()
    q = _fix_array(p.rot,4).tolist()
    return _pose_struct_fmt.pack(*p1,*q)

_confdata_struct_fmt = struct.Struct("<4f")
def _confdata_to_bin(c: confdata):
    return _confdata_struct_fmt.pack(c.cf1, c.cf4, c.cf6, c.cfx)

_robtarget_extax_struct_fmt = struct.Struct("<6f")
def _robtarget_to_bin(r: robtarget):
    pose_bin = _pose_to_bin(pose(r.trans,r.rot))
    robconf_bin = _confdata_to_bin(r.robconf)
    extax = _fix_array(r.extax,6).tolist()
    extax_bin = _robtarget_extax_struct_fmt.pack(*extax)
    return pose_bin + robconf_bin + extax_bin

_num_struct_fmt = struct.Struct("<f")
def _num_to_bin(f):
    return _num_struct_fmt.pack(f)

_intnum_struct_fmt = struct.Struct("<i")
def _intnum_to_bin(f):
    return _intnum_struct_fmt.pack(f)

def _str_to_bin(s: str):
    return _num_to_bin(len(s)) + s.encode('ascii')

_loaddata_struct_fmt = struct.Struct("<11f")
def _loaddata_to_bin(l: loaddata):
    cog = _fix_array(l.cog,3)
    aom = _fix_array(l.aom,4)
    return _loaddata_struct_fmt.pack(
        l.mass, *cog, *aom, l.ix, l.iy, l.iz
    )

def _tooldata_to_bin(td: tooldata):
    robhold_bin = _num_to_bin(0.0 if not td.robhold else 1.0)
    tframe_bin = _pose_to_bin(td.tframe)
    tload_bin = _loaddata_to_bin(td.tload)
    return robhold_bin + tframe_bin + tload_bin

def _wobjdata_to_bin(wd: wobjdata):
    robhold_bin = _num_to_bin(0.0 if not wd.robhold else 1.0)
    ufprog_bin = _num_to_bin(0.0 if not wd.ufprog else 1.0)
    ufmec_bin = _str_to_bin(wd.ufmec)
    uframe_bin = _pose_to_bin(wd.uframe)
    oframe_bin = _pose_to_bin(wd.oframe)
    return robhold_bin + ufprog_bin + ufmec_bin + uframe_bin + oframe_bin

def _read_num(f: io.IOBase):
    b = f.read(4)
    return _num_struct_fmt.unpack(b)[0]

def _read_nums(f: io.IOBase, n: int):
    return [_read_num(f) for _ in range(n)]

def _read_struct_io(f: io.IOBase, s: struct.Struct):
    return s.unpack(f.read(s.size))

def _read_str_to_rapid(f: io.IOBase):
    l = int(_read_num(f))
    s_ascii = f.read(l)
    return "\"" + s_ascii.decode('ascii') + "\""

def _nums_to_rapid_array(nums: List[float]):
    return "[" + ", ".join([str(n) for n in nums]) + "]"

def _read_struct_io_to_rapid_array(f: io.IOBase, s: struct.Struct):
    nums = _read_struct_io(f,s)
    return _nums_to_rapid_array(nums)

def _speeddata_io_to_rapid(f: io.IOBase):
    return _read_struct_io_to_rapid_array(f,_speeddata_struct_fmt)

def _zonedata_io_to_rapid(f: io.IOBase):
    nums =  list(_read_struct_io(f, _zonedata_struct_fmt))
    nums[0] = "TRUE" if nums[0] != 0 else "FALSE"
    return _nums_to_rapid_array(nums)

def _jointtarget_io_to_rapid(f: io.IOBase):
    nums = _read_struct_io(f,_jointtarget_struct_fmt)
    return f"[{_nums_to_rapid_array(nums[:6])},{_nums_to_rapid_array(nums[6:])}]"

def _pose_io_to_rapid(f: io.IOBase):
    nums = _read_struct_io(f, _pose_struct_fmt)
    return f"[{_nums_to_rapid_array(nums[:3])},{_nums_to_rapid_array(nums[3:])}]"

def _confdata_io_to_rapid(f: io.IOBase):
    return _read_struct_io_to_rapid_array(f, _confdata_struct_fmt)

def _extax_io_to_rapid(f: io.IOBase):
    return _read_struct_io_to_rapid_array(f, _robtarget_extax_struct_fmt)

def _robtarget_io_to_rapid(f: io.IOBase):
    pose_nums = _read_struct_io(f,_pose_struct_fmt)
    trans_str = _nums_to_rapid_array(pose_nums[:3])
    rot_str = _nums_to_rapid_array(pose_nums[3:])
    robconf_str = _confdata_io_to_rapid(f)
    extax_str = _extax_io_to_rapid(f)
    return f"[{trans_str},{rot_str},{robconf_str},{extax_str}]"

def _loaddata_io_to_rapid(f: io.IOBase):
    nums = _read_struct_io(f, _loaddata_struct_fmt)
    return f"[{nums[0]},{_nums_to_rapid_array(nums[1:4])},{_nums_to_rapid_array(nums[4:8])},{nums[8]},{nums[9]},{nums[10]}]"

def _tooldata_io_to_rapid(f: io.IOBase):
    robhold_num = _read_num(f)
    robhold_str = "TRUE" if robhold_num != 0 else "FALSE"
    tframe_str = _pose_io_to_rapid(f)
    tload_str = _loaddata_io_to_rapid(f)
    return f"[{robhold_str},{tframe_str},{tload_str}]"

def _wobjdata_io_to_rapid(f: io.IOBase):
    robhold_num = _read_num(f)
    robhold_str = "TRUE" if robhold_num != 0 else "FALSE"
    ufprog_num = _read_num(f)
    ufprog_str = "TRUE" if ufprog_num != 0 else "FALSE"
    ufmec_str = _read_str_to_rapid(f)
    uframe_str = _pose_io_to_rapid(f)
    oframe_str = _pose_io_to_rapid(f)
    return f"[{robhold_str},{ufprog_str},{ufmec_str},{uframe_str},{oframe_str}]"

def _moveabsj_io_to_rapid(f: io.IOBase):
    cmd_num = _read_num(f)
    op = _read_num(f)
    assert op == 0x1
    to_joint_pos_str = _jointtarget_io_to_rapid(f)
    speed_str = _speeddata_io_to_rapid(f)
    zone_str = _zonedata_io_to_rapid(f)
    return f"MoveAbsJ {to_joint_pos_str}, {speed_str}, {zone_str}, motion_program_tool;"

def _movej_io_to_rapid(f: io.IOBase):
    cmd_num = _read_num(f)
    op = _read_num(f)
    assert op == 0x2
    to_point_str = _robtarget_io_to_rapid(f)
    speed_str = _speeddata_io_to_rapid(f)
    zone_str = _zonedata_io_to_rapid(f)
    
    return f"MoveJ {to_point_str}, {speed_str}, {zone_str}, motion_program_tool;"

def _movel_io_to_rapid(f: io.IOBase):
    cmd_num = _read_num(f)
    op = _read_num(f)
    assert op == 0x3
    to_point_str = _robtarget_io_to_rapid(f)
    speed_str = _speeddata_io_to_rapid(f)
    zone_str = _zonedata_io_to_rapid(f)
    return f"MoveL {to_point_str}, {speed_str}, {zone_str}, motion_program_tool;"

def _movec_io_to_rapid(f: io.IOBase):
    cmd_num = _read_num(f)
    op = _read_num(f)
    assert op == 0x4
    cir_point_str = _robtarget_io_to_rapid(f)
    to_point_str = _robtarget_io_to_rapid(f)
    speed_str = _speeddata_io_to_rapid(f)
    zone_str = _zonedata_io_to_rapid(f)
    return f"MoveC {cir_point_str}, {to_point_str}, {speed_str}, {zone_str}, motion_program_tool;"

def _waittime_io_to_rapid(f: io.IOBase):
    cmd_num = _read_num(f)
    op = _read_num(f)
    assert op == 0x5
    t = _read_num(f)
    return f"WaitTime {t};"

def _cirpathmode_io_to_rapid(f: io.IOBase):
    cmd_num = _read_num(f)
    op = _read_num(f)
    assert op == 0x6
    t = _read_num(f)
    if t == 1:
        return r"CirPathMode\PathFrame;"
    if t == 2:
        return r"CirPathMode\ObjectFrame;"
    if t == 3:
        return r"CirPathMode\CirPointOri;"
    if t == 4:
        return r"CirPathMode\Wrist45;"
    if t == 5:
        return r"CirPathMode\Wrist46;"
    if t == 6:
        return r"CirPathMode\Wrist56;"
    assert False, "Invalid CirPathMode switch"

tool0 = tooldata(True,pose([0,0,0],[1,0,0,0]),loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0))
wobj0 = wobjdata(False, True, "", pose([0,0,0],[1,0,0,0]), pose([0,0,0],[1,0,0,0]))        

class MotionProgram:
    def __init__(self,first_cmd_num: int=1, tool: tooldata = None, wobj: wobjdata = None, timestamp: str = None):

        if timestamp is None:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")[:-2]
        assert re.match(r"^\d{4}\-\d{2}\-\d{2}-\d{2}\-\d{2}\-\d{2}\-\d{4}$", timestamp)

        self._timestamp = timestamp

        self._f = io.BytesIO()
        # Version number
        self._f.write(_num_to_bin(10005))
        if tool is None:
            tool = tool0
        if wobj is None:
            wobj = wobj0
        self._f.write(_tooldata_to_bin(tool))
        self._f.write(_wobjdata_to_bin(wobj))
        self._f.write(_str_to_bin(timestamp))
        self._cmd_num=first_cmd_num

    def MoveAbsJ(self, to_joint_pos: jointtarget, speed: speeddata, zone: zonedata):
        to_joint_pos_b = _jointtarget_to_bin(to_joint_pos)
        speed_b = _speeddata_to_bin(speed)
        zone_b = _zonedata_to_bin(zone)
        self._f.write(_num_to_bin(self._cmd_num))
        self._f.write(_num_to_bin(0x1))
        self._f.write(to_joint_pos_b)
        self._f.write(speed_b)
        self._f.write(zone_b)
        self._cmd_num+=1

    def MoveJ(self, to_point: robtarget, speed: speeddata, zone: zonedata):
        to_point_b = _robtarget_to_bin(to_point)
        speed_b = _speeddata_to_bin(speed)
        zone_b = _zonedata_to_bin(zone)
        self._f.write(_num_to_bin(self._cmd_num))
        self._f.write(_num_to_bin(0x2))
        self._f.write(to_point_b)
        self._f.write(speed_b)
        self._f.write(zone_b)
        self._cmd_num+=1

    def MoveL(self, to_point: robtarget, speed: speeddata, zone: zonedata):
        to_point_b = _robtarget_to_bin(to_point)
        speed_b = _speeddata_to_bin(speed)
        zone_b = _zonedata_to_bin(zone)
        self._f.write(_num_to_bin(self._cmd_num))
        self._f.write(_num_to_bin(0x3))
        self._f.write(to_point_b)
        self._f.write(speed_b)
        self._f.write(zone_b)
        self._cmd_num+=1

    def MoveC(self, cir_point: robtarget, to_point: robtarget, speed: speeddata, zone: zonedata):
        cir_point_b = _robtarget_to_bin(cir_point)
        to_point_b = _robtarget_to_bin(to_point)
        speed_b = _speeddata_to_bin(speed)
        zone_b = _zonedata_to_bin(zone)
        self._f.write(_num_to_bin(self._cmd_num))
        self._f.write(_num_to_bin(0x4))
        self._f.write(cir_point_b)
        self._f.write(to_point_b)
        self._f.write(speed_b)
        self._f.write(zone_b)
        self._cmd_num+=1
    
    def WaitTime(self, t: float):
        assert t > 0, "Wait time must be >0"
        self._f.write(_num_to_bin(self._cmd_num))
        self._f.write(_num_to_bin(0x5))
        self._f.write(_num_to_bin(t))
        self._cmd_num+=1

    def CirPathMode(self, switch: CirPathModeSwitch):
        val = switch.value
        assert val >=1 and val <= 6, "Invalid CirPathMode switch"
        self._f.write(_num_to_bin(self._cmd_num))
        self._f.write(_num_to_bin(0x6))
        self._f.write(_num_to_bin(val))
        self._cmd_num+=1

    def get_program_bytes(self):
        return self._f.getvalue()

    def get_program_rapid(self, module_name="motion_program_exec_gen"):
        program_bytes = self.get_program_bytes()
        f = io.BufferedReader(io.BytesIO(program_bytes))
        o = io.StringIO()

        ver = _read_num(f)
        tooldata_str = _tooldata_io_to_rapid(f)
        wobjdata_str = _wobjdata_io_to_rapid(f)
        timestamp_str = _read_str_to_rapid(f)


        print(f"MODULE {module_name}", file=o)
        print(f"    ! abb_motion_program_exec format version {ver}", file=o)
        print(f"    ! abb_motion_program_exec timestamp {timestamp_str}", file=o)
        print(f"    PERS tooldata motion_program_tool := {tooldata_str};", file=o)
        print(f"    PERS wobjdata motion_program_wobj := {wobjdata_str};", file=o)
        print(f"    PROC main()", file=o)
        
        while True:
            nums_bytes = f.peek(8)
            if len(nums_bytes) < 8:
                break
            op = _num_struct_fmt.unpack_from(nums_bytes, 4)[0]
            cmd_num = _num_struct_fmt.unpack_from(nums_bytes, 0)[0]
            print(f"        ! cmd_num = {cmd_num}",file=o)
            if op == 0x1:
                print(f"        {_moveabsj_io_to_rapid(f)}",file=o)
            elif op == 0x2:
                print(f"        {_movej_io_to_rapid(f)}",file=o)
            elif op == 0x3:
                print(f"        {_movel_io_to_rapid(f)}",file=o)
            elif op == 0x4:
                print(f"        {_movec_io_to_rapid(f)}",file=o)
            elif op == 0x5:
                print(f"        {_waittime_io_to_rapid(f)}",file=o)
            elif op == 0x6:
                print(f"        {_cirpathmode_io_to_rapid(f)}",file=o)
            else:
                assert False, f"Invalid command opcode: {op}"
        
        print("    ENDPROC", file=o)
        print("ENDMODULE", file=o)
        
        return o.getvalue()

    def get_timestamp(self):
        return self._timestamp


class MotionProgramExecClient:
    def __init__(self, base_url='http://127.0.0.1:80', username='Default User', password='robotics'):
        self.base_url=base_url
        self.auth=requests.auth.HTTPDigestAuth(username, password)
        self._session=requests.Session()
        
    def _do_get(self, relative_url):
        url="/".join([self.base_url, relative_url])
        res=self._session.get(url, auth=self.auth)
        try:            
            return self._process_response(res)
        finally:
            res.close()
    

    def _do_post(self, relative_url, payload=None):
        url="/".join([self.base_url, relative_url])
        res=self._session.post(url, data=payload, auth=self.auth)
        try:
            return self._process_response(res)
        finally:
            res.close()

    def _process_response(self, response):        
        soup=BeautifulSoup(response.text, features="html.parser")

        if (response.status_code == 500):
            raise Exception("Robot returning 500 Internal Server Error")
    
        if (response.status_code == 200 or response.status_code == 201  \
            or response.status_code==202 or response.status_code==204):
            
            return soup.body
        
        if soup.body is None:
            raise Exception("Robot returning HTTP error " + str(response.status_code))
        
        error_code=int(soup.find('span', attrs={'class':'code'}).text)
        error_message1=soup.find('span', attrs={'class': 'msg'})
        if (error_message1 is not None):
            error_message=error_message1.text
        else:
            error_message="Received error from ABB robot: " + str(error_code)

        raise ABBException(error_message, error_code)

    def start(self, cycle='asis',tasks=['T_ROB1']):

        rob_tasks = self.get_tasks()
        for t in tasks:
            assert t in rob_tasks, f"Cannot start unknown task {t}"

        for rob_task in rob_tasks.values():
            if not rob_task.motiontask:
                continue
            if rob_task.name in tasks:
                if not rob_task.active:
                    self.activate_task(rob_task.name)
            else:
                if rob_task.active:
                    self.deactivate_task(rob_task.name)

        payload={"regain": "continue", "execmode": "continue" , "cycle": cycle, "condition": "none", "stopatbp": "disabled", "alltaskbytsp": "true"}
        res=self._do_post("rw/rapid/execution?action=start", payload)

    def activate_task(self, task):
        payload={}
        self._do_post(f"rw/rapid/tasks/{task}?action=activate",payload)

    def deactivate_task(self, task):
        payload={}
        self._do_post(f"rw/rapid/tasks/{task}?action=deactivate",payload)

    def stop(self):
        payload={"stopmode": "stop"}
        res=self._do_post("rw/rapid/execution?action=stop", payload)

    def resetpp(self):
        res=self._do_post("rw/rapid/execution?action=resetpp")

    def get_ramdisk_path(self):
        soup = self._do_get("ctrl/$RAMDISK")
        return soup.find('span', attrs={'class': 'value' }).text

    def get_execution_state(self):
        soup = self._do_get("rw/rapid/execution")
        ctrlexecstate=soup.find('span', attrs={'class': 'ctrlexecstate'}).text
        cycle=soup.find('span', attrs={'class': 'cycle'}).text
        return RAPIDExecutionState(ctrlexecstate, cycle)
    
    def get_controller_state(self):
        soup = self._do_get("rw/panel/ctrlstate")
        return soup.find('span', attrs={'class': 'ctrlstate'}).text
    
    def get_operation_mode(self):
        soup = self._do_get("rw/panel/opmode")        
        return soup.find('span', attrs={'class': 'opmode'}).text
    
    def get_digital_io(self, signal, network='Local', unit='DRV_1'):
        soup = self._do_get("rw/iosystem/signals/" + network + "/" + unit + "/" + signal)        
        state = soup.find('span', attrs={'class': 'lvalue'}).text
        return int(state)
    
    def set_digital_io(self, signal, value, network='Local', unit='DRV_1'):
        lvalue = '1' if bool(value) else '0'
        payload={'lvalue': lvalue}
        res=self._do_post("rw/iosystem/signals/" + network + "/" + unit + "/" + signal + "?action=set", payload)
    
    def get_rapid_variable(self, var):
        soup = self._do_get("rw/rapid/symbol/data/RAPID/T_ROB1/" + var)        
        state = soup.find('span', attrs={'class': 'value'}).text
        return state
    
    def set_rapid_variable(self, var, value):
        payload={'value': value}
        res=self._do_post("rw/rapid/symbol/data/RAPID/T_ROB1/" + var + "?action=set", payload)
        
    def read_file(self, filename):
        url="/".join([self.base_url, "fileservice", filename])
        res=self._session.get(url, auth=self.auth)
        try:            
            return res.content
        finally:
            res.close()

    def upload_file(self, filename, contents):
        url="/".join([self.base_url, "fileservice" , filename])
        res=self._session.put(url, contents, auth=self.auth)
        assert res.ok, res.reason
        res.close()

    def delete_file(self, filename):
        url="/".join([self.base_url, "fileservice" , filename])
        res=self._session.delete(url, auth=self.auth)
        res.close()

    def read_event_log(self, elog=0):
        o=[]
        soup = self._do_get("rw/elog/" + str(elog) + "/?lang=en")
        state=soup.find('div', attrs={'class': 'state'})
        ul=state.find('ul')
        
        for li in ul.findAll('li'):
            seqnum = int(li.attrs["title"].split("/")[-1])
            def find_val(v):
                return li.find('span', attrs={'class': v}).text
            msg_type=int(find_val('msgtype'))
            code=int(find_val('code'))
            tstamp=datetime.datetime.strptime(find_val('tstamp'), '%Y-%m-%d T  %H:%M:%S')
            title=find_val('title')
            desc=find_val('desc')
            conseqs=find_val('conseqs')
            causes=find_val('causes')
            actions=find_val('actions')
            args=[]
            nargs=int(find_val('argc'))
            for i in range(nargs):
                arg=find_val('arg%d' % (i+1))
                args.append(arg)
            
            o.append(RAPIDEventLogEntry(seqnum,msg_type,code,tstamp,args,title,desc,conseqs,causes,actions))
        return o

    def get_tasks(self):
        o = {}
        soup = self._do_get("rw/rapid/tasks")
        state=soup.find('div', attrs={'class': 'state'})
        ul=state.find('ul')
        
        for li in ul.findAll('li'):
            def find_val(v):
                return li.find('span', attrs={'class': v}).text
            name=find_val('name')
            type_=find_val('type')
            taskstate=find_val('taskstate')
            excstate=find_val('excstate')
            try:
                active=find_val('active') == "On"
            except:
                active=False
            try:
              motiontask=find_val("motiontask").lower() == "true"
            except:
                motiontask=False

            o[name]=RAPIDTaskState(name,type_,taskstate,excstate,active,motiontask)
        
        return o

    def execute_motion_program(self, motion_program: MotionProgram, task="T_ROB1"):
        b = motion_program.get_program_bytes()
        assert len(b) > 0, "Motion program must not be empty"
        ramdisk = self.get_ramdisk_path()
        filename = f"{ramdisk}/motion_program.bin"
        if task != "T_ROB1":
            task_m = re.match(r"^.*[A-Za-z_](\d+)$",task)
            if task_m:
                filename_ind = int(task_m.group(1))
                filename = f"{ramdisk}/motion_program{filename_ind}.bin"
        def _upload():            
            self.upload_file(filename, b)

        return self._execute_motion_program([task],_upload)

    def execute_multimove_motion_program(self, motion_programs: List[MotionProgram], tasks=None):

        if tasks is None:
            tasks = [f"T_ROB{i+1}" for i in range(len(motion_programs))]        

        assert len(motion_programs) == len(tasks), \
            "Motion program list and task list must have some length"

        assert len(tasks) > 1, "Multimove program must have at least two tasks"

        b = []
        for mp in motion_programs:
            b.append(mp.get_program_bytes())
        assert len(b) > 0, "Motion program must not be empty"
        filenames = []
        ramdisk = self.get_ramdisk_path()
        for task in tasks:
            filename = f"{ramdisk}/motion_program.bin"
            if task != "T_ROB1":
                task_m = re.match(r"^.*[A-Za-z_](\d+)$",task)
                if task_m:
                    filename_ind = int(task_m.group(1))
                    filename = f"{ramdisk}/motion_program{filename_ind}.bin"
            filenames.append(filename)

        assert len(filenames) == len(b)
        def _upload():
            for i in range(len(filenames)):
                self.upload_file(filenames[i], b[i])

        return self._execute_motion_program(tasks,_upload)

    def _execute_motion_program(self, tasks, upload_fn: Callable[[],None]):
        
        exec_state = self.get_execution_state()
        assert exec_state.ctrlexecstate == "stopped"
        #assert exec_state.cycle == "once"
        ctrl_state = self.get_controller_state()
        assert ctrl_state == "motoron"

        log_before = self.read_event_log()
        prev_seqnum = log_before[0].seqnum

        self.resetpp()
        upload_fn()
        
        self.start(cycle='once',tasks=tasks)
        while True:
            exec_state = self.get_execution_state()
            if exec_state.ctrlexecstate != "running":
                break
            time.sleep(0.05)

        log_after_raw = self.read_event_log()
        log_after = []
        for l in log_after_raw:
            if l.seqnum > prev_seqnum:
                log_after.append(l)
            elif prev_seqnum > 61440 and l.seqnum < 4096:
                # Handle uint16 wraparound
                log_after.append(l)
            else:
                break
        
        failed = False
        for l in log_after:
            if l.msgtype >= 2:
                if len(l.args) > 0 and l.args[0].lower() == "motion program failed":
                    assert False, l.args[1] + " " + l.args[2] + " " + l.args[3] + " " + l.args[4]
            if l.msgtype >= 3:
                failed = True

        if failed:
            assert False, "Motion Program Failed, see robot error log for details"

        found_log_open = False
        found_log_close = False
        log_filename = ""

        for l in log_after:
            if l.code == 80003:
                if l.args[0].lower() == "motion program log file closed":
                    assert not found_log_close, "Found more than one log closed message"
                    found_log_close = True
                
                if l.args[0].lower() == "motion program log file opened":
                    assert not found_log_open, "Found more than one log opened message"
                    found_log_open = True
                    log_filename_m = re.search(r"(log\-[\d\-]+\.csv)",l.args[1])
                    assert log_filename_m, "Invalid log opened message"
                    log_filename = log_filename_m.group(1)

        assert found_log_open and found_log_close and len(log_filename) > 0, "Could not find log file messages in robot event log"

        ramdisk = self.get_ramdisk_path()
        log_contents = self.read_file(f"{ramdisk}/{log_filename}")
        try:
            self.delete_file(f"{ramdisk}/{log_filename}")
        except:
            pass
        return log_contents


        
class ABBException(Exception):
    def __init__(self, message, code):
        super(ABBException, self).__init__(message)
        self.code=code

class RAPIDExecutionState(NamedTuple):
    ctrlexecstate: Any
    cycle: Any

class RAPIDEventLogEntry(NamedTuple):
    seqnum: int
    msgtype: int
    code: int 
    tstamp: datetime.datetime 
    args: List[Any]
    title: str
    desc: str
    conseqs: str
    causes: str
    actions: str

class RAPIDTaskState(NamedTuple):
    name: str
    type_: str
    taskstate: str
    excstate: str
    active: bool
    motiontask: bool

def main():
    j1 = jointtarget([10,20,30,40,50,60],[0]*6)
    j2 = jointtarget([90,-91,60,-93,94,-95],[0]*6)
    j3 = jointtarget([-80,81,-82,83,-84,85],[0]*6)

    mp = MotionProgram()
    mp.MoveAbsJ(j1,v1000,fine)
    mp.MoveAbsJ(j2,v5000,fine)
    mp.MoveAbsJ(j3,v500,fine)
    mp.MoveAbsJ(j2,v5000,z50)
    mp.MoveAbsJ(j3,v500,z200)
    mp.MoveAbsJ(j2,v5000,fine)
    mp.WaitTime(1)

    r1 = robtarget([0.1649235*1e3, 0.1169957*1e3, 0.9502961*1e3], [ 0.6776466, -0.09003431, 0.6362069, 0.3576725 ], confdata(0,0,0,0),[0]*6)
    r2 = robtarget([ 0.6243948*1e3, -0.479558*1e3 ,  0.7073749*1e3], [ 0.6065634, -0.2193409,  0.6427138, -0.4133877], confdata(-1,-1,0,1),[0]*6)

    r3 = robtarget([417.9236, 276.9956, 885.2959], [ 0.8909725 , -0.1745558 ,  0.08864544,  0.4096832 ], confdata( 0.,  1., -2.,  0.),[0]*6)
    r4 = robtarget([417.9235 , -11.00438, 759.2958 ], [0.7161292 , 0.1868255 , 0.01720813, 0.6722789 ], confdata( 0.,  2., -2.,  0.),[0]*6)
    r5 = robtarget([ 417.9235, -173.0044,  876.2958], [0.6757616, 0.3854275, 0.2376617, 0.5816431], confdata(-1.,  1., -1.,  0.),[0]*6)

    mp.MoveJ(r1,v500,fine)
    mp.MoveJ(r2,v400,fine)
    mp.MoveJ(r1,v500,z100)
    mp.MoveJ(r2,v400,z100)
    mp.MoveJ(r1,v500,fine)
    mp.WaitTime(1.5)

    mp.MoveJ(r3,v5000,fine)
    mp.MoveL(r4,v200,fine)
    mp.MoveL(r3,v200,fine)
    mp.MoveL(r4,v1000,z100)
    mp.MoveL(r3,v1000,z100)
    mp.MoveL(r4,v1000,fine)
    mp.WaitTime(2.5)

    mp.MoveJ(r3,v5000,fine)

    mp.CirPathMode(CirPathModeSwitch.CirPointOri)
    mp.MoveC(r4,r5,v200,z10)
    mp.MoveC(r4,r3,v50,fine)

    print(mp.get_program_rapid())

    client = MotionProgramExecClient()
    log_results = client.execute_motion_program(mp)

    # Write log csv to file
    # with open("log.csv","wb") as f:
    #    f.write(log_results)

    # Or convert to string and use in memory
    log_results_str = log_results.decode('ascii')
    print(log_results_str)

if __name__ == "__main__":
    main()