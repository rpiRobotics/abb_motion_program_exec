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


from typing import NamedTuple, Any
import struct
import numpy as np
import io
import requests
from bs4 import BeautifulSoup
import time

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
    tframe: pose
    robconf: confdata # 
    extax: np.ndarray # shape=(6,)

class loaddata(NamedTuple):
    mass: float
    cog: np.ndarray # shape=(3,)
    aom: np.ndarray # shape=(4,)
    ix: float
    iy: float
    iz: float

class tooldata(NamedTuple):
    robhold: bool
    tframe: pose
    tload : loaddata

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
    pose_bin = _pose_to_bin(r.tframe)
    robconf_bin = _confdata_to_bin(r.robconf)
    extax = _fix_array(r.extax,6).tolist()
    extax_bin = _robtarget_extax_struct_fmt.pack(*extax)
    return pose_bin + robconf_bin + extax_bin

_num_struct_fmt = struct.Struct("<f")
def _num_to_bin(f):
    return _num_struct_fmt.pack(f)

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

# [[0, 0, 0], [1, 0, 0, 0]], [0.001, [0, 0, 0.001],[1, 0, 0, 0], 0, 0, 0]]

tool0 = tooldata(True,pose([0,0,0],[1,0,0,0]),loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0))        

class MotionProgram:
    def __init__(self,first_cmd_num: int=1, tool: tooldata = None):
        self._f = io.BytesIO()
        # Version number
        self._f.write(_num_to_bin(10002))
        if tool is None:
            tool = tool0
        self._f.write(_tooldata_to_bin(tool))
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

    def get_program_bytes(self):
        return self._f.getvalue()



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

    def start(self, cycle='asis'):
        payload={"regain": "continue", "execmode": "continue" , "cycle": cycle, "condition": "none", "stopatbp": "disabled", "alltaskbytsp": "false"}
        res=self._do_post("rw/rapid/execution?action=start", payload)

    def stop(self):
        payload={"stopmode": "stop"}
        res=self._do_post("rw/rapid/execution?action=stop", payload)

    def resetpp(self):
        res=self._do_post("rw/rapid/execution?action=resetpp")

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

    def execute_motion_program(self, motion_program : MotionProgram):
        b = motion_program.get_program_bytes()
        assert len(b) > 0, "Motion program must not be empty"
        exec_state = self.get_execution_state()
        assert exec_state.ctrlexecstate == "stopped"
        assert exec_state.cycle == "once"
        self.resetpp()
        self.upload_file("$temp/motion_program.bin", b)

        self.start()
        while True:
            exec_state = self.get_execution_state()
            if exec_state.ctrlexecstate != "running":
                break
            time.sleep(0.05)
        
class ABBException(Exception):
    def __init__(self, message, code):
        super(ABBException, self).__init__(message)
        self.code=code

class RAPIDExecutionState(NamedTuple):
    ctrlexecstate: Any
    cycle: Any

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

    r1 = robtarget(pose([0.1649235*1e3, 0.1169957*1e3, 0.9502961*1e3], [ 0.6776466, -0.09003431, 0.6362069, 0.3576725 ]), confdata(0,0,0,0),[0]*6)
    r2 = robtarget(pose([ 0.6243948*1e3, -0.479558*1e3 ,  0.7073749*1e3], [ 0.6065634, -0.2193409,  0.6427138, -0.4133877]), confdata(-1,-1,0,1),[0]*6)

    r3 = robtarget(pose([417.9236, 276.9956, 885.2959], [ 0.8909725 , -0.1745558 ,  0.08864544,  0.4096832 ]), confdata( 0.,  1., -2.,  0.),[0]*6)
    r4 = robtarget(pose([417.9235 , -11.00438, 759.2958 ], [0.7161292 , 0.1868255 , 0.01720813, 0.6722789 ]), confdata( 0.,  2., -2.,  0.),[0]*6)
    r5 = robtarget(pose([ 417.9235, -173.0044,  876.2958], [0.6757616, 0.3854275, 0.2376617, 0.5816431]), confdata(-1.,  1., -1.,  0.),[0]*6)

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

    mp.MoveC(r4,r5,v200,z10)
    mp.MoveC(r4,r3,v50,fine)

    client = MotionProgramExecClient()
    client.execute_motion_program(mp)

if __name__ == "__main__":
    main()