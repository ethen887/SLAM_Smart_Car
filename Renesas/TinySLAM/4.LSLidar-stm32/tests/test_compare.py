"""比较瑞萨移植核心与 MapBuild_Demo 原核心的同输入输出。"""
import ctypes as c
import math
import random
import subprocess
from pathlib import Path

root = Path(__file__).resolve().parents[1]
demo = root.parents[1] / 'MapBuild_Demo' / 'core'
out = root / 'build'
out.mkdir(exist_ok=True)
subprocess.run(['gcc','-std=c99','-shared','-o',str(out/'map_reference.dll'),str(demo/'map_core.c'),'-lm'],check=True)
subprocess.run(['gcc','-std=c99','-shared','-o',str(out/'map_port.dll'),str(root/'src/algorithm/map_core.c'),'-lm'],check=True)
reference, port = c.CDLL(str(out/'map_reference.dll')), c.CDLL(str(out/'map_port.dll'))
class Config(c.Structure):
    _fields_=[('width',c.c_int),('height',c.c_int)]+[(k,c.c_float) for k in ('resolution','origin_x','origin_y','free_delta','occupied_delta','min_log','max_log')]
class Pose(c.Structure):_fields_=[(k,c.c_float) for k in ('x','y','theta')]
class Ray(c.Structure):_fields_=[('angle',c.c_float),('distance',c.c_float),('hit',c.c_int)]
class Cell(c.Structure):_fields_=[('log_odds',c.c_float),('last_frame',c.c_uint32),('observed',c.c_uint8)]
class Map(c.Structure):_fields_=[('c',Config),('cells',c.POINTER(Cell)),('marks',c.POINTER(c.c_uint8))]
reference.map_create.argtypes=[c.POINTER(Config)];reference.map_create.restype=c.c_void_p
reference.map_update.argtypes=[c.c_void_p,Pose,c.POINTER(Ray),c.c_int,c.c_uint32]
reference.map_copy.argtypes=[c.c_void_p,c.POINTER(Cell),c.c_int]
reference.map_destroy.argtypes=[c.c_void_p];reference.map_destroy.restype=None
port.map_init.argtypes=[c.POINTER(Map),c.POINTER(Config),c.POINTER(Cell),c.POINTER(c.c_uint8),c.c_int]
port.map_update.argtypes=[c.POINTER(Map),Pose,c.POINTER(Ray),c.c_int,c.c_uint32]
cfg=Config(100,100,.05,-2.5,-2.5,-.4,.85,-4,4)
ref=reference.map_create(c.byref(cfg));assert ref
cells=(Cell*10000)();marks=(c.c_uint8*10000)();m=Map()
assert port.map_init(c.byref(m),c.byref(cfg),cells,marks,10000)==0
rng=random.Random(42);copy=(Cell*10000)()
for frame in range(1,21):
    pose=Pose(rng.uniform(-1,1),rng.uniform(-1,1),rng.uniform(-math.pi,math.pi))
    rays=(Ray*180)(*[Ray(rng.uniform(-math.pi,math.pi),rng.uniform(.01,1),rng.randrange(2)) for _ in range(180)])
    assert reference.map_update(ref,pose,rays,len(rays),frame)==0
    assert port.map_update(c.byref(m),pose,rays,len(rays),frame)==0
    assert reference.map_copy(ref,copy,10000)==0
    for i,(a,b) in enumerate(zip(copy,cells)):
        assert a.observed==b.observed and a.last_frame==b.last_frame and abs(a.log_odds-b.log_odds)<1e-6,(frame,i,a.observed,a.last_frame,a.log_odds,b.observed,b.last_frame,b.log_odds,pose.x,pose.y,pose.theta)
reference.map_destroy(ref)
print('20 帧、无裁剪、混合命中/无回波的地图内位姿：移植核心与 Demo 完全一致')
