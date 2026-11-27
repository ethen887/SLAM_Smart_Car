import ctypes as ct
import math
import numpy as np

class Config(ct.Structure):
    _fields_ = [(k, ct.c_int if k in ('width','height') else ct.c_float) for k in ('width','height','resolution','origin_x','origin_y','free_delta','occupied_delta','min_log','max_log')]
class Pose(ct.Structure):
    _fields_ = [(k,ct.c_float) for k in ('x','y','theta')]
class Ray(ct.Structure):
    _fields_ = [('angle',ct.c_float),('distance',ct.c_float),('hit',ct.c_int)]
class Cell(ct.Structure):
    _fields_ = [('log_odds',ct.c_float),('last_frame',ct.c_uint32),('observed',ct.c_uint8)]

def close_obstacles(grid, library, resolution=0.05):
    """Run the same C closing used by the board on a simulation snapshot."""
    h, w = grid.shape; n = h*w
    cells = (Cell*n)()
    view = np.ctypeslib.as_array(cells)
    view['log_odds'] = grid['log'].ravel()
    view['observed'] = grid['seen'].ravel()
    out = (ct.c_uint8*n)(); scratch = (ct.c_uint8*n)()
    library.map_close_cells.argtypes = [ct.POINTER(Cell), ct.c_int, ct.c_int,
                                        ct.POINTER(ct.c_uint8), ct.POINTER(ct.c_uint8), ct.c_int]
    library.map_close_cells.restype = ct.c_int
    if library.map_close_cells(cells, w, h, out, scratch, n): raise RuntimeError('障碍物闭运算失败')
    queue = (ct.c_uint32*n)()
    library.map_classify_walls.argtypes = [ct.POINTER(ct.c_uint8), ct.c_int, ct.c_int, ct.c_float,
                                          ct.POINTER(ct.c_uint8), ct.POINTER(ct.c_uint32), ct.c_int]
    library.map_classify_walls.restype = ct.c_int
    if library.map_classify_walls(out, w, h, resolution, scratch, queue, n): raise RuntimeError('墙面分类失败')
    return np.ctypeslib.as_array(out).copy().reshape(h, w)
class Grid:
    def __init__(self, lib, settings):
        self.lib=ct.CDLL(str(lib)); self.lib.map_create.argtypes=[ct.POINTER(Config)]; self.lib.map_create.restype=ct.c_void_p
        self.lib.map_destroy.argtypes=[ct.c_void_p]; self.lib.map_update.argtypes=[ct.c_void_p,Pose,ct.POINTER(Ray),ct.c_int,ct.c_uint32]; self.lib.map_copy.argtypes=[ct.c_void_p,ct.POINTER(Cell),ct.c_int]
        self.cfg=Config(**settings); self.handle=self.lib.map_create(ct.byref(self.cfg))
        if not self.handle: raise RuntimeError('C 地图初始化失败，请检查地图参数')
    def close(self):
        if self.handle: self.lib.map_destroy(self.handle); self.handle=None
    def __del__(self): self.close()
    def update(self, pose, rays, frame):
        a=(Ray*len(rays))(*(Ray(*r) for r in rays))
        if self.lib.map_update(self.handle,Pose(*pose),a,len(a),frame): raise RuntimeError('C 地图更新失败')
    def snapshot(self):
        n=self.cfg.width*self.cfg.height; a=(Cell*n)()
        if self.lib.map_copy(self.handle,a,n): raise RuntimeError('C 地图读取失败')
        return np.array([(c.log_odds,c.observed,c.last_frame) for c in a],dtype=[('log','f4'),('seen','u1'),('frame','u4')]).reshape(self.cfg.height,self.cfg.width)

def defaults():
    return dict(obstacles=[dict(id='O1',x=-0.4,y=-0.2,w=0.6,h=1.0),dict(id='O2',x=1.0,y=0.9,w=0.6,h=0.5)],waypoints=[[-1.8,-1.7],[0.9,-1.7],[1.8,-0.5],[1.8,1.8],[-1.7,1.8]],closed=False,robot_radius=0.15,linear_speed=0.35,angular_speed=1.0,arrival=0.03,dt=0.05,lidar=dict(min_range=0.04,max_range=4.0,rays=180,frequency=5.0,noise=0.0),odom=dict(scale=1.0,heading_bias=0.0,noise=0.0),map=dict(width=100,height=100,resolution=0.05,origin_x=-2.5,origin_y=-2.5,free_delta=-0.4,occupied_delta=0.85,min_log=-4.0,max_log=4.0),seed=42,monitors=[])

def segment_rect(a,b,box):
    x,y,w,h=box; dx=b[0]-a[0];dy=b[1]-a[1];lo=0.;hi=1.
    for p,q in [(-dx,a[0]-x),(dx,x+w-a[0]),(-dy,a[1]-y),(dy,y+h-a[1])]:
        if abs(p)<1e-12:
            if q<0:return None
        else:
            t=q/p
            if p<0:lo=max(lo,t)
            else:hi=min(hi,t)
            if lo>hi:return None
    return lo

def invalid_segments(s):
    pts=s['waypoints']; radius=s['robot_radius']; bad=[]
    pairs=list(zip(pts,pts[1:]))
    if s['closed'] and len(pts)>1:pairs.append((pts[-1],pts[0]))
    for i,(a,b) in enumerate(pairs):
        if any(not (-2.5+radius<=p[0]<=2.5-radius and -2.5+radius<=p[1]<=2.5-radius) for p in (a,b)) or any(segment_rect(a,b,(o['x']-radius,o['y']-radius,o['w']+2*radius,o['h']+2*radius)) is not None for o in s['obstacles']):bad.append(i)
    return bad

def scan(s,pose,rng):
    x,y,theta=pose; cfg=s['lidar']; out=[]
    for a in np.linspace(-math.pi,math.pi,int(cfg['rays']),endpoint=False):
        ang=theta+a; dx=math.cos(ang);dy=math.sin(ang); d=cfg['max_range'];hit=False
        for t in ((2.5-x)/dx if dx>1e-12 else (-2.5-x)/dx if dx< -1e-12 else float('inf'),(2.5-y)/dy if dy>1e-12 else (-2.5-y)/dy if dy< -1e-12 else float('inf')):
            if 0<=t<d:d=t;hit=True
        for o in s['obstacles']:
            t=segment_rect((x,y),(x+dx*cfg['max_range'],y+dy*cfg['max_range']),(o['x'],o['y'],o['w'],o['h']))
            if t is not None and t*cfg['max_range']<d:d=t*cfg['max_range'];hit=True
        if cfg['noise']:d+=rng.normal(0,cfg['noise'])
        if d<cfg['min_range']:continue
        out.append((float(a),float(min(d,cfg['max_range'])),int(hit and d<=cfg['max_range'])))
    return out

class SlamStats(ct.Structure):
    _fields_ = [
        ('scans_processed', ct.c_uint32),
        ('frontend_accepted', ct.c_uint32),
        ('frontend_fallback', ct.c_uint32),
        ('keyframe_count', ct.c_uint32),
        ('loop_candidates', ct.c_uint32),
        ('loop_closures', ct.c_uint32),
        ('map_regenerations', ct.c_uint32),
        ('last_score', ct.c_float),
        ('last_hit_ratio', ct.c_float),
        ('last_eigen_ratio', ct.c_float),
        ('last_degenerate', ct.c_int),
    ]

class SlamGrid:
    """调用纯 C99 libslamcore 的仿真接口，与 Grid 保持一致的生命周期。"""
    def __init__(self, lib, settings, min_range=0.04, max_range=6.0, loop_closure=True, max_keyframes=256):
        self.lib = ct.CDLL(str(lib))
        self.lib.slam_demo_create.argtypes = [ct.POINTER(Config), ct.c_int, ct.c_int, ct.c_float, ct.c_float]
        self.lib.slam_demo_create.restype = ct.c_void_p
        self.lib.slam_demo_destroy.argtypes = [ct.c_void_p]
        self.lib.slam_demo_step_full_scan.argtypes = [ct.c_void_p, Pose, ct.POINTER(Ray), ct.c_int, ct.POINTER(Pose)]
        self.lib.slam_demo_step_full_scan.restype = ct.c_int
        self.lib.slam_demo_push_packet.argtypes = [ct.c_void_p, Pose, ct.POINTER(Ray), ct.c_int, ct.POINTER(Pose)]
        self.lib.slam_demo_push_packet.restype = ct.c_int
        self.lib.slam_demo_copy_map.argtypes = [ct.c_void_p, ct.POINTER(Cell), ct.c_int]
        self.lib.slam_demo_copy_map.restype = ct.c_int
        self.lib.slam_demo_get_stats.argtypes = [ct.c_void_p, ct.POINTER(SlamStats)]
        self.lib.slam_demo_get_stats.restype = ct.c_int
        self.cfg = Config(**settings)
        self.handle = self.lib.slam_demo_create(
            ct.byref(self.cfg), int(max_keyframes), 1 if loop_closure else 0,
            float(min_range), float(max_range)
        )
        if not self.handle:
            raise RuntimeError('C SLAM 引擎初始化失败，请检查地图参数')
    def close(self):
        if self.handle:
            self.lib.slam_demo_destroy(self.handle)
            self.handle = None
    def __del__(self):
        self.close()
    def update(self, odom_pose, rays, frame=0):
        a = (Ray * len(rays))(*(Ray(*r) for r in rays))
        out = Pose(*odom_pose)
        if self.lib.slam_demo_step_full_scan(self.handle, Pose(*odom_pose), a, len(a), ct.byref(out)):
            raise RuntimeError('C SLAM 前端/后端更新失败')
        return [float(out.x), float(out.y), float(out.theta)]
    def push_packet(self, odom_pose, rays):
        a = (Ray * len(rays))(*(Ray(*r) for r in rays))
        out = Pose(*odom_pose)
        rc = self.lib.slam_demo_push_packet(self.handle, Pose(*odom_pose), a, len(a), ct.byref(out))
        if rc < 0:
            raise RuntimeError('C SLAM 流式包处理失败')
        return (rc > 0), [float(out.x), float(out.y), float(out.theta)]
    def stats(self):
        st = SlamStats()
        if self.lib.slam_demo_get_stats(self.handle, ct.byref(st)):
            raise RuntimeError('C SLAM 统计读取失败')
        return {k: getattr(st, k) for k, _ in SlamStats._fields_}
    def snapshot(self):
        n = self.cfg.width * self.cfg.height
        a = (Cell * n)()
        if self.lib.slam_demo_copy_map(self.handle, a, n):
            raise RuntimeError('C SLAM 地图读取失败')
        return np.array([(c.log_odds, c.observed, c.last_frame) for c in a],
                        dtype=[('log', 'f4'), ('seen', 'u1'), ('frame', 'u4')]).reshape(self.cfg.height, self.cfg.width)

class Simulation:
    def __init__(self,s,lib):
        from pathlib import Path
        if len(s['waypoints'])<2:raise ValueError('至少需要两个航点')
        if any(math.dist(a,b)<1e-8 for a,b in zip(s['waypoints'],s['waypoints'][1:])):raise ValueError('相邻航点不能重合')
        bad=invalid_segments(s)
        if bad:raise ValueError('轨迹与障碍物或场地边界碰撞，请调整红色路径段')
        self.s=s; self.use_slam=bool(s.get('slam', False))
        if self.use_slam:
            slam_lib = Path(lib).with_name('libslamcore.dll' if Path(lib).suffix == '.dll' else 'libslamcore.so')
            self.grid = SlamGrid(slam_lib, s['map'],
                                 min_range=s['lidar']['min_range'],
                                 max_range=s['lidar']['max_range'],
                                 loop_closure=bool(s.get('slam_loop_closure', True)))
        else:
            self.grid=Grid(lib,s['map'])
        p=s['waypoints'][0];self.real=[*p,0.];self.odom=[*p,0.];self.estimate=[*p,0.];self.target=1;self.time=0.;self.frame=0;self.distance=0.;self.finished=False;self.rng=np.random.default_rng(s['seed']);self.history=[];self.next_scan=1/s['lidar']['frequency'];rays=scan(s,self.real,self.rng);self.frame=1
        if self.use_slam:
            self.estimate = self.grid.update(self.odom, rays, self.frame)
            st = self.grid.stats()
        else:
            self.grid.update(self.estimate,rays,self.frame)
            st = None
        self.history.append(dict(time=0.,frame=1,real=self.real.copy(),odom=self.odom.copy(),estimate=self.estimate.copy(),target=self.target,distance=0.,rays=rays,grid=self.grid.snapshot(),finished=False,linear_velocity=0.,angular_velocity=0.,slam_stats=st))
    def step(self):
        s=self.s;dt=s['dt']; old=self.real.copy(); p=s['waypoints'][self.target];dx=p[0]-old[0];dy=p[1]-old[1];dist=math.hypot(dx,dy);desired=math.atan2(dy,dx);diff=(desired-old[2]+math.pi)%(2*math.pi)-math.pi
        if abs(diff)>1e-5:
            self.real[2]+=(1 if diff>0 else -1)*min(abs(diff),s['angular_speed']*dt)
        elif dist>0:
            move=min(dist,s['linear_speed']*dt);self.real[0]+=move*math.cos(desired);self.real[1]+=move*math.sin(desired);self.distance+=move
            if dist-move<=s['arrival']:
                # Finish the remaining segment continuously rather than snapping to the waypoint.
                if dist-move<1e-7:
                    if self.target+1<len(s['waypoints']):self.target+=1
                    elif s['closed']:self.target=0
                    else:self.finished=True
        dtheta=self.real[2]-old[2];travel=math.hypot(self.real[0]-old[0],self.real[1]-old[1]);odom=s['odom']; self.odom[2]+=dtheta+odom['heading_bias']*dt
        travel=travel*odom['scale']+(self.rng.normal(0,odom['noise'])*math.sqrt(dt) if travel and odom['noise'] else 0)
        self.odom[0]+=travel*math.cos(self.odom[2]);self.odom[1]+=travel*math.sin(self.odom[2]);self.time+=dt
        if not self.use_slam:
            self.estimate = self.odom.copy()
        if self.finished: self.next_scan=min(self.next_scan,self.time)
        if self.time+1e-9>=self.next_scan:
            rays=scan(s,self.real,self.rng);self.frame+=1
            if self.use_slam:
                self.estimate = self.grid.update(self.odom, rays, self.frame)
                st = self.grid.stats()
            else:
                self.grid.update(self.estimate,rays,self.frame)
                st = None
            self.next_scan+=1/s['lidar']['frequency']; self.history.append(dict(time=self.time,frame=self.frame,real=self.real.copy(),odom=self.odom.copy(),estimate=self.estimate.copy(),target=self.target,distance=self.distance,rays=rays,grid=self.grid.snapshot(),finished=self.finished,linear_velocity=math.hypot(self.real[0]-old[0],self.real[1]-old[1])/dt,angular_velocity=dtheta/dt,slam_stats=st))
        return self.finished
