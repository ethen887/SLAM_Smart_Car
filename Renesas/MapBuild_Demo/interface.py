import copy, csv, json, math
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox, filedialog, simpledialog
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib import font_manager
from matplotlib.patches import Rectangle, Circle
from simulation import defaults, Simulation, invalid_segments

class App:
    def __init__(self,lib):
        self.lib=lib; self.s=defaults();self.sim=None;self.index=0;self.playing=False;self.mode='障碍物';self.selected=None;self.drag=None;self.monitors=[];self.visible={};self.updating=False;self.step_credit=0.0
        self.root=tk.Tk();self.root.title('二维概率栅格建图实验平台');self.root.geometry('1550x900')
        fonts={f.name for f in font_manager.fontManager.ttflist};matplotlib.rcParams['font.sans-serif']=[next((x for x in ['Microsoft YaHei','SimHei','Noto Sans CJK SC'] if x in fonts),'DejaVu Sans')];matplotlib.rcParams['axes.unicode_minus']=False
        bar=ttk.Frame(self.root);bar.pack(fill='x',padx=8,pady=5)
        for label,cmd in [('障碍物编辑',lambda:self.setmode('障碍物')),('航点编辑',lambda:self.setmode('航点')),('选择/移动',lambda:self.setmode('移动')),('删除选中',self.delete),('清空障碍物',self.clear),('编辑选中',self.edit),('航点前移',lambda:self.reorder(-1)),('航点后移',lambda:self.reorder(1)),('保存',self.save),('加载',self.load),('参数',self.settings),('导出',self.export),('清空监控',self.clear_monitors)]:ttk.Button(bar,text=label,command=cmd).pack(side='left',padx=2)
        self.closed=tk.BooleanVar(value=self.s['closed']);ttk.Checkbutton(bar,text='闭环',variable=self.closed,command=self.set_closed).pack(side='left')
        controls=ttk.Frame(self.root);controls.pack(fill='x',padx=8)
        for label,cmd in [('开始/继续',self.start),('暂停',self.pause),('单步前进',self.forward),('单步后退',self.back),('重置',self.reset)]:ttk.Button(controls,text=label,command=cmd).pack(side='left',padx=2)
        ttk.Label(controls,text='播放速度').pack(side='left',padx=(12,2));self.speed=tk.StringVar(value='1');ttk.Combobox(controls,textvariable=self.speed,values=['0.5','1','2','4'],width=5,state='readonly').pack(side='left')
        self.modevar=tk.StringVar();ttk.Label(controls,textvariable=self.modevar).pack(side='right');self.setmode('障碍物')
        self.timeline=tk.Scale(self.root,from_=0,to=0,orient='horizontal',label='历史扫描帧',showvalue=True);self.timeline.pack(fill='x',padx=12)
        self.timeline.bind('<ButtonPress-1>', self.timeline_start);self.timeline.bind('<B1-Motion>', self.timeline_change);self.timeline.bind('<ButtonRelease-1>', self.timeline_change);self.timeline.bind('<KeyRelease>', self.timeline_change)
        body=ttk.Frame(self.root);body.pack(fill='both',expand=True,padx=8,pady=4)
        charts=ttk.Frame(body);charts.pack(side='left',fill='both',expand=True)
        self.fig=Figure(figsize=(9,7),dpi=100,facecolor='#f6f6f4');gs=self.fig.add_gridspec(1,2,wspace=.28)
        self.axenv=self.fig.add_subplot(gs[0,0]);self.axmap=self.fig.add_subplot(gs[0,1])
        self.canvas=FigureCanvasTkAgg(self.fig,charts);self.canvas.get_tk_widget().pack(fill='both',expand=True)
        self.canvas.mpl_connect('button_press_event',self.press);self.canvas.mpl_connect('button_release_event',self.release);self.canvas.mpl_connect('motion_notify_event',self.motion)
        side=ttk.Frame(body,width=350);side.pack(side='right',fill='y',padx=(8,2));side.pack_propagate(False)
        ttk.Label(side,text='实验状态与监控点',font=('',11,'bold')).pack(anchor='w',pady=(4,4))
        info=ttk.Frame(side);info.pack(fill='both',expand=True)
        self.info_text=tk.Text(info,width=40,height=15,wrap='word',font=('',10),relief='flat',background='#f6f6f4',state='disabled')
        scrollbar=ttk.Scrollbar(info,orient='vertical',command=self.info_text.yview);self.info_text.configure(yscrollcommand=scrollbar.set);scrollbar.pack(side='right',fill='y');self.info_text.pack(fill='both',expand=True)
        self.plotfig=Figure(figsize=(3.5,3.1),dpi=100,facecolor='#f6f6f4');self.plotfig.subplots_adjust(left=.18,right=.96,bottom=.20,top=.88);self.axplot=self.plotfig.add_subplot(111)
        self.plotcanvas=FigureCanvasTkAgg(self.plotfig,side);self.plotcanvas.get_tk_widget().pack(fill='x',side='bottom');self.plotcanvas.mpl_connect('pick_event',self.pick)
        self.status=tk.StringVar();ttk.Label(self.root,textvariable=self.status,anchor='w').pack(fill='x',padx=10);self.draw();self.root.after(50,self.tick)
    def run(self):self.root.mainloop()
    def setmode(self,m):self.mode=m;self.modevar.set('当前模式：'+m+'　｜　点击地图可添加监控点')
    def invalidate(self):
        if self.sim:self.sim.grid.close()
        self.sim=None;self.playing=False;self.index=0;self.timeline.configure(to=0);self.draw()
    def set_closed(self):self.s['closed']=self.closed.get();self.invalidate()
    def reset(self):self.invalidate();self.status.set('已重置，可重新开始。')
    def start(self):
        if not self.sim:
            try:self.sim=Simulation(copy.deepcopy(self.s),self.lib);self.index=0
            except Exception as e:messagebox.showerror('无法启动',str(e));self.draw();return
        self.playing=True
    def pause(self):self.playing=False;self.step_credit=0.0
    def forward(self):
        self.playing=False
        if not self.sim:self.start();self.playing=False
        if not self.sim:return
        if self.index<len(self.sim.history)-1:self.index+=1
        elif not self.sim.finished:
            count=len(self.sim.history)
            while len(self.sim.history)==count and not self.sim.finished:self.sim.step()
            self.index=len(self.sim.history)-1
        self.sync();self.draw()
    def back(self):self.playing=False;self.index=max(0,self.index-1);self.sync();self.draw()
    def sync(self):
        self.updating=True;self.timeline.configure(to=max(0,len(self.sim.history)-1) if self.sim else 0);self.timeline.set(self.index);self.updating=False
    def timeline_start(self,event):
        if self.sim:self.pause()
    def timeline_change(self,event):
        if self.sim:
            self.pause();self.index=min(int(self.timeline.get()),len(self.sim.history)-1);self.draw()
    def seek(self,v):
        if self.sim:
            self.playing=False;self.index=max(0,min(int(float(v)),len(self.sim.history)-1));self.sync();self.draw()
    def tick(self):
        if self.playing and self.sim:
            try:
                self.step_credit+=float(self.speed.get())*0.05
                # Historical scans are spaced by their recorded timestamps;
                # live motion is integrated in fixed dt increments.
                for _ in range(1000):
                    if self.index<len(self.sim.history)-1:
                        duration=self.sim.history[self.index+1]['time']-self.sim.history[self.index]['time']
                        if self.step_credit+1e-9<duration:break
                        self.step_credit-=duration;self.index+=1
                    elif not self.sim.finished:
                        if self.step_credit+1e-9<self.s['dt']:break
                        self.step_credit-=self.s['dt'];before=len(self.sim.history);self.sim.step()
                        if len(self.sim.history)>before:self.index+=1
                    else:self.playing=False;self.step_credit=0;break
                self.sync();self.draw()
            except Exception as e:self.playing=False;messagebox.showerror('仿真错误',str(e))
        self.root.after(50,self.tick)
    def closest(self,x,y):
        pts=self.s['waypoints']; ds=[math.hypot(x-p[0],y-p[1]) for p in pts]
        if ds and min(ds)<.16:return ('p',ds.index(min(ds)))
        for i in reversed(range(len(self.s['obstacles']))):
            o=self.s['obstacles'][i]
            if o['x']<=x<=o['x']+o['w'] and o['y']<=y<=o['y']+o['h']:return ('o',i)
        return None
    def press(self,e):
        if e.xdata is None or e.ydata is None:return
        if e.inaxes==self.axmap:
            c=self.s['map'];x=int(math.floor((e.xdata-c['origin_x'])/c['resolution']));y=int(math.floor((e.ydata-c['origin_y'])/c['resolution']))
            if 0<=x<c['width'] and 0<=y<c['height'] and (x,y) not in self.monitors:self.monitors.append((x,y));self.visible[(x,y)]=True;self.s['monitors']=[list(p) for p in self.monitors];self.draw()
        elif e.inaxes==self.axenv:
            if self.mode=='障碍物':self.drag=('new',e.xdata,e.ydata)
            elif self.mode=='航点':self.s['waypoints'].append([e.xdata,e.ydata]);self.selected=('p',len(self.s['waypoints'])-1);self.invalidate()
            else:
                self.selected=self.closest(e.xdata,e.ydata)
                if self.selected:self.drag=('move',e.xdata,e.ydata,copy.deepcopy(self.s['waypoints'] if self.selected[0]=='p' else self.s['obstacles']))
                self.draw()
    def motion(self,e):
        if not self.drag or e.inaxes!=self.axenv or e.xdata is None:return
        if self.drag[0]=='move':
            _,x,y,orig=self.drag;kind,i=self.selected;dx=e.xdata-x;dy=e.ydata-y
            if kind=='p':self.s['waypoints'][i]=[orig[i][0]+dx,orig[i][1]+dy]
            else:self.s['obstacles'][i]['x']=orig[i]['x']+dx;self.s['obstacles'][i]['y']=orig[i]['y']+dy
            self.draw()
    def release(self,e):
        if not self.drag:return
        if self.drag[0]=='new' and e.inaxes==self.axenv and e.xdata is not None:
            _,x,y=self.drag;w=abs(e.xdata-x);h=abs(e.ydata-y)
            if w>.02 and h>.02:
                ids=[int(o['id'][1:]) for o in self.s['obstacles'] if o['id'][1:].isdigit()];self.s['obstacles'].append(dict(id='O'+str(max(ids,default=0)+1),x=min(x,e.xdata),y=min(y,e.ydata),w=w,h=h));self.selected=('o',len(self.s['obstacles'])-1)
        self.drag=None;self.invalidate()
    def clear_monitors(self):
        self.monitors=[];self.visible={};self.s['monitors']=[];self.draw()
    def pick(self,e):
        label=e.artist.get_label()
        if label.startswith('M') and label[1:].isdigit():
            i=int(label[1:])-1
            if 0<=i<len(self.monitors):
                p=self.monitors[i];self.visible[p]=not self.visible.get(p,True);self.draw()
    def delete(self):
        if self.selected:
            k,i=self.selected;arr=self.s['waypoints'] if k=='p' else self.s['obstacles']
            if i<len(arr):arr.pop(i)
            self.selected=None;self.invalidate()
    def clear(self):self.s['obstacles']=[];self.selected=None;self.invalidate()
    def reorder(self,d):
        if self.selected and self.selected[0]=='p':
            i=self.selected[1];j=i+d;a=self.s['waypoints']
            if 0<=j<len(a):a[i],a[j]=a[j],a[i];self.selected=('p',j);self.invalidate()
    def edit(self):
        if not self.selected:return
        k,i=self.selected
        if k=='o':
            o=self.s['obstacles'][i];v=simpledialog.askstring('编辑障碍物','输入 x, y, 长, 宽（米）',initialvalue=f"{o['x']}, {o['y']}, {o['w']}, {o['h']}")
            if v:
                try:
                    x,y,w,h=map(float,v.split(','));assert w>0 and h>0;o.update(x=x,y=y,w=w,h=h);self.invalidate()
                except Exception:messagebox.showerror('输入错误','请输入四个数字，长和宽必须大于零。')
    def save(self):
        p=filedialog.asksaveasfilename(initialdir=Path(__file__).parent,defaultextension='.json',filetypes=[('JSON','*.json')])
        if p:
            if not Path(p).resolve().is_relative_to(Path(__file__).parent.resolve()):messagebox.showerror('保存失败','请保存在 MapBuild_Demo 目录内。');return
            Path(p).write_text(json.dumps(self.s,ensure_ascii=False,indent=2),encoding='utf8')
    def load(self):
        p=filedialog.askopenfilename(initialdir=Path(__file__).parent,filetypes=[('JSON','*.json')])
        if p:
            try:
                data=json.loads(Path(p).read_text(encoding='utf8'));self.s=validate(data);self.monitors=[tuple(v) for v in self.s.get('monitors',[])];self.visible={m:True for m in self.monitors};self.closed.set(self.s['closed']);self.selected=None;self.invalidate()
            except Exception as e:messagebox.showerror('加载失败',str(e))
    def settings(self):
        from dialogs import ParametersDialog
        self.pause()
        dialog=ParametersDialog(self.root,self.s,validate)
        self.root.wait_window(dialog)
        if dialog.result is not None:
            self.s=dialog.result;self.closed.set(self.s['closed']);self.monitors=[tuple(p) for p in self.s['monitors']];self.visible={m:True for m in self.monitors};self.invalidate()
    def draw(self):
        a=self.axenv;b=self.axmap;d=self.axplot
        for ax in (a,b,d):ax.clear()
        a.set_title('真实环境 / 场景编辑');a.set(xlim=(-2.5,2.5),ylim=(-2.5,2.5),xlabel='X (m)',ylabel='Y (m)');a.set_aspect('equal');a.grid(color='#dddddd',linewidth=.5)
        a.add_patch(Rectangle((-2.5,-2.5),5,5,fill=False,edgecolor='black',linewidth=2))
        for i,o in enumerate(self.s['obstacles']):
            a.add_patch(Rectangle((o['x'],o['y']),o['w'],o['h'],facecolor='#555555',edgecolor='#e67e22' if self.selected==('o',i) else '#222222',linewidth=2));a.text(o['x'],o['y']+o['h'],o['id'],fontsize=8)
        pts=self.s['waypoints'];pairs=list(zip(pts,pts[1:]));
        if self.s['closed'] and len(pts)>1:pairs.append((pts[-1],pts[0]))
        bad=invalid_segments(self.s)
        for i,(p,q) in enumerate(pairs):a.plot([p[0],q[0]],[p[1],q[1]],color='#c93b31' if i in bad else '#8ea8b7',linestyle='--',linewidth=1.3)
        for i,p in enumerate(pts):a.plot(*p,'o',color='#e67e22' if self.selected==('p',i) else '#236ca3',markersize=5);a.annotate('P'+str(i),p,xytext=(4,4),textcoords='offset points',fontsize=8)
        if bad:self.status.set('轨迹碰撞：红色路径段不可启动。')
        elif not self.playing:self.status.set('拖拽创建矩形；选择/移动模式拖动对象；点击右侧地图添加监控点。')
        cfg=self.s['map'];extent=[cfg['origin_x'],cfg['origin_x']+cfg['width']*cfg['resolution'],cfg['origin_y'],cfg['origin_y']+cfg['height']*cfg['resolution']]
        b.set_title('C 概率栅格地图');b.set(xlabel='X (m)',ylabel='Y (m)');b.set_aspect('equal')
        item=self.sim.history[self.index] if self.sim and self.sim.history else None
        grid=item['grid'] if item else None
        if grid is None:img=np.full((cfg['height'],cfg['width']),.5)
        else:img=np.where(grid['seen'],1-1/(1+np.exp(-grid['log'])),.5)
        b.imshow(img,origin='lower',extent=extent,cmap='gray',vmin=0,vmax=1,interpolation='nearest')
        colors=['#e67e22','#8c44a8','#168c83','#c34444','#718333','#2357aa']
        for i,(x,y) in enumerate(self.monitors):
            wx=cfg['origin_x']+(x+.5)*cfg['resolution'];wy=cfg['origin_y']+(y+.5)*cfg['resolution'];b.plot(wx,wy,'+',color=colors[i%len(colors)],markersize=10,mew=2);b.annotate('M'+str(i+1),(wx,wy),color=colors[i%len(colors)],fontsize=8)
        lines=[]
        if item:
            real=item['real'];est=item['estimate'];h=self.sim.history[:self.index+1]
            a.plot([p['real'][0] for p in h],[p['real'][1] for p in h],color='#287eb3',linewidth=1.4)
            b.plot([p['estimate'][0] for p in h],[p['estimate'][1] for p in h],color='#287eb3',linewidth=1.4)
            for ax,p in ((a,real),(b,est)):
                ax.add_patch(Circle((p[0],p[1]),self.s['robot_radius'],fill=False,color='#16699e',linewidth=1.5));ax.arrow(p[0],p[1],.25*math.cos(p[2]),.25*math.sin(p[2]),color='#16699e',width=.018,head_width=.08,length_includes_head=True)
            for ang,dist,hit in item['rays'][::max(1,len(item['rays'])//60)]:
                a.plot([real[0],real[0]+dist*math.cos(real[2]+ang)],[real[1],real[1]+dist*math.sin(real[2]+ang)],color='#e99d45',alpha=.25,linewidth=.5)
            lines += [f"时间：{item['time']:.2f} s  /  帧：{item['frame']}",f"估计 X / Y：{est[0]:.2f} / {est[1]:.2f} m",f"估计航向：{math.degrees(est[2]):.1f}°",f"速度：{item['linear_velocity']:.2f} m/s  /  {item['angular_velocity']:.2f} rad/s",f"目标：P{item['target']}　里程：{item['distance']:.2f} m",f"已探索：{100*np.mean(grid['seen']):.1f}%",f"有效命中：{sum(r[2] for r in item['rays'])} / {len(item['rays'])}"]
        else:lines+=['尚未开始','点击栅格添加监控点']
        lines+=['','监控点  （点击曲线图例切换）']
        for i,(x,y) in enumerate(self.monitors):
            if grid is not None and 0<=x<cfg['width'] and 0<=y<cfg['height']:
                cell=grid[y,x];val=(f"L={cell['log']:.2f}  P={100/(1+math.exp(-cell['log'])):.1f}%  帧{cell['frame']}" if cell['seen'] else '未知')
            else:val='未知'
            wx=cfg['origin_x']+(x+.5)*cfg['resolution'];wy=cfg['origin_y']+(y+.5)*cfg['resolution'];lines.append(f'M{i+1} ({x},{y})  ({wx:.2f},{wy:.2f}m)');lines.append('   '+val)
            if self.sim:
                hs=self.sim.history[:self.index+1];xx=[z['time'] for z in hs];yy=[100/(1+math.exp(-z['grid'][y,x]['log'])) if z['grid'][y,x]['seen'] else np.nan for z in hs];d.plot(xx,yy,label=f'M{i+1}',color=colors[i%len(colors)],alpha=1 if self.visible.get((x,y),True) else 0)
        pos=self.info_text.yview()[0];self.info_text.configure(state='normal');self.info_text.delete('1.0','end');self.info_text.insert('1.0','\n'.join(lines));self.info_text.configure(state='disabled');self.info_text.yview_moveto(pos)
        d.set_title('占据概率历史');d.set(xlabel='时间 (s)',ylabel='概率 (%)',ylim=(0,100));d.grid(color='#dddddd',linewidth=.6)
        if d.lines:
            legend=d.legend(fontsize=8,loc='best')
            for line in legend.get_lines(): line.set_picker(5);line.set_alpha(1)
        self.canvas.draw_idle();self.plotcanvas.draw_idle()
    def export(self):
        if not self.sim:messagebox.showinfo('导出','请先运行仿真。');return
        from dialogs import ExportDialog
        from export_data import export_experiment
        self.pause();dialog=ExportDialog(self.root,self.sim.history[self.index]);self.root.wait_window(dialog)
        if dialog.result:
            try:
                out=export_experiment(Path(__file__).parent,self.s,self.sim.history,self.index,self.monitors,dialog.result)
                messagebox.showinfo('导出完成',f'已导出到：\n{out}\n\n地图为当前帧，时间序列截至当前帧。')
            except Exception as e:messagebox.showerror('导出失败',str(e))

def validate(s):
    base=defaults()
    for k in base:
        if k not in s:raise ValueError('缺少参数：'+k)
    if not (0<s['dt']<=1 and s['linear_speed']>0 and s['angular_speed']>0 and 0<s['robot_radius']<2.5 and s['arrival']>=0):raise ValueError('运动参数无效')
    l=s['lidar'];m=s['map']
    if not (1<=int(l['rays'])<=5000 and 0<l['frequency']<=1/s['dt'] and 0<=l['min_range']<l['max_range'] and l['noise']>=0):raise ValueError('LiDAR 参数无效')
    if not (1<=m['width']<=1000 and 1<=m['height']<=1000 and m['resolution']>0 and m['min_log']<=0<=m['max_log']):raise ValueError('地图参数无效')
    if any(o['w']<=0 or o['h']<=0 for o in s['obstacles']):raise ValueError('障碍物尺寸必须为正')
    if any(not (0<=p[0]<m['width'] and 0<=p[1]<m['height']) for p in s['monitors']):raise ValueError('监控点超出地图')
    return s
