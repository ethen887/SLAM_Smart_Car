import copy, math
import tkinter as tk
from tkinter import ttk, messagebox

# (label, nested path, type, explanatory text)
FIELDS = {
 '运动': [('线速度 (m/s)',('linear_speed',),float,''),('角速度 (rad/s)',('angular_speed',),float,''),('小车半径 (m)',('robot_radius',),float,''),('仿真步长 (s)',('dt',),float,''),('航点到达阈值 (m)',('arrival',),float,'当前版本仍连续移动到航点中心'),('随机种子',('seed',),int,'固定种子可重复实验')],
 'LiDAR': [('最小距离 (m)',('lidar','min_range'),float,''),('最大距离 (m)',('lidar','max_range'),float,''),('每帧射线数',('lidar','rays'),int,''),('扫描频率 (Hz)',('lidar','frequency'),float,''),('距离噪声标准差 (m)',('lidar','noise'),float,'0 表示无噪声')],
 '里程计': [('距离比例',('odom','scale'),float,'例如 0.85 表示增量低估 15%'),('航向偏差 (rad/s)',('odom','heading_bias'),float,''),('距离噪声',('odom','noise'),float,'0 表示无噪声')],
 '地图': [('宽度 (格)',('map','width'),int,''),('高度 (格)',('map','height'),int,''),('分辨率 (m/格)',('map','resolution'),float,''),('原点 X (m)',('map','origin_x'),float,''),('原点 Y (m)',('map','origin_y'),float,''),('空闲证据',('map','free_delta'),float,''),('占据证据',('map','occupied_delta'),float,''),('Log-Odds 最小值',('map','min_log'),float,''),('Log-Odds 最大值',('map','max_log'),float,'')],
}

def get_path(data,path):
    for key in path:data=data[key]
    return data

def parameters_from_values(settings, values, validate):
    result=copy.deepcopy(settings)
    for fields in FIELDS.values():
        for label,path,kind,_ in fields:
            try:value=kind(values[path])
            except (ValueError,TypeError):raise ValueError(f'{label}：请输入有效的{"整数" if kind is int else "数字"}。')
            if not math.isfinite(value):raise ValueError(f'{label}：请输入有限数字。')
            node=result
            for key in path[:-1]:node=node[key]
            node[path[-1]]=value
    if result['odom']['scale']<0 or result['odom']['noise']<0 or result['seed']<0:raise ValueError('里程计比例、噪声和随机种子不能为负。')
    if result['map']['free_delta']>=0 or result['map']['occupied_delta']<=0:raise ValueError('空闲证据须小于 0，占据证据须大于 0。')
    geometry=('width','height','resolution','origin_x','origin_y')
    changed=any(result['map'][key]!=settings['map'][key] for key in geometry)
    if changed:result['monitors']=[]
    validate(result)
    return result,changed

class ParametersDialog(tk.Toplevel):
    def __init__(self,parent,settings,validator):
        super().__init__(parent);self.title('实验参数');self.transient(parent);self.resizable(False,False);self.result=None;self.settings=settings;self.validator=validator;self.variables={}
        ttk.Label(self,text='修改参数后将重置当前仿真；场景和航点保留。',padding=(12,10)).pack(anchor='w')
        notebook=ttk.Notebook(self);notebook.pack(fill='both',expand=True,padx=12)
        for group,fields in FIELDS.items():
            page=ttk.Frame(notebook,padding=12);notebook.add(page,text=group)
            for row,(label,path,_,hint) in enumerate(fields):
                ttk.Label(page,text=label).grid(row=row,column=0,sticky='w',pady=4,padx=(0,12));v=tk.StringVar(value=str(get_path(settings,path)));self.variables[path]=v
                ttk.Entry(page,textvariable=v,width=15).grid(row=row,column=1,sticky='w');ttk.Label(page,text=hint,foreground='#666').grid(row=row,column=2,sticky='w',padx=10)
        ttk.Label(self,text='修改地图尺寸、分辨率或原点会清空监控点。',foreground='#666',padding=(12,8)).pack(anchor='w')
        buttons=ttk.Frame(self,padding=(12,2,12,12));buttons.pack(fill='x');ttk.Button(buttons,text='取消',command=self.destroy).pack(side='right');ttk.Button(buttons,text='应用并重置',command=self.apply).pack(side='right',padx=8)
        self.protocol('WM_DELETE_WINDOW',self.destroy);self.grab_set()
    def apply(self):
        try:
            result,_=parameters_from_values(self.settings,{p:v.get() for p,v in self.variables.items()},self.validator)
            self.result=result;self.destroy()
        except Exception as exc:messagebox.showerror('参数错误',str(exc),parent=self)

class ExportDialog(tk.Toplevel):
    def __init__(self,parent,item):
        super().__init__(parent);self.title('导出实验数据');self.transient(parent);self.resizable(False,False);self.result=None
        ttk.Label(self,text=f"当前显示：第 {item['frame']} 帧，{item['time']:.2f} 秒",padding=(16,12,16,2)).pack(anchor='w')
        ttk.Label(self,text='地图导出当前帧；轨迹和扫描导出至当前帧。\n每次在本工程 exports 下新建文件夹，不覆盖旧结果。',padding=(16,4)).pack(anchor='w')
        self.options={}
        for key,label in [('map','地图 PNG 与栅格数据 CSV'),('trajectory','真实与估计轨迹 CSV'),('lidar','LiDAR 扫描 CSV'),('monitors','监控点概率历史 CSV'),('parameters','实验参数 JSON')]:
            var=tk.BooleanVar(value=True);self.options[key]=var;ttk.Checkbutton(self,text=label,variable=var).pack(anchor='w',padx=16,pady=2)
        buttons=ttk.Frame(self,padding=12);buttons.pack(fill='x');ttk.Button(buttons,text='取消',command=self.destroy).pack(side='right');ttk.Button(buttons,text='导出所选内容',command=self.apply).pack(side='right',padx=8);self.grab_set()
    def apply(self):
        selected={k for k,v in self.options.items() if v.get()}
        if not selected:messagebox.showerror('请选择','请至少选择一项导出内容。',parent=self);return
        self.result=selected;self.destroy()
