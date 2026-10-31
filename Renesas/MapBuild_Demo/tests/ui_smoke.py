from types import SimpleNamespace
from pathlib import Path
from run_demo import build
from interface import App, validate
import json
app=App(build());app.root.update()
def event(ax,x,y):return SimpleNamespace(inaxes=ax,xdata=x,ydata=y)
old=len(app.s['obstacles']); app.setmode('障碍物'); app.press(event(app.axenv,-2.2,-.3));app.release(event(app.axenv,-2.,-.1));assert len(app.s['obstacles'])==old+1
app.setmode('移动');app.press(event(app.axenv,-2.1,-.2));app.motion(event(app.axenv,-2.,-.2));app.release(event(app.axenv,-2.,-.2));assert abs(app.s['obstacles'][-1]['x']+2.1)<1e-5
app.delete();assert len(app.s['obstacles'])==old
old=len(app.s['waypoints']);app.setmode('航点');app.press(event(app.axenv,-2.,1.));assert len(app.s['waypoints'])==old+1
app.setmode('移动');app.press(event(app.axenv,-2.,1.));app.motion(event(app.axenv,-2.1,1.));app.release(event(app.axenv,-2.1,1.));assert abs(app.s['waypoints'][-1][0]+2.1)<1e-5
app.delete();assert len(app.s['waypoints'])==old
p=Path('tests/scene_roundtrip.json');p.write_text(json.dumps(app.s),encoding='utf8');assert validate(json.loads(p.read_text(encoding='utf8')))==app.s
app.press(event(app.axmap,0,0));assert app.monitors
app.start();app.pause();app.forward();app.forward();app.back();assert app.index==1
app.root.update();app.fig.savefig('ui_verified.png');print('UI interaction smoke passed; history:',len(app.sim.history),'monitor:',app.monitors);app.root.destroy()
