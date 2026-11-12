from types import SimpleNamespace
from run_demo import build
from interface import App
app=App(build()); state={}; errors=[]
def check_play():
    try:
        assert app.playing and app.sim.frame>=3,(app.playing,app.sim.frame)
        app.pause();state['old']=app.index
        app.root.after(200,check_pause)
    except Exception as exc:errors.append(exc);app.root.destroy()
def check_pause():
    try:
        assert app.index==state['old'];app.back();assert app.index==state['old']-1;app.start();app.root.after(800,check_resume)
    except Exception as exc:errors.append(exc);app.root.destroy()
def check_resume():
    try:
        assert app.playing and app.index>state['old']-1
        app.pause();app.seek(0);assert app.index==0 and not app.playing
        for i in range(12):app.press(SimpleNamespace(inaxes=app.axmap,xdata=-2+i*.15,ydata=0))
        assert int(app.info_text.index('end-1c').split('.')[0])>20
        print('playback and scrolling passed:',app.sim.frame,'frames;',len(app.monitors),'monitors')
    except Exception as exc:errors.append(exc)
    finally:app.root.destroy()
app.start();app.root.after(1600,check_play);app.root.mainloop()
if errors:raise errors[0]
