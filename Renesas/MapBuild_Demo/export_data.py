import csv, json, math
from datetime import datetime
from pathlib import Path
import numpy as np

def export_experiment(root, settings, history, index, monitors, selected):
    if not selected or not 0<=index<len(history):raise ValueError('导出选项或帧无效')
    root=Path(root)/'exports';root.mkdir(exist_ok=True)
    stem=datetime.now().strftime('%Y%m%d_%H%M%S_%f');out=root/stem;out.mkdir()
    item=history[index];grid=item['grid'];h=history[:index+1]
    # Metadata records frame scope even when parameter export is deselected.
    (out/'导出说明.json').write_text(json.dumps({'frame':item['frame'],'time_s':item['time'],'history_frames':len(h),'selected':sorted(selected),'grid_rows':'y 从小到大','map_png':'图像顶部为较大 y'},ensure_ascii=False,indent=2),encoding='utf8')
    if 'parameters' in selected:(out/'experiment.json').write_text(json.dumps(settings,ensure_ascii=False,indent=2),encoding='utf8')
    if 'map' in selected:
        np.savetxt(out/'grid_log_odds.csv',grid['log'],delimiter=',');np.savetxt(out/'grid_observed.csv',grid['seen'],delimiter=',',fmt='%d');np.savetxt(out/'grid_last_frame.csv',grid['frame'],delimiter=',',fmt='%d')
        from matplotlib import image
        image.imsave(out/'map.png',np.flipud(np.where(grid['seen'],1-1/(1+np.exp(-grid['log'])),.5)),cmap='gray',vmin=0,vmax=1)
    if 'trajectory' in selected:
        with (out/'trajectory.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.writer(f);w.writerow(['time','frame','true_x','true_y','true_theta','estimated_x','estimated_y','estimated_theta']);w.writerows([[z['time'],z['frame'],*z['real'],*z['estimate']] for z in h])
    if 'lidar' in selected:
        with (out/'lidar.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.writer(f);w.writerow(['time','frame','angle_rad','distance_m','hit'])
            for z in h:
                for r in z['rays']:w.writerow([z['time'],z['frame'],*r])
    if 'monitors' in selected:
        with (out/'monitors.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.writer(f);w.writerow(['time','frame','monitor','grid_x','grid_y','observed','log_odds','probability','last_frame'])
            for z in h:
                for i,(x,y) in enumerate(monitors):
                    cell=z['grid'][y,x];w.writerow([z['time'],z['frame'],f'M{i+1}',x,y,int(cell['seen']),float(cell['log']) if cell['seen'] else '',1/(1+math.exp(-cell['log'])) if cell['seen'] else '',int(cell['frame']) if cell['seen'] else ''])
    return out
