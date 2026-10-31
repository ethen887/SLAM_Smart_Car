import sys, subprocess, shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def build():
    builddir=ROOT/'build'; dll=builddir/('libmapcore.dll' if sys.platform=='win32' else 'libmapcore.so')
    sources=[ROOT/'CMakeLists.txt',ROOT/'core/map_core.c',ROOT/'core/map_core.h']
    if dll.exists() and all(dll.stat().st_mtime>=p.stat().st_mtime for p in sources):return dll
    for tool in ('cmake','ninja','gcc'):
        if not shutil.which(tool):raise RuntimeError(f'找不到 {tool}，请安装并加入 PATH。')
    cache=builddir/'CMakeCache.txt'
    if cache.exists() and 'CMAKE_GENERATOR:INTERNAL=Ninja' not in cache.read_text(errors='ignore'):
        raise RuntimeError('build 目录使用了其他 CMake 生成器。请删除 MapBuild_Demo/build 后重试。')
    for cmd in (['cmake','-S',str(ROOT),'-B',str(builddir),'-G','Ninja','-DCMAKE_C_COMPILER=gcc'],['cmake','--build',str(builddir)]):
        p=subprocess.run(cmd,cwd=ROOT,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        if p.returncode:raise RuntimeError('C 核心构建失败：\n'+p.stdout)
    if not dll.exists():raise RuntimeError('构建完成但找不到地图动态库：'+str(dll))
    return dll
if __name__=='__main__':
    try:
        dll=build()
        from interface import App
        App(dll).run()
    except Exception as e:
        import traceback
        traceback.print_exc()
        try:
            import tkinter as tk
            from tkinter import messagebox
            root=tk.Tk();root.withdraw();messagebox.showerror('启动失败',str(e));root.destroy()
        except Exception:pass
        sys.exit(1)

