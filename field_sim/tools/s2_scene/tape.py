import sys;sys.path.insert(0,"C:/Users/Jindal/AppData/Local/Temp/claude/C--Users-Jindal-StudioProjects-MakEMinds-Sigmaz/110bea92-59fd-420e-a5cb-58471f8cde9a")
sys.path.insert(0,"C:/Users/Jindal/AppData/Local/Temp/claude/C--Users-Jindal-StudioProjects-MakEMinds-Sigmaz/110bea92-59fd-420e-a5cb-58471f8cde9a/scratchpad/A_S2")
from common import *
import re
m=json.load(open(MD+'model.json'))
cfg={x['n']:x['configuration'] for x in man}
tapes=[]
for d in m['instances']:
    if d['category']!='tape': continue
    n=int(d['part'][1:]);c=cfg[n]
    L=float(re.search(r'Length=([\d.eE+-]+)\+meter',c).group(1))/0.0254;W=float(re.search(r'Width=([\d.eE+-]+)\+meter',c).group(1))/0.0254
    T=np.array(d['transform']).reshape(4,4)
    p=lambda x,y,z=0:(T@np.array([x,y,z,1]))[:3]
    a,b=p(-L/2,0.005),p(L/2,0.005);top=max(p(0,0)[2],p(0,0.01)[2])
    ax=T[:3,0];wd=T[:3,2]
    tapes.append(dict(id=d['id'],name=d['name'],part=d['part'],color=d['alliance'],width=round(W,3),length=round(L,3),p0=[round(a[0],3),round(a[1],3)],p1=[round(b[0],3),round(b[1],3)],z=round(float(top),4),_axz=round(float(abs(ax[2])),3),_up=np.round(T[:3,1],2).tolist()))
for t in sorted(tapes,key=lambda t:(t['color'],t['width'],t['p0'])): print(t['color'],t['width'],t['length'],t['p0'],t['p1'],t['z'],t['_up'],t['name'][-28:])
json.dump(tapes,open('tapes_tmp.json','w'))
