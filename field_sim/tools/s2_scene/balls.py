import sys;sys.path.insert(0,"C:/Users/Jindal/AppData/Local/Temp/claude/C--Users-Jindal-StudioProjects-MakEMinds-Sigmaz/110bea92-59fd-420e-a5cb-58471f8cde9a")
sys.path.insert(0,"C:/Users/Jindal/AppData/Local/Temp/claude/C--Users-Jindal-StudioProjects-MakEMinds-Sigmaz/110bea92-59fd-420e-a5cb-58471f8cde9a/scratchpad/A_S2")
from common import *
import collections
m=json.load(open(MD+'model.json'))
tr=[d for d in m['instances'] if d['category']=='artifact_tray']
trc=[]
for d in tr:
    T=np.array(d['transform']).reshape(4,4);b=wbbox(T,23);trc.append(((b[0]+b[1])/2,d['id']))
flw={}
for d in m['instances']:
    if d['category']=='flower' and d['part']=='p101':
        flw[d['group']]=np.array(d['transform']).reshape(4,4)[:3,3]
print(flw,[ (c.round(1)) for c,_ in trc])
balls=[];typ={'scoring_pollen':'pollen','scoring_nectar_red':'nectar_red','scoring_nectar_blue':'nectar_blue'}
for d in m['instances']:
    if d['category'] not in typ: continue
    n=int(d['part'][1:]);T=np.array(d['transform']).reshape(4,4);b=wbbox(T,n);c=(b[0]+b[1])/2;dia=float(np.mean(b[1]-b[0]))
    x,y,z=c;t=typ[d['category']];loc='other';wh=''
    if t=='pollen':
        if x<0 or x>144: loc,wh='preloaded','robot pre-load stock staged outside the %s side wall (x=%.1f), row along y'%('red/low-x' if x<0 else 'blue/high-x',x)
        else:
            fn=min(flw,key=lambda k:np.hypot(flw[k][0]-x,flw[k][1]-y));dist=np.hypot(flw[fn][0]-x,flw[fn][1]-y)
            if dist<6: loc,wh='field','stack of 4 under/in %s (layer %d from bottom)'%(fn.replace('_',' '),round((z-1.4)/2.89)+1)
            elif y<10 and x<20: loc,wh='field','red garden (audience-left corner), row along x'
            elif y>134 and x>124: loc,wh='field','blue garden (far-right corner), row along x'
            else: loc,wh='field','unclassified'
    else:
        col=t.split('_')[1]
        if z>20: loc,wh='preloaded','%s hive %s cell (upward-facing), hive-mounted (rotates with hive)'%(col,'audience' if y<72 else 'far')
        else:
            k=0 if x<72 else 1
            loc,wh='tray','%s artifact tray %d (%s alliance area, x~%.1f)'%(col,k+1,'red' if x<72 else 'blue',trc[k][0][0]) 
    balls.append(dict(id=d['id'],type=t,position=[round(float(v),4) for v in c],diameter=round(dia,3),location=loc,where=wh,group=d['group']))
cnt=collections.Counter(b['type'] for b in balls)
json.dump(dict(schema=1,units='in',frame='pedro',balls=balls,counts=dict(cnt)),open(MD+'ball_start_positions.json','w'),indent=1)
print(cnt,collections.Counter((b['type'],b['location'],b['where'][:30]) for b in balls))
print(collections.Counter(round(b['diameter'],1) for b in balls if b['type']=='pollen'),collections.Counter(round(b['diameter'],1) for b in balls if b['type']!='pollen'))
