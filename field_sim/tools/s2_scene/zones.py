import sys;sys.path.insert(0,"C:/Users/Jindal/AppData/Local/Temp/claude/C--Users-Jindal-StudioProjects-MakEMinds-Sigmaz/110bea92-59fd-420e-a5cb-58471f8cde9a")
sys.path.insert(0,"C:/Users/Jindal/AppData/Local/Temp/claude/C--Users-Jindal-StudioProjects-MakEMinds-Sigmaz/110bea92-59fd-420e-a5cb-58471f8cde9a/scratchpad/A_S2")
from common import *
T=json.load(open('tapes_tmp.json'))
m=json.load(open(MD+'model.json'))
def rect(t):
    (x0,y0),(x1,y1)=t['p0'],t['p1'];h=t['width']/2
    if abs(y1-y0)<1e-6: return [min(x0,x1),y0-h,max(x0,x1),y0+h]
    return [x0-h,min(y0,y1),x0+h,max(y0,y1)]
def role(t):
    mx=(t['p0'][0]+t['p1'][0])/2;my=(t['p0'][1]+t['p1'][1])/2
    if mx<1 or mx>143: return 'alliance_area'
    if (my<10 or my>134): return 'garden'
    return 'loading_zone'
tapes=[];grp={}
for t in T:
    r=role(t);rc=rect(t)
    key=(r,t['color']);g=grp.setdefault(key,[1e9,1e9,-1e9,-1e9]);g[0]=min(g[0],rc[0]);g[1]=min(g[1],rc[1]);g[2]=max(g[2],rc[2]);g[3]=max(g[3],rc[3])
    tapes.append(dict(id=t['id'],name=t['name'],part=t['part'],color=t['color'],width=t['width'],length=t['length'],p0=t['p0'],p1=t['p1'],z=t['z'],role=r))
zones=[]
for (r,c),g in sorted(grp.items()):
    x0,y0,x1,y1=[round(v,3) for v in g]
    zones.append(dict(name='%s %s'%(c,r.replace('_',' ')),kind=r,alliance=c,polygon=[[x0,y0],[x1,y0],[x1,y1],[x0,y1]],source='tape',size=[round(x1-x0,2),round(y1-y0,2)],
      note='outer envelope of the alliance-coloured tape strips (1in and 2in variants overlap); loading/alliance polygons extend to the tape ends at the wall'))
# tile seams
tc=[]
for d in m['instances']:
    if d['category']=='tile':
        b=wbbox(np.array(d['transform']).reshape(4,4),int(d['part'][1:]));tc.append((b[0]+b[1])/2)
tc=np.array(tc)
def cl(v):
    v=np.sort(np.round(v,0));return sorted(set(v))
cx=cl(tc[:,0])
cx=sorted({round(float(c),1) for c in np.array([np.mean(tc[np.abs(tc[:,0]-c)<2,0]) for c in cx])})
seams=[(a+b)/2 for a,b in zip(cx,cx[1:])]
print('tile centres',cx,'seams',seams)
for s in seams:
    zones.append(dict(name='tile seam x=%.2f'%s,kind='tile_seam',alliance=None,polygon=[[round(s,3),1.42],[round(s,3),142.58]],source='tiles'))
    zones.append(dict(name='tile seam y=%.2f'%s,kind='tile_seam',alliance=None,polygon=[[1.42,round(s,3)],[142.58,round(s,3)]],source='tiles'))
zones.append(dict(name='field tile area',kind='other',alliance=None,polygon=[[1.42,1.42],[142.58,1.42],[142.58,142.58],[1.42,142.58]],source='tiles'))
zones.append(dict(name='field nominal interior (manual 144x144)',kind='other',alliance=None,polygon=[[0,0],[144,0],[144,144],[0,144]],source='manual'))
json.dump(dict(schema=1,units='in',frame='pedro',tapes=tapes,zones=zones),open(MD+'tape_zones.json','w'),indent=1)
for z in zones:
    if z['source']=='tape':print(z['name'],z['polygon'][0],z['polygon'][2],z['size'])
