import sys;sys.path.insert(0,"C:/Users/Jindal/AppData/Local/Temp/claude/C--Users-Jindal-StudioProjects-MakEMinds-Sigmaz/110bea92-59fd-420e-a5cb-58471f8cde9a/scratchpad/A_S2")
from common import *
import re,collections
mname={m['n']:m['name'] for m in man}
out,_=resolve()
def base(s): return re.sub(r'\s*<\d+>$','',s)
def kind(n):
    s=mname[n].lower()
    for k,t in (('cable tie','cable_tie'),('rib','rib'),('skin','skin'),('churro','churro'),('base tube','tube'),('plug','plug'),
      ('socket head','fastener'),('washer','fastener'),('nut','fastener'),('screw','fastener'),('bolt','fastener'),('rivnut','fastener'),('rivet','fastener'),
      ('damper holder','damper_holder'),('damper','damper'),('pivot bracket','pivot_bracket'),('spacer','spacer'),('bearing','bearing'),('0.196 id','axle'),
      ('april tag','tag')):
        if k in s: return t
    return 'part'
inst=[];dropped=[];stats=collections.Counter()
for r in out:
    i=r['inst'];ids='/'.join(r['ids']);names=[base(x) for x in r['names']]
    if i['type']!='Part':
        dropped.append(dict(id=ids,name=i['name'],reason='assembly node (no geometry)'));continue
    n=key(i);nm=mname[n];top=names[0];full='/'.join(names)
    cat,sub,grp,al,cell=None,None,'field',None,None
    k=kind(n)
    if nm.startswith('Gaffer Tape'):
        cat='tape';al='red' if ', Red' in nm else 'blue';sub=re.search(r'(\d)in Wide',nm).group(1)+'in'
    elif n==20: cat,al='scoring_nectar_red','red'
    elif n==22: cat,al='scoring_nectar_blue','blue'
    elif n==21: cat='scoring_pollen'
    elif n==23: cat='artifact_tray'
    elif 'Flower Assembly' in top:
        cat='flower';grp='flower_'+re.search(r'<(\d+)>',r['names'][0]).group(1);sub='scoring_volume' if n==113 else k
    elif 'Soft Tiles' in top:
        cat='tile';sub=re.search(r'am-2499-(\w+)',nm).group(1).lower()
    elif 'Perimeter' in top:
        if 'Quick Release' in full or n in (43,44,45): cat,sub='perimeter_pin','quick_release_pin'
        elif n==55: cat,sub='hardware','rivet'
        else:
            cat='perimeter_wall';sub={63:'rail',64:'glass',114:'panel_line',84:'strap',85:'strap_clip',53:'corner_hinge',54:'corner_hinge'}.get(n,'part')
    elif 'Quick Release' in top: cat,sub='perimeter_pin','quick_release_pin'
    elif 'Hive' in top:
        al='red' if 'Red Hive' in top else 'blue'
        cat='hive_'+al;grp='hive_'+al
        if len(names)>1 and 'Goal Pivot' in names[1]:
            sub={'axle':'axle','bearing':'bearing','pivot_bracket':'pivot_bracket','damper':'damper','damper_holder':'damper_holder'}.get(k,k)
            cell=None
        else:
            cell='audience' if 'Audience' in names[1] else 'far'
            if k=='tag': cat,sub='apriltag_sticker','tag_audience' if cell=='audience' else 'tag_far'
            else: sub=k
    elif top.startswith('am-5854'):
        grp='hive_frame'
        if n==95: cat,sub='decoration','panel_sticker'
        elif k=='fastener': cat,sub='hardware','fastener'
        else: cat='hive_frame';sub={86:'axle_holder',87:'a_frame_corner',88:'a_frame_top_bar',89:'a_frame_leg',90:'acm_panel',91:'foot_bar',92:'foot',93:'foot',94:'under_tile_bar'}.get(n,'part')
    else: cat='other'
    M=pedroT(r['T'])
    d=dict(id=ids,name=names[-1],path=names,category=cat,subcategory=sub,group=grp,alliance=al,part='p%03d'%n,transform=[round(float(x),6) for x in M.ravel()])
    if cell: d['cell']=cell
    inst.append(d)
# nectar inside hive -> hive group
for d in inst:
    if d['category'] in('scoring_nectar_red','scoring_nectar_blue') and d['transform'][11]>20:
        d['group']='hive_'+d['alliance']
        d['subcategory']='preloaded_in_cell'
assert len(inst)+len(dropped)==1094
manset={m['n'] for m in man}
assert all(int(d['part'][1:]) in manset for d in inst)
st=collections.Counter(d['category'] for d in inst)
parts={}
for d in inst:
    parts.setdefault(d['part'],{'name':mname[int(d['part'][1:])],'mesh':'meshes/%s.npz'%d['part']})
piv=lambda x:[x,72.0,43.95]
groups={}
for al,x,st_,ang in (('red',59.25,'audience_up',-30.0),('blue',84.75,'far_up',30.0)):
    groups['hive_'+al]=dict(pivot=piv(x),axis=[1,0,0],cad_pose_angle_deg=ang,cad_pose_state=st_,stable_angles_deg=dict(audience_up=-30.0,far_up=30.0))
model=dict(schema=1,units='in',frame='pedro',parts=dict(sorted(parts.items())),instances=inst,dropped=dropped,groups=groups,
 stats=dict(occurrences_total=1094,instances=len(inst),dropped=len(dropped),by_category=dict(st),dropped_reasons=dict(collections.Counter(x['reason'] for x in dropped)),by_group=dict(collections.Counter(d['group'] for d in inst))))
json.dump(model,open(MD+'model.json','w'),separators=(',',':'))
print(st,len(dropped),model['stats']['by_group'])
