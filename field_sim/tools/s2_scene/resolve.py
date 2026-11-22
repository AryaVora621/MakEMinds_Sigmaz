import json,sys
R='C:/Users/Jindal/StudioProjects/MakEMinds_Sigmaz/field_sim/data/raw/'
a=json.load(open(R+'assembly_raw.json'));man=json.load(open(R+'part_manifest.json'))
root=a['rootAssembly'];subs={}
for s in a['subAssemblies']: subs[(s['documentId'],s['elementId'],s['fullConfiguration'])]=s
pk={}
for m in man: pk[(m['documentId'],m['elementId'],m['partId'],m['configuration'])]=m['n']
def inst_map(asm): return {i['id']:i for i in asm['instances']}
def resolve():
    out=[];miss=[]
    for o in root['occurrences']:
        asm=root;names=[];ids=[];inst=None
        for iid in o['path']:
            inst=inst_map(asm).get(iid); 
            if inst is None: miss.append(o['path']);break
            names.append(inst['name']);ids.append(iid)
            if inst['type']=='Assembly':
                asm=subs.get((inst['documentId'],inst['elementId'],inst['fullConfiguration']))
                if asm is None: asm={'instances':[]}
        out.append(dict(ids=ids,names=names,inst=inst,T=o['transform']))
    return out,miss
if __name__=='__main__':
    out,miss=resolve();print(len(out),len(miss))
    from collections import Counter
    c=Counter();nokey=[]
    for r in out:
        i=r['inst']
        if i['type']=='Part':
            k=(i['documentId'],i['elementId'],i['partId'],i.get('configuration'))
            if k in pk:c['ok']+=1
            else:c['nokey']+=1;nokey.append((r['names'],k))
        else:c['asm']+=1
    print(c,nokey[:5])
