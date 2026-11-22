import sys;sys.path.insert(0,"C:/Users/Jindal/AppData/Local/Temp/claude/C--Users-Jindal-StudioProjects-MakEMinds-Sigmaz/110bea92-59fd-420e-a5cb-58471f8cde9a/scratchpad/A_S2")
from resolve import *
import numpy as np
MD='C:/Users/Jindal/StudioProjects/MakEMinds_Sigmaz/field_sim/data/field_model/'
def pedroT(T):
    M=np.array(T,float).reshape(4,4).copy()
    M[:3,3]=M[:3,3]/0.0254+np.array([72,72,0]);return M
def key(i): return pk[(i['documentId'],i['elementId'],i['partId'],i.get('configuration'))]
_mc={}
def mesh(n):
    if n not in _mc:
        import os
        p=MD+'meshes/p%03d.npz'%n
        _mc[n]=np.load(p)['vertices'].astype(float) if os.path.exists(p) else None
    return _mc[n]
def wbbox(M,n):
    v=mesh(n)
    if v is None: return None
    w=v@M[:3,:3].T+M[:3,3];return w.min(0),w.max(0)
