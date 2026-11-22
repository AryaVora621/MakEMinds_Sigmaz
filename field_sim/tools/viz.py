import sys,os; sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
import cadlib as c, numpy as np, fitz
# hive-local view of red audience cell, parts: list n
def render(hive, cellkw, ns, axes, out, scale=20, W=800,H=600):
    occ=c.occurrences()
    piv=[o for o in occ if o['names']==['am-5853-%s Hive <1>'%hive,'Goal Pivot Assembly <1>']][0]['T']
    Pinv=np.linalg.inv(piv)
    cols={24:'#888',25:'#a50',26:'#d22',27:'#2a2',28:'#22d',30:'#a2a',29:'#000',83:'#000',57:'#000',79:'#000'}
    paths=[]
    (a,b)=axes
    for o in occ:
        if o['n'] in ns and o['names'][0].startswith('am-5853-%s'%hive) and cellkw in o['names'][1]:
            t=c.xform(Pinv@o['T'],c.load_part(o['n']))
            for tr in t:
                pts=[(W/2+tr[i][a]*scale, H/2-tr[i][b]*scale) for i in range(3)]
                paths.append('<polygon points="%s" fill="%s" fill-opacity="0.35" stroke="%s" stroke-width="0.3"/>'%(' '.join('%.1f,%.1f'%p for p in pts),cols[o['n']],cols[o['n']]))
    svg='<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d"><rect width="100%%" height="100%%" fill="white"/>%s</svg>'%(W,H,''.join(paths))
    d=fitz.open(stream=svg.encode(),filetype='svg'); pdf=fitz.open('pdf',d.convert_to_pdf()); pdf[0].get_pixmap(dpi=96).save(out)
if __name__=='__main__':
    S=sys.argv[1]
    render('Red','Audience',[24,25,26,27,28,30],(0,2),S+'/A_S3/red_aud_xz.png')   # x-z (view along y)
    render('Red','Audience',[24,25,26,27,28,30],(1,2),S+'/A_S3/red_aud_yz.png')   # y-z (view along x)
