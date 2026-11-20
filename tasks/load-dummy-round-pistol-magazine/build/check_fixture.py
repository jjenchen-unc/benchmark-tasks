import trimesh,numpy as np,sys
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
d=sys.argv[1]
fx=trimesh.load("magazine_fixture.stl")
parts=[]
for n in ["Glock_mag-1.stl","Glock_mag_plate-1.stl"]:
    m=trimesh.load(d+n); m.apply_scale(25.4)
    v=m.vertices.copy(); m.vertices=np.c_[v[:,0]+5.25, v[:,2]-22.31, v[:,1]-1.08+6.0]; parts.append(m)
mag=trimesh.util.concatenate(parts)
print("fixture watertight",fx.is_watertight)
pts=mag.vertices[mag.vertices[:,2]<56]
ins=fx.contains(pts); print("magazine vertices inside fixture:",int(ins.sum()),"of",len(pts))
_,dist,_=trimesh.proximity.closest_point(fx,pts[::3]); print("clearance min/median mm",dist.min().round(3),np.median(dist).round(3))
fig,ax=plt.subplots(1,3,figsize=(18,6))
def sec(m,o,n,a,b,ax,c):
    s=m.section(plane_origin=o,plane_normal=n)
    if s is None: return
    for e in s.entities:
        p=s.vertices[e.points]; ax.plot(p[:,a],p[:,b],c,lw=1)
for (o,n,a,b,t,k) in [([0,0,0],[0,1,0],0,2,"side section, centre plane (X forward, Z up)",0),([0,0,30],[0,0,1],0,1,"top section at Z = 30 mm (pin plane is Z = 36)",1),([45,0,0],[1,0,0],1,2,"front section at X = 45 mm (Y sideways, Z up)",2)]:
    sec(fx,o,n,a,b,ax[k],"k"); sec(mag,o,n,a,b,ax[k],"r"); ax[k].set_aspect("equal"); ax[k].grid(alpha=.3); ax[k].set_title(t)
fig.suptitle("Magazine fixture v1 (black) with community-CAD magazine (red), mm"); fig.savefig("fixture_check.png",dpi=110)
