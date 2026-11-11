"""Task-local measured contact reader adapted from verified B5. Pins runtime unverified."""
import numpy as np
from kit import arr
class Contacts:
    def __init__(self,simulation_view,sensors,filters):
        if not sensors or not filters:raise ValueError('empty contact body selection')
        from contact_processing import enable
        self.processing=enable()
        self.capacity=8192
        self.view=simulation_view.create_rigid_contact_view(sensors,filter_patterns=[filters]*len(sensors),max_contact_data_count=self.capacity)
        if self.view.sensor_count<1 or self.view.filter_count<1:raise ValueError('unresolved contact sensors/filters')
        self.selection={'sensors':sensors,'filters':filters,'sensor_count':self.view.sensor_count,'filter_count':self.view.filter_count}
    def read(self,dt):
        from contact_processing import require_enabled
        require_enabled()
        force,point,normal,separation,count,start=self.view.get_contact_data(dt)
        count=arr(count).astype(int);start=arr(start).astype(int)
        if np.any(count<0) or int(count.sum())>=self.capacity:raise ValueError('contact buffer capacity/measurement invalid')
        f=arr(force).reshape(-1);d=arr(separation).reshape(-1);indices=[];pairs=[]
        for pair,(n,i) in enumerate(zip(count.ravel(),start.ravel())):
            if n:
                if i<0 or i+n>len(d):raise ValueError('contact indices invalid')
                indices.extend(range(i,i+n));pairs.extend([pair]*n)
        if not indices:self.last_contacts=[];return False,0.,0
        ids=np.array(indices,int)
        if not np.isfinite(np.r_[f[ids],d[ids]]).all():raise ValueError('nonfinite contact data')
        # Count active solver support even when separation is rounded slightly
        # positive.43237 measured38.6N at+0.094micrometres with old hit=False.
        hit=(d[ids]<=0.)|(abs(f[ids])>0.)
        pts=arr(point).reshape(-1,3);normals=arr(normal).reshape(-1,3)
        if not np.isfinite(np.r_[pts[ids].ravel(),normals[ids].ravel()]).all():raise ValueError('nonfinite contact vectors')
        self.last_contacts=[{'sensor_index':int(pair//count.shape[-1]),'filter_index':int(pair%count.shape[-1]),'separation_m':float(d[i]),'normal_force_n':float(f[i]),'point_w':pts[i].tolist(),'normal_w':normals[i].tolist()} for pair,i in zip(pairs,ids)]
        return bool(np.any(hit)),float(np.max(abs(f[ids][hit]))) if np.any(hit) else 0.,len(indices)

