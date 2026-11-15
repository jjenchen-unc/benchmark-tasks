"""Read-only, vectorized native OBB telemetry; never used to change score/dynamics.

Integration: construct once per reset, then observe(existing_poses, physics_dt)
at every physics substep. Poses map body names to (world_center, rotation_matrix)
with matrix COLUMNS the local axes in world. Native task bodies use geom centers.
At each control tick call interval_report(reset=True); overall_report() is final.
No per-substep trace storage. All thresholds are diagnostics, not pass/fail gates.
"""
import numpy as np

class NativeOverlapMonitor:
    names=('tablet','panel1','panel2')
    pairs=((0,1),(0,2),(1,2))
    def __init__(self,sizes):
        self.half=np.array([sizes[n] for n in self.names],dtype=float)/2
        self.indices=np.array(self.pairs)
        self.pair_names=[self.names[a]+'/'+self.names[b] for a,b in self.pairs]
        self.interval=self._blank();self.overall=self._blank()
        self.peak_snapshots={name:None for name in self.pair_names}
    @staticmethod
    def _blank():
        return dict(samples=0,duration_s=0.,max_mtd=np.zeros(3),over250=np.zeros(3),over1000=np.zeros(3))
    def observe(self,poses,dt):
        centers=np.array([poses[n][0] for n in self.names])
        rotations=np.array([poses[n][1] for n in self.names])
        a,b=self.indices.T;delta=centers[b]-centers[a]
        # Conservative world AABB broad phase skips pairs that cannot overlap.
        extents=np.einsum('nij,nj->ni',np.abs(rotations),self.half)
        active=np.all(np.abs(delta)<=extents[a]+extents[b]+1e-12,axis=1)
        depth=np.zeros(3)
        if active.any():
            aa,bb=a[active],b[active];ra,rb=rotations[aa],rotations[bb]
            axis_a=ra.transpose(0,2,1);axis_b=rb.transpose(0,2,1)
            cross=np.cross(axis_a[:,:,None,:],axis_b[:,None,:,:]).reshape(-1,9,3)
            axes=np.concatenate((axis_a,axis_b,cross),axis=1)
            lengths=np.linalg.norm(axes,axis=2);valid=lengths>1e-10
            axes=axes/np.maximum(lengths,1e-10)[:,:,None]
            radius_a=np.sum(np.abs(np.einsum('pai,pij->paj',axes,ra))*self.half[aa,None,:],axis=2)
            radius_b=np.sum(np.abs(np.einsum('pai,pij->paj',axes,rb))*self.half[bb,None,:],axis=2)
            separation=np.abs(np.einsum('pai,pi->pa',axes,delta[active]))
            overlaps=np.where(valid,radius_a+radius_b-separation,np.inf)
            depth[active]=np.maximum(0.,overlaps.min(axis=1))
        # Retain only the latest cumulative maximum per pair. These copied
        # geometries explain a peak between control ticks without dense storage.
        for i,(a_idx,b_idx) in enumerate(self.pairs):
            if depth[i]>self.overall['max_mtd'][i]:
                self.peak_snapshots[self.pair_names[i]]=dict(
                    physics_sample_index=int(self.overall['samples']),
                    time_s=float(self.overall['duration_s']),
                    sampling_phase='pre_step',index_origin=0,
                    max_mtd_m=float(depth[i]),
                    bodies={self.names[j]:dict(center_m=centers[j].tolist(),
                        rotation_matrix=rotations[j].tolist(),
                        half_extents_m=self.half[j].tolist()) for j in (a_idx,b_idx)})
        for aggregate in (self.interval,self.overall):
            aggregate['samples']+=1;aggregate['duration_s']+=dt
            np.maximum(aggregate['max_mtd'],depth,out=aggregate['max_mtd'])
            aggregate['over250']+=(depth>.00025)*dt
            aggregate['over1000']+=(depth>.001)*dt
    def _report(self,data):
        return dict(diagnostic_only=True,samples=data['samples'],duration_s=data['duration_s'],
            pairs={name:dict(max_mtd_m=float(data['max_mtd'][i]),
                duration_over_0p25mm_s=float(data['over250'][i]),
                duration_over_1mm_s=float(data['over1000'][i])) for i,name in enumerate(self.pair_names)})
    def interval_report(self,reset=True):
        report=self._report(self.interval)
        if reset:self.interval=self._blank()
        return report
    def overall_report(self):
        report=self._report(self.overall)
        report['peak_snapshots']=self.peak_snapshots.copy()
        return report
