"""Candidate robot-only leg grip with a yaw90 bottle; hold during the separate rocker press."""
from dataclasses import dataclass
import math
import numpy as np
from yam_arena import ik
from yam_arena.demo import ArmPlan,Waypoint
from yam_arena.urdf import quat_xyzw_to_matrix


@dataclass
class PoseWaypoint(Waypoint):
    rotation: tuple = (0.,0.,0.,1.)


class PoseArmPlan(ArmPlan):
    def target(self,t):
        if not self.waypoints or not hasattr(self.waypoints[0],'rotation'):
            return super().target(t)
        p,R,g=self.start_pos,self.start_rot,self.finger_start
        for w in self.waypoints:
            goal=np.asarray(w.pos);rotation=quat_xyzw_to_matrix(np.asarray(w.rotation))
            if t<w.duration:
                s=ik.smoothstep(t/w.duration)
                return p+s*(goal-p),ik.slerp_matrix(R,rotation,s),g+s*(w.finger-g),w.label
            t-=w.duration;p,R,g=goal,rotation,w.finger
        return p,R,g,'done'


def plan(top=.7475):
    def wp(seconds,pos,gap,rot,label):
        return PoseWaypoint(seconds,pos,-gap/2,label,rotation=tuple(rot))
    x=.27
    left_rot=(.5,-.5,.5,.5)
    right_fold=(.5,-.5,.5,-.5)
    right_hold=(0.,math.sqrt(.5),0.,math.sqrt(.5))
    left=[
        wp(2.,(.18,.10,.9635),.045,left_rot,'wait clear of holding hand'),
        wp(8.5,(.18,.10,.9635),.045,left_rot,'wait for sustained loop grip'),
        wp(1.,(.18,.10,.9635),.020,left_rot,'set20mm gap for upper-finger press'),
        wp(2.,(.18,.0105,.9635),.020,left_rot,'approach below loop using raised upper finger'),
        wp(2.,(.218,.0105,.9635),.020,left_rot,'press rocker with upper finger, palm kept low'),
        wp(2.,(.218,.0105,.9635),.020,left_rot,'keep pressing while loop held'),
        wp(1.5,(.18,.10,.9635),.045,left_rot,'withdraw pressing hand'),
    ]
    right=[
        wp(2.,(.19,-.12,top+.266),.045,right_fold,'approach outside loop'),
        wp(2.,(.19,0.,top+.266),.045,right_fold,'straddle loop without lateral sweep'),
        wp(1.5,(x-.049,0.,top+.266),.045,right_fold,'advance axially over crown'),
        wp(1.,(x-.049,0.,top+.266),.003,right_fold,'pinch crown for folding'),
    ]
    for degrees in (15,30,50,65,75):
        t=math.radians(degrees)
        right.append(wp(.7,(x-.0465-.0275*math.sin(t)-.0025,0.,
            top+.240+.0275*math.cos(t)-.0015),.003,right_fold,f'lower loop {degrees}'))
    last=right[-1]
    right.append(wp(20.,last.pos,.003,right_fold,'maintain crown grip without rolling'))
    return {'left':left,'right':right}


def patch_scripted(module):
    module.ArmPlan=PoseArmPlan
