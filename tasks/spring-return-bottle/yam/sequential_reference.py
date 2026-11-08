"""Matched negative controller: release the spring loop before the rocker press.

Uses the same facing-robot setup and crown approach as the sustained-hold reference.
After releasing the loop, the right hand steadies the round body instead. This
isolates sustained versus sequential release from a simple unsupported-body fall.
"""
import math
from dataclasses import replace
from spring_motion import plan as hold_plan,PoseWaypoint


def plan(top=.7475):
    plans=hold_plan(top)
    right=plans['right'][:-1];last=right[-1]
    rot=(-math.sqrt(.5),0.,0.,math.sqrt(.5))
    def wp(seconds,pos,gap,rotation,label):
        return PoseWaypoint(seconds,pos,-gap/2,label,rotation=rotation)
    right.extend([
        wp(.7,last.pos,.065,last.rotation,'release loop before working rocker'),
        wp(.7,(last.pos[0]-.020,0.,last.pos[2]),.065,last.rotation,'retreat axially from returning loop'),
        wp(.7,(last.pos[0]-.020,0.,top+.18),.065,last.rotation,'clear below returning loop'),
        wp(2.,(.27,-.14,top+.14),.09,rot,'approach body brace'),
        wp(1.5,(.27,-.013,top+.14),.09,rot,'straddle body'),
        wp(1.,(.27,-.013,top+.14),.060,rot,'brace body after releasing loop'),
        wp(10.,(.27,-.013,top+.14),.060,rot,'hold body during left rocker press'),
    ])
    left=plans['left']
    left[1]=replace(left[1],duration=left[1].duration+6.6,
        label='wait for loop release and body brace')
    return {'left':left,'right':right}
