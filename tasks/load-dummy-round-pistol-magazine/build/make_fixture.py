import cadquery as cq, math
C=0.3                      # clearance per side
TB=6.0                     # base thickness
H=50.0                     # tower height above base
T=math.tan(math.radians(21.0))
DX=5.25; DZ=TB-1.08        # mag CAD x -> X, mag CAD y -> Z ; Y = mag z - 22.31
PX0,PX1=19.75+DX-C,60.15+DX+C      # floor plate
PW=14.27+C; PT=6.2+0.4
BW=11.25+C; BL=36.67+2*C
def rear(Z): return 21.73+DX-C+(Z-DZ-8.8)*T
Y0,Y1=-PW-10,22.0
Ztop=TB+H
base=cq.Workplane("XY").box(120,80,TB,centered=False).translate((0,-40,0))
tower=cq.Workplane("XY").box(75,Y1-Y0,H,centered=False).translate((15,Y0,TB))
body=base.union(tower)
groove=cq.Workplane("XY").box(PX1-PX0,Y1+PW+1,PT,centered=False).translate((PX0,-PW,TB))
za,zb=TB+PT-0.01,Ztop+1
chan=(cq.Workplane("XZ").polyline([(rear(za),za),(rear(za)+BL,za),(rear(zb)+BL,zb),(rear(zb),zb)]).close()
      .extrude(-(Y1+BW+1)).translate((0,-BW,0)))
pin=cq.Workplane("YZ").circle(2.6).extrude(200).translate((-50,BW+C+2.6,TB+30))
body=body.cut(groove).cut(chan).cut(pin)
for x,y in [(7,-33),(7,33),(113,-33),(113,33)]:
    body=body.cut(cq.Workplane("XY").circle(2.75).extrude(TB+1).translate((x,y,-0.5)))
body=body.edges("|Z").fillet(1.0)
cq.exporters.export(body,"magazine_fixture.step")
cq.exporters.export(body,"magazine_fixture.stl",tolerance=0.02)
bb=body.val().BoundingBox(); print("bbox",round(bb.xlen,1),round(bb.ylen,1),round(bb.zlen,1),"vol cm3",round(body.val().Volume()/1000,1))
