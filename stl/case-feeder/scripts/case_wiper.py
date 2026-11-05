"""Case wiper v7: plain 3.8 mm x 8 deep holes, M4 screws thread directly into plastic (no inserts).
Case wiper v6: leading end narrower: tip depth 6, R2 nose, taper from u=26, underside bevel rises up to +12 mm higher at tip.
Case wiper v5: leading end thinned (depth 16->9 mm at tip, R3.5 nose), taper from u=30.
Case wiper v4: underside slope reversed (high at leading nose, lowest at trailing end).
Case wiper v3 (v2 flipped vertically: flat top, curved+chamfered underside lowest at leading end; print flat top down)
Case wiper v2 for Case Feeder Pro PETG 3.2 mm outer wall, profile copied from a 3D scan of the Dillon wiper.
Frame: bowl axis = Z, z=0 = bottom of printed wall; cases stand at z=SUPPORT_Z. Part modelled in place
centred at angle 0 (+X). Print orientation: flat underside on the bed (45 deg roof, no supports).
Scan (dillon_scan.stl) measured: length 50.6, height 22.6 (trailing) -> 35.5 (leading), depth 16.4,
lower front face vertical, top 15-16 mm a ~45 deg roof back to the wall, leading front corner rounded ~R8.
"""
import cadquery as cq, math
R_IN, SUPPORT_Z = 170.0, 15.0     # measured bowl inner radius / case support surface
CLEARANCE = 24.0                  # support surface -> wiper underside (standing 9mm = 19.15 mm)
LENGTH = 50.6                     # along the wall (arc length at R_IN)
H_TRAIL, H_LEAD = 22.6, 35.5      # heights at trailing / leading end
DEPTH  = 16.0                     # radial depth from wall (scan 16.4 incl. noise)
ROOF   = 15.0                     # vertical extent of the 45 deg roof down from the top edge
NOSE_R = 2.0
TIP_D, TAPER_U = 6.0, 26.0
BEVEL_EXTRA = 12.0
def S(u):
    if u<=TAPER_U: return 0.0
    t=(u-TAPER_U)/(LENGTH-TAPER_U); return t*t*(3-2*t)
def D(u):
    if u<=TAPER_U: return DEPTH
    t=(u-TAPER_U)/(LENGTH-TAPER_U); return DEPTH-(DEPTH-TIP_D)*t*t*(3-2*t)                      # rounding of the leading front vertical edge
HAND   = -1                       # -1: plate turns CLOCKWISE seen from above (user's); +1 CCW
INSERT_D, INSERT_L = 3.8, 8.0     # v7: plain M4 self-tap holes (no inserts), dia / depth
INSERTS = [(10.0, 31.0), (38.0, 28.0)]
HOLE_L = [8.0, 7.0]               # v7 per-hole depth: 38 mm hole limited to 7 by the bevel  # (u from trailing end, height above underside)
N_ST = 27
z0 = SUPPORT_Z + CLEARANCE
LEAD_SIGN = -HAND                 # leading end at +Y (angle +) for clockwise plate

def top(u):  # scan top edge: concave-up curve, fitted quadratic through (0,22.6),(25,26.4),(50.6,35.5)
    t=u/LENGTH; return H_TRAIL + (H_LEAD-H_TRAIL)*(0.30*t + 0.70*t*t)

def pt(u,d,h):
    th = LEAD_SIGN*(u-LENGTH/2)/R_IN; r=R_IN-d
    return cq.Vector(r*math.cos(th), r*math.sin(th), z0+h)

def build():
    wires=[]
    for i in range(N_ST):
        u=LENGTH*i/(N_ST-1); T=top(u)
        prof=[(-3,0),(DEPTH,0),(DEPTH,T-ROOF),(DEPTH-ROOF*DEPTH/ROOF if False else 0,T),(-3,T)]
        b=H_LEAD-top(LENGTH-u)   # v4: slope reversed - underside lowest at TRAILING end, rises toward leading nose
        Du=D(u); prof=[(-3,H_LEAD),(Du,H_LEAD),(Du,b+Du+BEVEL_EXTRA*S(u)),(0,b),(-3,b)]   # 45 deg chamfer underneath
        wires.append(cq.Wire.makePolygon([pt(u,d,h) for d,h in prof],close=True))
    body=cq.Workplane().add(cq.Solid.makeLoft(wires,True))
    body=body.intersect(cq.Workplane("XY").circle(R_IN).extrude(400))
    # round the leading front vertical edge (the nose that meets the cases)
    lead=pt(LENGTH,DEPTH,1)
    # round the leading front vertical edge: cut (square - quarter circle) prism
    R=NOSE_R; import numpy as np
    DT=D(LENGTH); arc=[(LENGTH-R+R*math.sin(a), DT-R+R*math.cos(a)) for a in np.linspace(0,math.pi/2,16)]
    poly=[(LENGTH+2,DEPTH+2)]+arc[::-1]+[(LENGTH+2,DEPTH-R)] if False else arc+[(LENGTH+2,DT-R),(LENGTH+2,DT+2),(LENGTH-R,DT+2)]
    xy=[(pt(u,d,0).x,pt(u,d,0).y) for u,d in poly]
    nose=cq.Workplane("XY").workplane(offset=z0-1).polyline(xy).close().extrude(H_LEAD+5)
    body=body.cut(nose)
    for (u,h),L in zip(INSERTS,HOLE_L):
        p=pt(u,-1,h); th=math.atan2(p.y,p.x)
        hole=cq.Solid.makeCylinder(INSERT_D/2, L+1, cq.Vector(*(pt(u,-1,h).toTuple())),
                                   cq.Vector(-math.cos(th),-math.sin(th),0))
        body=body.cut(cq.Workplane().add(hole))
    return body

if __name__=="__main__":
    b=build()
    cq.exporters.export(b,"case_wiper_in_bowl.stl")
    p=b.rotate((0,0,0),(1,0,0),180)   # print flat top on the bed
    bb=p.val().BoundingBox(); p=p.translate((-bb.xmin,-bb.ymin,-bb.zmin))
    cq.exporters.export(p,"case_wiper.stl"); cq.exporters.export(p,"case_wiper.step")
    print("z0",z0,"bbox",b.val().BoundingBox().xlen,b.val().BoundingBox().ylen,b.val().BoundingBox().zlen)
