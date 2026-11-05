"""Variant 2: glass flush in an outside-face pocket. Units mm, lid coords (+z = outside face)."""
import trimesh, numpy as np
from trimesh.creation import box, cylinder
SRC='/workspace/cfp/stl/controller-case/CFBF LCD Lid.stl'
PLATE_IN, PLATE_OUT = 31.92, 33.92
OLD_WIN=(-67.86,-28.16,-17.0,36.4)
OLD_STANDOFFS=[(-63.4,40.4),(-63.4,-37.6),(-21.4,40.4),(-21.4,-37.6)]
WALL_IN_Y_LOW=-39.41
# Waveshare 2D drawing
GLASS_W, GLASS_H = 50.54, 73.06
PCB_W, PCB_H = 49.90, 69.00
PCB_TOP_INSET = 1.9
HOLE_DX, HOLE_DY, HOLE_FROM_PCB_TOP = 41.0, 60.0, 4.5
LENS_T = 0.70            # flange (cover glass) thickness = pocket depth
TP_LCD_T = 3.60
PCB_T = 1.6              # ASSUMED
LCD_MOD_W, LCD_MOD_H = 50.0, 69.2   # ASSUMED typical 2.8in module outline (not on drawing); centred on PCB
GLASS_PROUD = 0.0        # >0 lets the glass stand proud
POCKET_CLR = 0.25        # per side
OPEN_CLR = 0.2           # per side (=0.4 total)
# placement (same as draft 1: x on USB slot, glass edge 0.5 from -y wall)
GLASS_CX=-42.43; GLASS_BOTTOM_Y=WALL_IN_Y_LOW+0.5; GLASS_TOP_Y=GLASS_BOTTOM_Y+GLASS_H
PCB_TOP=GLASS_TOP_Y-PCB_TOP_INSET; PCB_CY=PCB_TOP-PCB_H/2
OPEN_W=max(PCB_W,LCD_MOD_W)+2*OPEN_CLR; OPEN_H=max(PCB_H,LCD_MOD_H)+2*OPEN_CLR
POCKET_FLOOR=PLATE_OUT-LENS_T+GLASS_PROUD
PCB_BACK=POCKET_FLOOR-TP_LCD_T-PCB_T
BOSS=6.0; BOSS_HOLE=2.8; BOSS_H=PLATE_IN-PCB_BACK
BOSS_X_OFF=29.77
HOLE_Y=[PCB_TOP-HOLE_FROM_PCB_TOP, PCB_TOP-HOLE_FROM_PCB_TOP-HOLE_DY]
def bx(x0,y0,z0,x1,y1,z1):
    b=box([x1-x0,y1-y0,z1-z0]); b.apply_translation([(x0+x1)/2,(y0+y1)/2,(z0+z1)/2]); return b
def cyl(x,y,z0,z1,d):
    c=cylinder(radius=d/2,height=z1-z0,sections=48); c.apply_translation([x,y,(z0+z1)/2]); return c
lid=trimesh.load(SRC)
lid=trimesh.boolean.difference([lid]+[bx(x-3.3,y-3.3,27.5,x+3.3,y+3.3,PLATE_IN) for x,y in OLD_STANDOFFS],engine='manifold')
x0,y0,x1,y1=OLD_WIN
lid=trimesh.boolean.union([lid,bx(x0-0.5,y0-0.5,PLATE_IN,x1+0.5,y1+0.5,PLATE_OUT)],engine='manifold')
bosses=[(GLASS_CX+s*BOSS_X_OFF,y) for s in (-1,1) for y in HOLE_Y]
lid=trimesh.boolean.union([lid]+[bx(x-3,y-3,PLATE_IN-BOSS_H,x+3,y+3,PLATE_IN+0.1) for x,y in bosses],engine='manifold')
pocket=bx(GLASS_CX-GLASS_W/2-POCKET_CLR,GLASS_BOTTOM_Y-POCKET_CLR,POCKET_FLOOR,GLASS_CX+GLASS_W/2+POCKET_CLR,GLASS_TOP_Y+POCKET_CLR,PLATE_OUT+1)
opening=bx(GLASS_CX-OPEN_W/2,PCB_CY-OPEN_H/2,PLATE_IN-1,GLASS_CX+OPEN_W/2,PCB_CY+OPEN_H/2,PLATE_OUT+1)
holes=[cyl(x,y,PLATE_IN-BOSS_H-1,PLATE_IN-0.6,BOSS_HOLE) for x,y in bosses]
lid=trimesh.boolean.difference([lid,pocket,opening]+holes,engine='manifold')
lid.export('/workspace/CFBF LCD Lid Waveshare Flush.stl')
gl,gr=GLASS_CX-GLASS_W/2,GLASS_CX+GLASS_W/2; ol,orr=GLASS_CX-OPEN_W/2,GLASS_CX+OPEN_W/2
ob,ot=PCB_CY-OPEN_H/2,PCB_CY+OPEN_H/2
print('watertight',lid.is_watertight,'bodies',len(lid.split()))
print('pocket',round(2*POCKET_CLR+GLASS_W,2),'x',round(2*POCKET_CLR+GLASS_H,2),'depth',LENS_T-GLASS_PROUD,'floor z',round(POCKET_FLOOR,2),'floor thickness',round(POCKET_FLOOR-PLATE_IN,2))
print('opening',round(OPEN_W,2),'x',round(OPEN_H,2),'x',round(ol,2),round(orr,2),'y',round(ob,2),round(ot,2))
print('glass ledge (glass edge to opening edge): left',round(ol-gl,2),'right',round(gr-orr,2),'top',round(GLASS_TOP_Y-ot,2),'bottom(USB)',round(ob-GLASS_BOTTOM_Y,2))
print('PCB back z',round(PCB_BACK,2),'boss height',round(BOSS_H,2),'bosses',[(round(a,2),round(b,2)) for a,b in bosses])
