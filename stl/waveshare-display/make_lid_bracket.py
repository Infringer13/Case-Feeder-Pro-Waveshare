"""Flush lid v3: glass in outside pocket; two printed rear bars screw into the board standoffs and into tall lid bosses."""
exec(open('make_lid_flush.py').read().split("lid=trimesh.load")[0])
SCREW='M2.5'                       # switch to 'M2'
CLEAR={'M2.5':2.9,'M2':2.4}[SCREW]; PILOT={'M2.5':2.2,'M2':1.7}[SCREW]; HEAD={'M2.5':5.0,'M2':4.0}[SCREW]
STACK=10.0                         # measured glass front -> standoff ends
STANDOFF_END=POCKET_FLOOR+LENS_T-STACK   # =23.92
BAR_T=3.0; BAR_W=7.0
BAR_TOP=STANDOFF_END; BAR_BOT=BAR_TOP-BAR_T
lid=trimesh.load(SRC)
lid=trimesh.boolean.difference([lid]+[bx(x-3.3,y-3.3,27.5,x+3.3,y+3.3,PLATE_IN) for x,y in OLD_STANDOFFS],engine='manifold')
x0,y0,x1,y1=OLD_WIN
lid=trimesh.boolean.union([lid,bx(x0-0.5,y0-0.5,PLATE_IN,x1+0.5,y1+0.5,PLATE_OUT)],engine='manifold')
bosses=[(GLASS_CX+s*BOSS_X_OFF,y) for s in (-1,1) for y in HOLE_Y]
lid=trimesh.boolean.union([lid]+[bx(x-3,y-3,BAR_TOP,x+3,y+3,PLATE_IN+0.1) for x,y in bosses],engine='manifold')
pocket=bx(GLASS_CX-GLASS_W/2-POCKET_CLR,GLASS_BOTTOM_Y-POCKET_CLR,POCKET_FLOOR,GLASS_CX+GLASS_W/2+POCKET_CLR,GLASS_TOP_Y+POCKET_CLR,PLATE_OUT+1)
opening=bx(GLASS_CX-OPEN_W/2,PCB_CY-OPEN_H/2,PLATE_IN-1,GLASS_CX+OPEN_W/2,PCB_CY+OPEN_H/2,PLATE_OUT+1)
holes=[cyl(x,y,BAR_TOP-1,BAR_TOP+8,PILOT) for x,y in bosses]
lid=trimesh.boolean.difference([lid,pocket,opening]+holes,engine='manifold')
lid.export('/workspace/CFBF LCD Lid Waveshare Flush v3.stl')
# bars (exported flat, z from 0)
bars=[]
for y in HOLE_Y:
    b=bx(GLASS_CX-BOSS_X_OFF-3.5,y-BAR_W/2,BAR_BOT,GLASS_CX+BOSS_X_OFF+3.5,y+BAR_W/2,BAR_TOP)
    cut=[cyl(GLASS_CX+s*HOLE_DX/2,y,BAR_BOT-1,BAR_TOP+1,CLEAR) for s in (-1,1)]+[cyl(GLASS_CX+s*BOSS_X_OFF,y,BAR_BOT-1,BAR_TOP+1,CLEAR) for s in (-1,1)]
    bars.append(trimesh.boolean.difference([b]+cut,engine='manifold'))
asm=trimesh.util.concatenate([lid]+bars); asm.export('/workspace/waveshare-lid/assembly.stl')
out=[]
for i,b in enumerate(bars):
    c=b.copy(); c.apply_translation([-c.bounds[0][0],-c.bounds[0][1]+i*12,-c.bounds[0][2]]); out.append(c)
trimesh.util.concatenate(out).export('/workspace/Waveshare Rear Bar x2.stl')
print('watertight',lid.is_watertight,[b.is_watertight for b in bars])
print('standoff end z',round(STANDOFF_END,2),'bar z',BAR_BOT,BAR_TOP,'hole rows y',[round(y,2) for y in HOLE_Y],'glass cx',GLASS_CX)
print('lowest z',round(BAR_BOT,2),'vs lid wall bottom 12.8; speaker housing z 19.17-31.92 y -22.1..24.8')
