import sys,math,numpy as np,trimesh
sys.path.insert(0,'/workspace/case-wiper'); import case_wiper as W
R=170.0; RO=173.2; OFF=np.array([7.44889,0,0])
wall=trimesh.load('../work/case_wall_local.stl')
v=wall.vertices; A=np.degrees(np.arctan2(v[:,1],v[:,0])); Z=v[:,2]
def edge(z): s=(abs(Z-z)<1.0)&(A<0); return A[s].min()
SW=4.5; TRAV=2.0; EDGE_MM=80.0; H1=60.0     # cm reading of the tape: 8 cm from edge, 6 cm up
(u1,h1),(u2,h2)=W.INSERTS
z0=H1-h1                                   # wiper underside datum when hole1 at 60
a1=edge(H1)+math.degrees(EDGE_MM/RO); a2=a1+math.degrees((u2-u1)/R)
cut=[]
for a,zc in ((a1,H1),(a2,z0+h2)):
    zlo,zhi=zc-TRAV,zc+TRAV; rm=trimesh.transformations.rotation_matrix(math.pi/2,[0,1,0])
    p=[trimesh.creation.cylinder(SW/2,20,transform=rm) for _ in (0,1)]; p[0].apply_translation([0,0,zlo]); p[1].apply_translation([0,0,zhi])
    b=trimesh.creation.box([20,SW,zhi-zlo]); b.apply_translation([0,0,zc])
    s=trimesh.boolean.union(p+[b],engine='manifold'); s.apply_translation([R+1.6,0,0]); s.apply_transform(trimesh.transformations.rotation_matrix(math.radians(a),[0,0,1])); cut.append(s)
    print('slot angle %.2f deg, centre z %.1f (travel %.1f-%.1f), overall z %.2f-%.2f, outer-arc dist from edge(at z) %.1f mm'%(a,zc,zlo,zhi,zlo-SW/2,zhi+SW/2,math.radians(a-edge(zc))*RO))
new=trimesh.boolean.difference([wall]+cut,engine='manifold')
new.export('../work/case_wall_local - wiper slots v2.stl'); o=new.copy(); o.apply_translation(-OFF); o.export('../Case Feeder Outer Wall PETG 3.2mm - wiper slots v2.stl')
print('watertight',new.is_watertight,o.is_watertight)
trail=a1-math.degrees(u1/R)
print('wiper z0 (lowest underside) %.1f..%.1f, clearance over z15 support %.1f..%.1f; trailing end %.2f deg, %.1f mm (inner arc) from edge at z0'%(z0-TRAV,z0+TRAV,z0-TRAV-15,z0+TRAV-15,trail,math.radians(trail-edge(z0))*R))
print('trailing top corner z %.1f, edge there %.2f vs trailing %.2f'%(z0+W.H_LEAD,edge(z0+W.H_LEAD),trail))
wb=trimesh.load('/workspace/case-wiper/case_wiper_in_bowl.stl'); wb.apply_transform(trimesh.transformations.rotation_matrix(math.radians(trail+math.degrees(W.LENGTH/2/R)),[0,0,1])); wb.apply_translation([0,0,z0-W.z0])
sys.path.insert(0,'../work'); from render import render
c=math.radians(a1); d=[math.cos(c),math.sin(c),0]; near=new.slice_plane([120*d[0],120*d[1],0],d)
render([(near,(.55,.35,.15)),(wb,(.95,.6,.1))],[d[0],d[1],-0.1],[0,0,1],1100,900,'preview_slots_v2_outside.png')
render([(near,(.55,.35,.15)),(wb,(.95,.6,.1))],[-d[0],-d[1],-0.3],[0,0,1],1100,900,'preview_slots_v2_inside.png')
