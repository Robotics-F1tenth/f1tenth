import numpy as np, json
from scipy import ndimage
acc=np.load("/tmp/acc.npy"); g=np.load("/tmp/mapg.npy"); E=np.load("/tmp/ego.npy")
A=np.load("/tmp/arr.npy")   # (t,x,y) control이 실제로 받은 장애물
mi=json.load(open("/tmp/mapinfo.json")); dets=json.load(open("/tmp/dets.json"))
res,ox,oy=mi["res"],mi["ox"],mi["oy"]
k=int(np.ceil(0.25/res)); occ=(g>0)
occ_d=ndimage.binary_dilation(occ,structure=np.ones((2*k+1,2*k+1)))
lbl,n=ndimage.label((acc>=40)&(~occ_d),structure=np.ones((3,3)))
dist_wall=ndimage.distance_transform_edt(~occ,sampling=res)
D=np.array([[d[0],d[1]] for d in dets])
print(f"{'x':>7} {'y':>7} {'크기':>5} {'벽거리':>6} {'hits':>7} {'ego':>5} | {'control수신':>10} {'랜드마크':>8}")
rows=[]
for i in range(1,n+1):
    m=(lbl==i)
    if m.sum()<3: continue
    ys,xs=np.nonzero(m)
    X,Y=xs.mean()*res+ox, ys.mean()*res+oy
    hits=int(acc[m].sum()); ext=max(xs.max()-xs.min()+1,ys.max()-ys.min()+1)*res
    de=float(np.hypot(E[:,0]-X,E[:,1]-Y).min())
    if hits<3000 or de>2.0 or ext>1.3: continue
    dw=float(dist_wall[int(ys.mean()),int(xs.mean())])
    narr=int((np.hypot(A[:,1]-X,A[:,2]-Y)<0.6).sum())
    lm = "O" if (len(D) and np.hypot(D[:,0]-X,D[:,1]-Y).min()<0.6) else "."
    rows.append((hits,X,Y,ext,dw,de,narr,lm))
rows.sort(reverse=True)
for hits,X,Y,ext,dw,de,narr,lm in rows:
    ctl = f"{narr:6d}건" if narr>0 else "     0건 !!"
    print(f"{X:7.2f} {Y:7.2f} {ext:5.2f} {dw:6.2f} {hits:7d} {de:5.2f} | {ctl:>10} {lm:>8}")
print()
print("control수신 = 실차에서 /f1/perception/object/obstacles/arr 에 이 위치 0.6m 내로 실린 횟수")
print("랜드마크    = /f1/perception/static_obstacles 확정 여부")
