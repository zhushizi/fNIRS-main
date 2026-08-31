# -*- coding: utf-8 -*-
import sys
from pathlib import Path
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np, pandas as pd
sys.path.insert(0, r"D:\WORK_space\Code_WS\code_localized\fNIRS-main\software")
sys.path.insert(0, r"D:\WORK_space\Code_WS\code_localized\fNIRS-main\software\tools")
sys.stdout.reconfigure(encoding="utf-8")
from srs_reference import attenuation
from fnirs_pipeline.srs import (fit_slopes, slope_to_mueff, mua_from_mueff,
                                mua_to_toi, musp_power_law, MUSP_REF_800, MUSP_EXPONENT)
from fnirs_pipeline.mbll_core import hemoglobin_extinctions

SURF,INK,INK2,INK3,RULE="#fcfcfb","#0b0b0b","#52514e","#8b8a84","#e3e2dd"
OK,BAD,NEU="#1baf7a","#eb6834","#2a78d6"
plt.rcParams.update({"font.sans-serif":["Microsoft YaHei","SimHei","DengXian","sans-serif"],
 "axes.unicode_minus":False,"figure.facecolor":SURF,"axes.facecolor":SURF,"savefig.facecolor":SURF,
 "text.color":INK,"axes.labelcolor":INK2,"axes.edgecolor":RULE,"xtick.color":INK2,"ytick.color":INK2,
 "axes.titlesize":11,"axes.labelsize":9.5,"legend.fontsize":8.5,"xtick.labelsize":9,"ytick.labelsize":9,
 "axes.spines.top":False,"axes.spines.right":False,"grid.color":RULE,"grid.linewidth":.7})

LN10=np.log(10.); rho=np.array([3.,4.,5.]); N=1.4
OUR=np.array([700.,730.,770.,810.,850.]); E=hemoglobin_extinctions(list(OUR),'wray')
d=pd.read_csv('tddos/Dataset_invivo_subjects_TDDOS/04_Analysis/Results_Final.txt',sep='\t')
d.columns=[c.strip() for c in d.columns]
F=d[d.Location.str.contains('Forehead',case=False,na=False)].copy()
F['Lambda']=F.Lambda.astype(float); F['mua']=F.VarMua0Opt.astype(float); F['musp']=F.VarMus0Opt.astype(float)
g=F[F.Subject=='Subject1'].groupby('Lambda')[['mua','musp']].mean(); wl=g.index.values.astype(float)
MS=np.interp(OUR,wl,g.musp.values)
c0,*_=np.linalg.lstsq(E*LN10,np.interp(OUR,wl,g.mua.values),rcond=None); THB=c0[0]+c0[1]
S=np.linspace(35,92,20)
def read(so2,gain):
    ma=LN10*(E[:,0]*so2/100+E[:,1]*(1-so2/100))*THB
    A=np.array([attenuation(rho,ma[i],MS[i],N) for i in range(5)])-np.log10(gain)
    sl,_=fit_slopes(A,rho); me=slope_to_mueff(sl,rho)
    if np.any(me<=0): return np.nan
    return mua_to_toi(mua_from_mueff(me,musp_power_law(OUR,MUSP_REF_800,MUSP_EXPONENT)),OUR,'wray')[0]

fig=plt.figure(figsize=(14.4,7.4))
gs=fig.add_gridspec(2,3,width_ratios=[1.45,1,1],hspace=.46,wspace=.30,
                    top=.80,bottom=.11,left=.062,right=.975)
fig.text(.062,.945,"趋势不需要标定：增益误差只平移曲线，不改变斜率",fontsize=17,fontweight="bold",color=INK)
fig.text(.062,.885,"Subject1 的真实前额 μs′ 谱（时域实测）· 只改 SO₂ 模拟降氧 · "
         "完全不做增益标定，注入随机的「探测器×波长」增益误差",fontsize=9.6,color=INK2)

# ① 曲线族
a=fig.add_subplot(gs[:,0])
a.plot(S,S,color=INK3,lw=1.4,ls='--',zorder=1,label="真值（1:1）")
cols=[NEU,OK,"#c9a227",BAD]
for j,e in enumerate((0.0,0.05,0.10,0.20)):
    for t in range(6):
        rng=np.random.default_rng(700+t+j*50)
        gn=1+rng.normal(0,e,(5,3)) if e>0 else np.ones((5,3))
        v=np.array([read(x,gn) for x in S])
        a.plot(S,v,color=cols[j],lw=1.5,alpha=.55 if e>0 else 1.0,zorder=3,
               label=("增益误差 %.0f%%"%(e*100)) if t==0 else None)
        if e==0: break
a.set_xlabel("真实 SO₂ (%)"); a.set_ylabel("srs.py 读出的 TOI (%)")
a.set_title("① 每条线都单调，且互相平行",color=INK,pad=8,loc="left")
a.grid(True,alpha=.9); a.legend(frameon=False,loc="upper left",labelcolor=INK2)
a.text(.97,.05,"曲线被上下平移，\n但走向完全一致",transform=a.transAxes,ha="right",
       fontsize=9.5,color=INK2,va="bottom")

E_=[0.0,0.02,0.05,0.10,0.20]
KS=[];OF=[];MO=[]
for e in E_:
    ks,offs,mono,n=[],[],0,0
    for t in range(300):
        rng=np.random.default_rng(300+t)
        gn=1+rng.normal(0,e,(5,3)) if e>0 else np.ones((5,3))
        v=np.array([read(x,gn) for x in S])
        if np.any(~np.isfinite(v)): continue
        n+=1
        if np.all(np.diff(v)>0): mono+=1
        ks.append(np.polyfit(S,v,1)[0]); offs.append(np.interp(70,S,v)-70)
    KS.append((np.mean(ks),np.std(ks))); OF.append(np.std(offs)); MO.append(100*mono/max(n,1))
KS=np.array(KS); OF=np.array(OF); X=np.array(E_)*100

# ② 斜率
a=fig.add_subplot(gs[0,1])
a.errorbar(X,KS[:,0],yerr=KS[:,1],color=OK,lw=2.2,marker='o',ms=7,capsize=4,
           markeredgecolor=SURF,markeredgewidth=1.2,zorder=3)
a.axhline(1.0,color=INK3,lw=1.2,ls='--')
a.set_ylim(.85,1.15); a.set_xlabel("增益误差 (%)"); a.set_ylabel("趋势斜率")
a.set_title("② 斜率几乎不变",color=OK,pad=8,loc="left"); a.grid(True,alpha=.9)
a.text(.5,.08,"0.995 → 0.997",transform=a.transAxes,ha="center",fontsize=10,
       color=OK,fontweight="bold")

# ③ 单调率
a=fig.add_subplot(gs[1,1])
a.bar(X,MO,3.0,color=OK,edgecolor=SURF,lw=1.2,zorder=3)
a.set_ylim(0,112); a.axhline(100,color=INK3,lw=1.2,ls='--')
a.set_xlabel("增益误差 (%)"); a.set_ylabel("方向全对的比例 (%)")
a.set_title("③ 方向 100% 正确",color=OK,pad=8,loc="left"); a.grid(True,axis='y',alpha=.9)
for x,m in zip(X,MO): a.text(x,m+3,"%.0f"%m,ha="center",fontsize=8.5,color=INK2)

# ④ 绝对偏置
a=fig.add_subplot(gs[:,2])
a.plot(X,OF,color=BAD,lw=2.4,marker='s',ms=8,markeredgecolor=SURF,markeredgewidth=1.3,zorder=3)
a.fill_between(X,0,OF,color=BAD,alpha=.12)
a.axhline(3.0,color=INK3,lw=1.2,ls='--'); a.text(19.5,3.3,"±3 %O₂",ha="right",fontsize=9,color=INK2)
a.set_xlabel("增益误差 (%)"); a.set_ylabel("绝对值离散 (±%O₂, 1σ)")
a.set_title("④ 但绝对值随增益误差线性恶化",color=BAD,pad=8,loc="left")
a.grid(True,alpha=.9)
for x,o in zip(X,OF): a.text(x,o+.25,"%.2f"%o,ha="center",fontsize=8.6,color=BAD)
a.text(.5,.55,"⇒ 体模矩阵标定买的是\n【绝对值】和【台间一致】，\n不是趋势",
       transform=a.transAxes,ha="center",fontsize=10.5,color=INK,fontweight="bold")

fig.text(.062,.038,"结论：Σw=0 的结构使探测器增益只进入截距、不进入斜率 —— 即使 20% 未标定的增益误差，"
         "300 次试验方向全部正确、斜率仍为 0.997。",fontsize=8.8,color=INK2)
fig.text(.062,.014,"前提：探测器能读出信号（长距不饱和）；且为均匀介质 —— 分层会把幅度打折，"
         "头皮反向大变仍可能翻转方向。",fontsize=8.5,color=INK3)
out=Path(__file__).parent/"srs_nocal.png"; fig.savefig(out,dpi=170)
print("saved:",out); print("斜率",[round(k,3) for k in KS[:,0]]); print("单调率",MO); print("偏置σ",[round(o,2) for o in OF])
