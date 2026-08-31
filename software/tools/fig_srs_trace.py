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
OK,BAD,NEU,PUR="#1baf7a","#eb6834","#2a78d6","#8e5bc9"
plt.rcParams.update({"font.sans-serif":["Microsoft YaHei","SimHei","DengXian","sans-serif"],
 "axes.unicode_minus":False,"figure.facecolor":SURF,"axes.facecolor":SURF,"savefig.facecolor":SURF,
 "text.color":INK,"axes.labelcolor":INK2,"axes.edgecolor":RULE,"xtick.color":INK2,"ytick.color":INK2,
 "axes.titlesize":11,"axes.labelsize":9.5,"legend.fontsize":9,"xtick.labelsize":9,"ytick.labelsize":9,
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

# ---- 构造一段监护场景：基线 → 缓慢下降 → 平台 → 恢复 ----
fs=1.0; T=600
t=np.arange(0,T,1/fs)
true=np.full_like(t,70.0)
true+= -13*np.clip((t-120)/90,0,1)                      # 120~210s 降到 57
true+=  13*np.clip((t-330)/120,0,1)                     # 330~450s 恢复
true+= 0.6*np.sin(2*np.pi*t/45)+0.3*np.sin(2*np.pi*t/13)   # 生理波动

rng=np.random.default_rng(7)
GAIN=1+rng.normal(0,0.05,(5,3))                          # 未标定，5% 增益误差
SLOPE_NOISE=0.002                                        # OD/cm
def read(so2,gain,noise_rng,f_scalp=0.0,scalp=70.0):
    ma=LN10*(E[:,0]*(f_scalp*scalp+(1-f_scalp)*so2)/100
             +E[:,1]*(1-(f_scalp*scalp+(1-f_scalp)*so2)/100))*THB
    A=np.array([attenuation(rho,ma[i],MS[i],N) for i in range(5)])-np.log10(gain)
    sl,_=fit_slopes(A,rho); sl=sl+noise_rng.normal(0,SLOPE_NOISE,5)
    me=slope_to_mueff(sl,rho)
    if np.any(me<=0): return np.nan
    return mua_to_toi(mua_from_mueff(me,musp_power_law(OUR,MUSP_REF_800,MUSP_EXPONENT)),OUR,'wray')[0]

raw=np.array([read(x,GAIN,rng) for x in true])
lay=np.array([read(x,GAIN,rng,f_scalp=0.25) for x in true])
k,b=np.polyfit(raw[t<100],true[t<100],1) if False else (1.0,0.0)
# 两参数标定：用真值在若干点上拟合（模拟降氧标定）
idx=np.linspace(0,len(t)-1,8).astype(int)
kk,bb=np.polyfit(raw[idx],true[idx],1)
cal=kk*raw+bb

fig=plt.figure(figsize=(14.4,8.4))
gs=fig.add_gridspec(2,3,height_ratios=[1.55,1],hspace=.50,wspace=.28,
                    top=.815,bottom=.105,left=.062,right=.976)
fig.text(.062,.945,"血氧监护曲线：一次降氧事件",fontsize=17,fontweight="bold",color=INK)
fig.text(.062,.888,"Subject1 真实前额 μs′ 谱（时域实测）· 1 Hz · 未标定增益误差 5% · 斜率噪声 0.002 OD/cm · "
         "基线 70% → 降至 57% → 恢复",fontsize=9.6,color=INK2)

a=fig.add_subplot(gs[0,:])
a.axvspan(120,210,color=BAD,alpha=.055); a.axvspan(210,330,color=BAD,alpha=.10)
a.axvspan(330,450,color=OK,alpha=.055)
a.plot(t,true,color=INK,lw=2.6,zorder=5,label="真值 SO₂")
a.plot(t,raw,color=BAD,lw=1.3,alpha=.85,zorder=3,label="srs.py 读数（未标定）")
a.plot(t,cal,color=OK,lw=1.3,alpha=.9,zorder=4,label="两参数标定后")
a.plot(t,lay,color=PUR,lw=1.3,alpha=.8,ls=(0,(4,2)),zorder=2,label="叠加分层（头皮占比 25%）")
a.set_xlim(0,T); a.set_ylim(50,80)
a.set_xlabel("时间 (s)"); a.set_ylabel("SO₂ / TOI (%)")
a.set_title("实际曲线", color=INK, pad=8, loc="left")
a.grid(True,alpha=.9); a.legend(frameon=False,loc="lower left",ncol=4,labelcolor=INK2)
for x,lab in ((165,"下降"),(270,"平台"),(390,"恢复")):
    a.text(x,78.3,lab,ha="center",fontsize=9.5,color=INK2)
a.annotate("",xy=(270,true[270]),xytext=(270,70),arrowprops=dict(arrowstyle="<->",color=INK3,lw=1.3))
a.text(276,63.5,"真降 %.1f 点"%(70-true[270]),fontsize=9.5,color=INK2)

a=fig.add_subplot(gs[1,0])
for v,c,lab in ((raw,BAD,"未标定"),(cal,OK,"标定后"),(lay,PUR,"含分层")):
    a.hist(v-true,bins=np.linspace(-2.5,5,46),color=c,alpha=.6,edgecolor=SURF,lw=.5,
           label="%s  %+.2f±%.2f"%(lab,np.mean(v-true),np.std(v-true)))
a.axvline(0,color=INK,lw=1.4,ls='--')
a.set_xlabel("读数 − 真值 (%O₂)"); a.set_ylabel("样本数")
a.set_title("① 偏差分布",color=INK,pad=8,loc="left")
a.grid(True,axis='y',alpha=.9); a.legend(frameon=False,labelcolor=INK2,fontsize=8.3)

a=fig.add_subplot(gs[1,1])
base=slice(0,100); plat=slice(240,320)
names,vals=[],[]
for v,c,lab in ((true,INK,"真值"),(raw,BAD,"未标定"),(cal,OK,"标定后"),(lay,PUR,"含分层")):
    names.append(lab); vals.append(np.mean(v[base])-np.mean(v[plat]))
x=np.arange(4)
a.bar(x,vals,.6,color=[INK,BAD,OK,PUR],edgecolor=SURF,lw=1.3,zorder=3)
for i,v in enumerate(vals): a.text(i,v+.25,"%.1f"%v,ha="center",fontsize=10,color=INK,fontweight="bold")
a.axhline(vals[0],color=INK3,lw=1.2,ls='--')
a.set_xticks(x); a.set_xticklabels(names,fontsize=9)
a.set_ylabel("检出的下降幅度 (点)"); a.set_ylim(0,max(vals)*1.28)
a.set_title("② 幅度检出",color=INK,pad=8,loc="left"); a.grid(True,axis='y',alpha=.9)
a.text(.5,-.235,"分层使幅度打折至 %.0f%%"%(100*vals[3]/vals[0]),transform=a.transAxes,
       ha="center",fontsize=9.4,color=PUR,fontweight="bold")

a=fig.add_subplot(gs[1,2]); a.axis("off")
a.text(0,.95,"③ 读数",fontsize=11,color=INK,fontweight="bold",transform=a.transAxes)
rows=[("真值：基线 → 平台","%.1f → %.1f"%(np.mean(true[base]),np.mean(true[plat])),INK),
      ("未标定读数","%.1f → %.1f"%(np.mean(raw[base]),np.mean(raw[plat])),BAD),
      ("标定后读数","%.1f → %.1f"%(np.mean(cal[base]),np.mean(cal[plat])),OK),
      ("含分层读数","%.1f → %.1f"%(np.mean(lay[base]),np.mean(lay[plat])),PUR)]
for i,(k_,v_,c_) in enumerate(rows):
    y=.80-i*.115
    a.text(.02,y,k_,fontsize=9.6,color=INK,transform=a.transAxes,va="center")
    a.text(.98,y,v_,fontsize=10.5,color=c_,fontweight="bold",ha="right",transform=a.transAxes,va="center")
a.add_patch(plt.Rectangle((0,.02),1.0,.30,transform=a.transAxes,
            facecolor="#f2f8f5",edgecolor=OK,lw=1.3,zorder=0))
a.text(.03,.245,"未标定：绝对值偏 %+.1f，但降幅只差 %.1f 点"%(np.mean(raw[base])-np.mean(true[base]),
       abs(vals[1]-vals[0])),fontsize=9.2,color=INK,transform=a.transAxes,va="center")
a.text(.03,.155,"标定后：绝对值和降幅都对上",fontsize=9.2,color=OK,fontweight="bold",
       transform=a.transAxes,va="center")
a.text(.03,.07,"含分层：方向对，幅度打折至 %.0f%%"%(100*vals[3]/vals[0]),fontsize=9.2,color=PUR,
       transform=a.transAxes,va="center")

fig.text(.062,.018,"曲线为仿真：μs′ 谱与 tHb 取自真实前额实测，SO₂ 变化过程为构造。含 1 Hz 采样、"
         "斜率噪声 0.002 OD/cm、未标定的 5% 增益误差。",fontsize=8.6,color=INK3)
out=Path(__file__).parent/"srs_trace.png"; fig.savefig(out,dpi=170)
print("saved:",out)
print("真降 %.2f  未标定 %.2f  标定后 %.2f  含分层 %.2f"%tuple(vals))
print("偏差 未标定 %+.2f±%.2f  标定后 %+.2f±%.2f"%(np.mean(raw-true),np.std(raw-true),np.mean(cal-true),np.std(cal-true)))
