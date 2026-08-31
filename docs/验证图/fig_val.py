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
                                mua_to_toi, fit_scattering_and_toi, musp_power_law,
                                MUSP_REF_800, MUSP_EXPONENT)
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
subs=sorted(F.Subject.unique(),key=lambda x:int(x.replace('Subject','')))
MUA=[];MUS=[];TRU=[];FIX=[];FIT=[];BT=[];BF=[]
for s in subs:
    g=F[F.Subject==s].groupby('Lambda')[['mua','musp']].mean(); wl=g.index.values.astype(float)
    ma=np.interp(OUR,wl,g.mua.values); ms=np.interp(OUR,wl,g.musp.values)
    MUA.append(ma); MUS.append(ms)
    BT.append(-np.polyfit(np.log(OUR/800.),np.log(ms),1)[0])
    c,*_=np.linalg.lstsq(E*LN10,ma,rcond=None); TRU.append(100*c[0]/(c[0]+c[1]))
    A=np.array([attenuation(rho,ma[i],ms[i],N) for i in range(5)])
    sl,_=fit_slopes(A,rho); me=slope_to_mueff(sl,rho)
    FIX.append(mua_to_toi(mua_from_mueff(me,musp_power_law(OUR,MUSP_REF_800,MUSP_EXPONENT)),OUR,'wray')[0])
    t,b,_=fit_scattering_and_toi(sl,rho,OUR,table='wray'); FIT.append(t); BF.append(b)
MUA=np.array(MUA);MUS=np.array(MUS);TRU=np.array(TRU);FIX=np.array(FIX);FIT=np.array(FIT)
BT=np.array(BT);BF=np.array(BF)

fig,ax=plt.subplots(2,2,figsize=(14.2,9.4))
fig.subplots_adjust(hspace=.42,wspace=.26,top=.845,bottom=.085,left=.07,right=.975)
fig.text(.07,.955,"SRS 反演的验证：10 名成人前额的时域实测光学参数",fontsize=17,fontweight="bold",color=INK)
fig.text(.07,.900,"数据 Zenodo 10.5281/zenodo.15828014（CC-BY 4.0）·时域 DOS 为 μa/μs′ 分离的金标准 · "
         "由实测谱正演多距离衰减，再用 fnirs_pipeline/srs.py 反演",fontsize=9.6,color=INK2)

# ① 实测谱
a=ax[0,0]; cm=plt.get_cmap("viridis")
for i,s in enumerate(subs):
    a.plot(OUR,MUS[i],color=cm(i/9),lw=1.5,marker='o',ms=4,alpha=.85)
a.set_xlabel("波长 (nm)"); a.set_ylabel("μs′ (cm⁻¹)",color=NEU)
a.tick_params(axis='y',colors=NEU); a.grid(True,alpha=.9)
a2=a.twinx(); a2.spines['right'].set_visible(True)
for i in range(10): a2.plot(OUR,MUA[i],color=cm(i/9),lw=1.2,ls='--',alpha=.7)
a2.set_ylabel("μa (cm⁻¹)  虚线",color=BAD); a2.tick_params(colors=BAD)
a.set_title("① 10 人的真实前额光学参数（每色一人）",color=INK,pad=8,loc="left")
a.text(.03,.06,"μs′@810 = 10.64 ± 0.85\nμa@810 = 0.1500 ± 0.0133",transform=a.transAxes,
       fontsize=9,color=INK2,va="bottom")

# ② 真值 vs 读数
a=ax[0,1]
lim=[62,90]
a.plot(lim,lim,color=INK3,lw=1.2,ls='--',zorder=1)
for v,c,lab in ((FIX,OK,"固定假设 μs′（默认）"),(FIT,BAD,"自解谱指数")):
    k,b0=np.polyfit(TRU,v,1); r=np.corrcoef(TRU,v)[0,1]
    a.scatter(TRU,v,s=58,color=c,alpha=.85,edgecolor=SURF,lw=1.2,zorder=3,
              label="%s\n  偏差 %+.2f±%.2f  r=%.4f  斜率 %.3f"%(lab,(v-TRU).mean(),(v-TRU).std(),r,k))
    xx=np.array(lim); a.plot(xx,k*xx+b0,color=c,lw=1.6,alpha=.6,zorder=2)
a.set_xlim(lim); a.set_ylim(lim); a.set_xlabel("真值 StO₂ (%)  ← 时域实测 μa 解出")
a.set_ylabel("srs.py 反演的 TOI (%)")
a.set_title("② 趋势验证：相关 0.94，无量程压缩",color=INK,pad=8,loc="left")
a.grid(True,alpha=.9); a.legend(frameon=False,loc="upper left",labelcolor=INK2,fontsize=8)

# ③ b 的实测 vs 自解
a=ax[1,0]; x=np.arange(10)
a.bar(x-.2,BT,.4,color=NEU,edgecolor=SURF,lw=1.2,label="实测 b（时域）",zorder=3)
a.bar(x+.2,BF,.4,color=BAD,edgecolor=SURF,lw=1.2,label="自解出的 b",zorder=3)
a.axhline(1.6,color=BAD,lw=1.6,ls='--',zorder=4)
a.text(9.4,1.62,"约束上界 1.6",ha="right",fontsize=9,color=BAD,fontweight="bold")
a.axhline(0.4,color=INK3,lw=1.2,ls=':',zorder=4); a.text(-.4,.43,"下界 0.4",fontsize=8.5,color=INK2)
a.set_xticks(x); a.set_xticklabels([s.replace('Subject','S') for s in subs],fontsize=8.5)
a.set_ylabel("μs′ 谱指数 b"); a.set_ylim(0,1.85)
a.set_title("③ 自解谱指数失效：%d/10 顶到上界，其余也远高于实测"%(BF>1.59).sum(),
            color=BAD,pad=8,loc="left")
a.grid(True,axis='y',alpha=.9); a.legend(frameon=False,loc="upper left",labelcolor=INK2)
a.text(.98,.04,"实测 b = 0.538~1.303（均值 0.927）\n根因：水/脂质吸收未建模，非 μs′ 偏离幂律",
       transform=a.transAxes,ha="right",fontsize=8.6,color=INK2)

# ④ 覆盖矩阵
a=ax[1,1]; a.axis("off")
rows=[("代码实现无误（符号/量纲）",1,"srs_reference 自检 10 项"),
      ("正演模型描述真实介质",1,"BRUNO 血液体模，残差 0.3%"),
      ("真实光学参数下的趋势",1,"本图②：r=0.94，斜率 1.08"),
      ("自解谱指数在真实数据可用",0,"本图③：顶边界，默认已关"),
      ("时间维度的真实氧变化",0,"无此类公开数据"),
      ("分层几何（头皮/颅骨/CSF）",0,"零验证"),
      ("绝对准确度",0,"需人体受控降氧")]
a.text(0,1.0,"④ 验证覆盖",fontsize=11.5,color=INK,fontweight="bold",transform=a.transAxes)
for i,(k,v,note) in enumerate(rows):
    y=.90-i*.098
    a.text(.03,y,k,fontsize=9.6,color=INK,transform=a.transAxes,va="center")
    a.add_patch(plt.Rectangle((.575,y-.028),.055,.056,transform=a.transAxes,
                facecolor=OK if v else BAD,edgecolor="none",zorder=3))
    a.text(.6025,y,"OK" if v else "X",fontsize=8.5,color="white",fontweight="bold",
           ha="center",va="center",transform=a.transAxes,zorder=4)
    a.text(.655,y,note,fontsize=8.6,color=INK3,transform=a.transAxes,va="center")
a.add_patch(plt.Rectangle((0,-.03),1.0,.185,transform=a.transAxes,
            facecolor="#fff4ef",edgecolor=BAD,lw=1.3,zorder=0))
a.text(.025,.105,"已验的是【受试者间】真实差异，不是【同一个人】的时间变化",
       fontsize=9.3,color=BAD,fontweight="bold",transform=a.transAxes,va="center")
a.text(.025,.045,"文献对照：真实血液体模上 SRS 类算法仅恢复 39~80%（真值 0~100%）",
       fontsize=8.8,color=INK2,transform=a.transAxes,va="center")
a.text(.025,-.005,"仿真表现不可外推到真实测量",
       fontsize=8.8,color=INK2,transform=a.transAxes,va="center")

fig.text(.07,.032,"② 的真值由时域实测 μa 谱经 wray 表解出；ε 表这一步两侧共用，故本图验证的是"
         "「斜率 → μeff → μa」链路与五波长采样的充分性，不含 ε 表本身的正确性。",fontsize=8.5,color=INK3)
out=Path(__file__).parent/"srs_validation.png"; fig.savefig(out,dpi=170)
print("saved:",out)
print("固定假设 偏差 %+.2f±%.2f  r=%.4f  斜率 %.3f"%((FIX-TRU).mean(),(FIX-TRU).std(),np.corrcoef(TRU,FIX)[0,1],np.polyfit(TRU,FIX,1)[0]))
print("自解     偏差 %+.2f±%.2f  r=%.4f  斜率 %.3f  b顶界 %d/10"%((FIT-TRU).mean(),(FIT-TRU).std(),np.corrcoef(TRU,FIT)[0,1],np.polyfit(TRU,FIT,1)[0],(BF>1.59).sum()))
