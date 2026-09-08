#!/usr/bin/env python3
"""Plot recorded rear-axle transfer paths; never generate synthetic results."""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

parser=argparse.ArgumentParser()
parser.add_argument("result",type=Path)
parser.add_argument("output",type=Path)
args=parser.parse_args()
data=json.loads(args.result.read_text(encoding="utf-8"))
if data["status"]!="passed" or not data["trace"]:
    raise ValueError("a passed recorded cycle is required")
fig,ax=plt.subplots(figsize=(10,7),layout="constrained")
colors={-1:"#bd6a1b",1:"#1965a3"}
chunks=[]
for row in data["trace"]:
    if not chunks or chunks[-1][0][1]!=row[1]:
        chunks.append([])
    chunks[-1].append(row)
seen=set()
for rows in chunks:
    gear=rows[0][1]
    label=("Reverse" if gear<0 else "Forward") if gear not in seen else None
    seen.add(gear)
    ax.plot([r[2] for r in rows],[r[3] for r in rows],color=colors[gear],
            linestyle="--" if gear<0 else "-",linewidth=2,label=label)
    a,b=rows[len(rows)//2],rows[min(len(rows)-1,len(rows)//2+30)]
    ax.annotate("",xy=(b[2],b[3]),xytext=(a[2],a[3]),arrowprops={"arrowstyle":"->","color":colors[gear],"lw":2})
source=data["config"]["source_center"]
target=data["config"]["unload_center"]
ax.add_patch(Circle(source,1.8/math.tan(math.radians(34)),color="#a78349",alpha=.25))
ax.add_patch(Circle(target,data["config"]["unload_radius_m"],fill=False,color="#258450",linestyle=":",linewidth=2))
ax.scatter(*source,color="#865e29",s=60,zorder=5)
ax.scatter(*target,color="#258450",s=60,zorder=5)
ax.annotate("A: source pile",source,xytext=(8,12),textcoords="offset points")
ax.annotate("B: deposition zone",target,xytext=(8,12),textcoords="offset points")
parking=data["config"]["unload_pose"]
ax.plot([parking[0],target[0]],[parking[1],target[1]],color="#258450",linestyle=":",alpha=.6)
ax.annotate("Rear axle while unloading",parking[:2],xytext=(12,-10),textcoords="offset points",fontsize=9)
ax.set(xlabel="World X (m)",ylabel="World Y (m)",title="Recorded A/B cycle: four transfer legs (ground-truth debug)")
ax.set_aspect("equal"); ax.grid(alpha=.18); ax.legend(loc="lower right")
ax.text(.45,.03,f"Loaded / unloaded: {data['loaded_volume_m3']:.3f} m³\nMaterial inside B: {data['inside_B_fraction']:.0%}",transform=ax.transAxes,
        bbox={"facecolor":"white","edgecolor":"#dddddd","alpha":.95})
args.output.parent.mkdir(parents=True,exist_ok=True)
fig.savefig(args.output,dpi=160)
plt.close(fig)
