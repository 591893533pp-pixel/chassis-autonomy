#!/usr/bin/env python3
import json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from footprint_check import inflate,placed
ROOT=Path(__file__).resolve().parents[1];g=json.loads((ROOT/'model/body_geometry.json').read_text());p=np.array(g['footprint']);margin=inflate(p,.03)
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name(),'axes.unicode_minus':False,'font.size':11})
fig,axes=plt.subplots(1,3,figsize=(14,4.6),layout='constrained')
for ax in axes:ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('前后方向 x / m');ax.set_ylabel('左右方向 y / m')
ax=axes[0];ax.add_patch(Polygon(margin,facecolor='#e5edf5',edgecolor='#6d8398',linestyle='--'));ax.add_patch(Polygon(p,facecolor='#4fa4d6',alpha=.7));ax.arrow(0,0,.2,0,width=.005,head_width=.04,color='#273d4c');ax.set_xlim(-.45,.45);ax.set_ylim(-.45,.45);ax.set_title('CAD 保守外廓：约 0.52 × 0.52 m\n虚线外缘：每侧加 0.03 m')
for i,angle in [(1,0),(2,np.pi/4)]:
 ax=axes[i];ax.axhspan(.32,.65,color='#525e69');ax.axhspan(-.65,-.32,color='#525e69');shape=placed(margin,[0,0,angle]);ax.add_patch(Polygon(shape,facecolor='#52b788' if i==1 else '#ec7867',alpha=.8));ax.set_xlim(-.6,.6);ax.set_ylim(-.55,.55);ax.set_title('示例 0.64 m 通道：直行可容纳' if i==1 else '同一通道：转到 45° 时会碰墙')
fig.suptitle('通道是否可过，取决于完整车体和转弯扫掠空间\n几何示意，尚未包含实车安装误差、线缆和定位误差',fontsize=14)
fig.savefig(ROOT/'reports/body_clearance.png',dpi=160)
