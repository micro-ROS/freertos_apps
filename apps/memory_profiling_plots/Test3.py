#import msparser
import os
import signal
import subprocess
import sys
import time
from tabulate import tabulate

import numpy as np
import matplotlib.pyplot as plt

from matplotlib.ticker import EngFormatter

with open("results_test3.txt",'r') as p:
    file = p.read()
        
file = " ".join(file.split("\n")[1:])
data = [int(x) for x in file.split(" ") if len(x)]
data = np.reshape(data, (20, 2))
data = data.T

fig = plt.figure()
ax = plt.axes()

plt.plot(data[0], data[1], 'o-', markersize=2.5, linewidth=2)

#plt.ylabel("Static memory usage (B)")
#plt.xlabel("RMW history")
formatter1 = EngFormatter(places=0, sep="\N{THIN SPACE}")  # U+2009
ax.xaxis.set_major_formatter(formatter1)
ax.yaxis.set_major_formatter(formatter1)
ax.tick_params(axis='both', which='major', labelsize=12)

plt.grid(b=True, which='major', linestyle='--', color='grey', linewidth=0.3)
plt.grid(b=True, which='minor', linestyle='--', color='grey', linewidth=0.3)

fig.text(0.5, -0.03, 'RMW history', ha='center', fontsize=15)
fig.text(-0.04, 0.5, 'Static memory usage (B)', va='center', rotation='vertical', fontsize=16)
fig.tight_layout()

plt.savefig("rmw_history" + ".png", dpi=600, bbox_inches='tight')
