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

names = ["results-REL-pubs.txt", "results-BE-pubs.txt", "results-REL-subs.txt", "results-BE-subs.txt"]

descriptions = ["Publishers - Reliable streams",
                "Publishers - BE streams",
                "Subscribers - Reliable streams",
                "Subscribers - BE streams"]

memo_cons = []
topic_sizes = []
entities_sizes = []

for i in range(len(names)):
    with open(names[i],'r') as p:
        file = p.read()
        
    file = " ".join(file.split("\n")[1:])
    data = [int(x) for x in file.split(" ") if len(x)]
    data = np.reshape(data, (20, 3))

    ts = list(dict.fromkeys([x[1] for x in data]))
    es = list(dict.fromkeys([x[0] for x in data]))
    
    topic_sizes.append(ts)
    entities_sizes.append(es)
    mem = []
    
    for topic_size in ts:
        for entities_size in es:
            line = [x for x in data if x[0] == entities_size and x[1] == topic_size][0]
            mem.append(line[2])
    
    mem = np.reshape(mem, (len(topic_sizes[i]), len(entities_sizes[i])))
    memo_cons.append(mem)

fig, axs = plt.subplots(2, 2, sharex='col', sharey='row')

axs = axs.flatten()

formatter1 = EngFormatter(places=0, sep="\N{THIN SPACE}")

for i in range(len(names)):
    for j,n in zip(memo_cons[i], topic_sizes[i]):
        axs[i].plot(entities_sizes[i], j, 'o-', markersize=1.5, linewidth=1, label="{:d} B".format(n))
        axs[i].legend(frameon=False, fontsize='small')
        axs[i].set_title(descriptions[i], fontsize=10)
        axs[i].grid(b=True, which='major', linestyle='--', color='grey', linewidth=0.3)
        axs[i].grid(b=True, which='minor', linestyle='--', color='grey', linewidth=0.3)
        axs[i].yaxis.set_major_formatter(formatter1)
        axs[i].tick_params(axis='both', which='major', labelsize=7)

#fig.text(0.5, 1.02, 'Realiable communication', ha='center', fontsize=15)

fig.text(0.5, -0.03, 'Entities Number', ha='center', fontsize=10)
fig.text(-0.04, 0.5, 'Memory usage (B)', va='center', rotation='vertical', fontsize=11)

fig.tight_layout()

#plt.savefig("overall" + ".svg", dpi=600, bbox_inches='tight')
plt.savefig("overall" + ".png", dpi=600, bbox_inches='tight')
