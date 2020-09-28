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

names = ["results-REL-pubs.txt", "results-REL-subs.txt", "results-BE-pubs.txt", "results-BE-subs.txt"]

descriptions = ["Publishers - Reliable streams",
                "Subscribers - Reliable streams",
                "Publishers - BE streams",
                "Subscribers - BE streams"]

total_usages_steady = []

for name, desc in zip(names, descriptions):
    with open(name,'r') as p:
        file = p.read()
        
    file = " ".join(file.split("\n")[1:])
    data = [int(x) for x in file.split(" ") if len(x)]
    data = np.reshape(data, (20, 3))
    
    topic_sizes = list(dict.fromkeys([x[1] for x in data]))
    entities_sizes = list(dict.fromkeys([x[0] for x in data]))
    
    usages_steady = []

    for entities_size in entities_sizes:
        for topic_size in topic_sizes:
            line = [x for x in data if x[0] == entities_size and x[1] == topic_size][0]

            min_buff = 400 if "XML" in name else 150
            topic_size_adjs = min_buff if topic_size + 32 < min_buff else topic_size + 32

            usages_steady.append(line[2])  #Removing topic size buffer since topic data memory can be shared with data source
            
    usages_steady = np.reshape(usages_steady, (len(entities_sizes), len(topic_sizes)))
    total_usages_steady.append(usages_steady)

fig, axs = plt.subplots(2, 2, sharex='col', sharey='row')
axs = axs.flatten()

formatter1 = EngFormatter(places=1, sep="\N{THIN SPACE}")

for i in range(len(names)):
    for j,n in zip(total_usages_steady[i], topic_sizes):
        axs[i].plot(entities_sizes, j, 'o-', markersize=1.5, linewidth=1, label="{:d} B".format(n))
        axs[i].legend(frameon=False, fontsize='small')
        axs[i].set_ylim([42717,172539])
        axs[i].set_title(descriptions[i], fontsize=10)
        axs[i].grid(b=True, which='major', linestyle='--', color='grey', linewidth=0.3)
        axs[i].grid(b=True, which='minor', linestyle='--', color='grey', linewidth=0.3)
        axs[i].xaxis.set_major_formatter(formatter1)
        axs[i].yaxis.set_major_formatter(formatter1)
        axs[i].tick_params(axis='both', which='major', labelsize=7)

#fig.text(0.5, 1.02, 'Realiable communication', ha='center', fontsize=15)

fig.text(0.5, -0.03, 'Entities Number', ha='center', fontsize=10)
fig.text(-0.04, 0.5, 'Memory usage (B)', va='center', rotation='vertical', fontsize=11)

fig.tight_layout()

#plt.savefig("overall_rel" + ".svg", dpi=600, bbox_inches='tight')
plt.savefig("overall_rel" + ".png", dpi=600, bbox_inches='tight')
