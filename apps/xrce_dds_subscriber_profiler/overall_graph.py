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

names = ["results-XML-RELIABLE.txt", "results-XML-BEST_EFFORT.txt", "results-REF-RELIABLE.txt", "results-REF-BEST_EFFORT.txt"]

descriptions = ["XML & Reliable streams",
                "XML & Best-effort streams",
                "References & Reliable streams",
                "References & Best-effort streams"]

total_usages_steady = []
topic_sizes = []
entities_sizes = []

for i in range(len(names)):
    with open(names[i],'r') as p:
        file = p.read()
        
    file = " ".join(file.split("\n")[1:])
    data = [int(x) for x in file.split(" ") if len(x)]
    data = np.reshape(data, (40, 4))
    
    ts = list(dict.fromkeys([x[1] for x in data]))
    es = list(dict.fromkeys([x[0] for x in data]))
    
    topic_sizes.append(ts)
    entities_sizes.append(es)
    
    usages_steady = []

    for entities_size in entities_sizes[i]:
        for topic_size in topic_sizes[i]:
            line = [x for x in data if x[0] == entities_size and x[1] == topic_size][0]

            min_buff = 400 if "XML" in names[i] else 150
            topic_size_adjs = min_buff if topic_size + 32 < min_buff else topic_size + 32

            usages_steady.append(line[2]+line[3]-topic_size_adjs)  #Removing topic size buffer since topic data memory can be shared with data source
            
    usages_steady = np.reshape(usages_steady, (len(entities_sizes[i]), len(topic_sizes[i])))
    total_usages_steady.append(usages_steady)

fig, axs = plt.subplots(2, 2, sharex='col', sharey='row')
axs = axs.flatten()

formatter1 = EngFormatter(places=1, sep="\N{THIN SPACE}")

for i in range(len(names)):
    for j,n in zip(total_usages_steady[i], entities_sizes[i]):
        axs[i].plot(topic_sizes[i], j, 'o-', markersize=1.5, linewidth=1, label="{:d} pubs".format(n))
        axs[i].legend(frameon=False, fontsize='small')
        axs[i].set_ylim([2000,8000])
        #axs[i].set_xlim([0,3000])
        axs[i].set_title(descriptions[i], fontsize=10)
        axs[i].grid(b=True, which='major', linestyle='--', color='grey', linewidth=0.3)
        axs[i].grid(b=True, which='minor', linestyle='--', color='grey', linewidth=0.3)
        axs[i].xaxis.set_major_formatter(formatter1)
        axs[i].yaxis.set_major_formatter(formatter1)
        axs[i].tick_params(axis='both', which='major', labelsize=7)

fig.text(0.5, 1.02, 'Subscriber application', ha='center', fontsize=15)

fig.text(0.5, -0.03, 'Topic size (B)', ha='center', fontsize=10)
fig.text(-0.04, 0.5, 'Memory usage (B)', va='center', rotation='vertical', fontsize=11)

fig.tight_layout()

plt.savefig("overall_sub" + ".svg", dpi=600, bbox_inches='tight')
plt.savefig("overall_sub" + ".png", dpi=600, bbox_inches='tight')