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

names = ["pubs-REL-3memories.txt", "pubs-BE-3memories.txt", "subs-REL-3memories.txt", "subs-BE-3memories.txt"]

descriptions = ["Publishers - Reliable streams",
                "Publishers - BE streams",
                "Subscribers - Reliable streams",
                "Subscribers - BE streams"]

fig, axs = plt.subplots(2, 2, sharex='col', sharey='row')
axs = axs.flatten()
formatter1 = EngFormatter(places=0, sep="\N{THIN SPACE}")

mem_types = ["Static", "Stack", "Dynamic"]

for k in range(len(names)): # ranges over the 4 setups explored
    with open(names[k],'r') as p:
        file = p.read()
    file = " ".join(file.split("\n")[1:]) # removes first line and returns string of all elements in file that is being read separated by a white space

    data = [int(x) for x in file.split(" ") if len(x)] # returns same as above as a LIST
    data = np.reshape(data, (4, 4)) # returns same as before arranged into 4x4 ARRAY
    data = data.T # Transposes matrix
    
    for i in range(len(mem_types)): # iterates on memory types
        axs[k].plot(data[0], data[i + 1], 'o-', markersize=1.5, linewidth=1, label="{}".format(mem_types[i]))
    axs[k].legend(frameon=False, fontsize='small')
    axs[k].set_title(descriptions[k], fontsize=10)
    axs[k].grid(b=True, which='major', linestyle='--', color='grey', linewidth=0.3)
    axs[k].grid(b=True, which='minor', linestyle='--', color='grey', linewidth=0.3)
    axs[k].yaxis.set_major_formatter(formatter1)
    axs[k].tick_params(axis='both', which='major', labelsize=7)

fig.text(0.5, -0.03, 'Entities Number', ha='center', fontsize=10)
fig.text(-0.04, 0.5, 'Memory usage (B)', va='center', rotation='vertical', fontsize=11)
fig.tight_layout()

plt.savefig("3mems" + ".png", dpi=600, bbox_inches='tight')