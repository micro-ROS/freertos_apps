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

names = ["servs.txt", "clis.txt"]
descriptions = ["Servers", "Clients"]
mem_types = ["Static", "Stack", "Dynamic", "Total"]

fig, axs = plt.subplots(1, 2, sharex='col', sharey='row', figsize=(6.4, 2.4))
#axs = axs.flatten()
formatter1 = EngFormatter(places=0, sep="\N{THIN SPACE}")

for i in range(len(names)):
    with open(names[i],'r') as p:
        file = p.read()
        
    file = " ".join(file.split("\n")[1:])
    data = [int(x) for x in file.split(" ") if len(x)]
    data = np.reshape(data, (4, 5))
    data = data.T

    for k in range(len(mem_types) - 1): # iterates on memory types except for the total one: 
        axs[i].plot(data[0], data[k + 1], 'o-', markersize=1.5, linewidth=1, linestyle='--', label="{}".format(mem_types[k])) # single ccontituents
    axs[i].plot(data[0], data[4], 'o-', markersize=1.5, linewidth=1, color='black', label="{}".format(mem_types[3])) #total memory
    axs[i].legend(frameon=False, fontsize='small')
    axs[i].set_title(descriptions[i], fontsize=10)
    axs[i].grid(b=True, which='major', linestyle='--', color='grey', linewidth=0.3)
    axs[i].grid(b=True, which='minor', linestyle='--', color='grey', linewidth=0.3)
    axs[i].yaxis.set_major_formatter(formatter1)
    axs[i].tick_params(axis='both', which='major', labelsize=7)


fig.text(0.5, -0.03, 'Entities Number', ha='center', fontsize=10)
fig.text(-0.04, 0.5, 'Memory usage (B)', va='center', rotation='vertical', fontsize=11)

fig.tight_layout()
plt.savefig("servcli" + ".png", dpi=600, bbox_inches='tight')
