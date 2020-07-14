import msparser
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
names = ["results-XML-RELIABLE.txt", "results-REF-RELIABLE.txt","results-XML-BEST_EFFORT.txt", "results-REF-BEST_EFFORT.txt"]

descriptions = ["Subscriber using XML and reliable streams\nFreeRTOS - UART Transport",
                "Subscriber using references and reliable streams\nFreeRTOS - UART Transport",
                "Subscriber using XML and best-effort streams\nFreeRTOS - UART Transport",
                "Subscriber using references and best-effort streams\nFreeRTOS - UART Transport"]

entities_sizes = [1, 5, 10, 20]

for name, desc in zip(names, descriptions):
    with open(name,'r') as p:
        file = p.read()
    
    file = " ".join(file.split("\n")[1:])
    data = [int(x) for x in file.split(" ") if len(x)]
    data = np.reshape(data, (40, 4))

    topic_sizes = list(dict.fromkeys([x[1] for x in data]))
    entities_sizes = list(dict.fromkeys([x[0] for x in data]))
    
    total_usages_steady = []

    for entities_size in entities_sizes:
        for topic_size in topic_sizes:
            line = [x for x in data if x[0] == entities_size and x[1] == topic_size][0]

            min_buff = 400 if "XML" in name else 150
            topic_size_adjs = min_buff if topic_size + 32 < min_buff else topic_size + 32

            total_usages_steady.append(line[2]+line[3]-topic_size_adjs)  #Removing topic size buffer since topic data memory can be shared with data source

    total_usages_steady = np.reshape(total_usages_steady, (len(entities_sizes), len(topic_sizes)))

    fig = plt.figure()
    ax = plt.axes()

    for data,n in zip(total_usages_steady, entities_sizes):
        plt.plot(topic_sizes, data, 'o-', markersize=1.5, linewidth=1, label="{:d} subs".format(n))

    plt.legend(frameon=False, fontsize='small')
    plt.ylabel("Memory usage (B)")
    plt.xlabel("Topic size (B)")
    plt.suptitle("eProsima XRCE-DDS Client -- Memory consumption", fontsize=14)
    plt.title(desc, fontsize=8)

    ax.set_ylim([2000,8000])

    formatter1 = EngFormatter(places=1, sep="\N{THIN SPACE}")  # U+2009
    ax.xaxis.set_major_formatter(formatter1)
    ax.yaxis.set_major_formatter(formatter1)

    plt.grid(b=True, which='major', linestyle='--',
             color='grey', linewidth=0.3)
    plt.grid(b=True, which='minor', linestyle='--',
             color='grey', linewidth=0.3)

    plt.savefig(name + ".svg", dpi=400, bbox_inches='tight')
    plt.savefig(name + ".png", dpi=400, bbox_inches='tight')

    # print(tabulate(zip(topic_sizes, [x/1000 for x in stack_usages_steady], [x/1000 for x in heap_usages_steady], [
    #     x/1000 for x in total_usages_steady]), headers=["Topic size (B)", "Stack Usage (KiB)", "Heap Usage (KiB)", "Total Usage (KiB)"]))
