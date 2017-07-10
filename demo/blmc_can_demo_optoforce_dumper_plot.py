# Copyright 2017, Max Planck Society. All rights reserved.

import matplotlib
import matplotlib.pyplot as plt

from numpy import genfromtxt

my_data = genfromtxt('/tmp/blmc_can_demo_dumper.csv', delimiter=',')

IDX_TIME_MS = 0
IDX_COUNTER = 1
IDX_SAMPLE_COUNTER = 2

fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(10, 12), dpi=80)

[ax.set_title(title) for (ax, title) in zip(axes, [
	'F_{x, y, z} [N] vs Program Counter',
	'F_{x, y, z} [N] vs Package Counter',
	'Ratio "Package Counter" vs "Program Counter"',
])]
axes[0].plot(my_data[:, IDX_COUNTER], my_data[:, -3:])

axes[1].plot(my_data[:, IDX_SAMPLE_COUNTER], my_data[:, -3:])

axes[2].plot(my_data[:, IDX_COUNTER], 
	(my_data[:, IDX_SAMPLE_COUNTER] - my_data[0, IDX_SAMPLE_COUNTER])/
    (my_data[:, IDX_COUNTER] + 1))
axes[2].set_ylim([0, 1])

plt.tight_layout()
plt.show()
