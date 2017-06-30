import matplotlib
import matplotlib.pyplot as plt

from numpy import genfromtxt

my_data = genfromtxt('/tmp/blmc_can_demo_dumper.csv', delimiter=',')

IDX_TIME_MS = 0
IDX_COUNTER = 1
IDX_SAMPLE_COUNTER = 2

plt.plot(my_data[:, IDX_COUNTER], my_data[:, -3:])
plt.show(block=False)

plt.plot(my_data[:, IDX_SAMPLE_COUNTER], my_data[:, -3:])
plt.show(block=False)


plt.plot(my_data[:, IDX_COUNTER], 
         (my_data[:, IDX_SAMPLE_COUNTER] - my_data[0, IDX_SAMPLE_COUNTER])/(my_data[:, IDX_COUNTER] + 1))
plt.ylim([0, 1])
plt.show()
