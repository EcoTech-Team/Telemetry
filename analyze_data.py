import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import ticker
from matplotlib.ticker import MaxNLocator
from matplotlib.widgets import Slider

print("matplotlib version:", matplotlib.__version__)

dirPath = sys.argv[1]
if len(sys.argv) > 2:
    print("To many arguments. Only path to directory with data is valid.")
    exit()

if dirPath[-1] != "/":
    dirPath = dirPath+"/"

buttonA = dirPath+"MotorController/buttonA.txt"
buttonB = dirPath+"MotorController/buttonB.txt"
buttonC = dirPath+"MotorController/buttonC.txt"
buttonD = dirPath+"MotorController/buttonD.txt"
current = dirPath+"MotorDriver/current.txt"
rpm = dirPath+"MotorDriver/rpm.txt"
voltage = dirPath+"MotorDriver/voltage.txt"

f1 = open(buttonA)
f2 = open(buttonB)
f3 = open(buttonC)
f4 = open(buttonD)
CurrF = open(current)
RpmF = open(rpm)
VoltF = open(voltage)

file_list = [f1, f2, f3, f4, CurrF, RpmF, VoltF]
data_list = []
for elem in file_list:
    data_elem = elem.read().split('\n')
    data_elem.pop()
    data_elem = [int(num_str) for num_str in data_elem]
    data_list.append(data_elem)

f1.close()
f2.close()
f3.close()
f4.close()
CurrF.close()
RpmF.close()
VoltF.close()

x = []
tw = 7 # time window
space = 10 #10us between measure points

for elem in range(0, 7, 1):
    maxTime = len(data_list[elem])/100 # scale to ms
    x_range = np.arange(0, maxTime, space/1000)
    x.append(x_range)

# define three y axis depend on receive data
y0 = data_list[0]
y1 = data_list[1]
y2 = data_list[2]
y3 = data_list[3]
y4 = data_list[4]
y5 = data_list[5]
y6 = data_list[6]

# Create 4 subplots in one figure
fig, axs = plt.subplots(4)
plt.subplots_adjust(top=0.955)
plt.subplots_adjust(bottom=0.08)
plt.tight_layout(h_pad=0.1)
axs[0].axis([0, tw, -0.1,1.1])
axs[1].axis([0, tw, -0.1+min(y4),0.1+max(y4)])
axs[2].axis([0, tw, -0.1+min(y5),0.1+max(y5)])
axs[3].axis([0, tw, -0.1+min(y6),0.1+max(y6)])
axs[0].plot(x[0], y0, label='A')
axs[0].plot(x[1], y1, label='B')
axs[0].plot(x[2], y2, label='C')
axs[0].plot(x[3], y3, label='D')
axs[1].plot(x[4], y4)
axs[2].plot(x[5], y5)
axs[3].plot(x[6], y6)

axs[0].yaxis.set_ticks(np.arange(0, 1.1, 1.0))
axs[0].legend(loc='upper left', fontsize='x-small')
axs[0].set_title('Buttons states')
axs[1].set_title('Current')
axs[2].set_title('RPM')
axs[3].set_title('Voltage')

axs[0].set_ylabel('ON=1 / OFF=0')
axs[1].set_ylabel('Current[mA]')
axs[2].set_ylabel('rpm')
axs[3].set_ylabel('Voltage[V]')
axs[3].set_xlabel('time (ms)')

axs[0].xaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
axs[1].xaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
axs[2].xaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
axs[3].xaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))

axsPos = plt.axes([0.01, 0.15, 0.03, 0.65])
spos = Slider(axsPos, 'Pos', 0, maxTime-tw, orientation='vertical')

def slider(val):
    pos = spos.val
    axs[0].axis([pos, pos+tw,-0.1,1.1])
    axs[1].axis([pos, pos+tw,-0.1+min(y4),0.1+max(y4)])
    axs[2].axis([pos, pos+tw,-0.1+min(y5),0.1+max(y5)])
    axs[3].axis([pos, pos+tw,-0.1+min(y6),0.1+max(y6)])
    fig.canvas.draw_idle()

spos.on_changed(slider)
plt.show()
