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

f_data = dirPath+"data_from_ride.txt"

buttonA = []
buttonB = []
buttonC = []
buttonD = []
voltage = []
current = []
rpm = []

with open(f_data) as f:
    next(f)
    lines = f.readlines()
    for l in lines:
        temp = l.split('|')
        temp[1] = temp[1].strip('\n') # remove symbol of end line
        # assign proper value to the list and cast it to integer
        buttonA.append(int(temp[0].split('-')[0]))
        buttonB.append(int(temp[0].split('-')[1]))
        buttonC.append(int(temp[0].split('-')[2]))
        buttonD.append(int(temp[0].split('-')[3]))
        voltage.append(int(temp[1].split('-')[0]))
        current.append(int(temp[1].split('-')[1]))
        rpm.append(int(temp[1].split('-')[2]))

# figure section
tw = 1000 # time window - 1000ms
space = 10 #10ms between measure points

maxTime = len(buttonA) * 10 # scale to ms
x = np.arange(0, maxTime, space)

# Create 4 subplots in one figure
fig, axs = plt.subplots(4)
plt.subplots_adjust(top=0.955)
plt.subplots_adjust(bottom=0.08)
plt.tight_layout(h_pad=0.1)
axs[0].axis([0, tw, -0.1,1.1])
axs[1].axis([0, tw, -0.1+min(voltage),0.1+max(voltage)])
axs[2].axis([0, tw, -0.1+min(current),0.1+max(current)])
axs[3].axis([0, tw, -0.1+min(rpm),0.1+max(rpm)])
axs[0].plot(x, buttonA, label='A')
axs[0].plot(x, buttonB, label='B')
axs[0].plot(x, buttonC, label='C')
axs[0].plot(x, buttonD, label='D')
axs[1].plot(x, voltage)
axs[2].plot(x, current)
axs[3].plot(x, rpm)

axs[0].yaxis.set_ticks(np.arange(0, 1.1, 1.0))
axs[0].legend(loc='upper left', fontsize='x-small')
axs[0].set_title('Buttons states')
axs[1].set_title('Voltage')
axs[2].set_title('Current')
axs[3].set_title('RPM')

axs[0].set_ylabel('ON=1 / OFF=0')
axs[1].set_ylabel('Voltage[V]')
axs[2].set_ylabel('Current[mA]')
axs[3].set_ylabel('RPM [obr/min]')
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
    axs[1].axis([pos, pos+tw,-0.1+min(voltage),0.1+max(voltage)])
    axs[2].axis([pos, pos+tw,-0.1+min(current),0.1+max(current)])
    axs[3].axis([pos, pos+tw,-0.1+min(rpm),0.1+max(rpm)])
    fig.canvas.draw_idle()

spos.on_changed(slider)
plt.show()
