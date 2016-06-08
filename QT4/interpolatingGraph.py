import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np


pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')
win = pg.GraphicsWindow()
pg.setConfigOptions(antialias=True)
win.setWindowTitle('Interpolating ADC Values')

adc = [300,310,315,309,350,400,3750,3800,410,310,350,300]
polated = []
n = 20
summ = 0
for i in range(11):
	s0 = adc[i]
	s1 = adc[i+1]
	for j in range(n):
		polated.append((s0*(n-j)+s1*j)/n)
		summ += (s0*(n-j)+s1*j)/n

halfsum = summ/2
summ = 0
step = 0
stepped=[]
counter = 0
while(summ < halfsum):
	summ += polated[step]
	step += 1

for i in range(220):
	stepped.append(0)
stepped[step] = 4095

# 1) Simplest approach -- update data in the array such that plot appears to scroll
#    In these examples, the array size is fixed.
p1 = win.addPlot()
p2 = win.addPlot()
#p3 = win.addPlot()
curve1 = p1.plot(adc,pen=(200,200,200), symbolBrush=(255,0,0), symbolPen='w')
curve2 = p2.plot(polated,pen=(200,200,200), symbolBrush=(0,0,255), symbolPen='w')
curve3 = p2.plot(stepped,pen=(200,200,200), symbolBrush=(0,255,0), symbolPen='w')
#self.curve8 = self.plot2.plot(self.stm_r_adcData, pen=(200,200,200), symbolBrush=(255,0,0), symbolPen='w')
		
## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
	import sys
	if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
		QtGui.QApplication.instance().exec_()
