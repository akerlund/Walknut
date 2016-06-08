

from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg

class plottingClass(QtGui.QWidget):
	def __init__(self):
		QtGui.QWidget.__init__(self)
		self.setupUi(self)

	def setupUi(self, QWidget):
		#QtGui.QApplication.setGraphicsSystem('raster')
		app = QtGui.QApplication([])
		#mw = QtGui.QMainWindow()
		#mw.resize(800,800)

		win = pg.GraphicsWindow(title="Basic plotting examples")
		win.resize(1000,600)
		win.setWindowTitle('pyqtgraph example: Plotting')

		# Enable antialiasing for prettier plots
		pg.setConfigOptions(antialias=True)

		p1 = win.addPlot(title="Basic array plotting", y=np.random.normal(size=100))