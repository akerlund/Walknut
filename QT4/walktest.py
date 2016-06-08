# -*- coding: utf-8 -*-#
#
#   @brief   Walknut GUI
#   @author  Fredrik Åkerlund
#   @version 1.0
#   @date    April-2015
#
from PyQt4 import QtCore, QtGui
import pyqtgraph as pg
import numpy as np
import serial, threading, socket, pickle, struct, bluetooth, sys, time, os, Queue
################################################################################
try:
	_fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
	def _fromUtf8(s):
		return s

try:
	_encoding = QtGui.QApplication.UnicodeUTF8
	def _translate(context, text, disambig):
		return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
	def _translate(context, text, disambig):
		return QtGui.QApplication.translate(context, text, disambig)
################################################################################
#	Data for bluetooth module and STM32.
startHeader = bytes( )
startHeader += struct.pack('BBB', *(85,85,85))

initz = bytes()
for val in range(0,9):
	initz += struct.pack('B', 0)

stopSend = bytes()
stopSend += struct.pack('f',12.0)
stopSend += struct.pack('f'*3,*((0.0,)*3))

startSend = bytes()
startSend += struct.pack('f',11.0)
startSend += struct.pack('f'*3,*((0.0,)*3))

################################################################################
class BlueToothClass(threading.Thread):

	def __init__(   self, 
					data_q, error_q, 
					device,
					port_timeout = 0.01):

		threading.Thread.__init__(self)	
		self.alive = threading.Event( )
		self.alive.set()	
		self.timeout = 0.01

		self.sock = None
		self.commands = '$$$SM,0\r\nSU,115200\r\n---'
		self.blueRecieve = bytes( )
		self.data_q  = data_q
		self.error_q = error_q
		self.device  = device
		self.connected = False

		self.requestData = bytes( )
		self.requestData += struct.pack('f',2.0)
		self.requestData += struct.pack('f'*3,*((0.0,)*3))

		self.ping = bytes( )
		self.ping += struct.pack('f', 42.0)
		self.ping += struct.pack('f'*3,*((0.0,)*3))

		self.isReceiving = False
		self.threadAlive = False
		self.recieve_thread = None

	def join(self, timeout=None):
		
		if self.sock:
			self.sock.shutdown(1)
			self.sock.close( )

		self.alive.clear( )
		threading.Thread.join(self, self.timeout)

	def run(self):
				
		while self.alive.isSet( ):
			time.sleep(0.25)
		print("BlueToothClass Stop")

	def connect(self):

		try:
			if not self.sock == None: 
				self.sock.close( )

			self.sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
			#self.sock.settimeout(1)
			self.sock.connect((self.device, 1))
			self.sock.send(self.commands)
			time.sleep(1.0)
			self.sock.send(initz)
			
			self.connected = True

		except bluetooth.btcommon.BluetoothError as error:			
			self.sock.close( )
			self.error_q.put(error.message)
			print ("Could not connect: ", error)			
			return

	def disconnect(self):

		if self.isReceiving:
			self.stopRecieve( )

		if self.sock:
			self.sock.close( )
			self.sock = None

	def isConnected(self):

		return self.connected

	def send(self, data):

		self.sock.send(data)

	def startRecieve(self):
	
		self.isReceiving = True
		self.recieve_thread = threading.Thread(target=self.incoming)
		self.recieve_thread.start( )
		self.threadAlive = True

	def stopRecieve(self):

		self.isReceiving = False
		self.sock.shutdown(1)

		while self.threadAlive:
			pass

		self.recieve_thread = None

	def incoming(self):
		try:
			while self.isReceiving:
				self.blueRecieve += self.sock.recv(26*4)			
				
				if (len(self.blueRecieve))==104:
					self.data_q.put(struct.unpack('fffffffffffffffffffiffiiff', self.blueRecieve))
					self.blueRecieve = bytes( )
						
			self.threadAlive = False		
		except EOFError:
			print("Klar")


	# def req_data(self):

	# 	self.sock.send(self.requestData)
	# 	self.blueRecieve = bytes( )

	# 	while not len(self.blueRecieve) == 24*4:

	# 		self.blueRecieve += self.sock.recv(24*4)			
				
	# 		if (len(self.blueRecieve))==24*4:
	# 			self.data_q.put(struct.unpack('ffffffffffffffffffffffff', self.blueRecieve))
	# 	return

	def incoming2(self):
		try:
			while self.isReceiving:
				self.blueRecieve += self.sock.recv(24*4)			
				
				if (len(self.blueRecieve))==24*4:
					self.data_q.put(struct.unpack('ffffffffffffffffffffffff', self.blueRecieve))
					self.blueRecieve = bytes( )
						
			self.threadAlive = False		
		except EOFError:
			print("Klar")

	def req_data(self):

		#print("Sending request")

		self.sock.send(self.requestData)
		self.blueRecieve = bytes( )

		while not len(self.blueRecieve) == 96:

			#print("Awaiting data")

			self.blueRecieve += self.sock.recv(96)	

			#print("Got so far:%i")%(len(self.blueRecieve))	
			
			if (len(self.blueRecieve))==96:
				self.data_q.put(struct.unpack('ffffffffffffffffffffffff', self.blueRecieve))
		return
################################################################################
class Ui_MainWindow(QtGui.QMainWindow):
	
	def __init__(self):

		QtGui.QWidget.__init__(self)
		self.setupUi(self)

		self.timer = pg.QtCore.QTimer( )
		self.startPlotUpdater( )
		self.plot_update_freq = 10
		
		# Bluetooth
		self.blueTooth_active = False
		self.blueTooth_thread = None
		self.blueTooth_data_q = Queue.Queue()
		self.blueTooth_error_q = Queue.Queue()
		self.blueTimer = QtCore.QTimer( )
		self.blueTimer_update_freq = 40.0
		self.foundDevices = None
		self.currentDevice = "00:06:66:05:11:41"
		self.dataRequested = False

		# STM Settings data
		self.stm_Kp = 0.0
		self.stm_Ki = 0.0
		self.stm_Kd = 0.0
		self.stm_i_filter = 0.0
		self.stm_d_filter = 0.0
		self.stm_pololu = 0.0
		self.stm_edf27 = 0.0

		self.stm_weight1 = 1.0
		self.stm_weight2 = 1.0
		self.stm_weight3 = 1.0
		self.stm_weight4 = 1.0
		self.stm_weight5 = 1.0
		self.stm_weight6 = 1.0
		self.stm_weight7 = 1.0
		self.stm_nrOfSamples = 8
		self.stm_whitePercent = 0.1
		self.stm_blackPercent = 0.5
		self.stm_stopTimerLimit = 3000

		#STM Recieved Settings
		self.stm_r_Kp 			= 0.0	# f
		self.stm_r_Ki 			= 0.0 	# f
		self.stm_r_Kd 			= 0.0 	# f
		self.stm_r_nrOfSamples  = 8
		self.stm_r_whitePercent = 0.0 	# i
		self.stm_r_blackPercent = 0.0 	# i

		# STM Recieve Data		
		self.stm_r_adcData = [0 for x in range(12)]	# f
		self.stm_r_error  			= 0.0
		self.stm_r_integral  		= 0.0
		self.stm_r_derivative   	= 0.0
		self.stm_r_motorSpeed   	= 0.0#
		self.stm_r_leftLapCnt   	= 0.0
		self.stm_r_rightLapCnt  	= 0.0
		self.stm_r_motorBaseA 		= 0.0#
		self.stm_r_motorBaseB 		= 0.0#
		self.stm_r_leftReg 			= 0.0
		self.stm_r_rightReg 		= 0.0
		self.stm_r_virtual 			= 0.0
		self.stm_r_isFindingLine	= 0.0
		self.stm_r_stopTimerValue	= 0.0

		# Plotting data
		self.plot_error 	 = [0 for x in range(300)]
		self.plot_integral   = [0 for x in range(300)]
		self.plot_derivative = [0 for x in range(300)]
		self.plot_leftMotor  = [0 for x in range(300)]
		self.plot_rightMotor = [0 for x in range(300)]

	def startPlotUpdater(self):

		self.timer = pg.QtCore.QTimer()
		self.timer.timeout.connect(self.updatePlotData)
		self.timer.start(10)

	def updatePlotData(self):

		self.plot_error[:-1]	  = self.plot_error[1:]
		self.plot_integral[:-1]	  = self.plot_integral[1:]
		self.plot_derivative[:-1] = self.plot_derivative[1:]
		self.plot_leftMotor[:-1]  = self.plot_leftMotor[1:]
		self.plot_rightMotor[:-1] = self.plot_rightMotor[1:]

		self.plot_error[-1]	  	 = self.stm_r_error
		self.plot_integral[-1]	 = self.stm_r_integral
		self.plot_derivative[-1] = self.stm_r_derivative
		self.plot_leftMotor[-1]  = self.stm_r_leftReg/10.0
		self.plot_rightMotor[-1] = self.stm_r_rightReg/10.0

		self.ptr += 0.01

		self.curve1.setData(self.plot_error)
		self.curve1.setPos(self.ptr, 0)

		self.curve2.setData(self.plot_integral)
		self.curve2.setPos(self.ptr, 0)

		self.curve3.setData(self.plot_derivative)
		self.curve3.setPos(self.ptr, 0)

		self.curve4.setData(self.plot_leftMotor)
		self.curve4.setPos(self.ptr, 0)

		self.curve5.setData(self.plot_rightMotor)
		self.curve5.setPos(self.ptr, 0)

		self.curve6.setData(self.stm_r_adcData)


	def on_timer(self):

		if self.blueTooth_active:
			
			#self.read_serial_data( )
			self.read_serial_req( )
			self.update_raw( )	

	def on_connect(self):
						
		self.blueTooth_data_q = Queue.Queue( )
		self.blueTooth_error_q = Queue.Queue( )

		self.blueTooth_thread = BlueToothClass(self.blueTooth_data_q, self.blueTooth_error_q, self.currentDevice)
		self.blueTooth_thread.start( )
		self.blueTooth_thread.connect( )

		timeout = time.time( )+10
		timedOut = False
		while True:
			if self.blueTooth_thread.isConnected( ):
				break
			if time.time() > timeout:
				timedOut = True
				break
	
		if not timedOut:

			#self.blueTooth_thread.startRecieve( )
			time.sleep(0.05)

			if not self.blueTooth_error_q.empty( ):
				com_error = self.blueTooth_error_q.get( )

			else:			
				com_error = None

			if com_error is not None:
				QtGui.QMessageBox.critical(self, 'BlueToothClass error', com_error)
				self.blueTooth_thread = None

			else:
				self.blueTooth_active = True
				self.label_21.setPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/available.png")))
				self.blueTimer = QtCore.QTimer( )
				self.connect(self.blueTimer, QtCore.SIGNAL('timeout()'), self.on_timer)
				self.blueTimer.start(self.blueTimer_update_freq)
		else:
			print("Couldn't establish connection to Bluetooth. Timeout.")

	def on_disconnect(self):

		if self.blueTooth_thread is not None:

			self.blueTooth_thread.disconnect( )
			self.blueTooth_thread.join(0.01)
			self.blueTooth_thread = None

		if self.blueTooth_active:

			self.blueTooth_active = False
			self.label_21.setPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/available_offline.png")))
			self.blueTimer.stop( )

	def read_serial_data(self):

		if self.blueTooth_thread is not None:
		
			self.blueTooth_thread.req_data( )
		

		if not self.blueTooth_data_q.empty( ):
				
			qdata = self.blueTooth_data_q.get()

			self.stm_r_adcData = []
			for i in range(12):
				self.stm_r_adcData.append(qdata[i])

			self.stm_r_Kp 			= qdata[12]		# f
			self.stm_r_Ki 			= qdata[13]		# f
			self.stm_r_Kd 			= qdata[14]		# f
			self.stm_r_virtual 		= qdata[15]		# f
			self.stm_r_error  		= qdata[16]		# f
			self.stm_r_integral 	= qdata[17]		# f
			self.stm_r_derivative 	= qdata[18]		# f		
			self.stm_r_motorSpeed 	= qdata[19]		# i
			self.stm_r_leftLapCnt 	= qdata[20]		# i
			self.stm_r_rightLapCnt 	= qdata[21]		# i
			self.stm_r_motorBaseA 	= qdata[22]		# i
			self.stm_r_motorBaseB 	= qdata[23]		# i
			self.stm_r_leftReg 		= qdata[24]		# i
			self.stm_r_rightReg 	= qdata[25]		# i

		# else:
		# 	print("No data")

	def read_serial_req(self):

		if self.blueTooth_thread is not None:

			if self.dataRequested:

				if not self.blueTooth_data_q.empty( ):
				
					qdata = self.blueTooth_data_q.get( )

					self.stm_r_adcData = []

					for i in range(14):
						self.stm_r_adcData.append(qdata[i])

					self.stm_r_error  			= qdata[14]
					self.stm_r_integral 		= qdata[15]
					self.stm_r_derivative 		= qdata[16]
					self.stm_r_leftLapCnt 		= qdata[17]
					self.stm_r_rightLapCnt 		= qdata[18]
					self.stm_r_leftReg 			= qdata[19]
					self.stm_r_rightReg 		= qdata[20]
					self.stm_r_virtual 			= qdata[21]
					self.stm_r_isFindingLine	= qdata[22]
					self.stm_r_stopTimerValue	= qdata[23]	

					self.dataRequested = False
				else:
					print "waiing for requested data"			
			else:
				self.blueTooth_thread.req_data( )
				self.dataRequested = True
				self.read_serial_req( )

	def read_settings_req(self):

		# if self.blueTooth_thread is not None:
		#self.blueTooth_thread.req_data( )
		if not self.blueTooth_data_q.empty( ):

			self.stm_r_Kp = qdata[0]		# f
			# self.stm_r_Ki = qdata[1]		# f
			# self.stm_r_Kd = qdata[2]		# f	
			# self.stm_i_filter = qdata[3]		# i
			# self.stm_d_filter = qdata[4]		# i
			# self.stm_pololu = qdata[5]		# i
			# self.stm_edf27 	= qdata[6]		# i
			# self. 		= qdata[7]		# f	
			# self.= qdata[8]		# i


	def f_SaveFile(self):
		print("f_SaveFile")

	def f_LoadFile(self):
		
		with open ("data.txt", "r") as myfile:
			data = myfile.read().replace('\n', '')
		
		with open("data.txt") as myfile:
			data="".join(line.rstrip() for line in myfile)

	def f_Find_Devices(self):

		print("f_Find_Devices")
		self.listWidget.clear()
		self.listWidget.addItem("Performing inquiry...")

		self.foundDevices =  bluetooth.discover_devices(duration=5, lookup_names=True,
														lookup_class=False)		
		self.listWidget.clear()
		
		for addr, name in self.foundDevices:
			device = str(addr) + " " + str(name)
			self.listWidget.addItem(device)

	def f_Use_Selected(self):
		print("f_Use_Selected")

		selected = self.listWidget.currentRow( )
		if not selected == -1:
			self.currentDevice = self.foundDevices[selected][0]
			print(self.foundDevices[selected][0])
			print(len(self.foundDevices[selected]))

	def is_number(self, nr):
		try:
			float(nr)
			return True

		except ValueError:
			return False

	def f_btn_Connect(self):

		print("btn_Connect")
		if not self.blueTooth_active:
			self.on_connect()			
		else:
			self.on_disconnect()	

	def update_raw(self):

		self.label_adc1.setText(str(format(self.stm_r_adcData[0],'.4f')))
		self.label_adc2.setText(str(format(self.stm_r_adcData[1],'.4f')))
		self.label_adc3.setText(str(format(self.stm_r_adcData[2],'.4f')))
		self.label_adc4.setText(str(format(self.stm_r_adcData[3],'.4f')))
		self.label_adc5.setText(str(format(self.stm_r_adcData[4],'.4f')))
		self.label_adc6.setText(str(format(self.stm_r_adcData[5],'.4f')))
		self.label_adc7.setText(str(format(self.stm_r_adcData[6],'.4f')))
		self.label_adc8.setText(str(format(self.stm_r_adcData[7],'.4f')))
		self.label_adc9.setText(str(format(self.stm_r_adcData[8],'.4f')))
		self.label_adc10.setText(str(format(self.stm_r_adcData[9],'.4f')))
		self.label_adc11.setText(str(format(self.stm_r_adcData[10],'.4f')))
		self.label_adc12.setText(str(format(self.stm_r_adcData[11],'.4f')))
		
		self.label_error.setText(str(format(self.stm_r_error,'.4f')))
		self.label_int.setText(str(format(self.stm_r_integral,'.4f')))
		self.label_der.setText(str(format(self.stm_r_derivative,'.4f')))
		self.label_lw.setText(str(self.stm_r_leftLapCnt))
		self.label_rw.setText(str(self.stm_r_rightLapCnt))
		self.label_lpwm.setText(str(self.stm_r_leftReg))
		self.label_rpwm.setText(str(self.stm_r_rightReg))
		self.label_virt.setText(str(self.stm_r_virtual))
		# self.stm_r_isFindingLine	= 0.0
		# self.stm_r_stopTimerValue	= 0.0
		
		self.label_speed.setText("BLANK")

		self.label_kp.setText(str(self.stm_r_Kp))
		self.label_ki.setText(str(self.stm_r_Ki))
		self.label_kd.setText(str(self.stm_r_Kd))
		

	def f_box_Usart(self):

		if self.box_usart.isChecked( ):
			
			usartStart = bytes()
			usartStart += struct.pack('f', float(3.0))
			usartStart += struct.pack('f', 1.0)
			usartStart += struct.pack('ff', *(0,0))
			self.blueTooth_thread.send(usartStart)
		else:
			usartStop = bytes( )
			usartStop += struct.pack('f', float(3.0))
			usartStop += struct.pack('f', 0.0)
			usartStop += struct.pack('ff', *(0,0))
			self.blueTooth_thread.send(usartStop)

	def f_btn_Start(self):

		print("btn_Start")
		if self.blueTooth_active:
			self.blueTooth_thread.send(startSend)

	def f_btn_Stop(self):

		print("btn_Stop")
		if self.blueTooth_active:
			self.blueTooth_thread.send(stopSend)

	def f_btn_PID_Apply(self):

		Kp = self.edit_Kp.text( )
		if self.is_number(Kp):
			self.stm_Kp = float(Kp)
		else:
			self.edit_Kp.setText(str(self.stm_Kp))

		Ki = self.edit_Ki.text( )
		if self.is_number(Ki):
			self.stm_Ki = float(Ki)
		else:
			self.edit_Ki.setText(str(self.stm_Ki))

		Kd = self.edit_Kd.text( )
		if self.is_number(Kd):
			self.stm_Kd = float(Kd)
		else:
			self.edit_Kd.setText(str(self.stm_Kd))

		i_filter = self.edit_i_filter.text( )
		if self.is_number(i_filter):
			self.stm_i_filter = float(i_filter)
		else:
			self.edit_i_filter.setText(str(self.stm_i_filter))

		d_filter = self.edit_d_filter.text( )
		if self.is_number(d_filter):
			self.stm_d_filter = float(d_filter)
		else:
			self.edit_d_filter.setText(str(self.stm_d_filter))


		if self.blueTooth_active:

			pidSend = bytes( )
			pidSend += struct.pack('f', 20.0)
			pidSend += struct.pack('f', self.stm_Kp)
			pidSend += struct.pack('f', self.stm_Ki)
			pidSend += struct.pack('f', self.stm_Kd)
			self.blueTooth_thread.send(pidSend)

			time.sleep(0.01)

			filterSend = bytes( )
			filterSend += struct.pack('ffff', *(21.0,self.stm_i_filter,self.stm_d_filter,0.0))
			self.blueTooth_thread.send(filterSend)

	def f_btn_Reset(self):

		print("btn_Reset")
		if self.blueTooth_active:
			resetSend = bytes( )
			resetSend += struct.pack('f', 22.0)
			resetSend += struct.pack('fff', *(0.0,0.0,0.0))
			self.blueTooth_thread.send(resetSend)

	def f_btn_Motors_Apply(self):
		
		print("btn_Motors_Apply")

		pololu = self.edit_PWM1.text( )

		if self.is_number(pololu):
			self.stm_pololu = int(pololu)			
		else:
			self.edit_PWM1.setText(str(self.stm_pololu))

		edf27 = self.edit_PWM2.text( )
		if self.is_number(edf27):
			self.stm_edf27 = int(edf27)			
		else:
			self.edit_PWM2.setText(str(self.stm_edf27))

		if self.blueTooth_active:
			motorSend = bytes( )
			motorSend += struct.pack('f', 30.0)
			motorSend += struct.pack('iii', *(self.stm_pololu,self.stm_edf27,0))
			self.blueTooth_thread.send(motorSend)

	def f_box_Motor(self):

		print("box_Motor")
		if self.blueTooth_active:			
			motorStart = bytes( )
			motorStart += struct.pack('f', 31.0)

			if self.box_motor.isChecked( ):
				motorStart += struct.pack('f', 1.0)

			else:
				motorStart += struct.pack('f', 0.0)

			motorStart += struct.pack('ff', (0.0,0.0))
			self.blueTooth_thread.send(motorStart)

	def f_btn_Calibrate(self):

		print("btn_Calibrate")
		if self.blueTooth_active:
			calibrate = bytes( )
			calibrate += struct.pack('f', 40.0)
			calibrate += struct.pack('fff', *(0.0,0.0,0.0))
			self.blueTooth_thread.send(calibrate)

	def f_btn_Sensors_Apply(self):

		print("btn_Sensors_Apply")

		adc1 = self.edit_ADC1.text( )
		if self.is_number(adc1):
			self.stm_weight1 = float(adc1)
		else:
			self.edit_ADC1.setText(str(self.stm_weight1))
		
		adc2 = self.edit_ADC2.text( )
		if self.is_number(adc2):
			self.stm_weight2 = float(adc2)
		else:
			self.edit_ADC2.setText(str(self.stm_weight2))
		
		adc3 = self.edit_ADC3.text( )
		if self.is_number(adc3):
			self.stm_weight3 = float(adc3)
		else:
			self.edit_ADC3.setText(str(self.stm_weight3))
		
		adc4 = self.edit_ADC4.text( )
		if self.is_number(adc4):
			self.stm_weight4 = float(adc4)
		else:
			self.edit_ADC4.setText(str(self.stm_weight4))
		
		adc5 = self.edit_ADC5.text( )
		if self.is_number(adc5):
			self.stm_weight5 = float(adc5)
		else:
			self.edit_ADC5.setText(str(self.stm_weight5))
		
		adc6 = self.edit_ADC6.text( )
		if self.is_number(adc6):
			self.stm_weight6 = float(adc6)
		else:
			self.edit_ADC6.setText(str(self.stm_weight6))
		
		adc7 = self.edit_ADC7.text( )
		if self.is_number(adc7):
			self.stm_weight7 = float(adc7)
		else:
			self.edit_ADC7.setText(str(self.stm_weight7))

		nrSamp = self.edit_nrOfSamples.text( )
		if self.is_number(nrSamp):
			self.stm_nrOfSamples = float(nrSamp)
		else:
			self.edit_nrOfSamples.setText(str(self.stm_nrOfSamples))

		if self.blueTooth_active:
			sensors = bytes( )
			sensors += struct.pack('f', 41.0)
			sensors += struct.pack('fff', *(self.stm_weight1,self.stm_weight2,self.stm_weight3))
			self.blueTooth_thread.send(sensors)
			sensors2 = bytes( )
			sensors2 += struct.pack('f', 42.0)
			sensors2 += struct.pack('fff', *(self.stm_weight4,self.stm_weight5,self.stm_weight6))
			self.blueTooth_thread.send(sensors2)
			sensors3 = bytes( )
			sensors3 += struct.pack('f', 43.0)
			sensors3 += struct.pack('fff', *(self.stm_weight7,self.stm_nrOfSamples,0.0))
			self.blueTooth_thread.send(sensors3)
			# sensors4 = bytes( )
			# sensors4 += struct.pack('f', 43.0)
			# sensors4 += struct.pack('fff', *(self.stm_whitePercent,self.stm_blackPercent,self.stm_stopTimerLimit))
			# self.blueTooth_thread.send(sensors4)

	def setupUi(self, MainWindow):

		MainWindow.setObjectName(_fromUtf8("MainWindow"))
		MainWindow.resize(989, 909)
		self.centralwidget = QtGui.QWidget(MainWindow)
		self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
		self.verticalLayout = QtGui.QVBoxLayout(self.centralwidget)
		self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
		self.horizontalLayout = QtGui.QHBoxLayout()
		self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
		self.btn_con = QtGui.QPushButton(self.centralwidget)
		self.btn_con.setAutoFillBackground(False)
		self.btn_con.setText(_fromUtf8(""))
		icon = QtGui.QIcon()
		icon.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/Bluetooth-icon (1).png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		icon.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/Bluetooth-icon (1).png")), QtGui.QIcon.Selected, QtGui.QIcon.Off)
		self.btn_con.setIcon(icon)
		self.btn_con.setIconSize(QtCore.QSize(56, 56))
		self.btn_con.setFlat(False)
		self.btn_con.setObjectName(_fromUtf8("btn_con"))
		self.horizontalLayout.addWidget(self.btn_con)
		self.verticalLayout_7 = QtGui.QVBoxLayout()
		self.verticalLayout_7.setObjectName(_fromUtf8("verticalLayout_7"))
		self.label_21 = QtGui.QLabel(self.centralwidget)
		self.label_21.setText(_fromUtf8(""))
		self.label_21.setPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/available_offline.png")))
		self.label_21.setObjectName(_fromUtf8("label_21"))
		self.verticalLayout_7.addWidget(self.label_21)
		self.horizontalLayout.addLayout(self.verticalLayout_7)
		self.line_3 = QtGui.QFrame(self.centralwidget)
		self.line_3.setFrameShape(QtGui.QFrame.VLine)
		self.line_3.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_3.setObjectName(_fromUtf8("line_3"))
		self.horizontalLayout.addWidget(self.line_3)
		self.btn_start = QtGui.QPushButton(self.centralwidget)
		self.btn_start.setAutoFillBackground(False)
		self.btn_start.setText(_fromUtf8(""))
		icon1 = QtGui.QIcon()
		icon1.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/Button-Play-icon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		icon1.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/Button-Play-icon.png")), QtGui.QIcon.Selected, QtGui.QIcon.Off)
		self.btn_start.setIcon(icon1)
		self.btn_start.setIconSize(QtCore.QSize(48, 48))
		self.btn_start.setAutoDefault(False)
		self.btn_start.setFlat(False)
		self.btn_start.setObjectName(_fromUtf8("btn_start"))
		self.horizontalLayout.addWidget(self.btn_start)
		self.btn_stop = QtGui.QPushButton(self.centralwidget)
		self.btn_stop.setText(_fromUtf8(""))
		icon2 = QtGui.QIcon()
		icon2.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/Button-Pause-icon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		self.btn_stop.setIcon(icon2)
		self.btn_stop.setIconSize(QtCore.QSize(48, 48))
		self.btn_stop.setDefault(False)
		self.btn_stop.setFlat(False)
		self.btn_stop.setObjectName(_fromUtf8("btn_stop"))
		self.horizontalLayout.addWidget(self.btn_stop)
		self.btn_reset = QtGui.QPushButton(self.centralwidget)
		self.btn_reset.setText(_fromUtf8(""))
		icon3 = QtGui.QIcon()
		icon3.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/Button-Refresh-icon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		self.btn_reset.setIcon(icon3)
		self.btn_reset.setIconSize(QtCore.QSize(48, 48))
		self.btn_reset.setDefault(False)
		self.btn_reset.setFlat(False)
		self.btn_reset.setObjectName(_fromUtf8("btn_reset"))
		self.horizontalLayout.addWidget(self.btn_reset)
		self.btn_Calibrate = QtGui.QPushButton(self.centralwidget)
		self.btn_Calibrate.setText(_fromUtf8(""))
		icon4 = QtGui.QIcon()
		icon4.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/Actions-svn-update-icon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		self.btn_Calibrate.setIcon(icon4)
		self.btn_Calibrate.setIconSize(QtCore.QSize(48, 48))
		self.btn_Calibrate.setDefault(False)
		self.btn_Calibrate.setFlat(False)
		self.btn_Calibrate.setObjectName(_fromUtf8("btn_Calibrate"))
		self.horizontalLayout.addWidget(self.btn_Calibrate)
		self.line_4 = QtGui.QFrame(self.centralwidget)
		self.line_4.setFrameShape(QtGui.QFrame.VLine)
		self.line_4.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_4.setObjectName(_fromUtf8("line_4"))
		self.horizontalLayout.addWidget(self.line_4)
		self.verticalLayout_5 = QtGui.QVBoxLayout()
		self.verticalLayout_5.setObjectName(_fromUtf8("verticalLayout_5"))
		self.box_motor = QtGui.QCheckBox(self.centralwidget)
		self.box_motor.setChecked(True)
		self.box_motor.setTristate(False)
		self.box_motor.setObjectName(_fromUtf8("box_motor"))
		self.verticalLayout_5.addWidget(self.box_motor)
		self.box_usart = QtGui.QCheckBox(self.centralwidget)
		self.box_usart.setChecked(False)
		self.box_usart.setObjectName(_fromUtf8("box_usart"))
		self.verticalLayout_5.addWidget(self.box_usart)
		self.horizontalLayout.addLayout(self.verticalLayout_5)
		spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout.addItem(spacerItem)
		self.label_23 = QtGui.QLabel(self.centralwidget)
		self.label_23.setMaximumSize(QtCore.QSize(71, 71))
		self.label_23.setText(_fromUtf8(""))
		self.label_23.setPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/walknut_c1.png")))
		self.label_23.setScaledContents(True)
		self.label_23.setObjectName(_fromUtf8("label_23"))
		self.horizontalLayout.addWidget(self.label_23)
		self.label_22 = QtGui.QLabel(self.centralwidget)
		self.label_22.setText(_fromUtf8(""))
		self.label_22.setObjectName(_fromUtf8("label_22"))
		self.horizontalLayout.addWidget(self.label_22)
		self.verticalLayout.addLayout(self.horizontalLayout)
		self.line_19 = QtGui.QFrame(self.centralwidget)
		self.line_19.setFrameShape(QtGui.QFrame.HLine)
		self.line_19.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_19.setObjectName(_fromUtf8("line_19"))
		self.verticalLayout.addWidget(self.line_19)
		self.tabWidget_1 = QtGui.QTabWidget(self.centralwidget)
		self.tabWidget_1.setIconSize(QtCore.QSize(32, 32))
		self.tabWidget_1.setObjectName(_fromUtf8("tabWidget_1"))
		self.tab_2 = QtGui.QWidget()
		self.tab_2.setObjectName(_fromUtf8("tab_2"))
		self.verticalLayout_3 = QtGui.QVBoxLayout(self.tab_2)
		self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
		self.line_16 = QtGui.QFrame(self.tab_2)
		self.line_16.setFrameShape(QtGui.QFrame.HLine)
		self.line_16.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_16.setObjectName(_fromUtf8("line_16"))
		self.verticalLayout_3.addWidget(self.line_16)
		self.tabWidget_3 = QtGui.QTabWidget(self.tab_2)
		self.tabWidget_3.setObjectName(_fromUtf8("tabWidget_3"))
		self.tab = QtGui.QWidget()
		self.tab.setObjectName(_fromUtf8("tab"))
		self.verticalLayout_10 = QtGui.QVBoxLayout(self.tab)
		self.verticalLayout_10.setObjectName(_fromUtf8("verticalLayout_10"))
		self.line_11 = QtGui.QFrame(self.tab)
		self.line_11.setFrameShape(QtGui.QFrame.HLine)
		self.line_11.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_11.setObjectName(_fromUtf8("line_11"))
		self.verticalLayout_10.addWidget(self.line_11)

 ################################################################################

		# self.plottingArea = QtGui.QWidget(self.tab_2)
 #1		
		# self.plottingArea = pg.PlotWidget(background=[255,255,255,0])
		# self.plottingArea.setDownsampling(mode='peak')
		# self.plottingArea.setClipToView(True)
		# self.plottingArea.setRange(xRange=[-100, 0])
		# self.plottingArea.setLimits(xMax=0)
		# self.plottingArea.setObjectName(_fromUtf8("plottingArea"))
		# self.curve = self.plottingArea.plot()
		# self.data = np.empty(100)
		# self.ptr = 0

 #2
		self.plottingArea1 = pg.GraphicsWindow()
		pg.setConfigOptions(antialias=True)

		self.plot1 = self.plottingArea1.addPlot(title="Walknut Data",background=[0,0,0,0])
		self.plot1.showGrid(x=True, y=True)
		self.plot1.setLabel('left', "Amplitude", units='A')
		self.plot1.setLabel('bottom', "Time", units='s')

		#self.data = np.random.normal(size=300)

		self.plot_error = [0 for x in range(300)]
		self.plot_integral = [0 for x in range(300)]
		self.plot_derivative = [0 for x in range(300)]
		self.plot_leftMotor = [0 for x in range(300)]
		self.plot_rightMotor = [0 for x in range(300)]

		self.curve1 = self.plot1.plot(self.plot_error, pen=(255,0,0), name="Error curve")
		self.curve2 = self.plot1.plot(self.plot_integral, pen=(0,255,0), name="Integral curve")
		self.curve3 = self.plot1.plot(self.plot_derivative, pen=(0,0,255), name="Derivative curve")
		self.curve4 = self.plot1.plot(self.plot_leftMotor, pen=(255,255,0), name="Leftmotor curve")
		self.curve5 = self.plot1.plot(self.plot_rightMotor, pen=(0,255,255), name="Rightmotor curve")
		self.ptr = 0

		self.verticalLayout_10.addWidget(self.plottingArea1)
 ################################################################################

 #3
		# self.pen1 = QtGui.QPen()
		# self.pen1.setColor(QtGui.QColor(255,0,0))
		# self.pen1.setWidth(.7)
		# self.pen1.setStyle(QtCore.Qt.DashLine)
		# self.pen2 = QtGui.QPen()
		# self.pen2.setColor(QtGui.QColor(0,255,0))
		# self.pen2.setWidth(.7)
		# self.pen2.setStyle(QtCore.Qt.DashLine)
		# self.pen3 = QtGui.QPen()
		# self.pen3.setColor(QtGui.QColor(0,0,255))
		# self.pen3.setWidth(.7)
		# self.pen3.setStyle(QtCore.Qt.DashLine)

		# self.plottingWidget = pg.PlotWidget(background=[0,0,0,0])
		# self.plottingWidget.setDownsampling(mode='peak')
		# self.plottingWidget.setClipToView(True)
		# self.plottingWidget.setRange(xRange=[-100, 0])
		# self.plottingWidget.setLimits(xMax=0)
		# self.plottingWidget.setObjectName(_fromUtf8("plottingWidget"))

		# #self.data = np.empty(100)
		# self.data = np.random.normal(size=300)
		# self.curve1 = self.plottingWidget.plotItem.plot(self.data+10,pen=(255,0,0))
		# # self.curve2 = self.plottingWidget.plotItem.plot(self.data+05,pen=(0,255,0))
		# # self.curve3 = self.plottingWidget.plotItem.plot(self.data+10,pen=(0,0,255))
		
		# self.ptr = 0

		# self.verticalLayout_3.addWidget(self.plottingWidget)

 ################################################################################

		icon5 = QtGui.QIcon()
		icon5.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/Control-Panel-icon (1).png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		self.tabWidget_3.addTab(self.tab, icon5, _fromUtf8(""))
		self.tab_7 = QtGui.QWidget()
		self.tab_7.setObjectName(_fromUtf8("tab_7"))
		self.verticalLayout_11 = QtGui.QVBoxLayout(self.tab_7)
		self.verticalLayout_11.setObjectName(_fromUtf8("verticalLayout_11"))
		self.line_18 = QtGui.QFrame(self.tab_7)
		self.line_18.setFrameShape(QtGui.QFrame.HLine)
		self.line_18.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_18.setObjectName(_fromUtf8("line_18"))
		self.verticalLayout_11.addWidget(self.line_18)


		#self.plottingArea2 = QtGui.QWidget(self.tab_7)
		self.plottingArea2 = pg.GraphicsWindow()
		self.plot2 = self.plottingArea2.addPlot(title="ADC")
		self.plot2.showGrid(x=True, y=True)
		self.plot2.setLabel('left', "Percentage", units='%')
		self.plot2.setLabel('bottom', "Left To Right Sensor", units='Nr')

		self.stm_r_adcData = [0 for x in range(12)]	

		self.curve6 = self.plot2.plot(self.stm_r_adcData, pen=(200,200,200), symbolBrush=(255,0,0), symbolPen='w')
		self.plottingArea2.setObjectName(_fromUtf8("plottingArea2"))
		self.verticalLayout_11.addWidget(self.plottingArea2)


 ################################################################################

		icon6 = QtGui.QIcon()
		icon6.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/lightning-icon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		self.tabWidget_3.addTab(self.tab_7, icon6, _fromUtf8(""))
		self.tab_8 = QtGui.QWidget()
		self.tab_8.setObjectName(_fromUtf8("tab_8"))
		self.verticalLayout_14 = QtGui.QVBoxLayout(self.tab_8)
		self.verticalLayout_14.setObjectName(_fromUtf8("verticalLayout_14"))
		self.line_20 = QtGui.QFrame(self.tab_8)
		self.line_20.setFrameShape(QtGui.QFrame.HLine)
		self.line_20.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_20.setObjectName(_fromUtf8("line_20"))
		self.verticalLayout_14.addWidget(self.line_20)
		self.verticalLayout_13 = QtGui.QVBoxLayout()
		self.verticalLayout_13.setObjectName(_fromUtf8("verticalLayout_13"))
		self.horizontalLayout_11 = QtGui.QHBoxLayout()
		self.horizontalLayout_11.setObjectName(_fromUtf8("horizontalLayout_11"))
		self.gridLayout_13 = QtGui.QGridLayout()
		self.gridLayout_13.setObjectName(_fromUtf8("gridLayout_13"))
		self.label_adc7 = QtGui.QLabel(self.tab_8)
		self.label_adc7.setObjectName(_fromUtf8("label_adc7"))
		self.gridLayout_13.addWidget(self.label_adc7, 6, 1, 1, 1)
		self.label_adc6 = QtGui.QLabel(self.tab_8)
		self.label_adc6.setObjectName(_fromUtf8("label_adc6"))
		self.gridLayout_13.addWidget(self.label_adc6, 5, 1, 1, 1)
		self.label_24 = QtGui.QLabel(self.tab_8)
		self.label_24.setObjectName(_fromUtf8("label_24"))
		self.gridLayout_13.addWidget(self.label_24, 0, 0, 1, 1)
		self.label_adc11 = QtGui.QLabel(self.tab_8)
		self.label_adc11.setObjectName(_fromUtf8("label_adc11"))
		self.gridLayout_13.addWidget(self.label_adc11, 10, 1, 1, 1)
		self.label_adc12 = QtGui.QLabel(self.tab_8)
		self.label_adc12.setObjectName(_fromUtf8("label_adc12"))
		self.gridLayout_13.addWidget(self.label_adc12, 11, 1, 1, 1)
		self.label_adc10 = QtGui.QLabel(self.tab_8)
		self.label_adc10.setObjectName(_fromUtf8("label_adc10"))
		self.gridLayout_13.addWidget(self.label_adc10, 9, 1, 1, 1)
		self.label_adc8 = QtGui.QLabel(self.tab_8)
		self.label_adc8.setObjectName(_fromUtf8("label_adc8"))
		self.gridLayout_13.addWidget(self.label_adc8, 7, 1, 1, 1)
		self.label_adc4 = QtGui.QLabel(self.tab_8)
		self.label_adc4.setObjectName(_fromUtf8("label_adc4"))
		self.gridLayout_13.addWidget(self.label_adc4, 3, 1, 1, 1)
		self.label_adc2 = QtGui.QLabel(self.tab_8)
		self.label_adc2.setObjectName(_fromUtf8("label_adc2"))
		self.gridLayout_13.addWidget(self.label_adc2, 1, 1, 1, 1)
		self.label_adc14 = QtGui.QLabel(self.tab_8)
		self.label_adc14.setObjectName(_fromUtf8("label_adc14"))
		self.gridLayout_13.addWidget(self.label_adc14, 13, 1, 1, 1)
		self.label_adc13 = QtGui.QLabel(self.tab_8)
		self.label_adc13.setObjectName(_fromUtf8("label_adc13"))
		self.gridLayout_13.addWidget(self.label_adc13, 12, 1, 1, 1)
		self.label_adc3 = QtGui.QLabel(self.tab_8)
		self.label_adc3.setObjectName(_fromUtf8("label_adc3"))
		self.gridLayout_13.addWidget(self.label_adc3, 2, 1, 1, 1)
		self.label_37 = QtGui.QLabel(self.tab_8)
		self.label_37.setObjectName(_fromUtf8("label_37"))
		self.gridLayout_13.addWidget(self.label_37, 12, 0, 1, 1)
		self.label_38 = QtGui.QLabel(self.tab_8)
		self.label_38.setObjectName(_fromUtf8("label_38"))
		self.gridLayout_13.addWidget(self.label_38, 13, 0, 1, 1)
		self.label_adc5 = QtGui.QLabel(self.tab_8)
		self.label_adc5.setObjectName(_fromUtf8("label_adc5"))
		self.gridLayout_13.addWidget(self.label_adc5, 4, 1, 1, 1)
		self.label_36 = QtGui.QLabel(self.tab_8)
		self.label_36.setObjectName(_fromUtf8("label_36"))
		self.gridLayout_13.addWidget(self.label_36, 11, 0, 1, 1)
		self.label_29 = QtGui.QLabel(self.tab_8)
		self.label_29.setObjectName(_fromUtf8("label_29"))
		self.gridLayout_13.addWidget(self.label_29, 4, 0, 1, 1)
		self.label_25 = QtGui.QLabel(self.tab_8)
		self.label_25.setObjectName(_fromUtf8("label_25"))
		self.gridLayout_13.addWidget(self.label_25, 1, 0, 1, 1)
		self.label_26 = QtGui.QLabel(self.tab_8)
		self.label_26.setObjectName(_fromUtf8("label_26"))
		self.gridLayout_13.addWidget(self.label_26, 2, 0, 1, 1)
		self.label_27 = QtGui.QLabel(self.tab_8)
		self.label_27.setObjectName(_fromUtf8("label_27"))
		self.gridLayout_13.addWidget(self.label_27, 3, 0, 1, 1)
		self.label_adc9 = QtGui.QLabel(self.tab_8)
		self.label_adc9.setObjectName(_fromUtf8("label_adc9"))
		self.gridLayout_13.addWidget(self.label_adc9, 8, 1, 1, 1)
		self.label_32 = QtGui.QLabel(self.tab_8)
		self.label_32.setObjectName(_fromUtf8("label_32"))
		self.gridLayout_13.addWidget(self.label_32, 7, 0, 1, 1)
		self.label_31 = QtGui.QLabel(self.tab_8)
		self.label_31.setObjectName(_fromUtf8("label_31"))
		self.gridLayout_13.addWidget(self.label_31, 6, 0, 1, 1)
		self.label_30 = QtGui.QLabel(self.tab_8)
		self.label_30.setObjectName(_fromUtf8("label_30"))
		self.gridLayout_13.addWidget(self.label_30, 5, 0, 1, 1)
		self.label_33 = QtGui.QLabel(self.tab_8)
		self.label_33.setObjectName(_fromUtf8("label_33"))
		self.gridLayout_13.addWidget(self.label_33, 8, 0, 1, 1)
		self.label_35 = QtGui.QLabel(self.tab_8)
		self.label_35.setObjectName(_fromUtf8("label_35"))
		self.gridLayout_13.addWidget(self.label_35, 10, 0, 1, 1)
		self.label_34 = QtGui.QLabel(self.tab_8)
		self.label_34.setObjectName(_fromUtf8("label_34"))
		self.gridLayout_13.addWidget(self.label_34, 9, 0, 1, 1)
		self.label_adc1 = QtGui.QLabel(self.tab_8)
		self.label_adc1.setObjectName(_fromUtf8("label_adc1"))
		self.gridLayout_13.addWidget(self.label_adc1, 0, 1, 1, 1)
		self.horizontalLayout_11.addLayout(self.gridLayout_13)
		self.gridLayout_14 = QtGui.QGridLayout()
		self.gridLayout_14.setObjectName(_fromUtf8("gridLayout_14"))
		self.label_54 = QtGui.QLabel(self.tab_8)
		self.label_54.setObjectName(_fromUtf8("label_54"))
		self.gridLayout_14.addWidget(self.label_54, 1, 0, 1, 1)
		self.label_kp = QtGui.QLabel(self.tab_8)
		self.label_kp.setObjectName(_fromUtf8("label_kp"))
		self.gridLayout_14.addWidget(self.label_kp, 0, 1, 1, 1)
		self.label_lpwm = QtGui.QLabel(self.tab_8)
		self.label_lpwm.setObjectName(_fromUtf8("label_lpwm"))
		self.gridLayout_14.addWidget(self.label_lpwm, 7, 1, 1, 1)
		self.label_60 = QtGui.QLabel(self.tab_8)
		self.label_60.setObjectName(_fromUtf8("label_60"))
		self.gridLayout_14.addWidget(self.label_60, 7, 0, 1, 1)
		self.label_53 = QtGui.QLabel(self.tab_8)
		self.label_53.setObjectName(_fromUtf8("label_53"))
		self.gridLayout_14.addWidget(self.label_53, 0, 0, 1, 1)
		self.label_lw = QtGui.QLabel(self.tab_8)
		self.label_lw.setObjectName(_fromUtf8("label_lw"))
		self.gridLayout_14.addWidget(self.label_lw, 10, 1, 1, 1)
		self.label_rpwm = QtGui.QLabel(self.tab_8)
		self.label_rpwm.setObjectName(_fromUtf8("label_rpwm"))
		self.gridLayout_14.addWidget(self.label_rpwm, 8, 1, 1, 1)
		self.label_63 = QtGui.QLabel(self.tab_8)
		self.label_63.setObjectName(_fromUtf8("label_63"))
		self.gridLayout_14.addWidget(self.label_63, 11, 0, 1, 1)
		self.label_55 = QtGui.QLabel(self.tab_8)
		self.label_55.setObjectName(_fromUtf8("label_55"))
		self.gridLayout_14.addWidget(self.label_55, 2, 0, 1, 1)
		self.label_kd = QtGui.QLabel(self.tab_8)
		self.label_kd.setObjectName(_fromUtf8("label_kd"))
		self.gridLayout_14.addWidget(self.label_kd, 2, 1, 1, 1)
		self.label_56 = QtGui.QLabel(self.tab_8)
		self.label_56.setObjectName(_fromUtf8("label_56"))
		self.gridLayout_14.addWidget(self.label_56, 3, 0, 1, 1)
		self.label_61 = QtGui.QLabel(self.tab_8)
		self.label_61.setObjectName(_fromUtf8("label_61"))
		self.gridLayout_14.addWidget(self.label_61, 8, 0, 1, 1)
		self.label_58 = QtGui.QLabel(self.tab_8)
		self.label_58.setObjectName(_fromUtf8("label_58"))
		self.gridLayout_14.addWidget(self.label_58, 5, 0, 1, 1)
		self.label_error = QtGui.QLabel(self.tab_8)
		self.label_error.setObjectName(_fromUtf8("label_error"))
		self.gridLayout_14.addWidget(self.label_error, 4, 1, 1, 1)
		self.label_virt = QtGui.QLabel(self.tab_8)
		self.label_virt.setObjectName(_fromUtf8("label_virt"))
		self.gridLayout_14.addWidget(self.label_virt, 3, 1, 1, 1)
		self.label_57 = QtGui.QLabel(self.tab_8)
		self.label_57.setObjectName(_fromUtf8("label_57"))
		self.gridLayout_14.addWidget(self.label_57, 4, 0, 1, 1)
		self.label_62 = QtGui.QLabel(self.tab_8)
		self.label_62.setObjectName(_fromUtf8("label_62"))
		self.gridLayout_14.addWidget(self.label_62, 10, 0, 1, 1)
		self.label_der = QtGui.QLabel(self.tab_8)
		self.label_der.setObjectName(_fromUtf8("label_der"))
		self.gridLayout_14.addWidget(self.label_der, 6, 1, 1, 1)
		self.label_int = QtGui.QLabel(self.tab_8)
		self.label_int.setObjectName(_fromUtf8("label_int"))
		self.gridLayout_14.addWidget(self.label_int, 5, 1, 1, 1)
		self.label_ki = QtGui.QLabel(self.tab_8)
		self.label_ki.setObjectName(_fromUtf8("label_ki"))
		self.gridLayout_14.addWidget(self.label_ki, 1, 1, 1, 1)
		self.label_speed = QtGui.QLabel(self.tab_8)
		self.label_speed.setObjectName(_fromUtf8("label_speed"))
		self.gridLayout_14.addWidget(self.label_speed, 12, 1, 1, 1)
		self.label_rw = QtGui.QLabel(self.tab_8)
		self.label_rw.setObjectName(_fromUtf8("label_rw"))
		self.gridLayout_14.addWidget(self.label_rw, 11, 1, 1, 1)
		self.label_59 = QtGui.QLabel(self.tab_8)
		self.label_59.setObjectName(_fromUtf8("label_59"))
		self.gridLayout_14.addWidget(self.label_59, 6, 0, 1, 1)
		self.label_77 = QtGui.QLabel(self.tab_8)
		self.label_77.setObjectName(_fromUtf8("label_77"))
		self.gridLayout_14.addWidget(self.label_77, 12, 0, 1, 1)
		self.label_64 = QtGui.QLabel(self.tab_8)
		self.label_64.setObjectName(_fromUtf8("label_64"))
		self.gridLayout_14.addWidget(self.label_64, 14, 0, 1, 1)
		self.label_78 = QtGui.QLabel(self.tab_8)
		self.label_78.setObjectName(_fromUtf8("label_78"))
		self.gridLayout_14.addWidget(self.label_78, 14, 1, 1, 1)
		self.horizontalLayout_11.addLayout(self.gridLayout_14)
		self.verticalLayout_13.addLayout(self.horizontalLayout_11)
		self.verticalLayout_14.addLayout(self.verticalLayout_13)
		spacerItem1 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
		self.verticalLayout_14.addItem(spacerItem1)
		icon7 = QtGui.QIcon()
		icon7.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/database-icon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		self.tabWidget_3.addTab(self.tab_8, icon7, _fromUtf8(""))
		self.verticalLayout_3.addWidget(self.tabWidget_3)
		icon8 = QtGui.QIcon()
		icon8.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/graph-icon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		self.tabWidget_1.addTab(self.tab_2, icon8, _fromUtf8(""))
		self.tab_1 = QtGui.QWidget()
		self.tab_1.setObjectName(_fromUtf8("tab_1"))
		self.gridLayout_8 = QtGui.QGridLayout(self.tab_1)
		self.gridLayout_8.setObjectName(_fromUtf8("gridLayout_8"))
		self.line_6 = QtGui.QFrame(self.tab_1)
		self.line_6.setFrameShape(QtGui.QFrame.HLine)
		self.line_6.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_6.setObjectName(_fromUtf8("line_6"))
		self.gridLayout_8.addWidget(self.line_6, 2, 0, 1, 1)
		self.tabWidget_2 = QtGui.QTabWidget(self.tab_1)
		self.tabWidget_2.setToolTip(_fromUtf8("Reset PID"))
		self.tabWidget_2.setIconSize(QtCore.QSize(24, 24))
		self.tabWidget_2.setObjectName(_fromUtf8("tabWidget_2"))
		self.tab_4 = QtGui.QWidget()
		self.tab_4.setObjectName(_fromUtf8("tab_4"))
		self.gridLayout_3 = QtGui.QGridLayout(self.tab_4)
		self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
		self.line_2 = QtGui.QFrame(self.tab_4)
		self.line_2.setFrameShape(QtGui.QFrame.HLine)
		self.line_2.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_2.setObjectName(_fromUtf8("line_2"))
		self.gridLayout_3.addWidget(self.line_2, 6, 0, 1, 1)
		self.gridLayout = QtGui.QGridLayout()
		self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
		self.label_Kd = QtGui.QLabel(self.tab_4)
		self.label_Kd.setObjectName(_fromUtf8("label_Kd"))
		self.gridLayout.addWidget(self.label_Kd, 2, 0, 1, 1)
		self.label_Kp = QtGui.QLabel(self.tab_4)
		self.label_Kp.setObjectName(_fromUtf8("label_Kp"))
		self.gridLayout.addWidget(self.label_Kp, 0, 0, 1, 1)
		self.label_Ki = QtGui.QLabel(self.tab_4)
		self.label_Ki.setObjectName(_fromUtf8("label_Ki"))
		self.gridLayout.addWidget(self.label_Ki, 1, 0, 1, 1)
		spacerItem2 = QtGui.QSpacerItem(80, 80, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.gridLayout.addItem(spacerItem2, 0, 5, 3, 1)
		spacerItem3 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.gridLayout.addItem(spacerItem3, 1, 3, 1, 1)
		spacerItem4 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.gridLayout.addItem(spacerItem4, 1, 4, 1, 1)
		spacerItem5 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.gridLayout.addItem(spacerItem5, 1, 2, 1, 1)
		self.edit_Kp = QtGui.QLineEdit(self.tab_4)
		self.edit_Kp.setObjectName(_fromUtf8("edit_Kp"))
		self.gridLayout.addWidget(self.edit_Kp, 0, 1, 1, 1)
		self.edit_Ki = QtGui.QLineEdit(self.tab_4)
		self.edit_Ki.setObjectName(_fromUtf8("edit_Ki"))
		self.gridLayout.addWidget(self.edit_Ki, 1, 1, 1, 1)
		self.edit_Kd = QtGui.QLineEdit(self.tab_4)
		self.edit_Kd.setObjectName(_fromUtf8("edit_Kd"))
		self.gridLayout.addWidget(self.edit_Kd, 2, 1, 1, 1)
		self.gridLayout_3.addLayout(self.gridLayout, 1, 0, 1, 1)
		self.gridLayout_6 = QtGui.QGridLayout()
		self.gridLayout_6.setObjectName(_fromUtf8("gridLayout_6"))
		spacerItem6 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.gridLayout_6.addItem(spacerItem6, 0, 1, 1, 1)
		self.btn_PID_Apply = QtGui.QPushButton(self.tab_4)
		self.btn_PID_Apply.setObjectName(_fromUtf8("btn_PID_Apply"))
		self.gridLayout_6.addWidget(self.btn_PID_Apply, 0, 0, 1, 1)
		self.gridLayout_3.addLayout(self.gridLayout_6, 7, 0, 1, 1)
		self.gridLayout_7 = QtGui.QGridLayout()
		self.gridLayout_7.setObjectName(_fromUtf8("gridLayout_7"))
		self.label_6 = QtGui.QLabel(self.tab_4)
		self.label_6.setObjectName(_fromUtf8("label_6"))
		self.gridLayout_7.addWidget(self.label_6, 2, 0, 1, 1)
		self.label_4 = QtGui.QLabel(self.tab_4)
		self.label_4.setObjectName(_fromUtf8("label_4"))
		self.gridLayout_7.addWidget(self.label_4, 0, 0, 1, 1)
		spacerItem7 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.gridLayout_7.addItem(spacerItem7, 0, 5, 1, 1)
		spacerItem8 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.gridLayout_7.addItem(spacerItem8, 0, 4, 1, 1)
		spacerItem9 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.gridLayout_7.addItem(spacerItem9, 0, 2, 1, 1)
		self.edit_d_filter = QtGui.QLineEdit(self.tab_4)
		self.edit_d_filter.setObjectName(_fromUtf8("edit_d_filter"))
		self.gridLayout_7.addWidget(self.edit_d_filter, 2, 1, 1, 1)
		self.edit_i_filter = QtGui.QLineEdit(self.tab_4)
		self.edit_i_filter.setObjectName(_fromUtf8("edit_i_filter"))
		self.gridLayout_7.addWidget(self.edit_i_filter, 0, 1, 1, 1)
		spacerItem10 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.gridLayout_7.addItem(spacerItem10, 0, 3, 1, 1)
		self.gridLayout_3.addLayout(self.gridLayout_7, 3, 0, 1, 1)
		self.verticalLayout_6 = QtGui.QVBoxLayout()
		self.verticalLayout_6.setObjectName(_fromUtf8("verticalLayout_6"))
		self.label_16 = QtGui.QLabel(self.tab_4)
		self.label_16.setObjectName(_fromUtf8("label_16"))
		self.verticalLayout_6.addWidget(self.label_16)
		self.label_9 = QtGui.QLabel(self.tab_4)
		self.label_9.setObjectName(_fromUtf8("label_9"))
		self.verticalLayout_6.addWidget(self.label_9)
		self.label_10 = QtGui.QLabel(self.tab_4)
		self.label_10.setObjectName(_fromUtf8("label_10"))
		self.verticalLayout_6.addWidget(self.label_10)
		spacerItem11 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
		self.verticalLayout_6.addItem(spacerItem11)
		self.gridLayout_3.addLayout(self.verticalLayout_6, 5, 0, 1, 1)
		self.line = QtGui.QFrame(self.tab_4)
		self.line.setFrameShape(QtGui.QFrame.HLine)
		self.line.setFrameShadow(QtGui.QFrame.Sunken)
		self.line.setObjectName(_fromUtf8("line"))
		self.gridLayout_3.addWidget(self.line, 2, 0, 1, 1)
		self.line_10 = QtGui.QFrame(self.tab_4)
		self.line_10.setFrameShape(QtGui.QFrame.HLine)
		self.line_10.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_10.setObjectName(_fromUtf8("line_10"))
		self.gridLayout_3.addWidget(self.line_10, 4, 0, 1, 1)
		self.line_14 = QtGui.QFrame(self.tab_4)
		self.line_14.setFrameShape(QtGui.QFrame.HLine)
		self.line_14.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_14.setObjectName(_fromUtf8("line_14"))
		self.gridLayout_3.addWidget(self.line_14, 0, 0, 1, 1)
		icon9 = QtGui.QIcon()
		icon9.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/Control-Panel-icon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		self.tabWidget_2.addTab(self.tab_4, icon9, _fromUtf8(""))
		self.tab_3 = QtGui.QWidget()
		self.tab_3.setObjectName(_fromUtf8("tab_3"))
		self.verticalLayout_4 = QtGui.QVBoxLayout(self.tab_3)
		self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
		self.line_13 = QtGui.QFrame(self.tab_3)
		self.line_13.setFrameShape(QtGui.QFrame.HLine)
		self.line_13.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_13.setObjectName(_fromUtf8("line_13"))
		self.verticalLayout_4.addWidget(self.line_13)
		self.label_28 = QtGui.QLabel(self.tab_3)
		self.label_28.setObjectName(_fromUtf8("label_28"))
		self.verticalLayout_4.addWidget(self.label_28)
		self.label_5 = QtGui.QLabel(self.tab_3)
		self.label_5.setObjectName(_fromUtf8("label_5"))
		self.verticalLayout_4.addWidget(self.label_5)
		self.line_5 = QtGui.QFrame(self.tab_3)
		self.line_5.setFrameShape(QtGui.QFrame.HLine)
		self.line_5.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_5.setObjectName(_fromUtf8("line_5"))
		self.verticalLayout_4.addWidget(self.line_5)
		self.gridLayout_2 = QtGui.QGridLayout()
		self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
		self.edit_ADC5 = QtGui.QLineEdit(self.tab_3)
		self.edit_ADC5.setObjectName(_fromUtf8("edit_ADC5"))
		self.gridLayout_2.addWidget(self.edit_ADC5, 7, 3, 1, 1)
		self.label_20 = QtGui.QLabel(self.tab_3)
		self.label_20.setObjectName(_fromUtf8("label_20"))
		self.gridLayout_2.addWidget(self.label_20, 9, 0, 1, 1)
		self.label_19 = QtGui.QLabel(self.tab_3)
		self.label_19.setObjectName(_fromUtf8("label_19"))
		self.gridLayout_2.addWidget(self.label_19, 8, 0, 1, 1)
		self.label_14 = QtGui.QLabel(self.tab_3)
		self.label_14.setObjectName(_fromUtf8("label_14"))
		self.gridLayout_2.addWidget(self.label_14, 2, 0, 1, 1)
		self.label_12 = QtGui.QLabel(self.tab_3)
		self.label_12.setObjectName(_fromUtf8("label_12"))
		self.gridLayout_2.addWidget(self.label_12, 5, 0, 1, 1)
		self.label_11 = QtGui.QLabel(self.tab_3)
		self.label_11.setObjectName(_fromUtf8("label_11"))
		self.gridLayout_2.addWidget(self.label_11, 3, 0, 1, 1)
		self.label_13 = QtGui.QLabel(self.tab_3)
		self.label_13.setObjectName(_fromUtf8("label_13"))
		self.gridLayout_2.addWidget(self.label_13, 4, 0, 1, 1)
		self.label_18 = QtGui.QLabel(self.tab_3)
		self.label_18.setObjectName(_fromUtf8("label_18"))
		self.gridLayout_2.addWidget(self.label_18, 7, 0, 1, 1)
		self.label_17 = QtGui.QLabel(self.tab_3)
		self.label_17.setObjectName(_fromUtf8("label_17"))
		self.gridLayout_2.addWidget(self.label_17, 6, 0, 1, 1)
		self.label_15 = QtGui.QLabel(self.tab_3)
		self.label_15.setObjectName(_fromUtf8("label_15"))
		self.gridLayout_2.addWidget(self.label_15, 2, 3, 1, 1)
		spacerItem12 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.gridLayout_2.addItem(spacerItem12, 2, 6, 1, 1)
		self.edit_ADC1 = QtGui.QLineEdit(self.tab_3)
		self.edit_ADC1.setObjectName(_fromUtf8("edit_ADC1"))
		self.gridLayout_2.addWidget(self.edit_ADC1, 3, 3, 1, 1)
		self.edit_ADC2 = QtGui.QLineEdit(self.tab_3)
		self.edit_ADC2.setObjectName(_fromUtf8("edit_ADC2"))
		self.gridLayout_2.addWidget(self.edit_ADC2, 4, 3, 1, 1)
		self.edit_ADC7 = QtGui.QLineEdit(self.tab_3)
		self.edit_ADC7.setObjectName(_fromUtf8("edit_ADC7"))
		self.gridLayout_2.addWidget(self.edit_ADC7, 9, 3, 1, 1)
		spacerItem13 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.gridLayout_2.addItem(spacerItem13, 2, 4, 1, 1)
		self.edit_ADC6 = QtGui.QLineEdit(self.tab_3)
		self.edit_ADC6.setObjectName(_fromUtf8("edit_ADC6"))
		self.gridLayout_2.addWidget(self.edit_ADC6, 8, 3, 1, 1)
		self.edit_ADC4 = QtGui.QLineEdit(self.tab_3)
		self.edit_ADC4.setObjectName(_fromUtf8("edit_ADC4"))
		self.gridLayout_2.addWidget(self.edit_ADC4, 6, 3, 1, 1)
		spacerItem14 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.gridLayout_2.addItem(spacerItem14, 2, 7, 1, 1)
		spacerItem15 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.gridLayout_2.addItem(spacerItem15, 2, 5, 1, 1)
		self.edit_ADC3 = QtGui.QLineEdit(self.tab_3)
		self.edit_ADC3.setObjectName(_fromUtf8("edit_ADC3"))
		self.gridLayout_2.addWidget(self.edit_ADC3, 5, 3, 1, 1)
		self.verticalLayout_4.addLayout(self.gridLayout_2)
		self.horizontalLayout_4 = QtGui.QHBoxLayout()
		self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
		self.label_3 = QtGui.QLabel(self.tab_3)
		self.label_3.setObjectName(_fromUtf8("label_3"))
		self.horizontalLayout_4.addWidget(self.label_3)
		self.edit_nrOfSamples = QtGui.QLineEdit(self.tab_3)
		self.edit_nrOfSamples.setObjectName(_fromUtf8("edit_nrOfSamples"))
		self.horizontalLayout_4.addWidget(self.edit_nrOfSamples)
		spacerItem16 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_4.addItem(spacerItem16)
		spacerItem17 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_4.addItem(spacerItem17)
		spacerItem18 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_4.addItem(spacerItem18)
		spacerItem19 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_4.addItem(spacerItem19)
		self.verticalLayout_4.addLayout(self.horizontalLayout_4)
		spacerItem20 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
		self.verticalLayout_4.addItem(spacerItem20)
		self.line_9 = QtGui.QFrame(self.tab_3)
		self.line_9.setFrameShape(QtGui.QFrame.HLine)
		self.line_9.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_9.setObjectName(_fromUtf8("line_9"))
		self.verticalLayout_4.addWidget(self.line_9)
		self.horizontalLayout_2 = QtGui.QHBoxLayout()
		self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
		self.btn_SensApply = QtGui.QPushButton(self.tab_3)
		self.btn_SensApply.setObjectName(_fromUtf8("btn_SensApply"))
		self.horizontalLayout_2.addWidget(self.btn_SensApply)
		spacerItem21 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_2.addItem(spacerItem21)
		self.verticalLayout_4.addLayout(self.horizontalLayout_2)
		icon10 = QtGui.QIcon()
		icon10.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/radar-icon (1).png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		self.tabWidget_2.addTab(self.tab_3, icon10, _fromUtf8(""))
		self.tab_5 = QtGui.QWidget()
		self.tab_5.setObjectName(_fromUtf8("tab_5"))
		self.gridLayout_9 = QtGui.QGridLayout(self.tab_5)
		self.gridLayout_9.setObjectName(_fromUtf8("gridLayout_9"))
		self.label_7 = QtGui.QLabel(self.tab_5)
		self.label_7.setObjectName(_fromUtf8("label_7"))
		self.gridLayout_9.addWidget(self.label_7, 4, 0, 1, 1)
		spacerItem22 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
		self.gridLayout_9.addItem(spacerItem22, 6, 0, 1, 1)
		self.horizontalLayout_9 = QtGui.QHBoxLayout()
		self.horizontalLayout_9.setObjectName(_fromUtf8("horizontalLayout_9"))
		self.btn_MotorApply = QtGui.QPushButton(self.tab_5)
		self.btn_MotorApply.setObjectName(_fromUtf8("btn_MotorApply"))
		self.horizontalLayout_9.addWidget(self.btn_MotorApply)
		spacerItem23 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_9.addItem(spacerItem23)
		self.gridLayout_9.addLayout(self.horizontalLayout_9, 8, 0, 1, 1)
		self.line_7 = QtGui.QFrame(self.tab_5)
		self.line_7.setFrameShape(QtGui.QFrame.HLine)
		self.line_7.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_7.setObjectName(_fromUtf8("line_7"))
		self.gridLayout_9.addWidget(self.line_7, 3, 0, 1, 1)
		self.label = QtGui.QLabel(self.tab_5)
		self.label.setObjectName(_fromUtf8("label"))
		self.gridLayout_9.addWidget(self.label, 1, 0, 1, 1)
		self.horizontalLayout_8 = QtGui.QHBoxLayout()
		self.horizontalLayout_8.setObjectName(_fromUtf8("horizontalLayout_8"))
		self.label_8 = QtGui.QLabel(self.tab_5)
		self.label_8.setObjectName(_fromUtf8("label_8"))
		self.horizontalLayout_8.addWidget(self.label_8)
		self.edit_PWM2 = QtGui.QLineEdit(self.tab_5)
		self.edit_PWM2.setObjectName(_fromUtf8("edit_PWM2"))
		self.horizontalLayout_8.addWidget(self.edit_PWM2)
		spacerItem24 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_8.addItem(spacerItem24)
		spacerItem25 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_8.addItem(spacerItem25)
		spacerItem26 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_8.addItem(spacerItem26)
		spacerItem27 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_8.addItem(spacerItem27)
		spacerItem28 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_8.addItem(spacerItem28)
		self.gridLayout_9.addLayout(self.horizontalLayout_8, 5, 0, 1, 1)
		self.horizontalLayout_5 = QtGui.QHBoxLayout()
		self.horizontalLayout_5.setObjectName(_fromUtf8("horizontalLayout_5"))
		self.label_2 = QtGui.QLabel(self.tab_5)
		self.label_2.setObjectName(_fromUtf8("label_2"))
		self.horizontalLayout_5.addWidget(self.label_2)
		self.edit_PWM1 = QtGui.QLineEdit(self.tab_5)
		self.edit_PWM1.setObjectName(_fromUtf8("edit_PWM1"))
		self.horizontalLayout_5.addWidget(self.edit_PWM1)
		spacerItem29 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_5.addItem(spacerItem29)
		spacerItem30 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_5.addItem(spacerItem30)
		spacerItem31 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_5.addItem(spacerItem31)
		spacerItem32 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_5.addItem(spacerItem32)
		spacerItem33 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
		self.horizontalLayout_5.addItem(spacerItem33)
		self.gridLayout_9.addLayout(self.horizontalLayout_5, 2, 0, 1, 1)
		self.line_8 = QtGui.QFrame(self.tab_5)
		self.line_8.setFrameShape(QtGui.QFrame.HLine)
		self.line_8.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_8.setObjectName(_fromUtf8("line_8"))
		self.gridLayout_9.addWidget(self.line_8, 7, 0, 1, 1)
		self.line_12 = QtGui.QFrame(self.tab_5)
		self.line_12.setFrameShape(QtGui.QFrame.HLine)
		self.line_12.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_12.setObjectName(_fromUtf8("line_12"))
		self.gridLayout_9.addWidget(self.line_12, 0, 0, 1, 1)
		icon11 = QtGui.QIcon()
		icon11.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/Transport-Piston-icon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		self.tabWidget_2.addTab(self.tab_5, icon11, _fromUtf8(""))
		self.tab_6 = QtGui.QWidget()
		self.tab_6.setObjectName(_fromUtf8("tab_6"))
		self.verticalLayout_9 = QtGui.QVBoxLayout(self.tab_6)
		self.verticalLayout_9.setObjectName(_fromUtf8("verticalLayout_9"))
		self.line_17 = QtGui.QFrame(self.tab_6)
		self.line_17.setFrameShape(QtGui.QFrame.HLine)
		self.line_17.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_17.setObjectName(_fromUtf8("line_17"))
		self.verticalLayout_9.addWidget(self.line_17)
		self.verticalLayout_8 = QtGui.QVBoxLayout()
		self.verticalLayout_8.setObjectName(_fromUtf8("verticalLayout_8"))
		self.btn_findDevice = QtGui.QPushButton(self.tab_6)
		self.btn_findDevice.setObjectName(_fromUtf8("btn_findDevice"))
		self.verticalLayout_8.addWidget(self.btn_findDevice)
		self.listWidget = QtGui.QListWidget(self.tab_6)
		self.listWidget.setObjectName(_fromUtf8("listWidget"))
		self.verticalLayout_8.addWidget(self.listWidget)
		self.btn_UseSel = QtGui.QPushButton(self.tab_6)
		self.btn_UseSel.setObjectName(_fromUtf8("btn_UseSel"))
		self.verticalLayout_8.addWidget(self.btn_UseSel)
		self.verticalLayout_9.addLayout(self.verticalLayout_8)
		icon12 = QtGui.QIcon()
		icon12.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/Apps-Bluetooth-Active-icon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		self.tabWidget_2.addTab(self.tab_6, icon12, _fromUtf8(""))
		self.gridLayout_8.addWidget(self.tabWidget_2, 1, 0, 1, 1)
		self.line_15 = QtGui.QFrame(self.tab_1)
		self.line_15.setFrameShape(QtGui.QFrame.HLine)
		self.line_15.setFrameShadow(QtGui.QFrame.Sunken)
		self.line_15.setObjectName(_fromUtf8("line_15"))
		self.gridLayout_8.addWidget(self.line_15, 0, 0, 1, 1)
		icon13 = QtGui.QIcon()
		icon13.addPixmap(QtGui.QPixmap(_fromUtf8("/home/akerlund/QT4Designer/Icons/Settings-icon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		self.tabWidget_1.addTab(self.tab_1, icon13, _fromUtf8(""))
		self.verticalLayout.addWidget(self.tabWidget_1)
		MainWindow.setCentralWidget(self.centralwidget)
		self.menubar = QtGui.QMenuBar(MainWindow)
		self.menubar.setGeometry(QtCore.QRect(0, 0, 989, 25))
		self.menubar.setNativeMenuBar(False)
		self.menubar.setObjectName(_fromUtf8("menubar"))
		self.menuFile = QtGui.QMenu(self.menubar)
		self.menuFile.setObjectName(_fromUtf8("menuFile"))
		self.menuSave_Settings = QtGui.QMenu(self.menuFile)
		self.menuSave_Settings.setObjectName(_fromUtf8("menuSave_Settings"))
		self.menuAbout = QtGui.QMenu(self.menubar)
		self.menuAbout.setObjectName(_fromUtf8("menuAbout"))
		MainWindow.setMenuBar(self.menubar)
		self.statusbar = QtGui.QStatusBar(MainWindow)
		self.statusbar.setObjectName(_fromUtf8("statusbar"))
		MainWindow.setStatusBar(self.statusbar)
		self.actionLoad_Settings = QtGui.QAction(MainWindow)
		self.actionLoad_Settings.setObjectName(_fromUtf8("actionLoad_Settings"))
		self.actionQuit = QtGui.QAction(MainWindow)
		self.actionQuit.setObjectName(_fromUtf8("actionQuit"))
		self.actionAbout = QtGui.QAction(MainWindow)
		self.actionAbout.setObjectName(_fromUtf8("actionAbout"))
		self.actionTo_File = QtGui.QAction(MainWindow)
		self.actionTo_File.setObjectName(_fromUtf8("actionTo_File"))
		self.actionSet_Current_As_Default = QtGui.QAction(MainWindow)
		self.actionSet_Current_As_Default.setObjectName(_fromUtf8("actionSet_Current_As_Default"))
		self.menuSave_Settings.addAction(self.actionTo_File)
		self.menuSave_Settings.addAction(self.actionSet_Current_As_Default)
		self.menuFile.addAction(self.menuSave_Settings.menuAction())
		self.menuFile.addAction(self.actionLoad_Settings)
		self.menuFile.addSeparator()
		self.menuFile.addAction(self.actionQuit)
		self.menuAbout.addAction(self.actionAbout)
		self.menubar.addAction(self.menuFile.menuAction())
		self.menubar.addAction(self.menuAbout.menuAction())

		self.retranslateUi(MainWindow)
		self.tabWidget_1.setCurrentIndex(0)
		self.tabWidget_2.setCurrentIndex(0)
		self.tabWidget_3.setCurrentIndex(0)
		QtCore.QObject.connect(self.actionQuit, QtCore.SIGNAL(_fromUtf8("activated()")), MainWindow.close)
		QtCore.QObject.connect(self.btn_stop, QtCore.SIGNAL(_fromUtf8("clicked()")), self.f_btn_Stop)
		QtCore.QObject.connect(self.btn_start, QtCore.SIGNAL(_fromUtf8("clicked()")), self.f_btn_Start)
		QtCore.QObject.connect(self.btn_reset, QtCore.SIGNAL(_fromUtf8("clicked()")), self.f_btn_Reset)
		QtCore.QObject.connect(self.btn_Calibrate, QtCore.SIGNAL(_fromUtf8("clicked()")), self.f_btn_Calibrate)
		QtCore.QObject.connect(self.btn_con, QtCore.SIGNAL(_fromUtf8("clicked()")), self.f_btn_Connect)
		QtCore.QObject.connect(self.box_motor, QtCore.SIGNAL(_fromUtf8("toggled(bool)")), self.f_box_Motor)
		QtCore.QObject.connect(self.box_usart, QtCore.SIGNAL(_fromUtf8("toggled(bool)")), self.f_box_Usart)
		QtCore.QObject.connect(self.btn_PID_Apply, QtCore.SIGNAL(_fromUtf8("clicked()")), self.f_btn_PID_Apply)
		QtCore.QObject.connect(self.btn_SensApply, QtCore.SIGNAL(_fromUtf8("clicked()")), self.f_btn_Sensors_Apply)
		QtCore.QObject.connect(self.btn_MotorApply, QtCore.SIGNAL(_fromUtf8("clicked()")), self.f_btn_Motors_Apply)
		QtCore.QObject.connect(self.actionTo_File, QtCore.SIGNAL(_fromUtf8("activated()")), self.f_SaveFile)
		QtCore.QObject.connect(self.actionLoad_Settings, QtCore.SIGNAL(_fromUtf8("activated()")), self.f_LoadFile)
		QtCore.QObject.connect(self.btn_findDevice, QtCore.SIGNAL(_fromUtf8("clicked()")), self.f_Find_Devices)
		QtCore.QObject.connect(self.btn_UseSel, QtCore.SIGNAL(_fromUtf8("clicked()")), self.f_Use_Selected)
		QtCore.QMetaObject.connectSlotsByName(MainWindow)


	def retranslateUi(self, MainWindow):
		MainWindow.setWindowTitle(_translate("MainWindow", "Walknut", None))
		self.btn_con.setToolTip(_translate("MainWindow", "<html><head/><body><p>Toggle Bluetooth Connection</p></body></html>", None))
		self.label_21.setToolTip(_translate("MainWindow", "<html><head/><body><p>Bluetooth Connection</p></body></html>", None))
		self.btn_start.setToolTip(_translate("MainWindow", "<html><head/><body><p>Start</p></body></html>", None))
		self.btn_stop.setToolTip(_translate("MainWindow", "<html><head/><body><p>Stop</p></body></html>", None))
		self.btn_reset.setToolTip(_translate("MainWindow", "<html><head/><body><p>Reset PID</p></body></html>", None))
		self.btn_Calibrate.setToolTip(_translate("MainWindow", "<html><head/><body><p>Calibrate On White Surface</p></body></html>", None))
		self.box_motor.setText(_translate("MainWindow", "Motors On", None))
		self.box_usart.setText(_translate("MainWindow", "USART On", None))
		self.tabWidget_3.setTabText(self.tabWidget_3.indexOf(self.tab), _translate("MainWindow", "Controller", None))
		self.tabWidget_3.setTabText(self.tabWidget_3.indexOf(self.tab_7), _translate("MainWindow", "Sensor Strip", None))
		self.label_adc7.setText(_translate("MainWindow", "0", None))
		self.label_adc6.setText(_translate("MainWindow", "0", None))
		self.label_24.setText(_translate("MainWindow", "ADC1", None))
		self.label_adc11.setText(_translate("MainWindow", "0", None))
		self.label_adc12.setText(_translate("MainWindow", "0", None))
		self.label_adc10.setText(_translate("MainWindow", "0", None))
		self.label_adc8.setText(_translate("MainWindow", "0", None))
		self.label_adc4.setText(_translate("MainWindow", "0", None))
		self.label_adc2.setText(_translate("MainWindow", "0", None))
		self.label_adc14.setText(_translate("MainWindow", "0", None))
		self.label_adc13.setText(_translate("MainWindow", "0", None))
		self.label_adc3.setText(_translate("MainWindow", "0", None))
		self.label_37.setText(_translate("MainWindow", "ADC13", None))
		self.label_38.setText(_translate("MainWindow", "ADC14", None))
		self.label_adc5.setText(_translate("MainWindow", "0", None))
		self.label_36.setText(_translate("MainWindow", "ADC12", None))
		self.label_29.setText(_translate("MainWindow", "ADC5", None))
		self.label_25.setText(_translate("MainWindow", "ADC2", None))
		self.label_26.setText(_translate("MainWindow", "ADC3", None))
		self.label_27.setText(_translate("MainWindow", "ADC4", None))
		self.label_adc9.setText(_translate("MainWindow", "0", None))
		self.label_32.setText(_translate("MainWindow", "ADC8", None))
		self.label_31.setText(_translate("MainWindow", "ADC7", None))
		self.label_30.setText(_translate("MainWindow", "ADC6", None))
		self.label_33.setText(_translate("MainWindow", "ADC9", None))
		self.label_35.setText(_translate("MainWindow", "ADC11", None))
		self.label_34.setText(_translate("MainWindow", "ADC10", None))
		self.label_adc1.setText(_translate("MainWindow", "0", None))
		self.label_54.setText(_translate("MainWindow", "Ki", None))
		self.label_kp.setText(_translate("MainWindow", "0", None))
		self.label_lpwm.setText(_translate("MainWindow", "0", None))
		self.label_60.setText(_translate("MainWindow", "LeftPWM", None))
		self.label_53.setText(_translate("MainWindow", "Kp", None))
		self.label_lw.setText(_translate("MainWindow", "0", None))
		self.label_rpwm.setText(_translate("MainWindow", "0", None))
		self.label_63.setText(_translate("MainWindow", "RightWheel", None))
		self.label_55.setText(_translate("MainWindow", "Kd", None))
		self.label_kd.setText(_translate("MainWindow", "0", None))
		self.label_56.setText(_translate("MainWindow", "Virtual", None))
		self.label_61.setText(_translate("MainWindow", "RightPWM", None))
		self.label_58.setText(_translate("MainWindow", "Integral", None))
		self.label_error.setText(_translate("MainWindow", "0", None))
		self.label_virt.setText(_translate("MainWindow", "0", None))
		self.label_57.setText(_translate("MainWindow", "Error", None))
		self.label_62.setText(_translate("MainWindow", "LeftWheel", None))
		self.label_der.setText(_translate("MainWindow", "0", None))
		self.label_int.setText(_translate("MainWindow", "0", None))
		self.label_ki.setText(_translate("MainWindow", "0", None))
		self.label_speed.setText(_translate("MainWindow", "0", None))
		self.label_rw.setText(_translate("MainWindow", "0", None))
		self.label_59.setText(_translate("MainWindow", "Derivative", None))
		self.label_77.setText(_translate("MainWindow", "Speed", None))
		self.label_64.setText(_translate("MainWindow", "Blank", None))
		self.label_78.setText(_translate("MainWindow", "Blank", None))
		self.tabWidget_3.setTabText(self.tabWidget_3.indexOf(self.tab_8), _translate("MainWindow", "Raw", None))
		self.tabWidget_1.setTabText(self.tabWidget_1.indexOf(self.tab_2), _translate("MainWindow", "Plot", None))
		self.label_Kd.setText(_translate("MainWindow", "Kd", None))
		self.label_Kp.setText(_translate("MainWindow", "Kp", None))
		self.label_Ki.setText(_translate("MainWindow", "Ki", None))
		self.edit_Kp.setText(_translate("MainWindow", "2.5", None))
		self.edit_Ki.setText(_translate("MainWindow", "0.5", None))
		self.edit_Kd.setText(_translate("MainWindow", "0.1", None))
		self.btn_PID_Apply.setText(_translate("MainWindow", "Apply", None))
		self.label_6.setText(_translate("MainWindow", "Derivative Filter", None))
		self.label_4.setText(_translate("MainWindow", "Integral Filter", None))
		self.edit_d_filter.setText(_translate("MainWindow", "0.01", None))
		self.edit_i_filter.setText(_translate("MainWindow", "0.001", None))
		self.label_16.setText(_translate("MainWindow", "Used In PID Function As:", None))
		self.label_9.setText(_translate("MainWindow", "    integral = (1-filter)*integral + filter*error*dt;", None))
		self.label_10.setText(_translate("MainWindow", "    derivative = (1-filter)*derivative + filter*(error-previousError)/dt;", None))
		self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_4), _translate("MainWindow", "PID", None))
		self.label_28.setText(_translate("MainWindow", "Sensor 1 is the one furthest out and 7 most in. ", None))
		self.label_5.setText(_translate("MainWindow", "Sensors on the other side will be set symmetrically.", None))
		self.edit_ADC5.setText(_translate("MainWindow", "1.0", None))
		self.label_20.setText(_translate("MainWindow", "7", None))
		self.label_19.setText(_translate("MainWindow", "6", None))
		self.label_14.setText(_translate("MainWindow", "Sensor", None))
		self.label_12.setText(_translate("MainWindow", "3", None))
		self.label_11.setText(_translate("MainWindow", "1", None))
		self.label_13.setText(_translate("MainWindow", "2", None))
		self.label_18.setText(_translate("MainWindow", "5", None))
		self.label_17.setText(_translate("MainWindow", "4", None))
		self.label_15.setText(_translate("MainWindow", "Weight", None))
		self.edit_ADC1.setText(_translate("MainWindow", "1.0", None))
		self.edit_ADC2.setText(_translate("MainWindow", "1.0", None))
		self.edit_ADC7.setText(_translate("MainWindow", "1.0", None))
		self.edit_ADC6.setText(_translate("MainWindow", "1.0", None))
		self.edit_ADC4.setText(_translate("MainWindow", "1.0", None))
		self.edit_ADC3.setText(_translate("MainWindow", "1.0", None))
		self.label_3.setText(_translate("MainWindow", "Nr Of Samples:", None))
		self.edit_nrOfSamples.setText(_translate("MainWindow", "8", None))
		self.btn_SensApply.setText(_translate("MainWindow", "Apply", None))
		self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_3), _translate("MainWindow", "Sensors", None))
		self.label_7.setText(_translate("MainWindow", "EDF27", None))
		self.btn_MotorApply.setText(_translate("MainWindow", "Apply", None))
		self.label.setText(_translate("MainWindow", "Pololu Motors", None))
		self.label_8.setText(_translate("MainWindow", "PWM", None))
		self.edit_PWM2.setText(_translate("MainWindow", "0", None))
		self.label_2.setText(_translate("MainWindow", "PWM", None))
		self.edit_PWM1.setText(_translate("MainWindow", "120", None))
		self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_5), _translate("MainWindow", "Motors", None))
		self.btn_findDevice.setText(_translate("MainWindow", "Find Devices", None))
		self.btn_UseSel.setText(_translate("MainWindow", "Use Selected", None))
		self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_6), _translate("MainWindow", "Bluetooth", None))
		self.tabWidget_1.setTabText(self.tabWidget_1.indexOf(self.tab_1), _translate("MainWindow", "Settings", None))
		self.menuFile.setTitle(_translate("MainWindow", "File", None))
		self.menuSave_Settings.setTitle(_translate("MainWindow", "Save Settings", None))
		self.menuAbout.setTitle(_translate("MainWindow", "About", None))
		self.actionLoad_Settings.setText(_translate("MainWindow", "Load Settings", None))
		self.actionQuit.setText(_translate("MainWindow", "Quit", None))
		self.actionAbout.setText(_translate("MainWindow", "About", None))
		self.actionTo_File.setText(_translate("MainWindow", "To File", None))
		self.actionSet_Current_As_Default.setText(_translate("MainWindow", "Set Current As Default", None))

if __name__ == "__main__":

	app = QtGui.QApplication(sys.argv)
	ex = Ui_MainWindow()
	ex.show()
	sys.exit(app.exec_())