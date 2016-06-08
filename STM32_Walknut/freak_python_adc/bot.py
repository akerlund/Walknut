#encoding=utf-8
import threading, socket, pickle, struct, bluetooth, sys, time, os

# one = bytes()
# one += struct.pack('B', 0)

initz = bytes()
for val in range(0,9):
	initz += struct.pack('B', 0)

stop = [1.0,1.0,1.0,1.0]
stopSend = bytes()
for val in stop:
	stopSend += struct.pack('f', float(val))

start = [2.0,0.0,0.0,0.0]
startSend = bytes()
for val in start:
	startSend += struct.pack('f', float(val))

def connectBlue():
	while(True):
		try:
			sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
			print("Establishing link to device 00:06:66:05:11:41")
			sock.connect(('00:06:66:05:11:41', 1))			
			print("Sending setup commands")
			#client_sock,address = sock.accept()
			#print ("Accepted connection from ",address)

			sock.send('$$$')
			#sock.send('SF,1\r\n')
			sock.send('SM,0\r\n')
			#sock.send('SO,\r\n')

			time.sleep(0.2)
			sock.send('SU,115200\r\n')
			# sock.send('SS,TAU_BOT\r\n')
			# sock.send('ST,60\r\n')
			# sock.send('S~,1\r\n')
			sock.send('---')
			time.sleep(0.2)						
			print("Linking done")
			break;
		except bluetooth.btcommon.BluetoothError as error:
			sock.close()
			print ("Could not connect: ", error, "; Retrying in 10s...")
			time.sleep(10)
	return sock;

class Bot:
	def __init__(self, device):
		self.device = device
		self.blueRecieve = bytes()

	def log_event(self, obj):
		self.target.send(pickle.dumps(obj, pickle.HIGHEST_PROTOCOL))		

	def connect(self):
		#self.ser = serial.Serial(self.device,baudrate=115200)
		#self.ser.flushInput( )

		self.target = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.target.connect(('127.0.0.1', 8100))

		self.sock = connectBlue()		
		self.sock.send(initz)
		#self.firstIncome = self.ser.read(4*4+7+9)

		self.incoming_thread = threading.Thread(target=self.handle_incoming)
		self.incoming_thread.start()

	def disconnect(self):
		self.sock.close()

	def user_shell(self):
		print("Welcome freaquencys' shell\n")

		while 1:
			print("Press '0' for commands")

			p=input()
			if(len(p)>0):
			
				argIn = p.split(" ")
				# 0 - PRINT COMMANDS
				if(int(argIn[0]) == 0):
					print ( "1: START\n"
							"2: STOP\n"
							"3: PID arg(Kp,Ki,Kd)\n"
							"4: MOTORBASE PWM arg(L,R)\n"
							"5: CALIBRATE arg((min=0 | max=1 | dif=2))\n"
							"6: USART ON/OFF: 0 == Off, 1 == On\n"
							"7: RESET RESET PID VALUES\n"
							"8: MODE: AUTO/MANUAL\n"
							"9: STOP MOTORS\n"
							"10: SEND ADC OR MIN_VALUE arg(ADC=0, MIN=1, DIF=2)\n")
					# print (self.firstIncome)
					# print (len(self.firstIncome))

				# 1 - START COMMAND
				if(int(argIn[0]) == 1):
					print ("START COMMAND\n")
					#self.ser.write(startSend)
					#self.sock.send(one)
					self.sock.send(startSend)

				# 2 - STOP COMMAND
				if(int(argIn[0]) == 2):
					print ("STOP COMMAND\n")
					#self.ser.write(startSend)
					self.sock.send(stopSend)
					#self.sock.send(stopSend)

				# 3 - PID COMMAND
				if((int(argIn[0])==3) and (len(argIn)>=4)):
					print ("PID COMMAND\n")
					pidSend = bytes()
					pidSend += struct.pack('f', float(3.0))
					pidSend += struct.pack('f', float(argIn[1]))
					pidSend += struct.pack('f', float(argIn[2]))
					pidSend += struct.pack('f', float(argIn[3]))
					#self.ser.write(pidSend)
					self.sock.send(pidSend)

				# 4 -MOTORBASE COMMAND
				if((int(argIn[0])==4) and (len(argIn)>=2)):
					print ("MOTORBASE COMMAND\n")
					motorSend = bytes()
					motorSend += struct.pack('f', float(4.0))
					motorSend += struct.pack('f', float(argIn[1]))
					motorSend += struct.pack('f', float(argIn[1]))
					motorSend += struct.pack('f', float(0.0))
					#self.ser.write(motorSend)
					self.sock.send(motorSend)
				
				# 5 - CALIBRATION
				if((int(argIn[0])==5) and (len(argIn)>=2)):
					print ("CALIBRATION\n")
					calSend = bytes()
					calSend += struct.pack('f', float(5.0))
					calSend += struct.pack('f', float(argIn[1]))
					calSend += struct.pack('ff', *(0,0))
					#self.ser.write(calSend)
					self.sock.send(calSend)				

				# 6 - USART ON/OFF
				if((int(argIn[0])==6)):
					print ("USART ON/OFF\n")
					usartSend = bytes()
					usartSend += struct.pack('f', float(6.0))
					usartSend += struct.pack('f', float(argIn[1]))
					usartSend += struct.pack('ff', *(0,0))
					self.sock.send(usartSend)
				
				# 7 - RESET PID VALUES
				if((int(argIn[0])==7)):
					print ("RESET PID VALUES\n")
					resetSend = bytes()
					resetSend += struct.pack('f', float(7.0))
					resetSend += struct.pack('fff', *(0,0,0))
					self.sock.send(resetSend)
								
				# 8 - MODE: AUTO/MANUAL
				if((int(argIn[0])==8)):
					print ("MODE: AUTO/MANUAL\n")
					modeSend = bytes()
					modeSend += struct.pack('f', float(8.0))
					modeSend += struct.pack('f', float(argIn[1]))
					modeSend += struct.pack('ff', *(0,0))
					self.sock.send(modeSend)				
				
				# 9 - STOP/START MOTORS
				if((int(argIn[0])==9)):
					print ("STOP/START MOTORS\n")
					motorStop = bytes()
					motorStop += struct.pack('f', float(9.0))
					motorStop += struct.pack('f', float(argIn[1]))
					motorStop += struct.pack('ff', *(0,0))
					self.sock.send(motorStop)

				# 10 - SEND ADC OR MIN_VALUE
				if((int(argIn[0])==10)):
					print ("SEND ADC OR MIN_VALUE\n")
					minmax = bytes()
					minmax += struct.pack('f', float(10.0))
					minmax += struct.pack('f', float(argIn[1]))
					minmax += struct.pack('ff', *(0,0))
					self.sock.send(minmax)					



				#print("Freak vill %s" % p)

	def handle_incoming(self):

		try:
			#self.ser.write(b'\xFF')
			while 1:
				#self.log_event(('values', struct.unpack( 'iiiiiiiiiiiifffiiffiiiiiii',  self.ser.read(4*26))))

				self.blueRecieve += self.sock.recv(26*4)
				l = len(self.blueRecieve)
				if l==104:
					#self.log_event(('values', struct.unpack( 'iiiiiiiiiiiifffffffiiiiiii', self.blueRecieve)))
					self.log_event(('values', struct.unpack( 'fffffffffffffffffffiiiiiii', self.blueRecieve)))
					self.blueRecieve = bytes()
				#print (l)

				# self.blueRecieve += self.ser.read(4*4)
				# l = len(self.blueRecieve)
				# if l==16:
				# 	self.log_event(('values', struct.unpack( 'ffff', self.blueRecieve)))
				# 	self.blueRecieve = bytes()


		except EOFError:
			print("Klar")

if __name__ == '__main__':
	bot = Bot('/dev/ttyUSB0')
	print("Connecting to bot")
	bot.connect()
	bot.user_shell()
	bot.disconnect()