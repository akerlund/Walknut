import io
import os
import serial
import time, math
import struct

#0.004096s @ 1.5MBaud
#dmesg | tail


ser = serial.Serial('/dev/ttyUSB0',baudrate=115200)
ser.flushInput( )
x = []
#ser.write(chr(0xFF))
#floats = [1024,2048,4096]
buf = bytes()
buf += struct.pack('f', 66.25)
buf += struct.pack('f', 123.75)
buf += struct.pack('f', 98.25)
buf += struct.pack('f', 13.75)
buf += struct.pack('f', 66.25)
buf += struct.pack('f', 123.75)
buf += struct.pack('f', 98.25)
buf += struct.pack('f', 13.75)
buf += struct.pack('f', 66.25)
buf += struct.pack('f', 123.75)
buf += struct.pack('f', 98.25)
buf += struct.pack('f', 13.75)
buf += struct.pack('f', 66.25)
buf += struct.pack('f', 123.75)
buf += struct.pack('f', 98.25)
buf += struct.pack('f', 13.75)
buf += struct.pack('f', 66.25)
buf += struct.pack('f', 123.75)
buf += struct.pack('f', 98.25)
buf += struct.pack('f', 13.75)
buf += struct.pack('B', 85)
buf += struct.pack('B', 85)
buf += struct.pack('B', 85)

buf += struct.pack('B', 16)

# buf += struct.pack('B', 7)
# buf += struct.pack('B', 8)
# buf += struct.pack('B', 9)
# buf += struct.pack('B', 63)
buf += struct.pack('f', 66.0)
buf += struct.pack('f', 123.75)
buf += struct.pack('f', 98.25)
buf += struct.pack('f', 13.75)
# buf += struct.pack('f', 2.0)
#buf += struct.pack('f', 512.0)
#buf += struct.pack('f', 512.0)


print "sending %i" % (len(buf))
ser.write(buf)
print "Int count: %i" % (struct.unpack('i', ser.read(4))[0])
# print "Int count: %B" % (struct.unpack('B', ser.read(1))[0])
# print "Int count: %B" % (struct.unpack('B', ser.read(1))[0])
# print "Int count: %B" % (struct.unpack('B', ser.read(1))[0])


	# USART_putn(USART1, (char*)&position, 4);	// i
	# USART_putn(USART1, (char*)&error, 4);		// i
	# USART_putn(USART1, (char*)&integral, 4);	// f
	# USART_putn(USART1, (char*)&derivative, 4);	// f
	# USART_putn(USART1, (char*)&motorSpeed, 4);	// i
	# USART_putn(USART1, (char*)&leftReg, 4);		// i
	# USART_putn(USART1, (char*)&rightReg, 4);	// i
# try:
	
# 	# Reading PID and shit
# 	while 1:

# 		#time.sleep(0.1)
# 		x = []

# 		for pid in range(7):
# 			x.append(ser.read(4))

# 		print("\x1b[2J")

# 		pid1 = struct.unpack('i', x[0])[0]
# 		pid2 = struct.unpack('i', x[1])[0]
# 		pid3 = struct.unpack('f', x[2])[0]
# 		pid4 = struct.unpack('f', x[3])[0]
# 		pid5 = struct.unpack('i', x[4])[0]
# 		pid6 = struct.unpack('i', x[5])[0]
# 		pid7 = struct.unpack('i', x[6])[0]

# 		print "position: %i" % (pid1)
# 		print "error: %i" % (pid2)
# 		print "integral: %f" % (pid3)
# 		print "derivative: %f" % (pid4)
# 		print "motorSpeed: %i" % (pid5)
# 		print "leftReg: %i" % (pid6)
# 		print "rightReg: %i" % (pid7)

# 	# p=raw_input()
# 	# p=p.split()
# 	# for i in p:
# 	# 	a.append(int(i))




# 	# Reading ADC and shit
# 	while 1:

# 		#time.sleep(0.1)
# 		x = []

# 		for adc_id in range(14):
# 			x.append(ser.read(2))

# 		x.append(ser.read(4))
# 		x.append(ser.read(4))

# 		print("\x1b[2J")
# 		for adc_id in range(12):
# 		#x = ser.read(2)
# 			xx = struct.unpack('h', x[adc_id])[0]
# 			print "ADC%i: %i" % ((adc_id+1), xx)
		
# 		nr12 = struct.unpack('h', x[12])[0]
# 		nr13 = struct.unpack('h', x[13])[0]
# 		nr14 = struct.unpack('f', x[14])[0]
# 		nr15 = struct.unpack('f', x[15])[0]
# 		print "Middle: %i" % (nr12)
# 		print "Turns: %i" % (nr13)
# 		print "L_Speed: %f cm/s" % (nr14)
# 		print "R_Speed: %f cm/s" % (nr15)	

# 	while 1:

# 		#time.sleep(0.1)
# 		x = []

# 		for adc_id in range(16):
# 			x.append(ser.read(2))

# 		print("\x1b[2J")
# 		for adc_id in range(12):
# 		#x = ser.read(2)
# 			xx = struct.unpack('h', x[adc_id])[0]
# 			print "ADC%i: %i" % ((adc_id+1), xx)
		
# 		nr12 = struct.unpack('h', x[12])[0]
# 		nr13 = struct.unpack('h', x[13])[0]
# 		nr14 = struct.unpack('h', x[14])[0]
# 		nr15 = struct.unpack('h', x[15])[0]
# 		print "Middle: %i" % (nr12)
# 		print "Middle: %i" % (nr13)
# 		print "L_Speed: %i" % (nr14)
# 		print "R_Speed: %i" % (nr15)



# 		# read = read+1
# 		# if read == 17:
# 		# 	read = 1
# 			#print "\n"

# except EOFError:
# 	pass # end of sequence
