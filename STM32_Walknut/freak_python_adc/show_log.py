#encoding=utf-8
from event_server import EventServer

def show_freak_data(arg):
	typename, data = arg
	if typename == 'values':
		print("\x1b[2J")
		#print("Mottagen data (%s)" % repr(data))
		#print()

		# print (("ADC nr:\n"
		# 		"1: %f\n2: %f\n3: %f\n4: %f\n") % data)		

		print (("ADC nr:\n"
				# "1: %i\n2: %i\n3: %i\n4: %i\n5: %i\n6: %i\n"
				# "7: %i\n8: %i\n9: %i\n10: %i\n11: %i\n12: %i\n\n\n"
				"1: %f\n2: %f\n3: %f\n4: %f\n5: %f\n6: %f\n"
				"7: %f\n8: %f\n9: %f\n10: %f\n11: %f\n12: %f\n\n\n"

				"Kp: %f  Ki: %f  Kd: %f\n"
				"Position: %f\t"
				"Error: %f\n"
				"Integral: %f\t"
				"Derivative: %f\n"
				"d_Motors: %i\n"
				"L_Cnt: %i\t"
				"R_Cnt: %i\n"
				"L_Base: %i\t"
				"R_Base: %i\n"
				"L_PWM: %i\t"
				"R_PWM: %i\n" ) % data)
	else:
		print("Vet inte vad jag får för skum data! %s" % data)

server = EventServer(verbose=True, data_handler=show_freak_data)
server_thread = server.start()
server_thread.join()