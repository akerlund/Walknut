#encoding=utf-8

import socket
import threading
import socketserver
import pickle



class ThreadedTCPRequestHandler(socketserver.BaseRequestHandler):

	def handle(self):
		print ("Opened connection with %s" % self.request)
		try:
			while 1:
				data = self.request.recv(65536)
				if not data:
					break
				self.server.data_handler(pickle.loads(data))

		finally:
			self.request.close()

		print ("Closed connection with %s" % self.request)

class EventServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
	def __init__(self, data_handler, bind_address='localhost', bind_port=8100, verbose=False):

		self.bind_address = bind_address
		self.bind_port = bind_port
		self.verbose = verbose
		self.thread = None
		self.data_handler = data_handler
		socketserver.ThreadingMixIn.__init__(self)
		socketserver.TCPServer.__init__(self, (self.bind_address, self.bind_port), ThreadedTCPRequestHandler)
		self.allow_reuse_address = True
		if self.verbose:
			print("%s: Listening on %s:%s" % (self.__class__.__name__, self.bind_address, self.bind_port))

	def start(self):
		self.thread = threading.Thread(target=self.serve_forever)
		if self.verbose:
			print("%s: Started server thread" % (self.__class__.__name__))
		self.thread.start()
		return self.thread

	def shutdown(self):
		if self.verbose:
			print("%s: Shutting down" % (self.__class__.__name__))
		socketserver.TCPServer.shutdown(self)
		self.wait()

	def wait(self):
		self.thread.join()

