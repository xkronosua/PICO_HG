import queue
import threading
import time
import traceback
import serial
from serial.tools import list_ports


class ComMonitorThread(threading.Thread):
	""" A thread for monitoring a COM port. The COM port is
		opened when the thread is started.

		data_q:
			Queue for received data. Items in the queue are
			(data, timestamp) pairs, where data is a binary
			string representing the received data, and timestamp
			is the time elapsed from the thread's start (in
			seconds).

		error_q:
			Queue for error messages. In particular, if the
			serial port fails to open for some reason, an error
			is placed into this queue.

		port:
			The COM port to open. Must be recognized by the
			system.

		port_baud/stopbits/parity:
			Serial communication parameters

		port_timeout:
			The timeout used for reading the COM port. If this
			value is low, the thread will return data in finer
			grained chunks, with more accurate timestamps, but
			it will also consume more CPU.
	"""
	def __init__(   self,
					data_q, error_q,
					port_num,
					port_baud,
					port_stopbits=serial.STOPBITS_ONE,
					port_parity=serial.PARITY_NONE,
					port_timeout=0.05):
		threading.Thread.__init__(self)

		self.serial_port = None
		self.serial_arg = dict( port=port_num,
								baudrate=port_baud,
								stopbits=port_stopbits,
								parity=port_parity,
								timeout=port_timeout)

		self.data_q = data_q
		self.error_q = error_q
		self.lock = threading.Lock()
		self.alive = threading.Event()
		self.alive.set()
		#self.lock.acquire()

	def run(self):
		try:
			if self.serial_port:
				self.serial_port.close()
			self.serial_port = serial.Serial(**self.serial_arg)
		except serial.SerialException:
			self.error_q.put(traceback.print_exc())
			return

		# Restart the clock
		time.clock()
		data = b''
		while self.alive.isSet():
			# Reading 1 byte, followed by whatever is left in the
			# read buffer, as suggested by the developer of
			# PySerial.
			#
			try:

				data += self.serial_port.read(1)
				#time.sleep(0.1)
				data += self.serial_port.read(self.serial_port.inWaiting())
				#print(data)
				
				if len(data) > 0:
					if data[-1]!=b'\n':
						data+=self.serial_port.readline()
					timestamp = time.clock()
					self.data_q.put((data, timestamp))
					data = b''
					#time.sleep(0.2)
			except serial.SerialException as e:
				self.error_q.put(e)
				return
			time.sleep(0.05)
		# clean up
		if self.serial_port:
			self.serial_port.close()
	def read(self):
		try:
			time.sleep(0.1)
			data = self.serial_port.read(1)
			#time.sleep(0.1)
			data += self.serial_port.read(self.serial_port.inWaiting())
			return data
		except serial.SerialException as e:
			self.error_q.put(e)
			return
	def join(self, timeout=None):
		self.alive.clear()
		threading.Thread.join(self, timeout)

	def stop(self):
		self.alive.clear()

	def lock_readout(self):
		self.alive.wait()
		self.lock.acquire()

	def release_readout(self):
		self.alive.set()
		self.lock.release()




if __name__ == '__main__':
	data_q = queue.Queue()
	error_q = queue.Queue()
	port = list(list_ports.grep("0403:0000"))[0][0]
	com_monitor = ComMonitorThread(data_q,error_q,port,9600)
	com_monitor.start()
	for i in range(10):
		if not data_q.empty():
			print(data_q.get())
		else:
			print('empty')
		time.sleep(1)
	#com_monitor.close()
