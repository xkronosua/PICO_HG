import queue
import threading
import time
import traceback
import serial
from serial.tools import list_ports
import numpy as np


class TCD1304(threading.Thread):
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

	data_q = queue.Queue()
	error_q = queue.Queue()

	highSpeedMode = False
	integration = 1
	trigger = False
	buf_sign = 0
	loop_sign = 1
	lastTime = time.time()
	def __init__(self, highSpeedMode=False, trigger=False, integration=1):
		threading.Thread.__init__(self)

		self.highSpeedMode = highSpeedMode
		self.integration = integration
		self.trigger = trigger
		self.serial_port = None
		self.lock = threading.Lock()
		self.alive = threading.Event()
		self.alive.set()
		#self.lock.acquire()

	def run(self):
		try:
			if self.serial_port:
				self.serial_port.close()
			else:
				ports = list(list_ports.grep("0403:6001"))
				self.port = '/dev/ttyUSB0'
				for i in ports:
					if i.description == 'TIT 2RCRP':
						self.port = i.device
				self.serial_port = serial.Serial(self.port,baudrate=115200,timeout=0.1)#, bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,rtscts=False,dsrdtr=False)
				self.setTrigger(self.trigger)
				self.setIntegration(self.integration)
				self.setHighSpeedMode(self.highSpeedMode)

		except serial.SerialException:
			self.error_q.put(traceback.print_exc())

			return self.error_q

		# Restart the clock
		time.clock()
		time.sleep(0.1)
		data = b''
		while self.alive.isSet():
			# Reading 1 byte, followed by whatever is left in the
			# read buffer, as suggested by the developer of
			# PySerial.
			#

			try:

				data = self.getData()
				timestamp = time.clock()
				self.data_q.put((data, timestamp))

				#print(data.sum(),self.error_q.get())

			except (serial.SerialException, OSError) as e:
				self.error_q.put(e)
				self.stop()

				return
			#time.sleep(0.05)
		# clean up
		if self.serial_port:
			self.serial_port.close()
		print('CCD Stopped')

	def setHighSpeedMode(self, active=False):
		if active:
			self.serial_port.write(b'#CSDTP:0%'+bytearray([0x11,0x13]))
			self.serial_port.write(b'#?data%'+bytearray([0x11,0x13]))
			#for i in range(10):
			self.serial_port.write(b'#CSDTP:1%'+bytearray([0x11,0x13]))
			time.sleep(1)
		else:
			pass#self.serial_port.write(b'#CSDTP:0%'+bytearray([0x11,0x13]))

	def setTrigger(self, active=False):
		n = self.serial_port.inWaiting()
		if n != 0:
			r=self.serial_port.read(n)	
			print('#Text:',r)
		if not active:
				self.serial_port.write(b'#Text:0%'+bytearray([0x11,0x13]))
		elif active:
			#for i in range(10):
			self.serial_port.write(b'#Text:1%'+bytearray([0x11,0x13]))
			#self.serial_port.write(b'#Text:1%'+bytearray([0x11,0x13]))

	def setIntegration(self, level=1):
		level_str = "#CCDInt:";

		if level<=0 or level>100:
				level = 1
		str1 = ''
		if level<10:
				str1="00"+str(level)
		elif level>=10 and level<100:
				str1="0"+str(level)
		elif level==100:
				str1=str(level)
		level_str=level_str+str1+"%";
		n = self.serial_port.inWaiting()
		if n != 0:
			r=self.serial_port.read(n)		
			print('#CCDInt:',r)
		self.serial_port.write(level_str.encode()+bytearray([0x11,0x13]))
		#self.serial_port.write(level_str.encode()+bytearray([0x11,0x13]))

	def getData(self):
		ccd_vol_array = np.zeros(3648,dtype=int)
		if not self.highSpeedMode:
			self.serial_port.write(b'#?data%'+bytearray([0x11,0x13]))
			read_buf = self.serial_port.read(1)#7296)
			l = 1
			for i in range(200):
				time.sleep(0.01)
				n=self.serial_port.inWaiting()
				read_buf += self.serial_port.read(n)
				l+=n
				if l == 7296:
					break
			#print(i)
			kk=0
			if len(read_buf) == 7296:
				for i in range(0,8,2):
					if (read_buf[i]*256+read_buf[i+1])==24930: kk+=1
					if (read_buf[7288+i]*256+read_buf[7289+i])==24930: kk+=1
				if kk>=2:
					for i in range(0,7296,2):
						#print(i/2)
						ccd_vol_array[int(i/2)]=read_buf[i]*256+read_buf[i+1]
				else:
					for i in range(1,7295,2):
						ccd_vol_array[int((i+1)/2)]=read_buf[i]*256+read_buf[i+1]
				ccd_vol_array[0]=ccd_vol_array[1]=ccd_vol_array[2]=ccd_vol_array[3]=ccd_vol_array[4]=ccd_vol_array[5]
				ccd_vol_array[3647]=ccd_vol_array[3646]=ccd_vol_array[3645]=ccd_vol_array[3644]=ccd_vol_array[3643]
		else:
			read_buf = b''
			
			k=0
			
			
			read_buf = self.serial_port.read(1)
			while self.loop_sign==1:
				l = 1
				while l<=7296:
					to_read = self.serial_port.inWaiting()
					read_buf += self.serial_port.read(to_read)
					l += to_read
				print(l)
				print('max:',max(read_buf))
				#time.sleep(5)
				for i in range(to_read-1):
					print(read_buf[i])
					if read_buf[i] == 255 and read_buf[i+1] == 252:
						print('='*10, read_buf[i:(i+5)])
						p = len(read_buf)-i-1
						
						read_buf = self.serial_port.read(7296-p)
						self.loop_sign = 0
						print('-'*10)
						#time.sleep(10)
						break
					
				k+=1
				print(k,to_read)
			print('shift_found')
			total_read_buf = read_buf
			if self.loop_sign==0:
				total_read_buf = self.serial_port.read(7296)
				#total_read_buf = self.serial_port.read(7296)
				self.loop_sign = 1
			for i in range(0,7296,2):
				ccd_vol_array[int(i/2)]=total_read_buf[i]*256+total_read_buf[i+1];
			ccd_vol_array[0]=ccd_vol_array[1]=ccd_vol_array[3647]=ccd_vol_array[2];

			'''

			buf_sign = 0
			temp_add=0;
			read_buf_B = []
			read_buf_A = []
			total_bytes = 0
			temp_read_buf = self.serial_port.read(128)

			for k in range(128):
				if buf_sign==0:
					read_buf_B
					
			'''
		#print(ccd_vol_array)
		
		return ccd_vol_array

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
	from pyqtgraph.Qt import QtGui, QtCore
	import numpy as np
	import pyqtgraph as pg
	from pyqtgraph.ptime import time as Time
	import random
	app = QtGui.QApplication([])

	p = pg.plot()
	p.setWindowTitle('pyqtgraph example: PlotSpeedTest')
	p.setYRange(0,45000)
	p.setXRange(0,4000)
	
	p.setLabel('bottom', 'Index', units='B')
	curves = []
	colors  = ["red","green","blue","orange","purple","pink","yellow"]
	for i in range(100):
		curve = p.plot()
		curve.setPen(QtGui.QColor(random.choice(colors)))
		#curve.setFillLevel(2)

		curves.append(curve)
	#curve.setFillLevel(0)

	#lr = pg.LinearRegionItem([100, 4900])
	#p.addItem(lr)
	ccd = TCD1304(highSpeedMode=False,trigger=True, integration=100)
	ccd.start()

	ptr = 0
	lastTime = Time()
	fps = None
	t1=0
	def update():
		global curve, data, ptr, p, lastTime, fps, t0, t1
		
		t0 = time.time()
		#print(t0-t1)
		cn = 0
		if ccd.data_q.empty():
			#time.sleep(0.2)
			return
		stack = ccd.data_q.get()[0]
		while not ccd.data_q.empty():
			stack += ccd.data_q.get()[0]
			cn+=1
			print("#",cn)
		data = stack/(cn+1)
		curves[0].setData(data)
			
		#ccd.data_q.clear()
		ptr += 1
		now = Time()
		dt = now - lastTime
		lastTime = now
		if fps is None:
			fps = 1.0/dt
		else:
			s = np.clip(dt*3., 0, 1)
			fps = fps * (1-s) + (1.0/dt) * s
		p.setTitle('%0.2f fps' % fps)
		app.processEvents()  ## force complete redraw for every plot
		t1 = time.time()
		print(t1-t0)
	timer = QtCore.QTimer()
	timer.timeout.connect(update)
	timer.start(0)


	## Start Qt event loop unless running in interactive mode.
	if __name__ == '__main__':
		import sys
		if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
			QtGui.QApplication.instance().exec_()
