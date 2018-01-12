import time
import serial
import threading

class HAMEG(object):
	
	ser = None
	connected = False
	serThread = None
	HAMEG_state = {}
	started = False
	vdiv1_ = None
	vdiv2_ = None
	tdiv_  = None
	CH1WF = None
	CH2WF = None

	def __init__(self):
		pass
	def connect(self):
		# configure the serial connections (the parameters differs on the device you are connecting to)
		self.ser = serial.Serial(
		#port='COM1',
		port='/dev/ttyS0',
		baudrate=19200,
		parity='N',
		stopbits=serial.STOPBITS_TWO,
		bytesize=8,
		timeout=3
		)
		self.connected = True
		self.serThread = threading.Timer(2,self.read_from_CH)
		self.serThread.start()
		return self.ser.isOpen()
	def handle_data(self,data):
		print(data)
	def read_from_CH(self):
		while not self.started:
			self.started = True
			while self.isConnected():
				try:
					#print('$'*100)
					data=self.getData()

					time.sleep(0.5)
					#SMs_state,(SM1_steps, SM1_mode),(SM2_steps, SM2_mode),endstops
					#print("@"*10,state,"@"*10)
					self.handle_data(data)
				except serial.serialutil.SerialException:
					print('----------')
	def getData(self):
		self.CH1WF = self.rdwf1(offset=b'\x00\x00',max_len=b'\x00\x01')
		print(self.CH1WF)
		self.CH2WF = self.rdwf2(offset=b'\x00\x00',max_len=b'\x00\x01')

	def disconnect(self):
		self.ser.write("RM0".encode('ASCII')+bytearray([13,10]))
		r = self.ser.read(3)
		return r

	def close(self):
		self.connected = False	
		self.serThread.cancel()
		self.started = False
		return self.ser.close()

	def init(self):
		self.ser.write(bytearray([32, 13]))
		r = self.ser.readline()
		return r

	def vers(self):
		#r = self.ser.write(bytearray([32, 13]))
		self.ser.write("VERS?\r".encode('ASCII'))
		#r = self.ser.read(3)
		r = self.ser.readline()
		return r

	def autoset(self):
		print("AUTOSET")
		self.ser.write("AUTOSET".encode('ASCII')+bytearray([13, 10]))
		r = self.ser.read(3)
		return r

	def isConnected(self):
		try:
			if self.connected == True:
				self.ser.inWaiting()
				return 1
			else:
				return 0
		except:
			print ("Lost connection!")
			return 0

	def ypos1(self, yp1):
		"""[0 30]"""
		print("Y1POS=")
		cyp1 = 0
		if yp1>15:
			cyp1 = yp1-15
		if yp1<=15:
			cyp1 = yp1+240
		c = [160, cyp1, 13]
		self.ser.write("Y1POS=".encode('ASCII')+bytearray(c))
		r = self.ser.read(3)
		return r
	def get_ypos1(self):
		self.ser.write("Y1POS?".encode('ASCII')+bytearray([13,10]))
		r = self.ser.read(8)
		val = r.split(b'Y1POS:')[-1]
		print(val[1])
		yp1 = val[1]
		cyp1 = 0
		if yp1<=15:
			cyp1 = yp1+15
		if yp1>15:
			cyp1 = yp1-240
		res = cyp1#int.from_bytes(val, byteorder='big')*30/2**16
		return res

	def ypos2(self, yp2):
		"""[0 30]"""
		print("Y2POS=")
		cyp2 = 0
		if yp2>15:
			cyp2 = yp2-15
		if yp2<=15:
			cyp2 = yp2+240
		c = [160, cyp2, 13]
		self.ser.write("Y2POS=".encode('ASCII')+bytearray(c))
		r = self.ser.read(3)
		return r
	def get_ypos2(self):
		self.ser.write("Y2POS?".encode('ASCII')+bytearray([13,10]))
		r = self.ser.read(8)
		val = r.split(b'Y2POS:')[-1]
		print(val[1])
		yp2 = val[1]
		cyp2 = 0
		if yp2<=15:
			cyp2 = yp2+15
		if yp2>15:
			cyp2 = yp2-240

		res = cyp2#int.from_bytes(val, byteorder='big')*30/2**16
		return res

	def vdiv1(self, vold1):
		"""[0 13]"""
		self.vdiv1_ = vold1
		vold1 += 16 
		"""[16 28]"""
		print("CH1=")
		f = [round(vold1), 13]
		self.ser.write("CH1=".encode('ASCII')+bytearray(f))
		r = self.ser.read(3)
		return r

	def vdiv2(self,vold2):
		"""[0 13]"""
		self.vdiv2_ = vold2
		vold2 += 16 
		"""[16 28]"""
		print("CH2=")
		f = [round(vold2), 13]
		self.ser.write("CH2=".encode('ASCII')+bytearray(f))
		r = self.ser.read(3)
		return r

	def tdiv(self,timed):
		"""[0 25]"""
		self.tdiv_ = timed
		timed += 1 
		"""[16 28]"""
		print("TBA=")
		h = [round(timed), 13]
		self.ser.write("TBA=".encode('ASCII')+bytearray(h))
		r = self.ser.read(3)
		return r

	def wtwf1(self):
		print("RDWF1=")
		i=bytearray([88,13])
		self.ser.write("STRMODE=".encode('ASCII')+i)
		i1 = ser.read(3)
		j = bytearray([0,4,0,1]+list(range(256))+[13])
		print(j)
		self.ser.write("WRREF1=".encode('ASCII')+j)
		r = self.ser.read(3)

		return r
	def dig2Volts(self, dig, vdiv, bit=256,div=8):
		return dig/bit*div*vdiv
	def dig2sec(self, dig,tdiv,bit=2048,div=10):
		return dig/bit*div*tdiv

	def rdwf1(self,offset=b'\x00\x00',max_len=b'\x00\x08'):
		#print("RDWF1=")
		#i=bytearray([16,13])
		#ser.write("STRMODE=".encode('ASCII')+i)
		#i1 = ser.read(3)
		j = offset + max_len+bytearray([13])
		print(j)
		#j = b"\x00\x00\x00\x08"
		self.ser.write("RDWFM1=".encode('ASCII')+j)
		length = int.from_bytes(j[2:4],byteorder='little')+j[4]
		j1 = self.ser.read(length)
		#l=0
		#v = j1[12:267]
		return j1


	def rdwf2(self,offset=b'\x00\x00',max_len=b'\x00\x08'):
		#print("RDWF1=")
		#i=bytearray([16,13])
		#ser.write("STRMODE=".encode('ASCII')+i)
		#i1 = ser.read(3)
		j = offset + max_len+bytearray([13])
		#j = b"\x00\x00\x00\x08"
		self.ser.write("RDWFM2=".encode('ASCII')+j)
		length = int.from_bytes(j[2:4],byteorder='little')+j[4]
		j1 = self.ser.read(length)
		#l=0
		#v = j1[12:267]
		return j1

	def trgval(self):
		self.ser.write("TRGVAL?\r".encode('ASCII'))
		r = self.ser.read(15)
		return r



	


	def get_vdiv1(self):
		print("CH1?")
		self.ser.write("CH1?".encode('ASCII')+bytearray([13,10]))
		#s = b""
		#while s != b":":
		#	print(s)
		#	s = ser.read(1)
		r = self.ser.read(5)
		val = r.split(b'CH1:')[-1]
		res = int.from_bytes(val, byteorder='big') - 16
		self.vdiv1_ = res
		return res

	def get_vdiv2(self):
		print("CH2?")
		self.ser.write("CH2?".encode('ASCII')+bytearray([13,10]))
		#s = b""
		#while s != b":":
		#	s = ser.read(1)
		r = self.ser.read(5)
		val = r.split(b'CH2:')[-1]
		res = int.from_bytes(val, byteorder='big') -16
		self.vdiv2_ = res
		return res

	def get_tdiv(self):
		print("TBA?")
		self.ser.write("TBA?".encode('ASCII')+bytearray([13,10]))
		#s = b""
		#while s != b":":
		#	s = ser.read(1)
		r = self.ser.read(5)
		val = r.split(b'TBA:')[-1]
		res = int.from_bytes(val, byteorder='big')-1
		self.tdiv_ = res
		return res

if __name__ == "__main__":
	import time


	hameg = HAMEG()
	hameg.connect() 
	hameg.init()

	r = hameg.vers()
	print(r)

	#r = autoset(ser)
	#print(r)

	r = hameg.vdiv1(8)
	print(r)
	r = hameg.vdiv2(8)
	print(r)


	r = hameg.tdiv(1)
	print(r)

	r = hameg.ypos1(0)
	#print(r)
	r = hameg.ypos2(15)
	print(r)

	res1 = hameg.CH1WF#hameg.rdwf1()
	print(res1)
	k1 = [int(i) for i in res1[50:]]

	res2 = hameg.CH1WF#hameg.rdwf2()
	print(res2)
	k2 = [int(i) for i in res2[50:]]



	rt = hameg.trgval()
	print(rt)

	
	ch1_v = hameg.get_vdiv1()
	print(ch1_v)

	ch2_v = hameg.get_vdiv2()
	print(ch2_v)
	
	yp1 = hameg.get_ypos1()
	print(yp1)

	td = hameg.get_tdiv()
	print(td)

	#time.sleep(5)
	r = hameg.disconnect()

	print(r)
	hameg.close()



	'''
	input=1
	while 1 :
		# get keyboard input
		input_ = input(">> ")
	        # Python 3 users
	        # input = input(">> ")
		if input_ == 'exit':
			ser.close()
			exit()
		else:
			# send the character to the device
			# (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
			ser.write(input_ + '\r\n')
			out = ''
			# let's wait one second before reading output (let's give device time to answer)
			time.sleep(1)
			while ser.inWaiting() > 0:
				out += ser.read(1)
				
			if out != '':
				print (">>" + out)
	'''
