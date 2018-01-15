import time
import serial

class HAMEG(object):
	
	ser = None
	
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
		timeout=0.1
		)
		return self.ser.isOpen()

	def disconnect(self):
		self.ser.write("RM0".encode('ASCII')+bytearray([13,10]))
		r = self.ser.readline()#(3)
		return r

	def close(self):
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
		r = self.ser.readline()#(3)
		return r



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
		vold1 += 16 
		"""[16 28]"""
		print("CH1=")
		f = [round(vold1), 13]
		self.ser.write("CH1=".encode('ASCII')+bytearray(f))
		r = self.ser.readline()#(3)
		return r

	def vdiv2(self,vold2):
		"""[0 13]"""
		vold2 += 16 
		"""[16 28]"""
		print("CH2=")
		f = [round(vold2), 13]
		self.ser.write("CH2=".encode('ASCII')+bytearray(f))
		r = self.ser.readline()#(3)
		return r

	def tdiv(self,timed):
		"""[0 25]"""
		timed += 1 
		"""[16 28]"""
		print("TBA=")
		h = [round(timed), 13]
		self.ser.write("TBA=".encode('ASCII')+bytearray(h))
		r = self.ser.readline()#(3)
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

#r = wtwf1(ser)
#print(r)



	def rdwf1(self,shift=5):
		#print("RDWF1=")
		#i=bytearray([16,13])
		#ser.write("STRMODE=".encode('ASCII')+i)
		#i1 = ser.read(3)
		j = bytearray([0,shift,0,1,13])
		#j = b"\x00\x00\x00\x08"
		self.ser.write("RDWFM1=".encode('ASCII')+j)
		j1 = self.ser.read(1)
		time.sleep(0.2)
		j1 += self.ser.read(self.ser.inWaiting())
		#j1 = self.ser.read(256+13)
		#l=0
		#v = j1[12:267]
		with open('HAMEG_out.txt','ab') as f:
			f.write("RDWFM1=".encode('ASCII')+j)
			f.write(j1)
		return j1


	def rdwf2(self,shift=5):
		#print("RDWF2=")
		#i=bytearray([16,13])	
		#ser.write("STRMODE=".encode('ASCII')+i)
		#i1 = ser.read(3)
		j = bytearray([0,shift,0,1,13])
		#j = b"\x00\x00\x00\x08"
		self.ser.write("RDWFM2=".encode('ASCII')+j)
		#j1 = self.ser.read(self.ser.inWaiting())
		j1 = self.ser.read(1)
		time.sleep(0.2)
		j1 += self.ser.read(self.ser.inWaiting())
		#j1 = self.ser.read(256+13)
		#l=0
		#v = j1[12:267]
		with open('HAMEG_out.txt','ab') as f:
			f.write("RDWFM2=".encode('ASCII')+j)
			f.write(j1)
			
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

	res1 = hameg.rdwf1()
	print(res1)
	k1 = [int(i) for i in res1[50:]]

	res2 = hameg.rdwf2()
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
