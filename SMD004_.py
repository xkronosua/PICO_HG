import serial  #pyserial-3.0 for XP
import time
import codecs
import base64
import threading
import traceback
import queue
from serial.tools import list_ports




class SMD004():
	ser = None
	connected = False
	serThread = None
	SM_state = {}
	started = False
	SM_position = [0,0]
	forceStop = False
	lockFlow = False
	Im = [2,2]
	Is = [1,1]
	threadKillCounter = 0
	waitUntilReadyState = False
	cSumOk = True
	commandQueue = queue.Queue()
	stateQueue = queue.Queue()
	def __init__(self, parent=None):
		self.eInit()
		ser = None

	def eInit(self):
		pass
		#self.ser = serial.Serial("COM3", baudrate=19200, timeout=0.1, bytesize=serial.EIGHTBITS)
		#print(self.write_(b'\xFF\x07\x03\x01\x04\x01'))#b"FF0703010101")
		#print(self.write_(b'\xFF\x06\x02\x01\x01'))#b"FF06020101")
		#print(self.write_(b'\xFF\x03\x05\x01\x60\x00\x00\x00'))#b"FF03050132000000")
		#print(self.write_(b"\xFF\x04\x00"))
		#print(self.write_(b"\xFF\x08\x02\x01\x01"))
	def add2queue(func):
		def wrapper(*args,**kwargs):
			self = args[0]
			self.commandQueue.put(lambda: func(*args,**kwargs))

		return wrapper
			

	def eOpenCOMPort(self,port=1):
		ports = list(list_ports.grep("0403:6001"))
		for p in ports:
			if p.usb_description() == 'USB <-> Serial':
				port = p.device

		self.ser = serial.Serial(port, baudrate=19200,timeout=0.1, bytesize=serial.EIGHTBITS,parity=serial.PARITY_MARK,stopbits=serial.STOPBITS_ONE)#,xonxoff=False,rtscts=False,dsrdtr=False)
		self.commandQueue = queue.Queue()
		self.stateQueue = queue.Queue()

		self.ser.write(b'\x00\x00\x00')
		r = b''
		for i in range(40):
			tmp = self.ser.read()
			r+=tmp
			if tmp == b'\x5c':
				break
		print(r)
		self.ser.write(b'\x00\x01\x00')
		self.ATrtAddr = self.ser.read(1)
		self.lockFlow = False
		print('ATrtAddr:',self.ATrtAddr)
		
		self.connected = True
		self.forceStop = False
		cSumOk = True
		self.eGetState()
		#self.SM_position[0] = self.SM_state['SM1_steps']
		#self.SM_position[1] = self.SM_state['SM2_steps']
		
		threading.Thread(target=self.procThread).start()
		self.threadKillCounter = 0
	
	def procThread(self):
		while not self.forceStop and self.ser:
			if not self.commandQueue.empty():
				#print(self.ser,self.isConnected())
				command = self.commandQueue.get()
				#print(command)
				command()
				self.commandQueue.task_done()
				
			else:

				time.sleep(0.5)


	def close(self):
		self.connected = False	
		self.forceStop = True
		#self.serThread.cancel()

		self.ser.close()
	@add2queue
	def eStop(self, stepper=3):
		u'''
		Назначение: стоп вращения двигателя.
		Байт		1-й		2-й		3-й		4-й											5-й
		Значение	Адрес	01 hex	01 hex	01 hex – стоп 1-го двигателя
											02 hex – стоп 2-го двигателя
											03 hex – стоп двух двигателей одновременно	Контроль¬ная сумма
		Ответ модуля: возвращает принятые байты без изменений.
		'''
		#self.write(b"FF010101")
		s = self.ATrtAddr +b"\x01\x01"+bytearray([stepper])
		print("eStop",end='')
		r = self.write_read(s)

	@add2queue
	def eStart(self, stepper=1):
		u'''
		Назначение: старт вращения двигателя.
		Байт		1-й		2-й		3-й		4-й								5-й
		Значение	Адрес	00 hex	01 hex	01 hex – старт 1-го двигателя	Контроль¬ная сумма
											02 hex – старт 2-го двигателя
											03 hex – старт двух двигателей одновременно	
		Ответ модуля: возвращает принятые байты без изменений.
		'''
		s = self.ATrtAddr +b"\x00\x01"+bytearray([stepper])
		print("eStart",end='')
		r = self.write_read(s)
	
	@add2queue	
	def eSetParams(self,stepper=1,mode='ccw_step',steps=0,prev=False):
		u'''
		Назначение: установка режима вращения двигателя.
		Байт		1-й		2-й		3-й		4-й		5-й
		Значение	Адрес	02 hex	04 hex	Номер двигателя	00 hex – вращение до кон¬це¬во-го выклю¬ча¬теля или ко¬манды "Стоп"
															01 hex – вращение на заданное количество шагов
															80 hex и 81 hex – то же, но в обратную сторону

		Байт		6-й							7-й							8-ой
		Значение	Младший байт числа шагов	Старший байт числа шагов	Контроль¬ная сумма
		Ответ модуля: возвращает принятые байты без изменений.     

		'''

		m = {'ccw_step' : b'\x01',
			 'ccw2stop' : b'\x00',
			 'cw_step'  : b'\x81',
			 'cw2stop'  : b'\x80',

		}
		st = int(steps).to_bytes(2,'little')
		
		#if len(s)<4:
		#	if len(s[0])<len(s[1]): s[0] = '0'+s[0]
		#	elif len(s[0])>len(s[1]): s[1] = '0'+s[1]
		
		#self.write_(b"\xFF\x02\x04"+bytearray([stepper]) + m[mode]+s)
		if prev:
			try:
				code_mode = self.SM_state['SM'+str(stepper)+"_mode"]
				s = self.ATrtAddr +b"\x02\x04"+bytearray([stepper]) + code_mode + st
			except:
				self.eStop(stepper)
				return
				
		else:
			s = self.ATrtAddr +b"\x02\x04"+bytearray([stepper]) + m[mode]+st
		print("eSetParams",end='')
		r = self.write_read(s)
		
	@add2queue
	def eSetTactFreq(self, stepper=1, freq=68):
		u'''
		Назначение: установка скорости вращения двигателя.
		Байт		1-й		2-й		3-й		4-й				5-й	6-й	7-й	8-й																								9-й
		Значение	Адрес	03 hex	05 hex	Номер двигателя	Длительность полу-периода тактовой частоты дви¬га¬теля в десятках микро¬се-кунд (млад¬шим байтом впе¬ред)	Контроль¬ная сумма
		Ответ модуля: возвращает принятые байты без изменений
		'''
		#print(
		s = self.ATrtAddr + b'\x03\x05'+bytearray([stepper])+(freq).to_bytes(4,'little')
		print("eSetTactFreq",end='')
		r = self.write_read(s)
		

	def isConnected(self):
		try:
			if self.ser:
				self.ser.inWaiting()
				return 1
			else:
				return 0
		except:
			print ("Lost connection!")
			return 0

	@add2queue
	def eHardReset(self,stepper=3):
		'''Назначение: сброс контроллеров шаговых двигателей.
			Байт 1-й 2-й 3-й 4-й 5-й
			Значение Адрес 09 hex 01 hex 01 hex – для 1-го двигателя Контрольная сумма
										 02 hex – для 2-го двигателя
										 03 hex – для двух двигателей одновременно

			Ответ модуля: возвращает принятые байты без изменений.
			Примечание к команде 9.
			После сброса устанавливается следущее исходное состояние фаз двигателя:
			Фаза 		 A   -A    B   -B
			Напряжение   0   Uпит  0   Uпит
			Это состояние устанавливается также после включения питания модуля.
			Команда имеет побочный эффект: если двигатель находился в волновом режиме полного шага, то он
			переключится в нормальный режим полного шага.
		'''
		s = self.ATrtAddr + b'\x09\x01'+bytearray([stepper])
		print("eHardReset",end='')
		r = self.write_read(s)
	@add2queue
	def eClearStep(self, stepper=3):
		u'''
		Назначение: обнуление счетчика шагов.
		Байт		1-й		2-й		3-й		4-й											5-й
		Значение	Адрес	05 hex	01 hex	01 hex – для 1-го двигателя
											02 hex – для 2-го двигателя
											03 hex – для двух двигателей одновременно	Контроль¬ная сумма
		Ответ модуля: возвращает принятые байты без изменений.

		'''
		\
		s = self.ATrtAddr + b'\x05\x01'+bytearray([stepper])
		print("eClearStep",end='')
		r = self.write_read(s)
	
	@add2queue	
	def eSetMulty(self,stepper=1, multy=1):
		u'''
		Назначение: установка множителя полупериода тактовой частоты двигателя.
		Байт		1-й		2-й		3-й		4-й					5-й					6-й
		Значение	Адрес	06 hex	02 hex	Номер дви¬га-те¬ля	Множитель (1…255)	Контроль¬ная сумма
		Ответ модуля: возвращает принятые байты без изменений.
		Примечание к команде 6.
		Фактическая длительность полупериода тактовой частоты равна величине, заданной командой 3, умноженной на величину множителя, заданную данной командой.
		'''
		
		s = self.ATrtAddr +b'\x06\x02'+bytearray([stepper, multy])
		print("eSetMulty",end='')
		r = self.write_read(s)
	@add2queue
	def eWriteMarchIHoldICode(self,stepper, Im=0, Is=0):
		u'''
		Назначение: установка маршевого тока и тока удержания двигателей.
		Байт		1-й		2-й		3-й		4-й					5-й					6-й						7-й
		Значение	Адрес	07 hex	03 hex	Номер дви¬га¬те¬ля	Марше-вый ток (0…7)	Ток удер-жа¬ния (0…7)	Контроль¬ная сумма
		Ответ модуля: возвращает принятые байты без изменений.
		Зависимость тока от передаваемого данной командой номера ступени:
		Ступень	Ток, A
		0	0
		1	0,4
		2	0,8
		3	1,2
		4	1,6
		5	2,0
		6	2,5
		7	3,0
		Примечание к команде 7.
		Следует помнить, что ток обмоток двигателя ограничивается также их омическим сопротивлением.
		Например, для двигателя, имеющего сопротивление обмоток 10 Ом при напряжении питания 12 В,
		 будет иметь влияние только установка ступеней с 0 по 3. Установка ступеней с 4 по 
		 7 не будет оказывать влияние, т.к. ток будет ограничен сопротивлением обмоток на уровне 
		 1,2 А. Изменить значения ступеней регулирования тока можно только изменением номиналов электронных 
		 компонентов внутри модуля.
		'''
		
		self.Im[stepper-1] = Im
		if Is!=0:
			self.Is[stepper-1] = Is
		s = self.ATrtAddr +b'\x07\x03'+bytearray([stepper,Im, Is])
		print("eWriteMarchIHoldICode",end='')
		r = self.write_read(s)
	@add2queue
	def eLock(self,stepper,state):
		if state:
			self.eWriteMarchIHoldICode(stepper,Im=self.Im[stepper-1],Is=self.Is[stepper-1])
		else:
			self.eWriteMarchIHoldICode(stepper,Im=self.Im[stepper-1],Is=0)

	@add2queue
	def eSetPhaseMode(self, stepper, mode='1x'):
		u'''
		•	Команда 8.
		Назначение: задание режима возбуждения фаз двигателя.
		Байт	1-й	2-й	3-й	4-й	5-й	6-й
		Значение	Адрес	08 hex	02 hex	Номер дви¬га¬те¬ля	Режим	Контроль¬ная сумма
		Ответ модуля: возвращает принятые байты без изменений.
		Режим кодируется 2 младшими битами байта 5:
		00 – волновой режим полного шага (в каждый момент времени включена только 1 фаза);
		01 – нормальный режим полного шаг (в каждый момент времени включены одновременно 2 смежные фазы);
		1х – половинный шаг.
		Примечание к команде 8.
		Задание любого из режимов полного шага возможно только из режима половинного шага.
		Подача команды на переключение одного режима полного шага в другой режим полного шага
		без промежуточного выхода в режим половинного шага не даст никакого эффекта. По умолчанию,
		после включения питания, модуль инициализируется в режим половинного шага.
		'''
		#m = {'00':}
		self.write(b'\xFF\x08\x02'+bytearray([stepper])+b'\x10')
		self.write(b'\xFF\x08\x02'+bytearray([stepper, mode]))
		s = self.ATrtAddr +b'\x08\x02'+bytearray([stepper])+b'\x10'
		print("eSetPhaseMode",end='')
		r = self.write_read(s)

		s = self.ATrtAddr +b'\x08\x02'+bytearray([stepper, mode])
		print("eSetPhaseMode",end='')
		r = self.write_read(s)

	@add2queue
	def makeStepCCW(self, stepper=1, steps=0, waitUntilReady=True,threadWait=False):
		
		#state = self.eGetState(0.02)
		#print('State', state)
		#def f():
		self.eSetParams(stepper,'ccw_step',steps)
		self.eStart(stepper)
		if waitUntilReady:
			def wait():
				self.waitUntilReadyState = True
				while not self.forceStop and self.threadKillCounter<5:
					if not self.lockFlow:
						time.sleep(0.5)
						self.eGetState()
						state = self.SM_state
						#print("makeStepCCW_thread:",state)
						try:
							if state['SMs_state'] == 0:
								self.SM_position[stepper-1]+=steps
								self.waitUntilReadyState = False
								break
						except:
							pass
					else:
						time.sleep(0.5)

			if threadWait:
				threading.Thread(target=wait).start()
			else:
				wait()
			
	@add2queue
	def makeStepCW(self, stepper=1, steps=0, waitUntilReady=True,threadWait=False):
		#state = self.eGetState(0.02)

		#print('State', state)
		#def f():
		self.eSetParams(stepper,'cw_step',steps)
		self.eStart(stepper)
		
		if waitUntilReady:
			def wait():
				self.waitUntilReadyState = True
				while not self.forceStop and self.threadKillCounter<5:
					if not self.lockFlow:

						self.eGetState()
						state = self.SM_state
						#print("makeStepCW_thread:",state)
						#time.sleep(0.5)
						try:
							if state['SMs_state'] == 0:
								self.SM_position[stepper-1]-=steps
								self.waitUntilReadyState = False
								break
						except:
							#self.waitUntilReadyState = False
							traceback.print_exc()
					else:
						time.sleep(1)
			if threadWait:
				threading.Thread(target=wait).start()
			else:
				wait()
	@add2queue	
	def moveCCW(self, stepper=1):
		steps=0
		self.eSetParams(stepper,'ccw2stop',steps)
		self.eStart(stepper)
	@add2queue
	def moveCW(self, stepper=1):
		steps=0
		self.eSetParams(stepper,'cw2stop',steps)
		self.eStart(stepper)
	@add2queue
	def eGetState(self,delay=0.01):
		u'''
		Назначение: запрос состояния двигателей.
		Байт	1-й	2-й	3-й	4-й
		Значение	Адрес	04 hex	00 hex	Контроль¬ная сумма
		Ответ модуля:
		Байт	1-й	2-й	3-й	4-й	5-й	6-й	7-й
		Значение	Адрес	04 hex	08 hex	Текущее со¬стояние двигателей	Режим дви¬га¬те-ля №1	Счетчик шагов дви-га¬те¬ля №1 (млад¬шим бай¬том впе¬ред)

		Байт	8-й	9-й	10-й	11-й	12 -ой
		Значение	Режим дви¬га¬те-ля №2	Счетчик шагов дви-га¬те¬ля №2 (млад¬шим бай¬том впе¬ред)	Состояние концевых выключателей	Контроль¬ная сумма
		Байт текущего состояния двигателей:
		00 hex – оба двигателя остановлены;
		01 hex – вращается 1-й двигатель, второй остановлен;
		02 hex – вращается 2-й двигатель, первый остановлен;
		03 hex – оба двигателя вращаются.
		Байты режима двигателей – так же, как и в команде 2, 5-й байт.
		Состояние концевых выключателей:
		Байт состояния концевых выключателей
		Бит 7	Бит 6	Бит 5	Бит 4	Бит 3	Бит 2	Бит 1	Бит 0
		K2.2	K2.1	K1.2	K1.1	x	x	x	x
		Младшие 4 бита – незначащие.
		Примечание к команде 4.
		Счетчики шагов двигателей – счетчики количества шагов с учетом направления вращения, с момента включения питания модуля или последнего обнуления счетчика (командой 5). Емкость счетчиков – 2 байта. Пример 1. После включения питания модуля двигатель сделал 200 шагов в прямом направлении и 50 – обратном. Значение счетчика шагов составит 150.
		Пример 2. После включения питания модуля двигатель сделал 100 шагов в обратном направлении. Значение счетчика шагов составит 65436 (т.е. 65536 –100).

		'''
		if self.isConnected():
			
			s = self.ATrtAddr +b'\x04\x00'
			print("eGetState",end='')
			r = self.write_read(s)
			if not len(r)==12 and len(r)>2:
				n = self.ser.inWaiting()
				r+= self.ser.read(n)
			if len(r)==12:
				address,hex04,hex08,SMs_state, SM1_mode, SM1_steps0,SM1_steps1,SM2_mode, SM2_steps0,SM2_steps1, endstops, cSum = r
				
				
				s1 = int.from_bytes(bytearray([SM1_steps0,SM1_steps1]), byteorder='little')#,signed=False)
				if s1==0:
					SM1_steps = 0
				else:
					SM1_steps = 65536-s1
				s2 = int.from_bytes(bytearray([SM2_steps0,SM2_steps1]), byteorder='little')#,signed=False)
				#if s2==0:
				#	SM2_steps = 0
				#else:
				SM2_steps = s2
				#print([int(i) for i in format(endstops, "08b")])
				endstops = [int(i) for i in format(endstops, "08b")[:4]]
				
				if len(endstops)==4:
					self.SM_state = {'SMs_state':SMs_state,	'SM1_steps':SM1_steps, 'SM1_mode':SM1_mode,
															'SM2_steps':SM2_steps, 'SM2_mode':SM2_mode,
															'SM2_end1':endstops[1], 'SM2_end2':endstops[0],
															'SM1_end1':endstops[3], 'SM1_end2':endstops[2]}
					self.stateQueue.put(self.SM_state)							
					return self.SM_state
			else:
				print(len(r))
	
	def write_read(self,data,delay=0.3):
		self.lockFlow = True
		if self.isConnected():
			data = self.cSum(data)
			if len(data)>4:
				self.ser.write(data[:4])
				time.sleep(0.02)
				self.ser.write(data[4:])
			else:
				self.ser.write(data)
			print("\t>>",data)
			time.sleep(0.02)
			r = self.ser.readline()
			print("\t\t<<",r)
			self.lockFlow = False
			return r
		else:
			self.ser.close()

	def write(self, data,delay=0.03):
		self.lockFlow = True
		if self.isConnected():
			data = self.cSum(data)
			self.ser.write(data)
			time.sleep(delay)
		return data

	def read(self, len_check=0):
		r = 'None'
		if self.lockFlow:
			r = self.ser.read(1)
			time.sleep(0.03)
			total_len = 1
			if len_check!=0:
				for i in range(len_check):
					time.sleep(0.05)
					n = self.ser.inWaiting()
					r += self.ser.read(n)
					total_len += n
					#time.sleep(0.03)
					if total_len == len_check:
						break
		self.lockFlow = False
		try:
			self.cSumOk = self.cSum(r[-1:])[-1]==r[-1]
		except:
			self.forceStop = True
			self.lockFlow = True
			print('SMD_connectionError')
			self.eHardReset(3)
			time.sleep(5)
			self.eStop(3)
			self.lockFlow = False
			self.forceStop = False
			self.ser.flush()
			self.close()
			
			self.eOpenCOMPort()
		return r

	def cSum(self, command):
		c_sum = sum(int(x) for x in command)
		if c_sum>255:
			command += c_sum.to_bytes(2,byteorder='little')[1:]
		else:
			command += c_sum.to_bytes(1,byteorder='little')
		return command


if __name__ == "__main__":
	e = SMD004()

	port = list(list_ports.grep("0403:6001"))[0][0]
	e.eOpenCOMPort(port)
	#print(e.str2hex(b"Hello"))
	#print(e.str2hex(b"\n\r"))
	#print(''.join('{:02x}'.format(ord(c)) for c in 'Hello'))
	
	try:
		#e.eStartLeft()
		#f = lambda : e.eStop()
		
		#time.sleep(2)
		#e.eStop()
		e.eClearStep(3)
		e.eSetTactFreq(1,60)
		e.eSetMulty(1,1)

		
		#e.makeStepCCW(steps=600)
		#e.makeStepCW(steps=300)
		#e.eStartLeft()
		#time.sleep(4)
		#e.makeStepCW(steps=600)

		#e.eStartRight()
		#e.eGetState()
		#time.sleep(2)
		#e.eStop()
		#e.eGetState()
		#print('+'*10,e.isConnected())
		print('-'*50)
		e.eWriteMarchIHoldICode(1,1,0)
		e.eSetPhaseMode(1,10)
		#e.makeStepCW(1,2000,True)
		e.makeStepCCW(1,5000)
		for i in range(1000):
			e.eGetState()
			time.sleep(5)
		#state=e.eGetState()
		#while state[1][0]!=0:
		#	state=e.eGetState()
		#	print('State:',state)
		#e.makeStepCCW(1,50,True)
		'''
		threading.Timer(5,f).start()
		time.sleep(10)
		#e.moveCW(1)
		threading.Timer(5,f).start()
		#time.sleep(5)
		e.eStop()
		'''
		#e.makeStepCW(1,50,True)
		#e.makeStepCCW(1,50,True)
	except:
		traceback.print_exc()
		#e.ser.close()	
	time.sleep(10)
	e.eStop()
	e.close()
	print(e.isConnected())

