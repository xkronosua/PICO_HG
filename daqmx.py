import numpy
import numpy as np
from PyDAQmx.DAQmxFunctions import *
from PyDAQmx.DAQmxConstants import *


class MultiChannelAnalogInput():
	"""Class to create a multi-channel analog input

	Usage: AI = MultiChannelInput(physicalChannel)
		physicalChannel: a string or a list of strings
	optional parameter: limit: tuple or list of tuples, the AI limit values
						reset: Boolean
	Methods:
		read(name), return the value of the input name
		readAll(), return a dictionary name:value
	"""
	Ncaptures = 1
	def __init__(self,physicalChannel, limit = None, reset = False, N=1):
		self.Ncaptures = N
		if type(physicalChannel) == type(""):
			self.physicalChannel = [physicalChannel]
		else:
			self.physicalChannel  =physicalChannel
		self.numberOfChannel = physicalChannel.__len__()
		if limit is None:
			self.limit = dict([(name, (-10.0,10.0)) for name in self.physicalChannel])
		elif type(limit) == tuple:
			self.limit = dict([(name, limit) for name in self.physicalChannel])
		else:
			self.limit = dict([(name, limit[i]) for  i,name in enumerate(self.physicalChannel)])
		if reset:
			DAQmxResetDevice(physicalChannel[0].split('/')[0] )
	def configure(self):
		# Create one task handle per Channel
		taskHandles = dict([(name,TaskHandle(0)) for name in self.physicalChannel])
		for name in self.physicalChannel:
			DAQmxCreateTask("",byref(taskHandles[name]))
			DAQmxCreateAIVoltageChan(taskHandles[name],name,"", DAQmx_Val_RSE,
									 self.limit[name][0],self.limit[name][1],
									 DAQmx_Val_Volts,None)
			DAQmxCfgSampClkTiming(taskHandles[name], "", 5000.0, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 1000);
			DAQmxCfgInputBuffer(taskHandles[name],100000)
		self.taskHandles = taskHandles
	def readAll(self):
		return dict([(name,self.read(name)) for name in self.physicalChannel])
	def read_(self):
		data = self.read()
		#data = data[data.keys()[0]]
		d1 = data[:len(data)//2]
		d2 = data[len(data)//2:]
		return np.array([d1,d2]).T
	def read(self, name=None):
		N = self.Ncaptures
		if name is None:
			name = self.physicalChannel[0]
		taskHandle = self.taskHandles[name]
		DAQmxStartTask(taskHandle)
		data = numpy.zeros((N*2,), dtype=numpy.float64)
#		data = AI_data_type()
		read = int32()
		DAQmxReadAnalogF64(taskHandle,N,10.0,DAQmx_Val_GroupByChannel,data,N*2,byref(read),None)
		DAQmxStopTask(taskHandle)
		return data


class MultiChannelAnalogInputSim():
	"""Class to create a multi-channel analog input

	Usage: AI = MultiChannelInput(physicalChannel)
		physicalChannel: a string or a list of strings
	optional parameter: limit: tuple or list of tuples, the AI limit values
						reset: Boolean
	Methods:
		read(name), return the value of the input name
		readAll(), return a dictionary name:value
	"""
	Ncaptures = 1
	def __init__(self,physicalChannel, limit=None, reset=False, N=1):
		self.Ncaptures = N
	def readAll(self, N=1):
		return dict([(name,(sum(bytearray(name))-633)+self.read(name, N=N)) for name in self.physicalChannel])
	def read(self, N=1, name=None):
		if N is None:
			N = self.Ncaptures

		data = [1,1,1,1,1,1,1,0,0,0,0,0,0]*N
		data*= np.random.rand(len(data))
		return data




if __name__ == '__main__':
	ports = [b"Dev1/ai3, Dev1/ai1"]
	multipleAI = MultiChannelAnalogInput(ports, N=10000)
	multipleAI.configure()
	data = multipleAI.read_()
	print(data)
	#print(data[ports[0]].shape)
