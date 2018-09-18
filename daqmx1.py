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
	NCaptures = 1

	def __init__(self,physicalChannel, limit = None, reset = False, N=1):
		self.NCaptures = N
		if type(physicalChannel) == type(""):
			self.physicalChannel = [physicalChannel]
		else:
			self.physicalChannel = physicalChannel
		self.numberOfChannel = physicalChannel.__len__()
		if limit is None:
			self.limit = dict([(name, (-10.0,10.0)) for name in self.physicalChannel])
		elif type(limit) == tuple:
			self.limit = dict([(name, limit) for name in self.physicalChannel])
		else:
			self.limit = dict([(name, limit[i]) for  i,name in enumerate(self.physicalChannel)])
		if reset:
			DAQmxResetDevice(physicalChannel[0].split(b'/')[0] )
	def configure(self):
		# Create one task handle per Channel
		taskHandles = dict([(name,TaskHandle(0)) for name in self.physicalChannel])
		for name in self.physicalChannel:
			DAQmxCreateTask(b"",byref(taskHandles[name]))
			DAQmxCreateAIVoltageChan(taskHandles[name],name,b"",DAQmx_Val_RSE,
									 self.limit[name][0],self.limit[name][1],
									 DAQmx_Val_Volts,None)
			DAQmxCfgSampClkTiming(taskHandles[name], "", 10000, DAQmx_Val_Rising,
				#DAQmx_Val_FiniteSamps,
				DAQmx_Val_ContSamps,
				10000	);

			#DAQmxCfgDigEdgeRefTrig(taskHandles[name], b"/Dev1/pfi0", DAQmx_Val_Rising, 100)
			DAQmxCfgDigEdgeStartTrig(taskHandles[name], b"/Dev1/pfi0", DAQmx_Val_Rising)
		#DAQmxCfgAnlgEdgeStartTrig(taskHandles[self.physicalChannel[0]],
		#	b"Dev1/ai0", DAQmx_Val_RisingSlope, 5)
		#DAQmxCfgSampClkTiming(taskHandles[self.physicalChannel[0]],"",51200.0,DAQmx_Val_Rising,DAQmx_Val_FiniteSamps,5120)
		
		self.taskHandles = taskHandles
	def readAll(self):
		return dict([(name,self.read(name)) for name in self.physicalChannel])
	def read_(self):
		data = self.readAll( )
		print(self.physicalChannel)
		out = [data[i] for i in self.physicalChannel]
		out = np.vstack(out).T
		print(out.shape,data)
		return out

	def read(self,name = None,N=None):
		if N is None:
			N = self.NCaptures
		if name is None:
			name = self.physicalChannel[0]
		taskHandle = self.taskHandles[name]
		DAQmxStartTask(taskHandle)
		data = numpy.zeros((N,), dtype=numpy.float64)
#		data = AI_data_type()
		read = int32()
		DAQmxReadAnalogF64(taskHandle,N,10.0,DAQmx_Val_GroupByChannel,data,N,byref(read),None)
		DAQmxStopTask(taskHandle)
		return data#[0]


if __name__ == '__main__':
	import time
	t0 = time.time()
	multipleAI = MultiChannelAnalogInput([b"Dev1/ai1",b"Dev1/ai3"], N=400)
	multipleAI.configure()
	out = []
	for i in range(10):
		t0 = time.time()
		data = multipleAI.readAll()
		#print(data)
		out.append(data)
		print(time.time()-t0)
	from pylab import *
	plot([0],'r')
	for d in out:
		plot(d[b'Dev1/ai1'],'r')
		plot(d[b'Dev1/ai3'],'b')
		draw()
	print(out)
	show()
	
