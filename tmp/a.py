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
    def __init__(self,physicalChannel, limit = None, reset = False):
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
            DAQmxResetDevice(physicalChannel[0].split(b'/')[0] )
    def configure(self):
        # Create one task handle per Channel
        taskHandles = dict([(name,TaskHandle(0)) for name in self.physicalChannel])
        for name in self.physicalChannel:
            DAQmxCreateTask(b"",byref(taskHandles[name]))
            DAQmxCreateAIVoltageChan(taskHandles[name],name,b"",DAQmx_Val_RSE,
                                     self.limit[name][0],self.limit[name][1],
                                     DAQmx_Val_Volts,None)
        self.taskHandles = taskHandles
    def readAll(self):
        return dict([(name,self.read(name)) for name in self.physicalChannel])
    def read(self,name = None):
        if name is None:
            name = self.physicalChannel[0]
        taskHandle = self.taskHandles[name]                    
        DAQmxStartTask(taskHandle)
        data = numpy.zeros((10000,), dtype=numpy.float64)
#        data = AI_data_type()
        read = int32()
        DAQmxReadAnalogF64(taskHandle,10000,10.0,DAQmx_Val_GroupByChannel,data,10000,byref(read),None)
        DAQmxStopTask(taskHandle)
        return data#[0]


if __name__ == '__main__':
    multipleAI = MultiChannelAnalogInput([b"Dev1/ai1",b"Dev1/ai3"])
    multipleAI.configure()
    data = multipleAI.readAll()
    print(data)
    from pylab import *
    plot(data[b'Dev1/ai1'],'r')
    plot(data[b'Dev1/ai3'],'b')
    show()
