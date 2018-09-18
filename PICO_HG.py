#!/usr/bin/env python3
# _*_ coding: utf-8 _*_

#from ET1255 import *
#from smd004b import *
#from SMD_test2 import *


import os
#import qdarkstyle

os.environ['PYQTGRAPH_QT_LIB'] = 'PyQt5' 

from HAMEG import HAMEG
import signal
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
from scipy.signal import medfilt, find_peaks_cwt
import pyqtgraph as pg
import signal
import re
from serial.tools import list_ports
import serial
import configparser
import threading
from multiprocessing import Process

import sys
from PyQt5 import uic
import time
#from PyQt5.QtCore import QtCore.QTimer, QtCore.QThread, QtCore.SIGNAL
#from PyQt4.QtGui import QtGui.QApplication, QtGui.QMessageBox, QtGui.QTableWidgetItem, QtGui.QFileDialog

import subprocess
import sys, os
import threading
import time
import signal
import struct

from datetime import datetime

import SMD004_ as SMD004
import traceback
from TCD1304 import TCD1304

MODE = 'lab'
if len(sys.argv)>1:
	if sys.argv[1]=='sim':
		MODE = 'sim'

if MODE == 'sim':
	from daqmx1 import MultiChannelAnalogInputSim as MultiChannelAnalogInput
else:
	from daqmx1 import MultiChannelAnalogInput


def sigint_handler(*args):
	"""Handler for the SIGINT signal."""
	sys.stderr.write('\r')
	QtGui.QApplication.quit()


"""
import csv
with open('eggs.csv', 'w', newline='') as csvfile:
	spamwriter = csv.writer(csvfile, delimiter=' ',
							quotechar='|', quoting=csv.QUOTE_MINIMAL)
	spamwriter.writerow(['Spam'] * 5 + ['Baked Beans'])
	spamwriter.writerow(['Spam', 'Lovely Spam', 'Wonderful Spam'])

"""
def calc_peaks(y,width=np.arange(20,40)):
	#y=medfilt(y)
	g = find_peaks_cwt(y, width)
	l = find_peaks_cwt(-y, width)
	return g,l

def freq_calc(y_mask):
	HIGH = 0
	LOW = 0
	prev = None
	T = []
	duty_cycle = []
	start = y_mask[0]==0
	for i in y_mask:
		#print(i,HIGH,LOW)
		if prev == int(start) and i==int(not start):
			if HIGH>5 and LOW>5:
				T.append(HIGH+LOW)
				duty_cycle.append(HIGH/(HIGH+LOW))
				HIGH=0
				LOW=1
				#print(T,duty_cycle)
		elif i==0:
			LOW+=1
		elif i==1:
			HIGH+=1
		prev = i
	#print("T:",T,duty_cycle)
	T_mean = np.mean(T[1:])
	duty_cycle_mean = np.mean(duty_cycle[1:	])
	return 1/T_mean, duty_cycle_mean

def peak2peak(ref,signal,smooth_ref=3,mean_calc=np.median, ref_mode='triangle'):
	if ref_mode == 'triangle':
		y_df1 = np.insert(np.diff(medfilt(ref,smooth_ref)), 0, 0)
	else:
		d = ref.max()-ref.mean()
		y_df1 = ref-ref.mean()+d/10
	#print(y_df1,type(y_df1))
	HIGH=np.mean(signal[y_df1>0])
	LOW=np.mean(signal[y_df1<0])
	res = abs(mean_calc(HIGH)-mean_calc(LOW))
	STD = np.sqrt(np.std(signal[y_df1>0])**2 )#+ np.std(signal[y_df1<0])**2)
	f,duty_cycle = freq_calc(y_df1>0)
	#print(STD)
	return res,HIGH,LOW,STD,f,duty_cycle

def d4sigma(a,bg=[]):
	a = a[300:-300]
	x = np.arange(len(a))
	if len(bg)==0:
		a0 = np.mean([a[:200],a[-200:]])
	else:
		a = a-bg[:len(a)]
		a0 = min(a)
	x0 = ((a-a0)*x).sum()/(a-a0).sum()
	#print(x0,a0)
	return 4*np.sqrt(np.dot(abs(a-a0),(x-x0)**2)/abs(a-a0).sum())




class PICO_HG(QtGui.QMainWindow):
	ui = None
	stepperCOMPort = None
	ATrtAddr = 255
	currentAngle = 0
	prevMove = ([0, 0], [0, 0])
	line0, line1, line2, line3, line4, line5, line6, line7 = None, None, None, None, None, None, None, None
	lines = [line0, line1, line2, line3, line4, line5, line6, line7]
	lineCCD = None
	steppingTimer = None
	calibrTimer = None
	measTimer = None
	stepperStateTimer = None
	FWCalibrationCounter = 0
	FWCalibrationCounterList = []

	angleSensorSerial = None
	angleUpdateTimer = None
	angleUpdateThread = None
	
	lastDirection = 1
	tmpData = None
	num = 10
	calibrData=np.array([])
	measData=np.array([])
	intStepCounter = 0
	moveFlag = 0
	extLaserStrob = None
	extDataBuffer = None
	CCDlastTime = 0
	oscillo = None
	oscReadTimerTimer = None
	lastFiltChTime = None
	lastDivChange = time.time()
	oscilloLastMaster = None
	oscilloLastIndex = [-1,-1]
	oscilloSleep = False
	SM_position = [0,0]
	serOscilloThread = None
	oscilloRefreshPlot = False
	CCD_data = []
	MAESTRO = None
	MAESTRO_port = None
	readMAESTROTimer = None
	#et = ET1255()
	def __init__(self, parent=None):
		QtGui.QMainWindow.__init__(self, parent)
		#from mainwindow import Ui_mw
		self.ui = uic.loadUi("mainwindow.ui")#Ui_mw()
		#self.ui.setupUi(self)
		#self.show()
		self.ui.show()
		
		self.ui.closeEvent = self.closeEvent
		self.steppingTimer = QtCore.QTimer()
		self.calibrTimer = QtCore.QTimer()
		self.calibrHomeTimer = QtCore.QTimer()
		self.calibrFWTimer = QtCore.QTimer()
		self.measTimer = QtCore.QTimer()
		self.angleUpdateTimer = QtCore.QTimer()
		self.updateCCDTimer = QtCore.QTimer()
		
		self.SMD_endstopsTimer = QtCore.QTimer()

		self.stepperStateTimer = QtCore.QTimer()
		self.oscReadTimer = QtCore.QTimer()
		self.readMAESTROTimer = QtCore.QTimer()
		self.config = configparser.ConfigParser()
		self.config.read('settings.ini')
		print(sys.argv[0],self.config.sections())
		

		self.globalSettingsDict = self.config['GLOBAL']#.read('globalSettings.ini')
		self.updateSettingsTable()
		self.initUi()
		self.uiConnect()
		
		
		self.fileDialog = QtGui.QFileDialog(self)

		#self.initET1255()
		self.tmpData = []#[[0,0,0]]
		self.calibrData = np.array([])#[0,0,0])
		#self.SMD = SMD004()
		#self.extLaserStrob = laserStrob(0.1)
		self.oscillo = HAMEG()
		self.oscilloLastMaster = 'HAMEG'

		self.SMD = SMD004.SMD004()
		
		self.CCD = TCD1304()
		self.niPorts = [b"Dev1/ai1", b"Dev1/ai3"]
		self.niAI = MultiChannelAnalogInput(self.niPorts, N=1000)
		self.niAI.configure()

		#self.startLaserStrob(0.02)
	def CCD_connect(self,state):
		if state:
			self.CCD = TCD1304(trigger=self.ui.CCD_trigger.currentIndex(),
				integration=self.ui.CCD_exposure.value())
			self.CCD.start()
			self.pwCCD.setVisible(True)
			if not self.CCD.error_q.empty():
				print(self.CCD.error_q.get())
				self.CCD.stop()
				self.ui.CCD_connect.setStyleSheet('background-color:#222222;')
				self.ui.CCD_connect.setChecked(False)
			else:
				self.updateCCDTimer.start(500)
				self.ui.CCD_connect.setStyleSheet('background-color:green;')
		else:
			self.updateCCDTimer.stop()
			self.CCD.stop()
			self.ui.CCD_connect.setStyleSheet('background-color:#222222;')

	def onUpdateCCDTimer(self):
		#self.updateTimer.stop()
		tDiff = time.time() - self.CCDlastTime
		self.CCDlastTime = time.time()
		print('tDiff:\t',tDiff)
		t = time.time()
		if not self.CCD.error_q.empty():
			self.CCD.stop()
			self.ui.CCD_connect.setStyleSheet('background-color:#222222;')
			self.ui.CCD_connect.setChecked(False)
			
			while not self.CCD.error_q.empty():
				err = self.CCD.error_q.get()
				print(err)	
				
			return	

		stack = []#self.CCD.data_q.get()[0]
		cn = 0
		while not self.CCD.data_q.empty():
			stack.append( self.CCD.data_q.get()[0])
			cn += 1
		if len(stack)==0: return
		data = np.sum(stack,axis=0)/(cn+1)
		if sum(data)==0:
			self.CCD.stop()
			self.CCD = TCD1304(trigger=False,
				integration=self.ui.CCD_exposure.value())
			print('Trigger: Int.')
			self.CCD.start()
			time.sleep(0.2)
			self.CCD.stop()
			self.CCD = TCD1304(trigger=self.ui.CCD_trigger.currentIndex(),
				integration=self.ui.CCD_exposure.value())
			self.CCD.start()
			print('Trigger: Ext.')
			
		print("#",cn,cn/tDiff)
		d4s = d4sigma(data)
		#print(time.time()-t,d4s)
		self.lineCCD.setData(data)
		self.CCD_data = data
		self.ui.CCD_sum.setText(str(int(data.sum())))
		try:
			self.ui.D4Sigma.setText(str(int(d4s)))
		except ValueError:
			self.ui.D4Sigma.setText("nan")

	def updateSettingsTable(self):
		if 'GLOBAL' in self.config.sections():
			section = self.config['GLOBAL']
			self.ui.globalConfig.setVerticalHeaderLabels(list(section))
			for row,key in enumerate(section):
				print(row,key)
				for column,val in enumerate(section[key].split(',')):
					print(column,val)
					newitem = QtGui.QTableWidgetItem(val)
					self.ui.globalConfig.setItem(row,column,newitem)

		if 'FILTERS' in self.config.sections():
			section = self.config['FILTERS']
			self.ui.filtersTab.setVerticalHeaderLabels(list(section))
			for row,key in enumerate(section):
				print(row,key)
				for column,val in enumerate(section[key].split(',')):
					print(column,val)
					newitem = QtGui.QTableWidgetItem(val)
					self.ui.filtersTab.setItem(row,column,newitem)
		#for column, key in enumerate(section):
			#	newitem = QtGui.QTableWidgetItem(item)
			#	for row, item in enumerate(self.data[key]):
			#		newitem = QtGui.QTableWidgetItem(item)
			#		self.setItem(row, column, newitem)

			
	def saveConfig(self):
		if 'GLOBAL' in self.config.sections():
			section = self.config['GLOBAL']
			rows = self.ui.globalConfig.rowCount()
			columns = self.ui.globalConfig.columnCount()
			
			for row in range(rows):
				key = self.ui.globalConfig.verticalHeaderItem(row).text()
				val = []
				for column in range(columns):
					try:
						val.append(self.ui.globalConfig.item(row,column).text())
					except:
						break
				section[key] = ','.join(val)
				print(key,val)

		if 'FILTERS' in self.config.sections():
			section = self.config['FILTERS']
			rows = self.ui.filtersTab.rowCount()
			columns = self.ui.filtersTab.columnCount()
			
			for row in range(rows):
				key = self.ui.filtersTab.verticalHeaderItem(row).text()
				val = []
				for column in range(columns):
					try:
						val.append(self.ui.filtersTab.item(row,column).text())
					except:
						break
				section[key] = ','.join(val)
				print(key,val)

		with open('settings.ini', 'w') as configfile:
			self.config.write(configfile)
		'''	
			for row,key in enumerate(section):
				print(row,key)
				for column,val in enumerate(section[key].split(',')):
					print(column,val)
					newitem = QtGui.QTableWidgetItem(val)
					self.ui.globalConfig.setItem(row,column,newitem)

		if 'FILTERS' in self.config.sections():
			section = self.config['FILTERS']
			self.ui.globalConfig.setVerticalHeaderLabels(list(section))
			for row,key in enumerate(section):
				print(row,key)
				newitem = QtGui.QTableWidgetItem(key)
				self.ui.filtersTab.setItem(row,0,newitem)
				for column,val in enumerate(section[key].split(',')):
					print(column,val)
					newitem = QtGui.QTableWidgetItem(val)
					self.ui.filtersTab.setItem(row,column+1,newitem)
		'''

	


	def setStepperParam(self):
		pass

	def getCorrectedAngle(self):
		angle = self.ui.steppingAngle.value()
		#calibr = float(self.config['STEPPER1']['calibrationCoefficient']) #self.ui.steppingCalibr.value()
		#tmp_angle = int(angle*calibr)/calibr
		#self.ui.steppingAngle.setValue(tmp_angle)
		#return int(tmp_angle*calibr), tmp_angle
		return angle, angle

	def steps2angle(self, steps, motor=1):
		calibr = float(self.config['STEPPER'+str(motor)]['calibrationCoefficient']) #self.ui.steppingCalibr.value()
		return steps/calibr

	def prepInternCounter(self, direction):
		if direction != self.lastDirection:
			#self.SMD.eClearSteps()#SMD_ClearStep(self.ATrtAddr)
			self.lastDirection = direction

	def CCWSingleMove(self):
		steps, angle = self.getCorrectedAngle()
		self.angleSensorSerial.write(b"CCW:"+str(steps).encode('utf-8')+b"\n")
		r = self.angleSensorSerial.readline()   
		print(r)



	def CWSingleMove(self):
		steps, angle = self.getCorrectedAngle()
		self.angleSensorSerial.write(b"CW:"+str(steps).encode('utf-8')+b"\n")
		r = self.angleSensorSerial.readline()   
		print(r)
	
	def CCWMoveToStop(self):
		print("CCWMoveToStop")
		self.angleSensorSerial.write(b"CCW\n")
		r = self.angleSensorSerial.readline()
		print(r)
		
		
	def CWMoveToStop(self):
		print("CWMoveToStop")
		self.angleSensorSerial.write(b"CW\n")
		r = self.angleSensorSerial.readline()
		print(r)
		

	def CCWHome(self):
		print("CCWHome")
		self.angleSensorSerial.write(b"HOME:CCW\n")
		r = self.angleSensorSerial.readline()
		print(r)
		

	def CWHome(self):
		print("CWHome")
		self.angleSensorSerial.write(b"HOME:CW\n")
		r = self.angleSensorSerial.readline()
		print(r)

	def onCalibrHomeTimer(self):
		info = self.getNanoScatInfo(['zero','angle'])
		if info['zero'] == 1:
			print("Zero at:", info['angle'])
			self.stepperStop()
			self.updateAngle(0)
			self.setNanoScatState(angle=0)
			self.currentAngle = 0
			self.calibrHomeTimer.stop()
		else:
			print("angle:", info['angle'])
		

	def getCurrentFilter(self):
		row = self.ui.filtersTab.currentRow()
		try:
			name = self.ui.filtersTab.item(row,0).text()
		except:
			name = 'none'
		try:
			val = self.ui.filtersTab.item(row,1).text()
		except:
			val = 'none'
		#   return (None, None)
		return name, val, row



	def prevFilter(self):
		row = self.ui.filtersTab.currentRow()
		if row>0:
			print(b"FW:"+str(row-1).encode('utf-8')+b"\n")
			self.angleSensorSerial.write(b"FW:"+str(row-1).encode('utf-8')+b"\n")
			r = self.angleSensorSerial.readline()   
			print(r)
			self.ui.filtersTab.setCurrentCell(row-1,0)
			self.lastFiltChTime = datetime.now()
			#self.ui.oscilloVdiv2.setCurrentIndex(10)
			#self.oscillo.vdiv2(10)
			#ch2_v = self.oscillo.get_vdiv2()
			#print(ch2_v)
			return 1
		else:
			return 0

	def nextFilter(self):
		row = self.ui.filtersTab.currentRow()
		if row<5:
			print(b"FW:"+str(row+1).encode('utf-8')+b"\n")
			self.angleSensorSerial.write(b"FW:"+str(row+1).encode('utf-8')+b"\n")
			r = self.angleSensorSerial.readline()   
			print(r)
			self.lastFiltChTime = datetime.now()
			#self.ui.oscilloVdiv2.setCurrentIndex(3)
			#self.oscillo.vdiv2(3)
			#ch2_v = self.oscillo.get_vdiv2()
			#print(ch2_v)
			return 1
		else:
			return 0

		

	def stepperStop(self):
		self.angleSensorSerial.write(b"SM:STOP\n")
		r = self.angleSensorSerial.readline()   
		print(r)

	def stepperLock(self,state):
		if state:
			self.angleSensorSerial.write(b"SM:OFF\n")
		else:
			self.angleSensorSerial.write(b"SM:ON\n")
		r = self.angleSensorSerial.readline()   
		print(r)

	def setStepperSpeed(self):
		speed = self.ui.stepperSpeed.value()
		self.angleSensorSerial.write(b"SPEED:"+str(speed).encode('utf-8')+b"\n")
		r = self.angleSensorSerial.readline()   
		print(r)

	def updateAngle(self, new_angle=0, mode='None'):
		if self.sender().objectName() == "resetAngle": mode = "reset"
		if mode == "reset":
			self.currentAngle = 0
		elif mode == "set":
			self.currentAngle = new_angle  # in deg
			val = new_angle / float(self.config['GLOBAL']['anglecalibr'])
			self.angleSensorSerial.write(b"ANGLE:"+str(val).encode('utf-8')+b"\n")
			r = self.angleSensorSerial.readline()   
			print(r)
		else:
			try:
				self.currentAngle = new_angle*float(self.config['GLOBAL']['anglecalibr'])
			except:
				pass
		self.ui.currentAngle.setText(str(self.currentAngle))
		return self.currentAngle

	def setCustomAngle(self):
		dlg =  QtGui.QInputDialog(self)
		dlg.setInputMode( QtGui.QInputDialog.DoubleInput)
		dlg.setLabelText("Angle:")
		dlg.setDoubleDecimals(6)
		dlg.setDoubleRange(-999999,999999)
		ok = dlg.exec_()
		angle = dlg.doubleValue()
		if ok:
			self.updateAngle(angle, mode='set')
	def moveToAngle(self):
		dlg =  QtGui.QInputDialog(self)
		dlg.setInputMode( QtGui.QInputDialog.DoubleInput)
		dlg.setLabelText("Angle:")
		dlg.setDoubleDecimals(6)
		dlg.setDoubleRange(-999999,999999)
		ok = dlg.exec_()
		angle = dlg.doubleValue()
		if ok:
			if angle>self.currentAngle:
				pass


	

	def startCalibr(self, state):
		if state:
			self.calibrTimer.start(self.ui.measDelay.value())
			
			direction = 1 if self.ui.measForward.isChecked() else -1
			self.line0.setData(x=[],y=[])
			self.line0Energy.setData(x=[],y=[])
			self.line1.setData(x=[],y=[])
			self.line2.setData(x=[],y=[])
			self.line3.setData(x=[],y=[])
			
		else:


			self.calibrTimer.stop()
			self.calibrData=np.array([])

			
	def onCalibrTimer(self):
		r=[]
		#chanels=[0,1,2]
		#####################################################################################
		#r=et.getDataProcN(2,100,chanels,True)
		#name,tmp_filt,index = self.getCurrentFilter()
		#print('Filter:',name,tmp_filt,index)
		r = self.getData()
		y0 = r[1]
		y1 = r[0]
		#y0 = float(self.ui.CH2Val.text())
		#y1 = float(self.ui.CH1Val.text())
		energy = float(self.ui.MAESTRO_val.text())

		
		if len(self.calibrData)>0:
			
			self.calibrData = np.vstack((self.calibrData,[y0,y1,energy]))
		else:
			self.calibrData = np.array([y0,y1,energy])
		data = self.calibrData
		
		try:
			self.line0.setData(x=np.arange(len(data)), y=data[:,0])
			self.line0Energy.setData(x=np.arange(len(data)), y=data[:,2])
			self.line1.setData(x=np.arange(len(data)), y=data[:,1])
			self.updateAngle(len(data))
		except:
			traceback.print_exc()
		app.processEvents()  

	def startContMeas(self, state):
		print("Start_State:",state)
		if state:
			#self.angleUpdateTimer.stop()
			self.oscReadTimer.stop()
			
			with open(self.ui.measFilePath.text(), 'a') as f:
				f.write("\n#$Filter:"+self.getCurrentFilter()[0]+"\n")
				for row in range(6):
					name = self.ui.filtersTab.item(row,0).text()
					val = self.ui.filtersTab.item(row,1).text()
					f.write("#$\t"+str(row)+"\t"+name+"\t"+val+'\n')
				f.write("#$POWER:"+self.config['GLOBAL']['power'].replace('\n','\t')+'\n')
				f.write("#$WAVELENGTH:"+self.config['GLOBAL']['wavelength'].replace('\n','\t')+'\n')
				f.write("#$OBJECT:"+self.config['GLOBAL']['object'].replace('\n','\t')+'\n')
				f.write("#$STEP_MODE:"+self.ui.measMode.currentText()+'\n')
				if self.ui.CCD_connect.isChecked():
					self.updateCCDTimer.stop()
					f.write("#position\tcounter\tref\tsignal\tENERGY\tTime\td4sigma\tCCDsum\t"+"\t".join(np.arange(3648).astype(str))+"\n")
				else:
					f.write("#position\tcounter\tref\tsignal\tENERGY\tTime\n")
			
			self.measTimer.start(self.ui.measDelay.value())
			
			direction = 1 if self.ui.measForward.isChecked() else -1
			self.SMD_endstopsTimer.stop()
			if self.ui.measMode.currentText() == 'SM1_EndStop':
				if direction == 1:
					self.SM1_left2end()
				else:
					self.SM1_right2end()
				#self.SMD_endstopsTimer.stop()
			elif self.ui.measMode.currentText() == 'SM1_Step':
				#self.oscilloSleep = True
				#self.SMD_endstopsTimer.stop()
				self.SMD.SMDSleep = True
			elif self.ui.measMode.currentText() == 'SM2_EndStop':
				if direction == 1:
					self.SM2_left2end()
				else:
					self.SM2_right2end()
				#self.SMD_endstopsTimer.stop()
			elif self.ui.measMode.currentText() == 'SM2_Step':
				#self.oscilloSleep = True
				#self.SMD_endstopsTimer.stop()
				self.SMD.SMDSleep = True

			self.line0.setData(x=[],y=[])
			self.line0Energy.setData(x=[],y=[])
			self.line1.setData(x=[],y=[])
			
			self.ui.measStart.setStyleSheet('background-color:red;')
		else:

			self.SM1_stop()
			self.SM2_stop()
			#et.ET_stopStrobData()
			#try:
			
			#   self.SMD.eStop()
			#except:
			#   print('Err')
			self.measTimer.stop()
			self.oscReadTimer.start(self.ui.measDelay.value())
			if self.ui.measMode.currentText() == 'smallStep':
				#self.oscilloSleep = False
				self.SMD.SMDSleep = False

			#self.openAngleSensorPort(True)
			#self.angleUpdateTimer.start()
			self.calibrData=np.array([])
			#data = np.loadtxt(self.ui.measFilePath.text(),comments='#')
			#self.line2.setData(x=data[:,4], y=data[:,1])
			#self.line3.setData(x=data[:,-1], y=data[:,2])
			self.ui.measStart.setStyleSheet('background-color:green;')
			self.SMD_endstopsTimer.start(2000)
			if self.ui.CCD_connect.isChecked():
				self.updateCCDTimer.start(500)
			
			
			
			
	def onContMeasTimer(self):
		if self.SMD.waitUntilReadyState:
			print('wait')
		else:
			if not self.ui.measStart.isChecked():
				self.measTimer.stop()
				self.oscReadTimer.start(self.ui.measDelay.value())
			r=[]
			#chanels=[0,1,2]
			############################################################################
			#r = self.oscilloGetData()
			#if self.ui.measMode.currentText() == 'EndStop':
			#	pass
			#else:
			self.measTimer.stop()
			r = self.getData()
			y0 = r[1]
			y1 = r[0]
			y1_std = r[2]
			energy = float(self.ui.MAESTRO_val.text())
			#self.SMD.SMDSleep = True
			#r = self.oscilloGetData()
			
			direction = 1 if self.ui.measForward.isChecked() else -1
			print(self.ui.measForward.isChecked())
			counter=0
			if self.ui.measMode.currentText() == 'SM1_Step':
				if direction == 1:
					self.SM1_stepRight()
				else:
					self.SM1_stepLeft()
				self.onSMD_endstopsTimer()
				x = float(self.ui.SM1_position.text())
				counter = float(self.ui.SM1_stepsCounter.text())
			elif self.ui.measMode.currentText() == 'SM2_Step':
				if direction == 1:
					self.SM2_stepRight()
				else:
					self.SM2_stepLeft()
				self.onSMD_endstopsTimer()
				x = float(self.ui.SM2_position.text())
				counter = float(self.ui.SM2_stepsCounter.text())
			if self.ui.measMode.currentText() == 'SM1_EndStop':
				self.onSMD_endstopsTimer()
				x = float(self.ui.SM1_position.text())
				counter = float(self.ui.SM1_stepsCounter.text())
			elif self.ui.measMode.currentText() == 'SM2_EndStop':
				self.onSMD_endstopsTimer()
				x = float(self.ui.SM2_position.text())
				counter = float(self.ui.SM2_stepsCounter.text())
			#else: x=0
			self.SMD.SMDSleep = False
			
			#y0 = float(self.ui.CH2Val.text())
			#y1 = float(self.ui.CH1Val.text())
			
			#name,tmp_filt,index = self.getCurrentFilter()
			#print('Filter:',name,float(tmp_filt),index)
			#info = self.getNanoScatInfo(['angle', 'FW'])
			#angle = float(self.ui.currentAngle.text())#info['angle']
			#filt = info['FW']
			#freq = float(self.ui.oscilloFreq.text())
			#duty_cycle = float(self.ui.duty_cycle.text())
			#try:
				#self.currentAngle = angle#et.getAngle()
			#except:
			#   pass
			#print("angle", angle)
			#self.ui.currentAngle.setText(str(self.currentAngle))

			self.onUpdateCCDTimer()
			
			
			with open(self.ui.measFilePath.text(), 'a') as f:
					
					if self.ui.CCD_connect.isChecked():
						d4s = self.ui.D4Sigma.text()
						y1 = float(d4s)
						CCD_sum = self.ui.CCD_sum.text()
						f.write("\t".join([str(x),str(counter),str(y0),str(y1),str(energy),str(time.time()),d4s,CCD_sum])+"\t")
						f.write("\t".join(self.CCD_data.astype(str))+"\n")
					else:
						f.write("\t".join([str(x),str(counter),str(y0),str(y1),str(energy),str(time.time())])+"\n")			

					
			#print(self.measData)
			try:
				
				self.measData = np.vstack((self.measData,[x,y0,y1,energy]))
				data = self.measData
				self.line0.setData(x=data[:,0], y=data[:,1])
				self.line0Energy.setData(x=data[:,0], y=data[:,3])
				self.line1.setData(x=data[:,0], y=data[:,2])
				
			except:
				traceback.print_exc()
				self.measData = np.array([x,y0,y1,energy])
			
			#print(data)

			self.measTimer.start(self.ui.measDelay.value())
			
			#self.updateAngle(x)
			app.processEvents()  
		

	def _saveToBtn(self):
		filename = self.fileDialog.getSaveFileName(self)
		print(filename)
		try:
			self.ui.measFilePath.setText(filename)
		except:
			traceback.print_exc()
			self.ui.measFilePath.setText(filename[0])

	def openAngleSensorPort(self,state):
		if state:
			
			print(">>>>>>open")
			#et.openSerialPort('COM'+str(self.ui.angleSensorPort.value()))
			#self.angleSensorSerial = serial.Serial('COM'+str(self.ui.angleSensorPort.value()), baudrate=19200, dsrdtr=False)
			#self.angleSensorSerial = serial.Serial()
			port = '/dev/ttyUSB'+str(self.ui.angleSensorPort.value())
			self.angleSensorSerial = serial.Serial(port,baudrate=9600,timeout=5)#.port = port
			print(port)
			time.sleep(1)
			#self.angleSensorSerial.port = port
			#self.angleSensorSerial.baudrate = 19200
			#self.angleSensorSerial.timeout = 4
			#self.angleSensorSerial.open()
			#self.angleSensorSerial.setDTR(True)
			#self.angleSensorSerial.write(b'')
			#self.angleSensorSerial.flush()
			#self.angleSensorSerial.flushInput()
			#self.angleSensorSerial.flushOutput()
			a=self.angleSensorSerial.read(5)
			print("0>>",a)
			self.angleSensorSerial.write(b"STATE?\n")
			a=self.angleSensorSerial.readline()
			print("1>>",a)
			#self.angleSensorSerial.write(b'1')
			#self.setNanoScatState(frecMode=[20,20])
			#self.setNanoScatState(angleCalibrCoef=0.49)
			self.angleUpdateTimer.start(1000)
			#self.angleSensorSerial.write(b'2')
			

		else:
			self.angleUpdateTimer.stop()
			#self.angleSensorSerial.write(b'RM')
			self.angleSensorSerial.close()

	
	def getNanoScatInfo(self,keys=['angle']):
		if not self.angleSensorSerial.isOpen():
			self.openAngleSensorPort(True)
			return None
		else:
			outDict = {}
			self.angleSensorSerial.flush()
			self.angleSensorSerial.flushInput()
			self.angleSensorSerial.flushOutput()
			self.angleSensorSerial.write(b"STATE?\n")
			line = self.angleSensorSerial.readline().decode("utf-8")
			print(">>",line)
			if 'angle' in keys:
				try:
					outDict['angle'] = float(line.split('A:')[-1].split('\t')[0])
					self.updateAngle(outDict['angle'])
				except ValueError:
					outDict['angle'] = None
			if 'zero' in keys:
				try:
					outDict['zero'] = int(line.split('Z:')[-1].split('\t')[0])
				except ValueError:
					outDict['zero'] = None
			if 'FW' in keys:
				try:
					outDict['FW'] = int(line.split('FW:')[-1].split('\t')[0])
				except ValueError:
					outDict['FW'] = None

			if 'freqMode' in keys:
				try:
					outDict['freqMode'] = [float(i) for i in (line.split('\tFM:')[-1].split('\t')[0]).split('x')]
				except ValueError:
					outDict['freqMode'] = [None, None]
			return outDict

	def setNanoScatState(self,angle=None, frecMode=[None, None], angleCalibrCoef=None):
		if not self.angleSensorSerial.isOpen():
			self.openAngleSensorPort(True)
			return None
		else:
			res = None
			if not angle is None:
				res = self.angleSensorSerial.write(b'A'+str(angle).encode())
			if not angleCalibrCoef is None:
				res = self.angleSensorSerial.write(b'CC'+str(angleCalibrCoef).encode())

			if sum([i is None for i in frecMode]) != 2:
				res = self.angleSensorSerial.write(b'FM'+str(frecMode[0]).encode()+b'x'+str(frecMode[1]).encode())
			print(res)
			return 1

		

	def onAngleUpdateTimer(self):
		
		info = self.getNanoScatInfo(['angle','FW'])
		angle = info['angle']
		filt = info['FW'] 
		try:
			#self.currentAngle = angle#et.getAngle()
			self.ui.filtersTab.setCurrentCell(filt,0)
		except:
			pass
		print("angle", angle)
		self.updateAngle(angle)
		#self.ui.currentAngle.setText(str(self.currentAngle))

	def checkStepperState(self):
		pass
	#def setADCAmplif(self, value):
		#et.ET_SetAmplif(value)

	def cleanPlot(self):
		self.calibrData=np.array([])
		self.measData=np.array([])
		self.line0.setData(x=[], y=[])
		self.line1.setData(x=[], y=[])

	

	def setLaserFreq(self, value):
		print('SetLaserFreq:')
		self.angleSensorSerial.write(b'f'+str(value).encode('ascii')+b'\n')

	def closeEvent(self, event):
		print("event")
		reply = QtGui.QMessageBox.question(self, 'Message',
			"Are you sure to quit?", QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)

		if reply == QtGui.QMessageBox.Yes:
			try:
				if self.angleSensorSerial:
					self.angleSensorSerial.close()
				self.oscillo.disconnect()
				self.oscillo.close()
				self.SMD.close()
				self.MAESTRO.close()
				#self.SMD_endstopsTimer.stop()

			except:
				traceback.print_exc()
			
			
			event.accept()
			#self.stopLaserStrob()
			#self.stopDataBuffering()
		else:
			event.ignore()
	def oscilloConnect(self, state):
		source = self.ui.dataSource.currentText()
		if source == 'HAMEG':


			if state:
				self.oscillo.connect()
				r = self.oscillo.init()
				print(r)
				r = self.oscillo.vers()
				print(r)
				ch1_v = self.oscillo.get_vdiv1()
				if self.ui.CH1_AC_DC.isChecked():
					ch1_v -= 64
				print(ch1_v)
				self.ui.oscilloVdiv1.setCurrentIndex(ch1_v)
				yp1 = self.oscillo.get_ypos1()
				print(yp1)
				self.ui.oscilloYpos1.setValue(yp1)

				ch2_v = self.oscillo.get_vdiv2()
				if self.ui.CH2_AC_DC.isChecked():
					ch1_v -= 64
				print(ch2_v)
				self.ui.oscilloVdiv2.setCurrentIndex(ch2_v)
				yp2 = self.oscillo.get_ypos2()
				self.oscilloLastIndex = [ch1_v, ch2_v]
				print(yp2)
				self.ui.oscilloYpos2.setValue(yp2)

				td = self.oscillo.get_tdiv()
				print(td)
				self.ui.oscilloTdiv.setCurrentIndex(td)

				self.oscReadTimer.start(1000)

			else:
				self.oscReadTimer.stop()
				self.oscillo.disconnect()
				self.oscillo.close()
		else:
			print('NI_power')
			if state:
				self.oscReadTimer.start(300)
			else:
				self.oscReadTimer.stop()
	"""
	def oscilloConnect(self, state):
		if state:
			self.oscillo.connect()
			r = self.oscillo.init()
			print(r)
			r = self.oscillo.vers()
			print(r)
			ch1_v = self.oscillo.get_vdiv1()
			print(ch1_v)
			self.ui.oscilloVdiv1.setCurrentIndex(ch1_v)
			yp1 = self.oscillo.get_ypos1()
			print(yp1)
			self.ui.oscilloYpos1.setValue(yp1)
			
			ch2_v = self.oscillo.get_vdiv2()
			print(ch2_v)
			self.ui.oscilloVdiv2.setCurrentIndex(ch2_v)
			yp2 = self.oscillo.get_ypos2()
			self.oscilloLastIndex = [ch1_v, ch2_v]
			print(yp2)
			self.ui.oscilloYpos2.setValue(yp2)

			td = self.oscillo.get_tdiv()
			print(td)
			self.ui.oscilloTdiv.setCurrentIndex(td)
			
			self.oscReadTimer.start(1000)
			
			self.oscilloConnected = True
			self.oscilloStarted = False
			self.oscilloSleep = False

			'''
			def read_from_CH():
				while not self.oscilloStarted:
					self.oscilloStarted = True
					while self.oscilloConnected:
						if not self.oscilloSleep:
							try:
								#print('$'*100)
								self.oscilloGetData()

								time.sleep(0.1)
								#SMs_state,(SM1_steps, SM1_mode),(SM2_steps, SM2_mode),endstops
								#print("@"*10,state,"@"*10)
								#self.handle_data(data)
							except serial.serialutil.SerialException:
								print('----------')
			self.serOscilloThread = threading.Thread(target=read_from_CH)
			self.serOscilloThread.start()
			'''
			#return self.ser.isOpen()
			self.oscReadTimer.start(self.ui.measDelay.value())
			self.ui.oscilloConnect.setStyleSheet('background-color:red;')
		else:
			self.oscilloConnected = False	
			#self.serOscilloThread.cancel()
			self.oscilloStarted = False
			self.oscReadTimer.stop()
			self.oscillo.disconnect()
			self.oscillo.close()
			self.oscilloSleep = True
			self.ui.oscilloConnect.setStyleSheet('background-color:green;')
	"""

	def oscilloGetData(self):
		div = ( 0.001,0.002,0.005,
				0.01, 0.02, 0.05,
				0.1,  0.2,  0.5,
				1,    2,    5,
				10,   20)
		div_t = self.ui.oscilloTdiv.currentText()
		print(div_t)
		tdiv_val, t_order = div_t.split(' ')
		
		scales={'ns':1e-9,'mks':1e-6,'ms':1e-3,'s':1}
		tdiv_val = float(tdiv_val)*scales[t_order]

		res1 = self.oscillo.rdwf1(shift=self.ui.oscilloRWShift.value())
		sig = np.array([int(i) for i in res1])
		self.line2.setData(x=np.arange(len(sig)),y=sig)
		vdiv1 = self.ui.oscilloVdiv1.currentIndex()
		

		res2 = self.oscillo.rdwf2(shift=self.ui.oscilloRWShift.value())
		ref = np.array([int(i) for i in res2])
		self.line3.setData(x=np.arange(len(ref)),y=ref)
		vdiv2 = self.ui.oscilloVdiv2.currentIndex()
		
		dx=(tdiv_val*10)/2048
		print("HAMEG_region:",self.lr.getRegion())
		b = ref[int(self.lr.getRegion()[0]):int(self.lr.getRegion()[1])]
		try:
			bg=ref[self.lr.getRegion()[1]:]
		except:
			bg=ref[20:]
		
		
		if self.ui.oscilloAuto.isChecked():
			index2 = self.ui.oscilloVdiv2.currentIndex()
			if b.min()<20 or b.max()>240:
				if index2<12:
					self.oscillo.vdiv2(index2+1)
					self.ui.oscilloVdiv2.setCurrentIndex(index2+1)
			elif abs(b.max() - b.min())<50:
				 
				if index2>1:
					self.oscillo.vdiv2(index2-1)
					self.ui.oscilloVdiv2.setCurrentIndex(index2-1)
			
				
		
		Vpp2=np.trapz(b-np.mean(bg)*np.ones(len(b)),dx=dx)
		
			
		#print("bg>",bg,"<",np.mean(bg), b,len(b), b-np.mean(bg)*np.ones(len(b)))
		
		b = sig[int(self.lr.getRegion()[0]):int(self.lr.getRegion()[1])]
		try:
			bg=sig[self.lr.getRegion()[1]:]
		except:
			bg=sig[20:]
		
		if self.ui.oscilloAuto.isChecked():
			index1 = self.ui.oscilloVdiv1.currentIndex() 
			if b.min()<20 or b.max()>240:
				if index1<12:
					self.oscillo.vdiv1(index1+1)
					self.ui.oscilloVdiv1.setCurrentIndex(index1+1)
			elif abs(b.max() - b.min())<50:
				if index1>1:
					self.oscillo.vdiv1(index1-1)
					self.ui.oscilloVdiv1.setCurrentIndex(index1-1)
		
		
		Vpp1=np.trapz(b-np.mean(bg)*np.ones(len(b)),dx=dx)
		
		val1 = self.oscillo.dig2Volts(Vpp1, div[vdiv1])#(Vpp1)/256*(div[index1]*8)*1.333333
		STD1 = 0
		self.ui.CH1Val.setText(str(round(val1,13)))


		val2 = self.oscillo.dig2Volts(Vpp2, div[vdiv2])#(Vpp2)/256*(div[index2]*8)*1.333333
		self.ui.CH2Val.setText(str(round(val2,13)))

		
		return [val1,val2,STD1]
	
	'''
	def onOscReadTimer(self):
		self.oscReadTimer.stop()
		try:
			res = self.oscilloGetData()
		except AttributeError:
			res=[np.nan,np.nan,np.nan]

		self.oscReadTimer.start(self.ui.measDelay.value())
		print(res)
	'''
		
	def onOscReadTimer(self):
		res = self.getData()
		print(res)

	def oscilloAutoSet(self):
		self.oscilloSleep = True
		r = self.oscillo.autoset()
		print(r)
		ch1_v = self.oscillo.get_vdiv1()
		print(ch1_v)
		self.ui.oscilloVdiv1.setCurrentIndex(ch1_v)
		yp1 = self.oscillo.get_ypos1()
		print(yp1)
		self.ui.oscilloYpos1.setValue(yp1)
		
		ch2_v = self.oscillo.get_vdiv2()
		print(ch2_v)
		self.ui.oscilloVdiv2.setCurrentIndex(ch2_v)
		yp2 = self.oscillo.get_ypos2()
		print(yp2)
		self.ui.oscilloYpos2.setValue(yp2)

		td = self.oscillo.get_tdiv()
		print(td)
		self.ui.oscilloTdiv.setCurrentIndex(td)
		self.oscilloSleep = False
		
	def oscilloSet(self):
		self.oscilloSleep = True
		v1 = self.ui.oscilloVdiv1.currentIndex()
		r = self.oscillo.vdiv1(v1)
		print(r)
		v2 = self.ui.oscilloVdiv2.currentIndex()
		r = self.oscillo.vdiv2(v2)
		print(r)

		td = self.ui.oscilloTdiv.currentIndex()
		r = self.oscillo.tdiv(td)
		print(r)

		y1 = self.ui.oscilloYpos1.value()
		r = self.oscillo.ypos1(y1)
		print(r)

		y2 = self.ui.oscilloYpos2.value()
		r = self.oscillo.ypos2(y2)
		print(r)
		self.oscilloSleep = False
		
	def SMD_connect(self,state):
		if state:
			port = list(list_ports.grep("0403:6001"))[0][0]
			
			self.SMD.eOpenCOMPort(port)
			time.sleep(2)
			#print(self.SMD.str2hex(b"Hello"))
			#print(self.SMD.str2hex(b"\n\r"))
			self.SMD.eSetTactFreq(1,224)
			self.SMD.eSetTactFreq(2,80)
			#self.SMD.eClearStep(3)
			self.SMD.eSetMulty(1,2)
			self.SMD.eSetMulty(2,2)
			self.SMD.eWriteMarchIHoldICode(1,3,2)
			self.SMD.eWriteMarchIHoldICode(2,2,1)
			self.SMD.eSetPhaseMode(1,0)
			self.SMD.eSetPhaseMode(2,0)
			self.ui.SM1_position.setText(str(round(self.SMD.SM_position[0],6)))
			self.ui.SM2_position.setText(str(round(self.SMD.SM_position[1],6)))
			self.SMD_endstopsTimer.start(2000)
			self.ui.SMD_connect.setStyleSheet('background-color:red;')
		
		else:
			self.SMD.close()
			self.SMD_endstopsTimer.stop()
			self.ui.SMD_connect.setStyleSheet('background-color:green;')
			
	def onSMD_endstopsTimer(self):
		self.SMD.commandQueue.join()
		self.SMD.eGetState()
		state = self.SMD.SM_state

		if not self.SMD.cSumOk:
			self.ui.SMD_connect.setStyleSheet('background-color:orange;')
		print('State:',state)
		self.SMD.threadKillCounter = 0
		self.ui.SM1_leftEndstop.setChecked(not  state['SM1_end1'])
		self.ui.SM1_rightEndstop.setChecked(not state['SM1_end2'])
		self.ui.SM1_stepsCounter.setText(str(state['SM1_steps']))
		self.ui.SM1_position.setText(str(round(self.SMD.SM_position[0],6)))
		
		self.ui.SM2_leftEndstop.setChecked(not  state['SM2_end1'])
		self.ui.SM2_rightEndstop.setChecked(not state['SM2_end2'])
		self.ui.SM2_stepsCounter.setText(str(state['SM2_steps']))
		self.ui.SM2_position.setText(str(-round(self.SMD.SM_position[1],6)))
		
		self.ui.SM1_moving.setChecked(state['SMs_state']==1 or state['SMs_state']==3)
		self.ui.SM2_moving.setChecked(state['SMs_state']==2 or state['SMs_state']==3)


	def SM1_stop(self):
		#self.SMDSleep = True
		self.SMD.eStop(1)
	def SM2_stop(self):
		self.SMD.eStop(2)
	def SMs_stop(self):
		#self.SMDSleep = True
		self.SMD.eStop(3)


	def stepCalc(self,stepper,position,direction=1):
		step_init = 0
		if stepper==1:
			tab = np.loadtxt('SM1_step.table')
		if stepper==2:
			tab = np.loadtxt('SM2_step.table')

		step=tab[0,1]
		step_init = step														      							
		if direction==1:	       
			if position > tab[-1,0]:   
				step = tab[-1,1]
			for i in range(len(tab)-1):
				if tab[i+1,0]>position:
					step = tab[i,1]    
					if position+step>tab[i+1,0]:
						step = tab[i+1,0]-position
						if step <= 4: step = step_init
					break			 
		else:				     
			if position < tab[0,0]:	       
				step = tab[0,1]					   
			for i in range(len(tab)-1):				     
					if tab[i+1,0]>=position:			      
						step = tab[i,1]				   
						if position-step<tab[i,0]:			
							step = position-tab[i,0]
							if step <= 4: step = step_init
						break			  
													  
		return step

	def SM1_stepLeft(self):
		#self.SMDSleep = True
		calibr = float(self.config['GLOBAL']['SM1_calibr_coef'])
		real_step = self.ui.SM1_step.value()
		
		if self.ui.SM1_tabStep.isChecked():
			pos = float(self.ui.SM1_position.text())
			real_step = self.stepCalc(stepper=1,position=pos,direction=0)
			self.ui.SM1_step.setValue(real_step)

		steps = round(real_step*calibr)
		self.SMD.makeStepCW(1,steps,True,threadWait=True)
		#self.SMDSleep = False
		state=self.SMD.SM_state#['SMs_state']
		#while state!=0:
		print('State:',self.SMD.SM_state)
		#self.SMD.SM_position[0] -= real_step
		self.ui.SM1_position.setText(str(round(self.SMD.SM_position[0],6)))
		self.ui.SM1_leftEndstop.setChecked(not  state['SM1_end1'])
		self.ui.SM1_rightEndstop.setChecked(not state['SM1_end2'])
		self.ui.SM1_stepsCounter.setText(str(state['SM1_steps']))
	
	def SM2_stepLeft(self):
		#self.SMDSleep = True
		calibr = self.config['GLOBAL']['SM2_calibr_coef']
		calibr = float(calibr.split(',')[0])
		real_step = self.ui.SM2_step.value()
		if self.ui.SM2_tabStep.isChecked():
			pos = float(self.ui.SM2_position.text())
			real_step = self.stepCalc(stepper=2,position=pos,direction=0)
			self.ui.SM2_step.setValue(real_step)

		steps = round(real_step*calibr)
		self.SMD.makeStepCCW(2,steps,True,threadWait=0)
		#self.SMDSleep = False
		state=self.SMD.SM_state#['SMs_state']
		#while state!=0:
		#	print('State:',self.SMD.SM_state)
		#self.SM_position[1] -= real_step
		self.ui.SM2_position.setText(str(-round(self.SMD.SM_position[1],6)))
		self.ui.SM2_leftEndstop.setChecked(not  state['SM2_end1'])
		self.ui.SM2_rightEndstop.setChecked(not state['SM2_end2'])
		self.ui.SM2_stepsCounter.setText(str(state['SM2_steps']))
	

	def SM1_stepRight(self):
		#self.SMDSleep = True
		calibr = float(self.config['GLOBAL']['SM1_calibr_coef'])
		real_step = self.ui.SM1_step.value()
		if self.ui.SM1_tabStep.isChecked():
			pos = float(self.ui.SM1_position.text())
			real_step = self.stepCalc(stepper=1,position=pos,direction=1)
			self.ui.SM1_step.setValue(real_step)

		steps = round(real_step*calibr)
		self.SMD.makeStepCCW(1,steps,True,threadWait=True)
		#self.SMDSleep = False
		state=self.SMD.SM_state#['SMs_state']
		#while state!=0:
		print('State:',self.SMD.SM_state)
		#self.SM_position[0] += real_step
		self.ui.SM1_position.setText(str(round(self.SMD.SM_position[0],6)))
		self.ui.SM1_leftEndstop.setChecked(not  state['SM1_end1'])
		self.ui.SM1_rightEndstop.setChecked(not state['SM1_end2'])
		self.ui.SM1_stepsCounter.setText(str(state['SM1_steps']))

	def SM2_stepRight(self):
		#self.SMDSleep = True
		calibr = self.config['GLOBAL']['SM2_calibr_coef']
		calibr = float(calibr.split(',')[0])
		real_step = self.ui.SM2_step.value()
		if self.ui.SM2_tabStep.isChecked():
			pos = float(self.ui.SM2_position.text())
			real_step = self.stepCalc(stepper=2,position=pos,direction=1)
			self.ui.SM2_step.setValue(real_step)

		steps = round(real_step*calibr)
		self.SMD.makeStepCW(2,steps,True,threadWait=0)
		
		#self.SMDSleep = False
		state=self.SMD.SM_state#['SMs_state']
		#while state!=0:
		#print('State:',self.SMD.SM_state)
		#self.SM_position[1] += real_step
		self.ui.SM2_position.setText(str(-round(self.SMD.SM_position[1],6)))
		self.ui.SM2_leftEndstop.setChecked(not  state['SM2_end1'])
		self.ui.SM2_rightEndstop.setChecked(not state['SM2_end2'])
		self.ui.SM2_stepsCounter.setText(str(state['SM2_steps']))

	def SM1_left2end(self):
		#self.SMDSleep = True
		calibr = float(self.config['GLOBAL']['SM1_calibr_coef'])
		real_step = self.ui.SM1_step.value()
		steps = round(real_step*calibr)
		
		self.SMD.moveCW(1)
		#self.SMDSleep = False
		#state=self.SMD.eGetState()
		#f = lambda : e.eStop()
		#while state[1][0]!=0:
		#	state=self.SMD.eGetState()
		#	print('State:',state)
		#self.SM_position[0] = 0
		#self.ui.SM1_position.setText(str(round(self.SM_position[0],6)))

	def SM2_left2end(self):
		#self.SMDSleep = True
		calibr = self.config['GLOBAL']['SM2_calibr_coef']
		calibr = float(calibr.split(',')[0])
		real_step = self.ui.SM2_step.value()
		steps = round(real_step*calibr)
		
		self.SMD.moveCCW(2)
		#self.SMDSleep = False
		#state=self.SMD.eGetState()
		#f = lambda : e.eStop()
		#while state[1][0]!=0:
		#	state=self.SMD.eGetState()
		#	print('State:',state)
		#self.SMD.SM_position[2] = 0
		#self.ui.SM1_position.setText(str(round(self.SM_position[0],6)))

	def SM1_right2end(self):
		calibr = float(self.config['GLOBAL']['SM1_calibr_coef'])
		real_step = self.ui.SM1_step.value()
		steps = round(real_step*calibr)
		#self.SMDSleep = True
		self.SMD.moveCCW(1)
		#self.SMDSleep = False
		#state=self.SMD.eGetState()
		#while state[1][0]!=0:
		#	state=self.SMD.eGetState()
		#	print('State:',state)
		#self.SMD.SM_position[0] = 0
		#self.ui.SM1_position.setText(str(round(self.SM_position[0],6)))

	def SM2_right2end(self):
		calibr = self.config['GLOBAL']['SM2_calibr_coef']
		calibr = float(calibr.split(',')[0])
		real_step = self.ui.SM2_step.value()
		steps = round(real_step*calibr)
		#self.SMDSleep = True
		self.SMD.moveCW(2)
		
		#self.SMDSleep = False
		#state=self.SMD.eGetState()
		#while state[1][0]!=0:
		#	state=self.SMD.eGetState()
		#	print('State:',state)
		#self.SM_position[0] = 0
		#self.ui.SM1_position.setText(str(round(self.SM_position[0],6)))

	def SM1_reset(self):
		self.SMD.SM_position[0] = 0
		self.ui.SM1_position.setText(str(round(self.SM_position[0],6)))
		self.SMD.eClearStep(1)

	def SM2_reset(self):
		self.SMD.SM_position[1] = 0
		self.ui.SM2_position.setText(str(round(self.SM_position[0],6)))
		self.SMD.eClearStep(2)
	
	def SM1_moveTo(self):
		dlg =  QtGui.QInputDialog(self)
		dlg.setInputMode( QtGui.QInputDialog.DoubleInput)
		dlg.setLabelText("Position:")
		dlg.setDoubleDecimals(6)
		dlg.setDoubleRange(-999999,999999)
		ok = dlg.exec_()
		to_pos = dlg.doubleValue()
		calibr = float(self.config['GLOBAL']['SM1_calibr_coef'])
		
		if ok:
			if to_pos>self.SMD.SM_position[0]:
				real_step = abs(to_pos-self.SMD.SM_position[0])
				steps = round(real_step*calibr)
				self.SMD.makeStepCCW(1,steps,True)
				state=self.SMD.eGetState()
				#while state[1][0]!=0:
				#	state=self.SMD.eGetState()
				#	print('State:',state)
				#self.SMD.SM_position[0] += real_step
			elif to_pos<self.SMD.SM_position[0]:
				real_step = abs(to_pos-self.SMD.SM_position[0])
				steps = round(real_step*calibr)
				self.SMD.makeStepCW(1,steps,True)
				state=self.SMD.eGetState()
				#while state[1][0]!=0:
				#	state=self.SMD.eGetState()
				#	print('State:',state)
				#self.SMD.SM_position[0] -= real_step
			self.ui.SM1_position.setText(str(round(self.SMD.SM_position[0],6)))

	def SM2_moveTo(self):
		dlg =  QtGui.QInputDialog(self)
		dlg.setInputMode( QtGui.QInputDialog.DoubleInput)
		dlg.setLabelText("Position:")
		dlg.setDoubleDecimals(6)
		dlg.setDoubleRange(-999999,999999)
		ok = dlg.exec_()
		to_pos = dlg.doubleValue()
		try:
			calibr = float(self.config['GLOBAL']['SM2_calibr_coef'])
		except:
			calibr = float(self.config['GLOBAL']['SM2_calibr_coef'].replace(',',''))
		if ok:
			if to_pos>-self.SMD.SM_position[1]:
				real_step = abs(to_pos+self.SMD.SM_position[1])
				steps = round(real_step*calibr)
				self.SMD.makeStepCW(2,steps,True)
				state=self.SMD.eGetState()
				#while state[1][0]!=0:
				#	state=self.SMD.eGetState()
				#	print('State:',state)
				#self.SMD.SM_position[0] += real_step
			elif to_pos<-self.SMD.SM_position[1]:
				real_step = abs(to_pos+self.SMD.SM_position[1])
				steps = round(real_step*calibr)
				self.SMD.makeStepCCW(2,steps,True)
				state=self.SMD.eGetState()
				#while state[1][0]!=0:
				#	state=self.SMD.eGetState()
				#	print('State:',state)
				#self.SMD.SM_position[0] -= real_step
			self.ui.SM2_position.setText(str(round(self.SMD.SM_position[1],6)))


	def SM1_lock(self,state):
		self.SMD.eLock(1,state)

	def SM2_lock(self,state):
		self.SMD.eLock(2,state)
		#self.SMD.eHardReset(2)


	def SM1_speed(self,val):
		#if(val<255):
		self.ui.SM1_speed_val.setValue(val)
		self.SMD.eSetTactFreq(1,val)
		#elif(val>255):
		#	self.SMD.eSetMulty(1,2)
		#	self.SMD.eSetTactFreq(1,val-255)
		self.SMD.eSetParams(stepper=1,steps=0,prev=True)

	def SM2_speed(self,val):
		#if(val<255):
		self.ui.SM2_speed_val.setValue(val)
		self.SMD.eSetTactFreq(2,val)
		#elif(val>255):
		#	self.SMD.eSetMulty(1,2)
		#	self.SMD.eSetTactFreq(1,val-255)
		self.SMD.eSetParams(stepper=2,steps=0,prev=True)

	def SM1_setPosition(self):
		dlg =  QtGui.QInputDialog(self)
		dlg.setInputMode( QtGui.QInputDialog.DoubleInput)
		dlg.setLabelText("SetPosition:")
		dlg.setDoubleDecimals(6)
		dlg.setDoubleRange(-999999,999999)
		ok = dlg.exec_()
		to_pos = dlg.doubleValue()

		self.ui.SM1_position.setText(str(to_pos))
		self.SMD.SM_position[0] = int(to_pos)

	def SM2_setPosition(self):
		dlg =  QtGui.QInputDialog(self)
		dlg.setInputMode( QtGui.QInputDialog.DoubleInput)
		dlg.setLabelText("SetPosition:")
		dlg.setDoubleDecimals(6)
		dlg.setDoubleRange(-999999,999999)
		ok = dlg.exec_()
		to_pos = dlg.doubleValue()

		self.ui.SM2_position.setText(str(to_pos))
		self.SMD.SM_position[1] = int(to_pos)
	
	def connectMAESTRO(self,state):
		if state:
			
			if self.MAESTRO:
				self.MAESTRO.close()
			else:
				ports = list(list_ports.grep("0403:6001"))
				self.MAESTRO_port = '/dev/ttyUSB0'
				for i in ports:
					print(i.description)
					if i.description == 'FT232R USB UART':
						self.MAESTRO_port = i.device
				self.MAESTRO = serial.Serial(self.MAESTRO_port,baudrate=115200,timeout=0.1)
				self.MAESTRO.write(b'*Ver')
				time.sleep(0.1)
				r = self.MAESTRO.readline()
				
				print('MAESTRO:',r)
				self.MAESTRO.write(b'*CVU')
				time.sleep(0.1)
				r = self.MAESTRO.readline()
			
				try:
					val = float(r)
				except:
					val=np.nan	
				
				self.ui.MAESTRO_val.setText(str(val))
				
				print('MAESTRO:',r)
				self.ui.connectMAESTRO.setStyleSheet('background-color:red;')
				self.readMAESTROTimer.start(500)
		else:
			print('MAESTRO_cloes')
			self.readMAESTROTimer.stop()
			self.MAESTRO.close()
			self.ui.connectMAESTRO.setStyleSheet('background-color:green;')
			self.MAESTRO = None
			
	def onReadMAESTROTimer(self):
		if not self.MAESTRO:
			self.MAESTRO.close()
			self.ui.connectMAESTRO.setStyleSheet('background-color:green;')
			self.ui.connectMAESTRO.setChecked(False)
		else:
			r=b''
			try:
				self.MAESTRO.write(b'*CVU')
				time.sleep(0.1)
				n = self.MAESTRO.inWaiting()
				r = self.MAESTRO.read(n)
			except serial.serialutil.SerialException:
				print('MAESTRO_IO_ERROR')
				self.readMAESTROTimer.stop()
				self.MAESTRO.close()
				self.MAESTRO = serial.Serial(self.MAESTRO_port,baudrate=115200,timeout=0.1)
				self.readMAESTROTimer.start(500)
				
			val = np.nan
			try:
				val = float(r)
			except:
				val=np.nan	
			print('MAESTRO:',r)
			self.ui.MAESTRO_val.setText(str(val))
		
	
	def initUi(self):
		
			
		###########   mainToolBar  #############
		


		##############    plot   ##############
		self.pw = pg.PlotWidget(name='PlotMain')  ## giving the plots names allows us to link their axes together
		self.ui.dataPlot.addWidget(self.pw)
		self.pw2 = pg.PlotWidget(name='PlotRef')
		self.ui.dataPlot.addWidget(self.pw2)
		self.pwCCD = pg.PlotWidget(name='PlotCCD')
		self.ui.CCDPlot.addWidget(self.pwCCD)
		self.pwCCD.showGrid(x=True, y=True)
		self.pwCCD.setXRange(0, 4000)
		self.pwCCD.setYRange(0, 65000)
		
		self.pwCCD.setVisible(False)
		self.osc_pw = pg.PlotWidget(name='PlotHAMEG')
		self.ui.oscilloPlot.addWidget(self.osc_pw)
		#self.osc_pw.setXRange(0, 360)
		#self.osc_pw.showAxis('bottom', False)
		self.osc_pw.showGrid(x=True, y=True)
		self.pw.showGrid(x=True, y=True)
		self.pw2.showGrid(x=True, y=True)
		

		#self.osc_pw.showAxis('bottom', False)
		self.osc_pw.setYRange(0, 256)
		self.osc_pw.setXRange(0, 256)
		self.osc_pw.setMaximumWidth(self.ui.dataSources.maximumWidth())
		#self.osc_pw.setMaximumHeight(200)
		#self.osc_pw.setMinimumHeight(100)
		self.pwCCD.setMinimumHeight(200)
		#self.pwCCD.setMaximumHeight(400)
		
		self.lr = pg.LinearRegionItem([20,200])
		self.lr.setZValue(-10)
		self.osc_pw.addItem(self.lr)
		#def updatePlot():
		#	b = data2[int(lr.getRegion()[0]):int(lr.getRegion()[1])]
		#	integr=np.trapz(b-np.median(data2),dx=dx)
		#	rint(integr,np.median(data2))
		#lr.sigRegionChanged.connect(updatePlot)
		#self.ui.show()


		## Create an empty plot curve to be filled later, set its pen
		colors = ['red', "green", 'blue', 'cyan', 'magenta', 'yellow', 'purple', 'olive']
		#for i in range(0,self.ui.channelsSettings.rowCount()):
		#   self.lines[i] = self.pw.plot()
		#   self.lines[i].setPen(QtGui.QColor(colors[i]))
		self.line0 = self.pw2.plot()
		self.line0.setPen(QtGui.QColor('cyan'))
		self.line0Energy = self.pw.plot()
		self.line0Energy.setPen(QtGui.QColor('red'))
		
		self.line1 = self.pw.plot()
		self.line1.setPen(QtGui.QColor("orange"))
		self.line2 = self.osc_pw.plot()
		self.line2.setPen(QtGui.QColor("orange"))
		self.line3 = self.osc_pw.plot()
		self.line3.setPen(QtGui.QColor("cyan"))

		self.lineCCD = self.pwCCD.plot()
		self.lineCCD.setPen(QtGui.QColor("magenta"))

		self.pw.setLabel('left', 'Signal', units='arb. un.')
		self.pw.setLabel('bottom', 'position', units='deg.')
		self.line_sig = self.osc_pw.plot()
		self.line_sig.setPen(QtGui.QColor("orange"))
		self.line_ref = self.osc_pw.plot()
		self.line_ref.setPen(QtGui.QColor("cyan"))
		self.line_ref_norm = self.osc_pw.plot()
		self.line_ref_norm.setPen(QtGui.QColor("red"))
		#self.pw.setXRange(0, 360)
		#self.pw.setYRange(0, 1e10)
		#self.pw2.setMaximumHeight(300)

	def getData(self):
		source = self.ui.dataSource.currentText()
		if source == 'HAMEG':
			return self.oscilloGetData()
		else:
			#data = self.niAI.readAll()
			#r = #data[self.niPorts[0]]
			#s = #data[self.niPorts[1]]
			data = self.niAI.read_()
			r = data[:,0]
			s = data[:,1]
			w = r>r.mean()
			Vpp1 = abs(s[w].mean() - s[~w].mean())
			#w1 = s>s.mean()
			#Vpp1_1 =  abs(s[w1].mean() - s[~w1].mean())
			#if Vpp1_1< Vpp1:
			#Vpp1 = Vpp1_1
			Vpp2 = r[w].mean() - r[~w].mean()
			STD1 = s[w].std()

			self.line_sig.setData(x=np.arange(len(s)),y=s)
			self.line_ref.setData(x=np.arange(len(r)),y=r)
			ref_norm = w*20-10
			#self.line_ref_norm.setData(x=np.arange(len(ref_norm)),y=ref_norm)
			self.ui.CH1Val.setText(str(Vpp1))
			self.ui.CH2Val.setText(str(Vpp2))
			self.ui.oscilloFreq.setText(str(0))
			#self.ui.duty_cycle.setText(str(0))

			self.ui.CH1Val_std.setText(str(STD1))
			return (Vpp1, Vpp2, STD1)

	def uiConnect(self):
		
		self.ui.connectMAESTRO.toggled[bool].connect(self.connectMAESTRO)
		self.readMAESTROTimer.timeout.connect(self.onReadMAESTROTimer)
		###############    SMD    ################################
		self.ui.CCD_connect.toggled[bool].connect(self.CCD_connect)

		self.ui.SMD_connect.toggled[bool].connect(self.SMD_connect)
		self.ui.SM1_speed.valueChanged[int].connect(self.SM1_speed)

		self.ui.SM1_lock.toggled[bool].connect(self.SM1_lock)
		self.ui.SM2_lock.toggled[bool].connect(self.SM1_lock)

		self.ui.SM1_speed_val.valueChanged[int].connect(self.ui.SM1_speed.setValue)
		self.ui.SM1_stop.clicked.connect(self.SM1_stop)
		self.ui.SM1_stepLeft.clicked.connect(self.SM1_stepLeft)
		self.ui.SM1_stepRight.clicked.connect(self.SM1_stepRight)
		self.ui.SM1_left2end.clicked.connect(self.SM1_left2end)
		self.ui.SM1_right2end.clicked.connect(self.SM1_right2end)
		self.ui.SM1_reset.clicked.connect(self.SM1_reset)
		self.ui.SM1_absMove.clicked.connect(self.SM1_moveTo)
		self.ui.SM1_setPosition.clicked.connect(self.SM1_setPosition)
		
		self.ui.SM2_speed.valueChanged[int].connect(self.SM2_speed)
		self.ui.SM2_speed_val.valueChanged[int].connect(self.ui.SM2_speed.setValue)
		self.ui.SM2_stop.clicked.connect(self.SM2_stop)
		self.ui.SM2_stepLeft.clicked.connect(self.SM2_stepLeft)
		self.ui.SM2_stepRight.clicked.connect(self.SM2_stepRight)
		self.ui.SM2_left2end.clicked.connect(self.SM2_left2end)
		self.ui.SM2_right2end.clicked.connect(self.SM2_right2end)
		self.ui.SM2_reset.clicked.connect(self.SM2_reset)
		self.ui.SM2_absMove.clicked.connect(self.SM2_moveTo)
		self.ui.SM2_setPosition.clicked.connect(self.SM2_setPosition)

		self.ui.measCalibr.toggled[bool].connect(self.startCalibr)
		self.ui.measStart.toggled[bool].connect(self.startContMeas)

		
		###################    HAMEG  #################################
		self.ui.oscilloConnect.toggled[bool].connect(self.oscilloConnect)
		self.ui.oscilloAutoSet.clicked.connect(self.oscilloAutoSet)
		self.ui.oscilloSet.clicked.connect(self.oscilloSet)
		#self.ui.ADCAmplif.valueChanged[int].connect(self.setADCAmplif)
		#self.ui.laserFreq.valueChanged[int].connect(self.setLaserFreq)
		
		#self.ui.pauseMeasurements.toggled[bool].connect(self.pauseMeasurements)
		#self.ui.openCOMPort_btn.toggled[bool].connect(self.Open_CloseStepperCOMPort)
		self.ui.openAngleSensorPort.toggled[bool].connect(self.openAngleSensorPort)
		self.ui.setCustomAngle.clicked.connect(self.setCustomAngle)
		self.ui.moveToAngle.clicked.connect(self.moveToAngle)
		
		#self.ui.editStepperSettings.clicked.connect(self.setStepperParam)
		#self.ui.CCWSingleMove.clicked.connect(self.CCWSingleMove)
		#self.ui.CWSingleMove.clicked.connect(self.CWSingleMove)

		self.ui.actionClean.triggered.connect(self.cleanPlot)

		self.ui.CCWMoveToStop.clicked.connect(self.CCWMoveToStop)
		self.ui.CWMoveToStop.clicked.connect(self.CWMoveToStop)
		self.ui.CCWHome.clicked.connect(self.CCWHome)
		self.ui.CWHome.clicked.connect(self.CWHome)
		self.ui.setStepperSpeed.clicked.connect(self.setStepperSpeed)
		self.ui.stepperSpeed.valueChanged[int].connect(self.ui.stepperSpeedValue.setValue)
		self.ui.stepperSpeedValue.valueChanged[int].connect(self.ui.stepperSpeed.setValue)

		self.ui.nextFilter.clicked.connect(self.nextFilter)
		self.ui.prevFilter.clicked.connect(self.prevFilter)

		self.ui.stepperStop.clicked.connect(self.stepperStop)
		self.ui.stepperLock.toggled[bool].connect(self.stepperLock)

		self.ui.measSetFilePath.clicked.connect(self._saveToBtn)
		#self.ui.resetAngle.clicked.connect(self.updateAngle)
		#self.ui.setCustomAngle.toggled[bool].connect(self.setCustomAngle)

		#self.ui.calibrFW.clicked.connect(self.calibrFW)

		#self.ui.measStart.clicked.connect(self.startContMeas)

		#self.steppingTimer.timeout.connect(self.stepMeasurement)
		self.calibrTimer.timeout.connect(self.onCalibrTimer)
		self.measTimer.timeout.connect(self.onContMeasTimer)
		self.angleUpdateTimer.timeout.connect(self.onAngleUpdateTimer)
		self.calibrHomeTimer.timeout.connect(self.onCalibrHomeTimer)
		self.oscReadTimer.timeout.connect(self.onOscReadTimer)

		self.updateCCDTimer.timeout.connect(self.onUpdateCCDTimer)
		self.oscReadTimer.timeout.connect(self.onOscReadTimer)
		self.SMD_endstopsTimer.timeout.connect(self.onSMD_endstopsTimer)
		#self.calibrFWTimer.timeout.connect(self.onCalibrFWTimer)
		#self.stepperStateTimer.timeout.connect(self.checkStepperState)
		#self.laserStrobTimer.timeout.connect(self.laserStrob)
		
		self.ui.saveConfig.clicked.connect(self.saveConfig)

		self.ui.actionClose.triggered.connect(self.close)
		#self.ui.actionExit.triggered.connect(self.close)


		


if __name__ == "__main__":
	signal.signal(signal.SIGINT, sigint_handler)
	app = QtGui.QApplication(sys.argv)
	#app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
	myWindow = PICO_HG(None)
	app.exec_()
	#SMD_CloseComPort()
	print(":)")
	#os.kill(pid, signal.SIGTERM) #or signal.SIGKILL 

	sys.exit(1)
