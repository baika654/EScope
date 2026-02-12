# eshardware.py - This file is part of EScope/ESpark
# (C) 2024  Daniel A. Wagenaar
#
# EScope and ESpark are free software: you can redistribute it
# and/or modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# EScope and ESpark are distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software. If not, see <http://www.gnu.org/licenses/>.


# eshardware.py

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import sys
from . import esconfig
import numpy as np
import serial
from threading import Thread
from serial.serialutil import SerialException
from .esstopwatch import Stopwatch

BUFFER_SIZE = 20000

class DAQPacket:
    opcode:bytes
    payload = bytearray(512)
    payloadLength = None
    channel = None
    crc = None

    def Decode(self, packet:bytes):
        self.opcode = packet[0]
        self.payloadLength = (packet[1] << 8) + packet[2]
        #Array.Copy(packet, 3, self.payload, 0, self.payloadLength)
        self.channel = packet[3]
        self.payload = packet[4:4+self.payloadLength]
        self.crc = (packet[self.payloadLength + 4] << 8) + packet[self.payloadLength + 5]
    

    @staticmethod
    def Encode(opcode:bytes, payload:bytes): 
        payloadLength = 0
        
        if(payload != None):
            payloadLength = len(payload)
        
        #packet = 
        baPacket = bytearray(bytes(payloadLength + 5))
        baPacket[0] =opcode
        baPacket[1] = (payloadLength >> 8)
        baPacket[2] = payloadLength

        if(payloadLength > 0):
            #Array.Copy(payload, 0, packet, 3, payloadLength)
            #packet[3:] = payload[0:payloadLength]
            baPacket[3:3 + payloadLength] = bytearray(payload)


        crc = 0
        baPacket[payloadLength + 3]=(crc >> 8)
        baPacket[payloadLength + 4]=(crc)

        return bytes(baPacket)

class AnalogInCHStruct:
    channel = np.uint8
    #AnalogInCHConfigStruct config
    buffer = bytearray(BUFFER_SIZE)
    bufferReadIndex = 0 #np.uint16 
    bufferWriteIndex = 0 # np.uint16 

analogInAChannels = AnalogInCHStruct()

class Opcodes: 
		reset:bytes = 0x01
		connect:bytes = 0x02
		disconnect:bytes = 0x03
		setCurrentA:bytes = 0x11
		setCurrentB:bytes = 0x12
		setAnalogInA:bytes = 0x13
		setAnalogInB:bytes = 0x14
		setAnalogInACH:bytes = 0x15
		setAnalogInBCH:bytes = 0x16
		setAnalogOutACH:bytes = 0x17
		setAnalogOutBCH:bytes = 0x18
		txAnalogInA:bytes = 0x81
		txAnalogInB:bytes = 0x82

class ESStm32Hardware(QGroupBox):
    cfgChanged = pyqtSignal()

    _last_instance = None
    isConnected = False
    serialPort = None
    rxUSBThread = None
    rxUSBThreadRun = False
    usbRXStopwatch = Stopwatch()
    bytesRecieved = 0
    
    def __init__(self, cfg):
        super().__init__(title="STM32 Hardware Config")
        ESStm32Hardware._last_instance = self
        self.setWindowTitle("EScope: Stm32 Hardware")
        self.cfg = cfg
        lay = QGridLayout(self)
        lbl = QLabel("COM port:", self)
        lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lay.addWidget(lbl,0,0)

        lbl = QLabel("Baud Rate:", self)
        lbl.setAlignment(Qt.AlignRight)
        lay.addWidget(lbl,1,0)

        self.cnt = QPushButton()
        self.cnt.setText("Connect")
        self.cnt.setToolTip('Connect / Disconnect to STM board')
        self.cnt.setStyleSheet("background-color: red")
        self.cnt.clicked.connect(self.serialButtonClick)

        self.h_COM = QComboBox(self)
        self.h_baudrate = QComboBox(self)
        self.h_baudrate.setToolTip("Choose COM port speed")
        self.h_baudrate.addItems(["9600","19200","115200","921600", "1843200","2500000","3000000"])

        self.h_COM.activated.connect(self.selectCOMPort)
        self.h_baudrate.activated.connect(self.selectBaudRate)
        lay.addWidget(self.h_COM,0,1)
        lay.addWidget(self.h_baudrate,1,1)
        lay.addWidget(self.cnt,2,0)
        lay.setSpacing(10)

        self.buildAdapters()
        self.reconfig()

    @classmethod
    def get_last_instance(cls):
        return cls._last_instance

    def reconfig(self):
        self.findCOMPorts()
        self.buildRates()

    def USBRXProcessing(self, rxPacket:DAQPacket):
        
        if (rxPacket.channel!=1):
            return
        self.bytesRecieved = self.bytesRecieved + (rxPacket.payloadLength//2)*2
        if (rxPacket.payloadLength > 3):
            print("A payload of size ", rxPacket.payloadLength, " was recieved")
        if (self.bytesRecieved >=BUFFER_SIZE/2):
            #print(f"Recieved data from board. A total of 1024 bytes were transmitted")
            self.bytesRecieved = 0
        #self.bytesRecieved = self.bytesRecieved + (rxPacket.payloadLength//2)*2
        for x in range(rxPacket.payloadLength//2):    
            analogInAChannels.buffer[analogInAChannels.bufferWriteIndex+1] = rxPacket.payload[x*2]
            analogInAChannels.buffer[analogInAChannels.bufferWriteIndex] = rxPacket.payload[x*2+1]

			#Update Buffer index, buffer is circular
            analogInAChannels.bufferWriteIndex += 2
            if(analogInAChannels.bufferWriteIndex >= BUFFER_SIZE):
                analogInAChannels.bufferWriteIndex = 0
        #AuxLength:any = analogInAChannels.bufferWriteIndex-analogInAChannels.bufferReadIndex
        #if (AuxLength < 0):
        #    length = AuxLength + 2048
        #else:
        #    length = AuxLength
        #if (length >=1000):
        #    print(f"1000 data points are ready to be sent to the oscilloscope renderer")
        #    analogInAChannels.bufferReadIndex = analogInAChannels.bufferWriteIndex    	    

    def USBWrite(self, opcode, payload:bytes):
        if (self.serialPort.is_open):
            packet:bytes = DAQPacket.Encode(opcode, payload)
            self.serialPort.write(packet)

    def GetBufferData(self, size):
        buffer_data = None
        if (analogInAChannels.bufferReadIndex + size < BUFFER_SIZE):
            buffer_data = analogInAChannels.buffer[analogInAChannels.bufferReadIndex:analogInAChannels.bufferReadIndex+size]
            analogInAChannels.bufferReadIndex += size
        else: 
            buffer_data =  analogInAChannels.buffer[analogInAChannels.bufferReadIndex:BUFFER_SIZE] + analogInAChannels.buffer[0: analogInAChannels.bufferReadIndex+size -BUFFER_SIZE]
            analogInAChannels.bufferReadIndex = analogInAChannels.bufferReadIndex+size - BUFFER_SIZE
        #AuxLength:any = analogInAChannels.bufferWriteIndex-analogInAChannels.bufferReadIndex
        #buffer_data = None
        #if (AuxLength <0):
        #    length = AuxLength + 2048
        #    buffer_data = analogInAChannels.buffer[analogInAChannels.bufferReadIndex:2048] + analogInAChannels.buffer[0:analogInAChannels.bufferWriteIndex]

        #else:
        #    length = AuxLength
        #    buffer_data = analogInAChannels.buffer[analogInAChannels.bufferReadIndex:analogInAChannels.bufferWriteIndex]

        #analogInAChannels.bufferReadIndex = analogInAChannels.bufferWriteIndex
        return buffer_data

    def GetBufferDataSize(self):
        AuxLength:any = analogInAChannels.bufferWriteIndex-analogInAChannels.bufferReadIndex
        
        if (AuxLength <0):
            length = AuxLength + BUFFER_SIZE
        else:
            length = AuxLength
        return length
    
    def GetWriteBufferIndex(self):
        return analogInAChannels.bufferWriteIndex
    
    def GetReadBufferIndex(self):
        return analogInAChannels.bufferReadIndex

    def USBRXThread(self): 
        rxPacket = DAQPacket()
        index = 0
        usbRXByteCount = 0
        usbRXErrorCount = 0
        rxDataPacketLength = 0
        usbRXPacketCount = 0
        rxDataPacket = bytearray(550)
        rxBuffer = bytearray(1024)
        while (self.rxUSBThreadRun):
            if (self.serialPort.is_open):
                rxLength = 0
                try:
                    #//rxLength = await serialPort.BaseStream.ReadAsync(rxBuffer, 0, rxBuffer.Length);
                    #rxLength = serialPort.BaseStream.Read(rxBuffer, 0, rxBuffer.Length);
                    rxLength = self.serialPort.readinto(rxBuffer)
                except: 
                    return
                
                #print("Total data received from serial port was ",rxLength," bytes")
                for i in range(rxLength):
                    if (index == 0):
                        #First Byte of package is the Opcode
                        rxDataPacket[index] = rxBuffer[i]
                        index+=1
                    elif (index == 1):
                        #Second and Third byte of package is the length
                        rxDataPacket[index] = rxBuffer[i]
                        index+=1
                        rxDataPacketLength = (rxBuffer[i] << 8)
                    elif (index == 2):
                        #Second and Third byte of package is the length
                        rxDataPacket[index] = rxBuffer[i]
                        index+=1
                        rxDataPacketLength += rxBuffer[i]

                        if(rxDataPacketLength > 512):
                            #RX Buffer overflow
                            index = 0
                            usbRXErrorCount += 1
                    elif (index < (rxDataPacketLength + 3 + 2)):
                        #Rest bytes are payload
                        rxDataPacket[index] = rxBuffer[i]
                        index+=1
                    else:
                        #Full packet received
                        rxPacket.Decode(rxDataPacket)

                        if(self.USBRXProcessing(rxPacket) != 0):
                            # Packet Processing Error
                            usbRXErrorCount += 1
                        usbRXPacketCount += 1
                        index = 0
                usbRXByteCount += rxLength
                if (self.usbRXStopwatch.ElapsedMilliseconds() > 1000):
                    usbRXDatarate = usbRXByteCount
                    usbRXByteCount = 0
                    usbRXErrorRate = usbRXErrorCount
                    usbRXErrorCount = 0
                    self.usbRXStopwatch.Restart()

    def findCOMPorts(self):
        if sys.platform.startswith('win'):
            self.ports = ['COM%s' % (i + 1) for i in range(256)]
        result = []
        for port in self.ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        self.h_COM.addItems(result)
        #return result
        
        #k=0
        #for ada in self.cfg.hw.adapters:
        #    if self.cfg.hw.adapter==ada:
        #        #self.h_ada.setCurrentIndex(k)
        #        pass
        #    k=k+1

    def findRate(self):
        k = np.argmin(abs(self.cfg.hw.acqrate.value -
                             self.cfg.hw.acqrate.values))
        #self.h_rate.setCurrentIndex(k)
        pass

    def buildAdapters(self):
        #self.h_ada.clear()
        #for ada in self.cfg.hw.adapters:
        #    name = ada[0]
        #    if len(ada)>1:
        #        name = f"{name}: {ada[1]}"
        #    self.h_ada.addItem(name)
        #self.findAdapter()
        pass

    def buildRates(self):
        #self.h_rate.clear()
        #for v in self.cfg.hw.acqrate.values:
        #    self.h_rate.addItem("%g kHz" % (v/1000))
        #self.findRate()
        pass

    def selectHardware(self, idx):
        self.cfg.hw.adapter = self.cfg.hw.adapters[idx]
        esconfig.confighardware(self.cfg)
        self.buildRates()
        self.cfgChanged.emit()

    def selectRate(self, idx):
        self.cfg.hw.acqrate.value = self.cfg.hw.acqrate.values[idx]
        self.cfgChanged.emit()

    def selectCOMPort(self, idx):
        pass

    def selectBaudRate(self, idx):
        pass

    def serialButtonClick(self):
        if not self.isConnected: 
            if ( self.connectToPort()): 
                self.cnt.setText("Disconnect")
                self.cnt.setStyleSheet("background-color: green")
        else: 
            if (self.disconnectFromPort()):
                self.cnt.setText("Connect")
                self.cnt.setStyleSheet("background-color: red")
    
    def disconnectFromPort(self):
        try:
            if self.serialPort.is_open:
                self.USBWrite(Opcodes.disconnect, None)
                self.usbRXStopwatch.Stop()
                self.rxUSBThreadRun = False  # alerts thread that it is time to stop
                self.rxUSBThread.join()  # wait for thread to end before continuing.
                self.serialPort.close()
                self.isConnected = False
                return True
            else:
                print(f"Serial port is already closed.")
                return False
        except SerialException as e:
            print(f"An unexpected error occurred: {e}")
            return False

    def connectToPort(self):
        try:
            self.serialPort = serial.Serial(port=self.h_COM.currentText(), baudrate=self.h_baudrate.currentText(), timeout=1)
            if self.serialPort.is_open:
                print(f"Serial port {self.h_COM.currentText()} is successfully opened.")
                # You can now proceed with your serial communication
                self.USBWrite(Opcodes.connect, None)

                self.usbRXStopwatch.Start()

                self.rxUSBThreadRun = True
                self.rxUSBThread = Thread(target = self.USBRXThread)
                self.rxUSBThread.start()
                self.isConnected = True
                #self.rxUSBThread.join()
				#rxUSBThread = new Thread(USBRXThread);
				#rxUSBThread.Start();
                #self.serialPort.close() # Close the port if you only needed to check its availability
                data = bytearray(b'\x01\x02\x01\x04\x01')
                self.USBWrite(Opcodes.setAnalogInACH, data)
                return True
            else:
                print(f"Serial port {self.h_COM.currentText()} could not be opened, but no exception was raised.")
                return False
        except SerialException as e:
            print(f"Failed to open serial port {self.h_COM.currentText()}: {e}")
            if "Port is already open" in str(e):
                print("The serial port is likely already in use by another process.")
                return False
            elif "No such file or directory" in str(e):
                print("The specified serial port does not exist or is not available.")
                return False
            else:
                print("An unknown error occurred while trying to open the serial port.")
                return False
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            return False
    
if __name__ == '__main__':
    app = QApplication(sys.argv)
    cfg = esconfig.basicconfig()
    win = ESHardware(cfg)
    win.show()
    app.exec_()
    
