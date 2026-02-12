
from .esdatasource import ESDataSource
from .esstm32hardware import ESStm32Hardware
import numpy as np


def deviceList():
    
    #system = nidaqmx.system.System.local()
    devs = ["STM32H723"]
    return devs

class ESDS_Stm32h7(ESDataSource):

    ref_to_ESStm32Hardware = None

    def __init__(self, cfg):
        ESDataSource.__init__(self, cfg)
        self.timerid = None
        self.acqtask = None
        #print("STM object has been initialised")

    def reconfig(self):
        ESDataSource.reconfig(self)
        self.t = 0
        self.ddt = np.arange(0,self.period_scans)/self.cfg.hw.acqrate.value
        print("Reconfiguring")

    def run(self):
        if (self.ref_to_ESStm32Hardware == None):
            self.ref_to_ESStm32Hardware = ESStm32Hardware._last_instance
        ESDataSource.run(self)
        if self.timerid is not None:
            self.killTimer(self.timerid)
        self.timerid = self.startTimer(int(self.period_s*1000))
        print(f"STM32H7 starting")

    def stop(self):
        if self.timerid is not None:
            self.killTimer(self.timerid)
        ESDataSource.stop(self)
        print(f"SMT32H7 stop")

    def timerEvent(self, evt):
        self.dataAvailable.emit()

    def getData(self, dst):
        now = min(self.ref_to_ESStm32Hardware.GetBufferDataSize()//2, dst.shape[0])
        data_in_buffer = self.ref_to_ESStm32Hardware.GetBufferData(now*2)
        print("Date in buffer: ", self.ref_to_ESStm32Hardware.GetBufferDataSize()//2)
        print("Write buffer index = ",self.ref_to_ESStm32Hardware.GetWriteBufferIndex()," and read buffer index = ", self.ref_to_ESStm32Hardware.GetReadBufferIndex())
        #now = len(data_in_buffer)//2
        #temp = np.frombuffer(data_in_buffer, dtype='<u2').reshape(-1,1)
        dst[:now,:] = np.frombuffer(data_in_buffer, dtype='<u2').reshape(now,1)/10000
        #dst[:now,:] = .1*np.random.standard_normal((now, self.nchan))
        #ff=[3,10,30,100,300,1,0.3,0.1]
        #for k in range(self.nchan):
        #    f = ff[int(self.chans[k])]
        #dst[:now,0]  = np.frombuffer(data_in_buffer, dtype='<u2')
        self.t += now/self.cfg.hw.acqrate.value
        return now