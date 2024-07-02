# This script reads data from the microcontroller connected via usb and carries out the lock in amplifier calculations. 
# It also plots the resulting data live on screen and records everything to a folder as a file named data in a folder named "Data"

import serial
import struct
import signal
import time
import numpy as np
import sys
import os
import collections as clns
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import threading
import cProfile

# if running on a windows machine this will need to be changed and process should be set to realtime.
os.nice(-20)


def signal_handler(signal, frame):
    global interrupted
    interrupted = True


signal.signal(signal.SIGINT, signal_handler)

sinwave = [0x80000000,0x819BC395,0x83377685,0x84D3082B,
0x866E67E4,0x8809850E,0x89A44F0B,0x8B3EB540,
0x8CD8A716,0x8E7213F9,0x900AEB5E,0x91A31CBC,
0x933A9794,0x94D14B6D,0x966727D5,0x97FC1C65,
0x999018BD,0x9B230C8A,0x9CB4E780,0x9E459961,
0x9FD511FA,0xA1634125,0xA2F016CA,0xA47B82DE,
0xA6057563,0xA78DDE6E,0xA914AE22,0xAA99D4B2,
0xAC1D4262,0xAD9EE789,0xAF1EB491,0xB09C99F5,
0xB2188846,0xB3927026,0xB50A4250,0xB67FEF92,
0xB7F368D1,0xB9649F09,0xBAD3834C,0xBC4006C5,
0xBDAA1AB9,0xBF11B085,0xC076B99E,0xC1D92797,
0xC338EC1A,0xC495F8F0,0xC5F03FFD,0xC747B340,
0xC89C44D8,0xC9EDE700,0xCB3C8C12,0xCC882686,
0xCDD0A8F5,0xCF160618,0xD05830C6,0xD1971BFB,
0xD2D2BAD1,0xD40B0086,0xD53FE07B,0xD6714E32,
0xD79F3D54,0xD8C9A1AC,0xD9F06F28,0xDB1399E0,
0xDC33160C,0xDD4ED80F,0xDE66D470,0xDF7AFFDC,
0xE08B4F2B,0xE197B759,0xE2A02D8D,0xE3A4A717,
0xE4A5196F,0xE5A17A36,0xE699BF39,0xE78DDE6E,
0xE87DCDF8,0xE9698423,0xEA50F768,0xEB341E6B,
0xEC12EFFE,0xECED631E,0xEDC36EF8,0xEE950AE4,
0xEF622E68,0xF02AD13A,0xF0EEEB3E,0xF1AE7485,
0xF2696552,0xF31FB617,0xF3D15F73,0xF47E5A3A,
0xF5269F6C,0xF5CA283D,0xF668EE0F,0xF702EA78,
0xF798173E,0xF8286E59,0xF8B3E9F4,0xF93A846B,
0xF9BC384D,0xFA39005C,0xFAB0D78C,0xFB23B906,
0xFB91A023,0xFBFA8874,0xFC5E6DBA,0xFCBD4BEC,
0xFD171F33,0xFD6BE3EE,0xFDBB96B1,0xFE063441,
0xFE4BB99B,0xFE8C23EF,0xFEC770A4,0xFEFD9D52,
0xFF2EA7CA,0xFF5A8E10,0xFF814E5D,0xFFA2E721,
0xFFBF5700,0xFFD69CD3,0xFFE8B7AA,0xFFF5A6CA,
0xFFFD69AC,0xFFFFFFFF,0xFFFD69AC,0xFFF5A6CA,
0xFFE8B7AA,0xFFD69CD3,0xFFBF5700,0xFFA2E721,
0xFF814E5D,0xFF5A8E10,0xFF2EA7CA,0xFEFD9D52,
0xFEC770A4,0xFE8C23EF,0xFE4BB99B,0xFE063441,
0xFDBB96B1,0xFD6BE3EE,0xFD171F33,0xFCBD4BEC,
0xFC5E6DBA,0xFBFA8874,0xFB91A023,0xFB23B906,
0xFAB0D78C,0xFA39005C,0xF9BC384D,0xF93A846B,
0xF8B3E9F4,0xF8286E59,0xF798173E,0xF702EA78,
0xF668EE0F,0xF5CA283D,0xF5269F6C,0xF47E5A3A,
0xF3D15F73,0xF31FB617,0xF2696552,0xF1AE7485,
0xF0EEEB3E,0xF02AD13A,0xEF622E68,0xEE950AE4,
0xEDC36EF8,0xECED631E,0xEC12EFFE,0xEB341E6B,
0xEA50F768,0xE9698423,0xE87DCDF8,0xE78DDE6E,
0xE699BF39,0xE5A17A36,0xE4A5196F,0xE3A4A717,
0xE2A02D8D,0xE197B759,0xE08B4F2B,0xDF7AFFDC,
0xDE66D470,0xDD4ED80F,0xDC33160C,0xDB1399E0,
0xD9F06F28,0xD8C9A1AC,0xD79F3D54,0xD6714E32,
0xD53FE07B,0xD40B0086,0xD2D2BAD1,0xD1971BFB,
0xD05830C6,0xCF160618,0xCDD0A8F5,0xCC882686,
0xCB3C8C12,0xC9EDE700,0xC89C44D8,0xC747B340,
0xC5F03FFD,0xC495F8F0,0xC338EC1A,0xC1D92797,
0xC076B99E,0xBF11B085,0xBDAA1AB9,0xBC4006C5,
0xBAD3834C,0xB9649F09,0xB7F368D1,0xB67FEF92,
0xB50A4250,0xB3927026,0xB2188846,0xB09C99F5,
0xAF1EB491,0xAD9EE789,0xAC1D4262,0xAA99D4B2,
0xA914AE22,0xA78DDE6E,0xA6057563,0xA47B82DE,
0xA2F016CA,0xA1634125,0x9FD511FA,0x9E459961,
0x9CB4E780,0x9B230C8A,0x999018BD,0x97FC1C65,
0x966727D5,0x94D14B6D,0x933A9794,0x91A31CBC,
0x900AEB5E,0x8E7213F9,0x8CD8A716,0x8B3EB540,
0x89A44F0B,0x8809850E,0x866E67E4,0x84D3082B,
0x83377685,0x819BC395,0x80000000,0x7E643C6B,
0x7CC8897B,0x7B2CF7D5,0x7991981C,0x77F67AF2,
0x765BB0F5,0x74C14AC0,0x732758EA,0x718DEC07,
0x6FF514A2,0x6E5CE344,0x6CC5686C,0x6B2EB493,
0x6998D82B,0x6803E39B,0x666FE743,0x64DCF376,
0x634B1880,0x61BA669F,0x602AEE06,0x5E9CBEDB,
0x5D0FE936,0x5B847D22,0x59FA8A9D,0x58722192,
0x56EB51DE,0x55662B4E,0x53E2BD9E,0x52611877,
0x50E14B6F,0x4F63660B,0x4DE777BA,0x4C6D8FDA,
0x4AF5BDB0,0x4980106E,0x480C972F,0x469B60F7,
0x452C7CB4,0x43BFF93B,0x4255E547,0x40EE4F7B,
0x3F894662,0x3E26D869,0x3CC713E6,0x3B6A0710,
0x3A0FC003,0x38B84CC0,0x3763BB28,0x36121900,
0x34C373EE,0x3377D97A,0x322F570B,0x30E9F9E8,
0x2FA7CF3A,0x2E68E405,0x2D2D452F,0x2BF4FF7A,
0x2AC01F85,0x298EB1CE,0x2860C2AC,0x27365E54,
0x260F90D8,0x24EC6620,0x23CCE9F4,0x22B127F1,
0x21992B90,0x20850024,0x1F74B0D5,0x1E6848A7,
0x1D5FD273,0x1C5B58E9,0x1B5AE691,0x1A5E85CA,
0x196640C7,0x18722192,0x17823208,0x16967BDD,
0x15AF0898,0x14CBE195,0x13ED1002,0x13129CE2,
0x123C9108,0x116AF51C,0x109DD198,0x0FD52EC6,
0x0F1114C2,0x0E518B7B,0x0D969AAE,0x0CE049E9,
0x0C2EA08D,0x0B81A5C6,0x0AD96094,0x0A35D7C3,
0x099711F1,0x08FD1588,0x0867E8C2,0x07D791A7,
0x074C160C,0x06C57B95,0x0643C7B3,0x05C6FFA4,
0x054F2874,0x04DC46FA,0x046E5FDD,0x0405778C,
0x03A19246,0x0342B414,0x02E8E0CD,0x02941C12,
0x0244694F,0x01F9CBBF,0x01B44665,0x0173DC11,
0x01388F5C,0x010262AE,0x00D15836,0x00A571F0,
0x007EB1A3,0x005D18DF,0x0040A900,0x0029632D,
0x00174856,0x000A5936,0x00029654,0x00000000,
0x00029654,0x000A5936,0x00174856,0x0029632D,
0x0040A900,0x005D18DF,0x007EB1A3,0x00A571F0,
0x00D15836,0x010262AE,0x01388F5C,0x0173DC11,
0x01B44665,0x01F9CBBF,0x0244694F,0x02941C12,
0x02E8E0CD,0x0342B414,0x03A19246,0x0405778C,
0x046E5FDD,0x04DC46FA,0x054F2874,0x05C6FFA4,
0x0643C7B3,0x06C57B95,0x074C160C,0x07D791A7,
0x0867E8C2,0x08FD1588,0x099711F1,0x0A35D7C3,
0x0AD96094,0x0B81A5C6,0x0C2EA08D,0x0CE049E9,
0x0D969AAE,0x0E518B7B,0x0F1114C2,0x0FD52EC6,
0x109DD198,0x116AF51C,0x123C9108,0x13129CE2,
0x13ED1002,0x14CBE195,0x15AF0898,0x16967BDD,
0x17823208,0x18722192,0x196640C7,0x1A5E85CA,
0x1B5AE691,0x1C5B58E9,0x1D5FD273,0x1E6848A7,
0x1F74B0D5,0x20850024,0x21992B90,0x22B127F1,
0x23CCE9F4,0x24EC6620,0x260F90D8,0x27365E54,
0x2860C2AC,0x298EB1CE,0x2AC01F85,0x2BF4FF7A,
0x2D2D452F,0x2E68E405,0x2FA7CF3A,0x30E9F9E8,
0x322F570B,0x3377D97A,0x34C373EE,0x36121900,
0x3763BB28,0x38B84CC0,0x3A0FC003,0x3B6A0710,
0x3CC713E6,0x3E26D869,0x3F894662,0x40EE4F7B,
0x4255E547,0x43BFF93B,0x452C7CB4,0x469B60F7,
0x480C972F,0x4980106E,0x4AF5BDB0,0x4C6D8FDA,
0x4DE777BA,0x4F63660B,0x50E14B6F,0x52611877,
0x53E2BD9E,0x55662B4E,0x56EB51DE,0x58722192,
0x59FA8A9D,0x5B847D22,0x5D0FE936,0x5E9CBEDB,
0x602AEE06,0x61BA669F,0x634B1880,0x64DCF376,
0x666FE743,0x6803E39B,0x6998D82B,0x6B2EB493,
0x6CC5686C,0x6E5CE344,0x6FF514A2,0x718DEC07,
0x732758EA,0x74C14AC0,0x765BB0F5,0x77F67AF2,
0x7991981C,0x7B2CF7D5,0x7CC8897B,0x7E643C6B]

datafile = open("Data/data.txt", 'w')
sinewave = np.array(sinwave)
sinmax = sinewave.max()
sinewave = sinewave - sinmax/2
sinewave = sinewave * (3.3/sinmax)
print(len(sinewave))

#port address will need to be changed depending on the operating system and what port the microcontroller is connected to.
m4 = serial.Serial(port = "/dev/cu.usbmodem2101", baudrate=2000000, timeout=None)

samples = 5000

indexdat =  clns.deque(maxlen=samples)
counterdat = clns.deque(maxlen=samples)
refdat = clns.deque(maxlen=samples)
ref90dat = clns.deque(maxlen=samples)
#refthetadat = clns.deque(maxlen=1000)
adcdat = clns.deque(maxlen=samples)
tempdat = clns.deque(maxlen=samples)
tempdatavg = clns.deque(maxlen=samples)
mixerxdat = clns.deque(maxlen=samples)
mixerydat = clns.deque(maxlen=samples)
#movAvX = clns.deque(maxlen=1000)
#moVavY = clns.deque(maxlen=1000)
phase = clns.deque(maxlen=samples)
amplitude = clns.deque(maxlen=samples)


interrupted = False
win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle('plotting test')

refplot=win.addPlot()
refplot.setLabel('bottom', text = "sample", units = None)
refplot.setLabel('left', text = "Reference", units = 'V')
refplot.plot(counterdat, refdat)


adcplot=win.addPlot()
adcplot.setLabel('bottom', text = "sample", units = None)
adcplot.setLabel('left', text = "ADC", units = 'V')
adcplot.plot(counterdat, adcdat)

win.nextRow()

ampPlot=win.addPlot()
ampPlot.setLabel('bottom', text = "sample", units = None)
ampPlot.setLabel('left', text = "Amplitude", units = 'V')
ampPlot.plot(counterdat, amplitude)

phaseplot=win.addPlot()
phaseplot.setLabel('bottom', text = "sample", units = None)
phaseplot.setLabel('left', text = "phase", units = 'degrees')
phaseplot.plot(counterdat, phase)

win.nextRow()
tempplot=win.addPlot()
tempplot.setLabel('bottom', text = "sample", units = None)
tempplot.setLabel('left', text = "temperature", units = 'degrees C')
tempplot.plot(counterdat, tempdat)

avtempplot=win.addPlot()
avtempplot.setLabel('bottom', text = "sample", units = None)
avtempplot.setLabel('left', text = "temperature", units = 'degrees C')
avtempplot.plot(counterdat, tempdatavg)


def getData():
    counter = 1
    lastindex=0
    sumx = 0.0
    sumy = 0.0
    tempsum = 0.0
    profiler = cProfile.Profile()
    profiler.enable()
    while 1:
        incoming = m4.read(m4.in_waiting)
        for i in range(int(len(incoming)/10)):
            adcRaw = incoming[i*10:(i+1)*10]
            adc1data = (int(adcRaw[0]<<24) | int(adcRaw[1]<<16) | int(adcRaw[2]<<8) | int(adcRaw[3]))
            if(adc1data & (1 << 31)) != 0:
                adc1data = adc1data - (1 << 32)
            convdata1= float(adc1data)*float(2.5/2147483648)
            adc2data = (int(adcRaw[4]<<24) | int(adcRaw[5]<<16) | int(adcRaw[6]<<8) | int(adcRaw[7]))
            if(adc2data & (1 << 31)) != 0:
                adc2data = adc2data - (1 << 32)
            convdata2= float(adc2data)*float(2.5/2147483648)
            degreesC = float((convdata2+2.5)-1.25)/0.005
            index = int(adcRaw[8]<<8) | int(adcRaw[9])
            counter = counter + 1
            
            

            counterdat.append(counter)
            indexdat.append(index)
            refdat.append(sinewave[index])
            ref90dat.append(sinewave[index-125])
            #refthetadat.append(sinewave[index-458])
            adcdat.append(convdata1)
            
            # Lock-in amplifier math
            mixedx = convdata1*sinewave[index]
            mixedy = convdata1*sinewave[index-125]

            sumx = sumx + mixedx
            sumy = sumy + mixedy
            tempsum = tempsum + degreesC
            if counter >= samples:
                sumx = sumx - mixerxdat[0]
                sumy = sumy - mixerydat[0]
                tempsum = tempsum - tempdat[0]
                mixerxdat.append(mixedx)
                mixerydat.append(mixedy)
                tempdat.append(degreesC)
                tempavg = np.divide(tempsum, samples)
                tempdatavg.append(tempavg)

                movavgx = np.divide(sumx, samples)
                movavgy = np.divide(sumy, samples)

                amp = np.sqrt(np.square(movavgx)+ np.square(movavgy))
                amplitude.append(amp)
                phaseRad = np.arctan2(movavgx,movavgy)
                phaseDeg = phaseRad * 180/np.pi
                phase.append(phaseDeg)
                datafile.write(str(counter) + ',' + str(index) + ',' + str(sinewave[index]) + ',' + str(sinewave[index-125]) + ',' + str(convdata1) + ',' + str(amp) + ',' + str(phaseDeg) + ',' + str(degreesC) + ',' + str(tempavg) + '\n')
            else:
                mixerxdat.append(mixedx)
                mixerydat.append(mixedy)
                tempdat.append(degreesC)
            

            if index-1 != lastindex:
                if index > 0:
                    print("skipped!")
                    print(index)
            lastindex = index
            if interrupted:  # this allows the program to exit gracefully when ctrl-c is used to stop it
                m4.close()
                profiler.disable()
                profiler.dump_stats("threadstats4.stats")
                datafile.close()
                print('Exited Gracefully')
                pg.exit()
                exit()


def update_plot():
    adcplot.plot(counterdat, adcdat, clear = True)
    refplot.plot(counterdat, refdat, clear = True)
    ampPlot.plot(counterdat, amplitude)
    phaseplot.plot(counterdat, phase)
    tempplot.plot(counterdat, tempdat)
    avtempplot.plot(counterdat, tempdatavg)
    #counts = counts +1
    #pg.QtWidgets.QApplication.processEvents()


    #datafile.write(str(counter) + ',' + str(index) + ',' + str(sinewave[index-125]) + ',' + str(sinewave[index-42]) + ',' + str(sinewave[index]) + ',' + str(convdata) + '\n')
    #print(index)
    #print(sinewave[index])
    #print(sinewave[index-125])
    #print(sinewave[index-42])

    
    #print(convdata)
    #print(index)

    if interrupted:  # this allows the program to exit gracefully when ctrl-c is used to stop it
        m4.close()
        datafile.close()
        print('Exited Gracefully')
        pg.exit()
        exit()

m4.reset_input_buffer
aqthread = threading.Thread(target = getData, daemon = True)
aqthread.start()

timer = pg.QtCore.QTimer()
timer.timeout.connect(update_plot)
timer.start(50)
pg.exec()
