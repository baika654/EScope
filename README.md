<img alt="EScope and ESpark" src="https://github.com/baika654/EScope/blob/main/docs/source/banner.svg" width="100%">
STM32 extension for Escope is python extension that allows a user to use a STM32H723 microdevice's ADC on Escope, a software oscilloscope.

## Screenshots

EScope sampling data from an STM32H723 card:

<img alt="EScope screenshot" src="https://github.com/baika654/EScope/blob/main/docs/source/EScopeSTM32.gif" width="80%">

In this animated gif an audio signal from a youtube video 
(https://www.youtube.com/watch?v=UZyb_wIjKCk) is being fed 
into the ADC of the STM32H723 running at a sampling rate of
20 kHz.

## Setup

To connect the audio output from some source device, the following
circuit is used. In this case the circuit connects the audio output
of a raspberry pi to the STM32H7 device.

<img alt="Circuit diagram for STM32H723" src="https://github.com/baika654/EScope/blob/main/docs/source/CircuitDiagram.png" width="80%">


## Features

EScope can display one trace from one input on the STM32H723 device,
optionally using the input as a trigger input. As on
physical digital storage oscilloscopes, input signals can be DC or AC
coupled. The vertical gain and offset can be adjusted by dragging
corresponding user interface elements.

## Installation

Open a terminal in your home directory and install the Escope package written by Daniel Wagenaar using the command

    git clone https://github.com/wagenadl/escope.git

Once installed create a new subdirectory in your home directory 

    mkdir ext

Enter the directory and install this repo into the directory

    cd ext
    git clone https://github.com/baika654/EScope.git

Once installed copy these three files from the repo to the Escope install

    copy <home>\ext\Escope\src\escope\escopelib\esstm32hardware.py <home>\Escope\src\escope\escopelib
    copy <home>\ext\Escope\src\escope\escopelib\esstm32h7.py <home>\Escope\src\escope\escopelib
    copy <home>\ext\Escope\src\escope\escopelib\esstopwatch.py <home>\Escope\src\escope\escopelib

Add the following lines of code to `<home>`\Escope\src\escope\escope.py
At about line 16 in the import section add

    from .escopelib.esstm32hardware import ESStm32Hardware

Add the following to the function makedocks()

    def makedocks(self):
    self.h_hw = ESHardware(self.cfg)
    self.h_hw.cfgChanged.connect(self.hwChanged)
    self.docklay.addWidget(self.hw)
    self.h_hw.hide()
    # ********** New code goes here ***********
    self.h_stm32hw = ESStm32Hardware(self.cfg, self)
    self.docklay.addWidget(self.h_stm32hw)
    self.h_stm32hw.hide()
    # *********** End of new code section ********

Add the following lines of code to `<home>`\Escope\src\escope\escopelib\esconfig.py
Add the following line in the import section

    from . import esstm32h7

Add the following to the function findadapters()

    def findadapters():
    lst=[('dummy',)]
    # ********** New code goes here ***********
    stmdevs = esstm32h7.deviceList()
    for dev in stmdevs:
        lst.append(('STM32H7', dev))
    # *********** End of new code section ********    

    
## Running

To run the software, open a terminal in your home directory and type

    cd escope
    python escope

The program may not initially run as python libraries that are needed by the software may not be installed. Use pip to fix this.

## Software for the STM32H723 board.

The software for this board can be downloaded from https://github.com/baika654/STM32H7DAQ.git

## To do

The STM32 device can only run at a maximum sampling frequency of 20 kHz. More work on the STM32H7 code is needed to reach higher sampling frequencies.

## License

EScope and ESpark are licensed under the GPL license, version 3 or—at your choice—any later version. See [LICENSE](LICENSE) for more information.

