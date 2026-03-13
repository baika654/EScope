<img alt="EScope and ESpark" src="https://github.com/baika654/EScope/blob/main/docs/source/banner.svg" width="100%">
EScope with STM32 extension a software oscilloscope designed to be used with the ADC on STM32 microdevices.

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

    pip install escope

Once installed create a new subdirectory in your home directory 

    mkdir ext

Enter the directory and install this repo into the directory

    cd ext
    git clone https://github.com/baika654/EScope.git

Once installed copy two files from the repo to the Escope install

    copy <home>\ext\Escope\src\escope\escopelib\esstm32hardware.py <home>\Escope\src\escope\escopelib
    copy <home>\ext\Escope\src\escope\escopelib\esstm32h7.py <home>\Escope\src\escope\escopelib


    
## Running

To run the software, open a terminal and type either ....

    To Do

## License

EScope and ESpark are licensed under the GPL license, version 3 or—at your choice—any later version. See [LICENSE](LICENSE) for more information.

