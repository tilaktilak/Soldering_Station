An arduino base project to control a simple soldering Iron from ebay.

The temperature is read and set through an rotary encoder and a LCD graphic display.
There is a temperature sensor in the soldering iron, a mosfet that control the voltage through a PWM command signal.
The system is modelized as a first order system and control with a simple PID algorithm

T = aV + b 
a = 1.16
b = -234.48

[![DSC00005-1551815028919.jpg](https://i.postimg.cc/N0tw53Cd/DSC00005-1551815028919.jpg)](https://postimg.cc/3ktqqcTG)
