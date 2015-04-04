BreezeUav

@author : Adrien Hadj-Salah
@url : breezeuav.com
=======================================

Breeze UAV is an open-source autopilot giving any fixed-wing aircraft full autonomous capability. 
Based on Arduino Mega  2560 microcontroller, Breeze provides basics and advanced functions such as 
automatic takeoff and landing, waypoints navigation, on-board and ground decisions.


=======================================
Entry source code :
The main function is written in Breeze.ino


=======================================
Compile the code :
1- Install arduino
2- Change Makefile to link to your arduino folder.
3- (TODO) Edit your arduino core library files to remove any link to Serial (Serial0)
4- Using makefile, run make -f BreezeMakefile.mk


=======================================
Upload to the board :
Using makefile, run make -f BreezeMakefile.mk upload

