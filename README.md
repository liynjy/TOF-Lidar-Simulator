# TOF-Lidar-Simulator
A simulation of TOF (Time of Flight) Lidar (Laser Radar).

=========================================================================
Liadr TX Trig and RX Data Simulation
=========================================================================
Author : Lin Junyang
Date   : 2021.10
Version: 01
-------------------------------------------------------------------------
Lidar  TX/RX ◖ ←-----------------------→  ❒ Object
-------------------------------------------------------------------------
Operation Sequency:
1. Lidar generates internal laser firing triger signal.
2. Lidar TX sends laser pulse out.Reference signal returns immediately.
3. Laser echo returns to Lidar and convert to data by ADC.
-------------------------------------------------------------------------
This simulation simulates the Lidar trigger signal and the received data.
Simulation of 3 targets echo signals.
=========================================================================

Run single_ch_basic.m in MATLAB. 
