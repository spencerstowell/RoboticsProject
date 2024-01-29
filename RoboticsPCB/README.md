# Overview
This folder contains the Autodesk EAGLE board (.brd) and schematic (.sch) files I created to assist with the hardware side of the robot we simulated for RRT.

The stepper motor drivers we used were the [ULN2003A](https://en.wikipedia.org/wiki/ULN2003A) with the [28BYJ](https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/) stepper motor.

I personally milled the pcb on my desktop CNC machine, a Genmitsu 3018PRO, using the .grbl files in the [useful](/RoboticsPCB/useful/) directory. I converted the files to .gcode via [Flatcam](http://flatcam.org/manual/introduction.html) and used a 30° drill bit to isolate the traces.

## Soldered Board w/ Milled Traces
![Image of the underside of the PCB, with soldered connections and milled traces](/RoboticsPCB/PCB_Traces.jpg)
## Fully Populated Board
![Image of the fully populated PCB w/ all IC's, JST's, & Capacitors in place.](/RoboticsPCB/Populated_PCB.jpg)
