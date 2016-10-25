MarlinKimbra OverAll Configuration by C/D
===============
<br>
Author: Cosimo Orlando.<br>
Blog: http://creativityslashdesign.com
<br>
<br>
These settings are developed for Kosssel Mini project by C/D and they are completely compatible with release 4.2.8 as well as 4.2.9 of
__<a href="https://github.com/MagoKimbra/MarlinKimbra"> MarlinKimbra 3D Printer Firmware for Arduino </a>__.
<br>
<br>
## Overall specification for MarlinKimbra 3D Printer Firmware 4.2.9.
<br>
In this case, the MagoKimbra has added a feature cannot be revised. For more info, we leave you the following link.
<br>
#### <a href="https://github.com/MagoKimbra/MarlinKimbra/issues/163">Home Issue #163</a>
<br>
A brand new feature, into the release 4.2.9, move down of 25.64mm (in our case) the position after homing. This feature is inside the code of new release and it not be easily defeated. So, this means if you have an Z axe of 293.30mm, your effective height is of 293.30mm - 25.64mm = 267.66mm.
Inside the 4.2.8 this feature is absent for Kossel Printers.
<br>
For info, send a message through our blog <a href="http://creativityslashdesign.com"> C/D </a>.
<br>
<br>
<br>
## Installation Instruction:
- Download the MarlinKimbra Firmware 4.2.8 or 4.2.9.
- Decompress the firmware in a directory.
- Overwrite the file "Configuration_Overall.h" with the one downloadable right here.
- Compile and write the firmware with Arduino IDE.
- Have fun with your new configurated Delta printer!
<br>
 
<br>
## Bug list
#### On MarlinKimbra 3D Printer Firmware 4.2.9.
- "Boot Screen" misaligned as opposed to previous firmware 4.2.8.
- "Printing Pause" remains on same position without interrupt the printing (4.2.8 no problem).
<br>
<br>
<br>
Creative Commons License (NC)
