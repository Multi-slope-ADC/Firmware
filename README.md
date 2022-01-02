# Firmware
[From eevblog forum: DIY high resolution multi-slope converter (with permissiom from Ulrich Harms)](https://www.eevblog.com/forum/metrology/diy-high-resolution-multi-slope-converter/msg3616117/#msg3616117)<br>
<br>
Changes for PCB from Rerouter:<br>
Changed mux7 from Nr. 5 (S6 buffered 7V) to Nr. 6 (S7 unbuffered 7V) - there is no buffered 7V awailable<br>
Use external Ref for ATMEGA ADC<br>
Swap ADC0 and ADC1 (change in rev of Kleinstein)<br>
<br>
[Original copy in branch: Kleinstein](https://github.com/Multi-slope-ADC/Firmware/tree/Kleinstein)<br>
<br>
High Resolution Multi-slope ADC Firmware (Atmel Studio 7) - Copyright: Ulrich Harms