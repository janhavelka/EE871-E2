# E2 Schnittstelle:

- Source PDF: `docs/EE871_E2_interface_addendum.pdf`
- Extraction date: 2026-05-09
- Page count: 3
- SHA256: `d925814dd54d176d42d487c14a579c60e462695d45b8e52f3dd4880dbda149f8`

## Page 1

created:
checked:
released:
26.04.2012/Kürnst.
31.05.2012/Scharinger
04.06.2012/SR

File: E2_Interface_EE871_v1.1.docx
E2 Interface for EE871-Series:
Addition al  to the specification E2 interface “specification E2 -interface_e x.doc”:
Readable parameters
These parameters/values [hex] can be read by the E2 interface:
command: return-value kind format measuring-range output
Group (two bytes) 0x0367 (871d)   unsigned int.
Sub-Group 0x09    byte
Available measured
variables
0x08    byte
Statusbyte:1) 0x0_    byte

Measuring value 1:   Not defined
Measuring value 2:   Not defined
Measuring value 3:  CO2
eg. for handhelds
and fast response
unsigned int.  0 – 2000 or
 0 – 5000 or
 0 – 10000
ppm
Measuring value 4:  CO2
Averaged Value
eg. for climate control
unsigned int.  0 – 2000 or
 0 – 5000 or
 0 – 10000
ppm

1) Gives information on whether last measurement was successful

Available parameters in custom area
 Firmware-Mainversion
 Firmware-Subversion
 Offset CO2
 Gain CO2
 Upper calibration point CO2
 Lower calibration point CO2
 Last customer adjustment
 Last customer adjustment of CO2
 Serial number
 Part name
 Error code
 global measurement time interval
 Special features

For additional information also view application notes AN0101 optimised power consumption and AN0102
changing the measuring time interval and response time.
Electrical requirements
Symbol Parameter Minimum Maximum Unit Remark
VDD Bus-High-Voltage 3,6 5,2 V For a minimum of supply
current use 4.5V to 5.0V
fCLK

Clock frequency 500 5000  Hz The maximum achievable
data rate depends on the
combination of line capacity
and the pull-up resistors. Rup Pull-up resistor 4,7 100 k

Error Code List
Error Code Description
1 Supply Voltage Low detected
200 Sensor Counts Low
possible damage of electronic or sensor-cell
201 Sensor Counts High
possible damage of electronic or sensor-cell
202 Supply Voltage Breakdown at current peak for measurement
maybe the internal resistance of supply unit is to high

## Page 2

created:
checked:
released:
26.04.2012/Kürnst.
31.05.2012/Scharinger
04.06.2012/SR

File: E2_Interface_EE871_v1.1.docx
Measurement Timing

 minimum typical maximum
t pwrup  5s 10s
t stab  4,3s 9,3s
t meas   0,7s
t mti 15s  3600s

Examples:

When the measuring time interval is > 15s, a measurement can be triggered by reading the status
byte.
Note: Because of sensor design, a measurement will only be triggered, if the last measurement dates
back longer than 10s.

tmti >15s Measurement
E2 interface
communication
…stabilizes sensor element
…measuring
…reading measurement value 3
…reading measurement value 4
…reading status byte
t
t
>10s
…reading status byte starts a new measurement
Next planned measurement
will not be performed
because of triggered
measurement before
t Power-UP
tpwrup
tmti
t Power-UP
Measurement
E2 interface
communication
…stabilizes sensor element
…measuring
…reading measurement value 3
…reading measurement value 4
…reading status byte

## Page 3

created:
checked:
released:
26.04.2012/Kürnst.
31.05.2012/Scharinger
04.06.2012/SR

File: E2_Interface_EE871_v1.1.docx
Timing for write commands
Writing a byte (with control byte 0x10) to the device is done by writing the flash memory which takes
<=150ms for each byte. During this time, E2 -communication interrupts are deactivated. Trying  to
communicate with the device while the flash is written forces the clock low extension which holds the
clock line low until the write routine has finished.
Note: When writing the measurement interval (address 0xC6 and 0xC7) both values will be written into
the flash together after both values were sent to the device. Writing will start after sending both bytes
and will cause a communication delay of <=300ms.
