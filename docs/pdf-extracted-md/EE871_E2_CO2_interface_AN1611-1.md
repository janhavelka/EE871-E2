# EE871 E2 CO2 interface AN1611 1

- Source PDF: `docs/EE871_E2_CO2_interface_AN1611-1.pdf`
- Extraction date: 2026-05-09
- Page count: 12
- SHA256: `5d4bf11d7c3e12779bb7fc6710ee6062b90a6dc1edf914792870b7bdce0e626c`

## Page 1

AN1611-1

Rev. 1.0 11/2016

APPLICATION NOTE AN1611-1

Application Note
E2 Interface for EE871, EE892 and EE893

Rev. 1.0 11/2016

Relevant for:
This application note applies to EE871, EE892 and EE893
Introduction:
The E2 interface is used for the digital, bidirectional data transmission between a Master and a Slave
device.
The data transmission takes place via synchronous and serial modes, the Master being responsible for
generating the clock signal. The Slave cannot send any data independently.

## Page 2

AN1611-1

Rev. 1.0 11/2016  1

## Page 3

AN1611-1

Rev. 1.0 11/2016  2
CONTENT
1 Readable parameters ....................................................................................................................... 3
1.1 Available parameters in custom area ...................................................................................... 3
2 Electrical requirements ..................................................................................................................... 3
3 Error Code List ................................................................................................................................. 3
4 Measurement Timing........................................................................................................................ 4
5 Timing for write commands .............................................................................................................. 5
6 Optimizing the power consumption .................................................................................................. 6
6.1 Operation Modes ..................................................................................................................... 6
6.2 Measuring Time Interval .......................................................................................................... 7
6.3 Communication ........................................................................................................................ 8
6.4 Bus voltage level ..................................................................................................................... 8
7 Setting the measuring time interval .................................................................................................. 8
7.1 Check if “Global Measurement Interval” function is supported ............................................... 8
7.2 Check current measuring time interval .................................................................................... 8
7.3 Set the measuring time interval ............................................................................................... 9
8 Selecting the Output Value .............................................................................................................. 9
9 Response time ............................................................................................................................... 10
10 Triggering a measurement ........................................................................................................ 10
Contact information ............................................................................................................................... 11

## Page 4

AN1611-1

Rev. 1.0 11/2016  3
1 Readable parameters

These parameters/values [hex] can be read via E2 interface:

command: return-value kind format Measuring range output
Group (two bytes)
EE871: 0x0367 (871d)
EE892: 0x037C (892d)
EE871: 0x037D (873d)
  unsigned int.
Sub-Group 0x09    byte
Available measured
variables 0x08    byte
Statusbyte:1) 0x0_    byte

Measuring value 1:   Not defined
Measuring value 2:   Not defined
Measuring value 3:
CO2
eg. for handhelds
and fast response
unsigned int.
 0 – 2000 or
 0 – 5000 or
 0 – 10000
ppm
Measuring value 4:
CO2
Averaged Value
eg. for climate control
unsigned int.
 0 – 2000 or
 0 – 5000 or
 0 – 10000
ppm
1) Gives information on whether last measurement was successful
1.1 Available parameters in custom area

• Firmware-Mainversion
• Firmware-Subversion
• Offset CO2
• Gain CO2
• Upper calibration point CO2
• Lower calibration point CO2
• Last customer adjustment
• Last customer adjustment of CO2
• Serial number
• Part name
• Error code
• global measurement time interval
2 Electrical requirements

Symbol Parameter Minimum Maximum Unit Remark
VDD Bus-High-Voltage 3,6 5,2 V For minimizing the supply
current use 4.5V to 5.0V
fCLK

Clock frequency 500 5000  Hz The highest achievable data
rate depends on the
combination of line capacity
and the pull-up resistors. Rup Pull-up resistor 4,7 100 kΩ
3 Error Code List

Error Code Description
1 Supply Voltage Low detected
200 Sensor Counts Low
possible damage of electronic or sensor-cell
201 Sensor Counts High
possible damage of electronic or sensor-cell
202 Supply Voltage Breakdown at current peak for measurement
maybe the internal resistance of supply unit is to high

## Page 5

AN1611-1

Rev. 1.0 11/2016  4
4 Measurement Timing

 minimum typical maximum
t pwrup  5s 10s
t stab  4,3s 9,3s
t meas   0,7s
t mti 15s  3600s

Examples:

For measuring time interval > 15s a measurement can be triggered by reading the status byte,
nevertheless only if the last measurement dates back longer than 10s

t
Power-UP
tpwrup
tmti
t
Power-UP
Measurement
E2 interface
communication
…stabilizes sensor element
…measuring
…reading measurement value 3
…reading measurement value 4
…reading status byte
tmti >15s
Measurement
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

## Page 6

AN1611-1

Rev. 1.0 11/2016  5
5 Timing for write commands

Writing a byte to the device (with control byte 0x10) takes <=150ms and can be done by writing the flash
memory. During the writing time E2 communication interrupts are deactivated. The attempt to
communicate with the device while the flash is being written forces the clock low extension which holds
the clock line low until the write routine has finished.
Note: When writing the measurement interval (address 0xC6 and 0xC7) both values will be written
together into the flash. Writing will start after sending both bytes and will cause a communication delay
of <=300ms.

## Page 7

AN1611-1

Rev. 1.0 11/2016  6
6 Optimizing the power consumption

6.1 Operation Modes

E+E CO2 modules are designed to change their operation mode based on the actual status of
measurement or communication. The supply current is different for each operation mode and it is
shown given below as well as in Figure 1.

Mode min. supply current description
Sleep mode 40µA The module is waiting for measurement or
communication request
Warm-up mode 1.7mA The module is in warm-up mode. Duration ~4.3s before a
measurement is taken.
Upon starting the warm-up mode a 10µF capacitor is
charged. The current peak of up to ~1.2A lasts for
~200µs.
Communication mode 3.4mA initiated by an interrupt on the E2 Bus and lasts for at
least 1s
Measuring mode 120mA average value over 350ms, caused by flashing the
infrared lamp.
For details see Figure 2.

Figure 1: Supply current for 15s measuring time interval @23°C
Setting a longer measuring interval extends the sleep mode time.

## Page 8

AN1611-1

Rev. 1.0 11/2016  7
After a reset the modules starts with the warm-up mode and continues with the first measurement
after 4.3s.

Figure 2: Supply current in measuring mode @23°C
6.2 Measuring Time Interval

The peak current during the measurement mode is conditioned by the infrared lamp and cannot be
reduced. The average power consumption can be reduced by increasing the measuring interval.
The measuring interval can be set by writing the interval time to customer addresses 0x C6 and 0xC7
as an unsigned integer value in unit s/10. Figure 3 shows the impact of the measuring time interval on
the average supply current.

Figure 3: average supply current as function of the measuring interval
Example:
For 15 s measuring time interval four AA batteries (1.5V, 2500mAh) would last approx. 30 days.
Changing the sampling rate to 600s (10min) would increase the battery lifetime  to about 30 months.

## Page 9

AN1611-1

Rev. 1.0 11/2016  8
6.3 Communication

Communication on the bus forces the module to switch to communication mode. The power
consumption of the CO2 modules can be reduced by avoiding unnecessary communication.
For low power consumption it is important to avoid disturbances on the bus, which  would prevent the
CO2 module from getting into sleep mode.
6.4 Bus voltage level

The high level on the bus is likely to be 3.6V ... 5.2V. For minimising the supply current the high level
on the bus should be 4.5V ... 5.0V.
7 Setting the measuring time interval

Following steps are necessary for setting the measurement time interval for E+E CO 2 modules.

7.1 Check if “Global Measurement Interval” function is supported

• Set the read pointer with control byte 0x50 Set internal custom pointer to address 0x07
• Read the value of byte 0x07 with control byte 0x51 Read from internal custom address

User setup of the measuring time interval is possible if bit 4 of the read byte is ‘1’.

7.2 Check current measuring time interval

• Set the read pointer with control byte 0x50 to address 0xC6
• Read the Global Measurement Interval low-byte with control byte 0x51
• Read the Global Measurement Interval high-byte with control byte 0x51 (pointer increments
automatically after reading a byte)
• Calculate the measuring time interval using the following formula
𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚 𝑡𝑡𝑚𝑚𝑚𝑚𝑚𝑚 𝑚𝑚𝑚𝑚𝑡𝑡𝑚𝑚𝑚𝑚𝑖𝑖𝑚𝑚𝑖𝑖= 𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚 𝑡𝑡𝑚𝑚𝑚𝑚𝑚𝑚 𝑚𝑚𝑚𝑚𝑡𝑡𝑚𝑚𝑚𝑚𝑖𝑖𝑚𝑚𝑖𝑖 𝑖𝑖𝑙𝑙𝑙𝑙 𝑏𝑏𝑏𝑏𝑡𝑡𝑚𝑚+ 𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚 𝑡𝑡𝑚𝑚𝑚𝑚𝑚𝑚 𝑚𝑚𝑚𝑚𝑡𝑡𝑚𝑚𝑚𝑚𝑖𝑖𝑚𝑚𝑖𝑖 ℎ𝑚𝑚𝑚𝑚ℎ 𝑏𝑏𝑏𝑏𝑡𝑡𝑚𝑚∗ 256
10
Example:
𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚 𝑡𝑡𝑚𝑚𝑚𝑚𝑚𝑚 𝑚𝑚𝑚𝑚𝑡𝑡𝑚𝑚𝑚𝑚𝑖𝑖𝑚𝑚𝑖𝑖= 150 + 0 ∗ 256
10 = 150
10 = 15𝑚𝑚

## Page 10

AN1611-1

Rev. 1.0 11/2016  9

7.3 Set the measuring time interval

For setting the measuring time interval one must always write both the low and the high byte of the
Global Measuring Interval (Bytes 0xC6 and 0xC7 in Custom Area). Values smaller than 150 (=15s)
and higher than 36000 (=3600s) will be ignored by the firmware.
Calculation of byte values:

𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚 𝑡𝑡𝑚𝑚𝑚𝑚𝑚𝑚 𝑚𝑚𝑚𝑚𝑡𝑡𝑚𝑚𝑚𝑚𝑖𝑖𝑚𝑚𝑖𝑖 𝑖𝑖𝑙𝑙𝑙𝑙 𝑏𝑏𝑏𝑏𝑡𝑡𝑚𝑚= (𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚 𝑡𝑡𝑚𝑚𝑚𝑚𝑚𝑚 𝑚𝑚𝑚𝑚𝑡𝑡𝑚𝑚𝑚𝑚𝑖𝑖𝑚𝑚𝑖𝑖∗ 10) 𝑀𝑀𝑀𝑀𝑀𝑀 256
 𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚 𝑡𝑡𝑚𝑚𝑚𝑚𝑚𝑚 𝑚𝑚𝑚𝑚𝑡𝑡𝑚𝑚𝑚𝑚𝑖𝑖𝑚𝑚𝑖𝑖 ℎ𝑚𝑚𝑚𝑚ℎ 𝑏𝑏𝑏𝑏𝑡𝑡𝑚𝑚= (𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚 𝑡𝑡𝑚𝑚𝑚𝑚𝑚𝑚 𝑚𝑚𝑚𝑚𝑡𝑡𝑚𝑚𝑚𝑚𝑖𝑖𝑚𝑚𝑖𝑖∗ 10)/256
MOD…modulo operation
For division only integer values are relevant.
Example for 60s interval:
 𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚 𝑡𝑡𝑚𝑚𝑚𝑚𝑚𝑚 𝑚𝑚𝑚𝑚𝑡𝑡𝑚𝑚𝑚𝑚𝑖𝑖𝑚𝑚𝑖𝑖 𝑖𝑖𝑙𝑙𝑙𝑙 𝑏𝑏𝑏𝑏𝑡𝑡𝑚𝑚= (60 ∗ 10) 𝑀𝑀𝑀𝑀𝑀𝑀 256 = 600 𝑀𝑀𝑀𝑀𝑀𝑀 256 =  88
 𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚𝑚 𝑡𝑡𝑚𝑚𝑚𝑚𝑚𝑚 𝑚𝑚𝑚𝑚𝑡𝑡𝑚𝑚𝑚𝑚𝑖𝑖𝑚𝑚𝑖𝑖 ℎ𝑚𝑚𝑚𝑚ℎ 𝑏𝑏𝑏𝑏𝑡𝑡𝑚𝑚=
(60∗10)
256 =
600
256 = 2
Writing values to custom area:
• Write the low byte with control byte 0x10 Direct write to custom area and address 0xC6.
• Write the high byte with control byte 0x10 Direct write to custom area and address 0xC7.
8 Selecting the Output Value

The E2 Interface provides up to 4 output values (measurement value 1 to measurement value 4).
Measurement value 4 (Value Index: 30 acc. E2 bus uniform measurement value table)is the standard
CO2 output value and it is appropriate for most applications. This CO2 output value is optimized for
low noise output signal and it is calculated out of the last 11 measurements
Measurement value 3 (Value Index: 31 acc. E2 bus uniform measurement value table) delivers fast
response CO2 value and is meaningful only for applications with fast changing CO2 concentration.
Measurement value 3 is the last measured CO2 value without any averaging. It implies a noise of
typ. ±30ppm which should be taken into account for the overall accuracy.
NOTE: When using measurement value 3 any disturbance affects immediately to the output
Example: The CO2 level increase caused by the exhaled air of an individual passing by the sensor
will be visible in the measurement value 3.
The choice of using one, the other or both measurement values depends on the application. Please
contact E+E Elektronik or a local distributor for any support you might need.

## Page 11

AN1611-1

Rev. 1.0 11/2016  10
9 Response time

The response time for measurement value 3 and 15s measurement interval is τ90 ≈ 60s.
For measurement value 4 with 15s measurement interval the response time is τ90 ≈ 105s. Longer
response time is of advantage in climate control applications because it filters out short time peaks
and consequently avoids changes in ventilation based on short time events.
Longer measuring interval increases the response time of measurement value 4.

10 Triggering a measurement

For measuring interval > 15s, the master device can trigger a measurement by reading the status byte
(control byte = 0x71) if the last measurement value is older than 10s.  The new measurement value
will be available typically 5s to 10s after the trigger.
After each triggered measurement the interval time counter is set to ‘0’.

## Page 12

AN1611-1

Rev. 1.0 11/2016  11

Contact information

E+E ELEKTRONIK GES.M.B.H.
Langwiesen 7
4209 Engerwitzdorf
Austria

Tel.: +43 (7235) 605-0
Fax: +43 (7235) 605-8
E-Mail: info@epluse.com

Homepage: www.epluse.com

For your local contact please visit the homepage.
The information in this document is believed to be accurate in all respects at the time of publication but is subject
to change without notice. E+E Elektronik assumes no responsibility for errors and omissions, and  shall not
accept responsibility for any consequences resulting from the use of information included herein. Additionally,
E+E Elektronik assumes no responsibility for the functioning of features or parameters  not described. E+E
Elektronik reserves the right to make changes without further notice. E+E Elektronik makes no warranty,
representation or guarantee regarding the suitability of its products for any particular purpose, nor does E+E
Elektronik assume any liability arising out of the application or use of any product or circuit, and specifically
disclaims any and all liability, inc luding consequential or incidental damages  without limitation. E+E Elektronik
products are not designed, intended, or authori sed for use in applications intended to support or sustain life, or
for any other application in which the failure of the E+E Elektronik product could create a situation where personal
injury or death may occur. Should the b uyer purchase or use E+E Elektronik products for any such unintended
or unauthorised application, the buyer shall indemnify and hold E+E Elektronik harmless agains t all claims and
damages.
