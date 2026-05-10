# E2 interface specification v4 1

- Source PDF: `docs/E2_interface_specification_v4_1.pdf`
- Extraction date: 2026-05-09
- Page count: 20
- SHA256: `efcfa6a4d0c2b620bf328929b2d78b03b864c7ece593b6b4a50a229c8421fc46`

## Page 1

Specification E2 Interface
Version 4.1
Name Date
Created: Haider A. 16.05.2011
Checked: Robert Mayr 16.05.2011
Approved: Schwarz Roswitha 20.05.2011
Reason for
change:
Text corrections

## Page 2

Specification E2-Interface_V4-1.docx Page 2/20
TABLE OF CONTENTS
1 INTRODUCTION ........................................................................................................... 3
1.1 Overview.................................................................................................................. 3
1.2 Agreements ............................................................................................................. 3
2 CHARACTERISTICS .................................................................................................... 3
2.1 Layer 1: Physical Layer ........................................................................................... 3
2.2 Layer 2: Data Link Layer.......................................................................................... 5
2.2.1 Bit-transfer ......................................................................................................... 5
2.2.2 Data Transfer Formats....................................................................................... 5
2.2.3 Control Byte....................................................................................................... 6
2.2.4 PEC Packet Error Code (Checksum)................................................................. 6
2.3 Layer 3: Network Layer (Protocol) ........................................................................... 7
2.3.1 Read Byte from Slave........................................................................................ 7
2.3.2 Write Byte to Slave .......................................................................................... 10
2.4 Memory areas........................................................................................................ 12
2.4.1 Custom memory .............................................................................................. 12
3 APPENDIX: FLOW-CHARTS ..................................................................................... 18
3.1 “Addressing + Read/Write distinction” flow chart ................................................... 18
3.2 “Read from Memory” flow chart ............................................................................. 19
3.2.1 “Read from Custom Memory” flow chart .......................................................... 19
3.3 “Write to Memory” flow chart.................................................................................. 20
3.3.1 “Write to Custom Memory” flow chart .............................................................. 20

## Page 3

Specification E2-Interface_V4-1.docx Page 3/20
1 Introduction
1.1 Overview
The E2 interface has been specified by E+E Elektronik in 2004 and represents a subset
of the E2 interface protocol. The E2 interface is based on similar principles to the
I²C Bus1 or the SMBus 1 introduced by Phillips Semiconductors in 1982. The most
significant differences to the I²C Bus are the slower transmission rate, the slightly
divergent addressing mechanism and the error detecti on (checksum). The E2 interface
is used for the digital, bi -directional data transmission between a master module (e.g.:
climate controller, microcontroller , etc.) and a slave module ( often an E+E transmitter).
The data transmission takes place in synchronous and serial modes, whereby the
master is responsible for generatin g the clock pulse. The slave cannot send any data
independently.
1.2 Agreements
The technical specifications in this document are to be regarded as recommendations
and apply for all E+E transmitters with E2 interface, assuming nothing to the contrary is
specified in their data sheets.
Terms:
Master: Module with E2 interface that initiates communication and can drive a clock
signal.
Slave: Module with E2 interface that cannot drive a clock signal
2 Characteristics
In the following, the E2 interface will be specified in accordance with the ISO-OSI Layer
Model up to Layer 3. In addition, it should also be noted that due to the significant
similarity to the SM Bus or the I²C Bus, the E2 interface is hardware -compatible with
popular microcontrollers with interfaces of tha t type. This means that E2-slaves can be
directly connected to the corresponding pins of these controllers.
2.1 Layer 1: Physical Layer
The E2 interface consists of two active lines (Clock and Data) as well as an earthing line
as reference potential. The Cloc k and Data lines are connected with the operating
voltage via pull-up resistors. In the idle state, both lines are at High -Level (positive
logic). The respective inputs and outputs of the modules are designed as Open Drain
(or Open Collector), and thus can be connected directly as „wired AND“.
1 All brands, names, product names and logos listed are registered trademarks or brands of their
respective owners that we hereby explicitly acknowledge.

## Page 4

Specification E2-Interface_V4-1.docx
Fig. 1: Principle joining together of master
Parameter
Symbol Parameter
VDD Operating Voltage
VIH Input High Level
VIL Input Low Level
VOL Output Low Level
Cmax Line capacity to ground
fCLK Clock frequency (
rate)
tCLKH Clock-High time
tCLKL Clock-Low time
Rup Pull-up resistor
For any other parameters not specified here, the specifications in the data sheets of the
modules used apply.
Remark 1: The maximum data rate that can be achieved
and the pull-up resistors. The RUP
Principle joining together of master- and slave Module
Minimum Maximum Unit
Operating Voltage V
0,8 VDD VDD + 0,3 V
VSS – 0,3 0,2 * VDD
or 0,8
V
Level 0,7 V
ine capacity to ground 1 nF
frequency (data 500 5000 Hz
100 s
100 s
1 100 k
parameters not specified here, the specifications in the data sheets of the
Remark 1: The maximum data rate that can be achieved depends on the combination of the line capacity
UP value is the value of all pull-up resistors connected in parallel.
Page 4/20
lave Module
Remark
See module
description
Smallest value is
valid
Iin = 0,5 mA
@ Rup = 22 k
see Remark 1
see Remark 1
see Remark 1
parameters not specified here, the specifications in the data sheets of the
on the combination of the line capacity
up resistors connected in parallel.

## Page 5

Specification E2-Interface_V4-1.docx
2.2 Layer 2: Data Link Layer
2.2.1 Bit-transfer
The data tran smission always
line is used as a synchronisation line, which is
exception of the start and stop conditions, a change of the level on the data line is only
permissible during a LOW
communication there is a start condition. This is reali
line, with simultaneous sustained high
4us the clock line is drawn to low
data line. The data transfer takes place during the high
data bits (incl. ACK/ NACK) have been sent, the communication is terminat
stop condition. A stop condition
simultaneous high-level on the clock line.
Clock Low Extension
The slave has the option of holding the clock line at low level for up to 25ms after every
transmitted data bit to gain a little time for internal operations. The total transmission
time for a complete byte may not exceed 35ms
2.2.2 Data Transfer Format
The data is transmitted from the transmitter in bytes, and confirmed by the receiver
individually (for each byte) with ACK (data line
ninth bit. The first data byte (control byte) is always sent from the master to the slave.
The first four bits represent the main command and the next three
„address“ of the slave module. The eigh
transfer (R/W=0 data from master to slave; R/W=
Layer 2: Data Link Layer
always takes place serial ly by bit and synchronous
a synchronisation line, which is always controlled by
exception of the start and stop conditions, a change of the level on the data line is only
ermissible during a LOW -phase of the clock line. At the beginning of every
communication there is a start condition. This is realised by a negative edge on the data
line, with simultaneous sustained high -level on the clock line. After a delay of at least
us the clock line is drawn to low-level, and the first data bit (MSB) can be placed on the
data line. The data transfer takes place during the high -phase of the clock line. After all
data bits (incl. ACK/ NACK) have been sent, the communication is terminat
stop condition is defined by a positive edge on the data line with
level on the clock line.
Fig. 2: Bit Transfer
of holding the clock line at low level for up to 25ms after every
transmitted data bit to gain a little time for internal operations. The total transmission
time for a complete byte may not exceed 35ms, however.
ormats
ted from the transmitter in bytes, and confirmed by the receiver
individually (for each byte) with ACK (data line = Low) or NACK (data line = High) as
ninth bit. The first data byte (control byte) is always sent from the master to the slave.
bits represent the main command and the next three
s“ of the slave module. The eigh th bit (R/W) specifies the direction of the data
transfer (R/W=0 data from master to slave; R/W=’1’ data from slave to master).
Fig. 3: Byte Transfer
Page 5/20
by bit and synchronous ly. The clock
the master. With the
exception of the start and stop conditions, a change of the level on the data line is only
phase of the clock line. At the beginning of every
ed by a negative edge on the data
After a delay of at least
level, and the first data bit (MSB) can be placed on the
phase of the clock line. After all
data bits (incl. ACK/ NACK) have been sent, the communication is terminat ed with a
a positive edge on the data line with
of holding the clock line at low level for up to 25ms after every
transmitted data bit to gain a little time for internal operations. The total transmission
ted from the transmitter in bytes, and confirmed by the receiver
= Low) or NACK (data line = High) as
ninth bit. The first data byte (control byte) is always sent from the master to the slave.
bits represent the main command and the next three bits represent the
bit (R/W) specifies the direction of the data
data from slave to master).

## Page 6

Specification E2-Interface_V4-1.docx Page 6/20
2.2.3 Control Byte
The control byte of the E2 interface is used only for the differentiation of various
command modes (which are defined in what is referred to as the m ain command) and
the data flow direction (R/W). The control byte is defined as follows:
Control Byte
Bit 7 Bit 6 Bit 5 Bit 4 Bit 3 Bit 2 Bit 1 Bit 0
Main Command Device Address R/W
Fig. 4: Control Byte Structure
Bit Meaning
0 (LSB) R/W
1 Device address low bit
2 Device address
3 Device address high bit
4 Main command low bit
5 Main command
6 Main command
7 (MSB) Main command high bit
2.2.4 PEC Packet Error Code (Checksum)
For the detection of transmission errors a checksum byte is transmitted as the last data
byte of every transmission. The checksum byte corresponds to the low byte of the sum
(unsigned char) of all transmitted bytes.
Read: Checksum byte = (Control byte + Data byte) MOD 0x100
Write: Checksum byte = (Control byte + Address byte + Data byte) MOD 0x100

## Page 7

Specification E2-Interface_V4-1.docx Page 7/20
2.3 Layer 3: Network Layer (Protocol)
All defined commands of the E2 interface are described in this section.Refer to the data
sheets of the modules used for the data format of the transmitted data bytes.
2.3.1 Read Byte from Slave
This command is supported by all modules with E2 interface and is used to read
individual data bytes.
Command structure:
Start Control Byte Data Byte Checksum Stop
7 6 5 4 3 2 1 0 ACK 7 6 5 4 3 2 1 0 ACK 7 6 5 4 3 2 1 0 ACK
Start 1 A x x x x x x x x A x x x x x x x x NA Stop
From Master to Slave
From Slave to Master
Attention: For the Read command, the bit 0 of the control byte is always ‘1’ (High). The
Read command is terminated by a NACK and a stop condition of the master.
For Read Byte from Slave , the Main Commands are organi sed so that important data
and information bytes (all measured values, status byte, sensor type , etc.) can be read
out with one single command (at the expense of I²C address space).
Non-implemented main commands will be answered as „0x55“or “0xFF”.
The following Main Commands are defined:
Control byte Bit number Type of command
7 6 5 4
0x11 0 0 0 1 Sensor type (group L-Byte)
0x21 0 0 1 0 Sensor type (subgroup)
0x31 0 0 1 1 Available physical measurements
0x41 0 1 0 0 Sensor type (group H-Byte)
0x51 0 1 0 1 Read from internal custom address
0x71 0 1 1 1 Status byte
0x81 1 0 0 0 Measurement value 1 low byte
0x91 1 0 0 1 Measurement value 1 high byte
0xA1 1 0 1 0 Measurement value 2 low byte
0xB1 1 0 1 1 Measurement value 2 high byte
0xC1 1 1 0 0 Measurement value 3 low byte
0xD1 1 1 0 1 Measurement value 3 high byte
0xE1 1 1 1 0 Measurement value 4 low byte
0xF1 1 1 1 1 Measurement value 4 high byte

## Page 8

Specification E2-Interface_V4-1.docx
2.3.1.2 Sensor type (group L
Identifies the current sensor type (1
Sensor Group
HIGH-Byte LOW-Byte
0x03 0x67
0x0367
2.3.1.3 Sensor type (sub-group
Indicate the slave sub-group (upper 4 bits) and the output type (lower 4 bits)
Example: sub-group = 19 hex f
sub-group = 46 hex f
For detailed information see slave product description.
measurements (control byte 0x31)
This byte explains the supported
physical measured from the Slave)
represents a physical measurement:
Bit 7 Bit 6 Bit 5
reserved reserved reserved
1...supported
0...not supported (unsupported)
2.3.1.5 Read from internal custom address (control byte 0x51)
When you read out the slave with the
(Control byte = 0x 51) you wi
(internal address pointer). After powering up the slave
always ‘0x00’ and increments after every reading (
operation to this address pointer is only
Byte to Slav e. If the internal custom address is greater than 1Byte the High
ignored (Internal address = 0x
address  new internal address = 0x00)
oup L-Byte & H-Byte, control byte 0x11 & 0x41)
sensor type (16 bits)
conversion to dec-value Product-series
EE 871
group, control byte 0x21)
group (upper 4 bits) and the output type (lower 4 bits)
= 19 hex for EExxx-1 in FT9 implementation
= 46 hex for EExxx-4 in FT6 implementation
For detailed information see slave product description.
(control byte 0x31)
explains the supported active physical measurements of the slave
from the Slave) , independent from the (analog) output.
a physical measurement:
Available physical measurements
Bit 5 Bit 4 Bit 3 Bit 2
reserved reserved CO2 air velocity temp
(unsupported)
Read from internal custom address (control byte 0x51)
When you read out the slave with the “read from internal custom address” command
will get the data (one Byte) from the actual
(internal address pointer). After powering up the slave , this internal address pointer is
and increments after every reading ( Control byte 0x
to this address pointer is only possible by a write command
If the internal custom address is greater than 1Byte the High
ignored (Internal address = 0x FF  read from internal address 
new internal address = 0x00)
Page 8/20
Byte, control byte 0x11 & 0x41)
group (upper 4 bits) and the output type (lower 4 bits)
implementation (E2 interface).
implementation (4-20mA).
Available physical
physical measurements of the slave (which is
ndependent from the (analog) output. Every bit
Bit 1 Bit 0
emperature humidity
“read from internal custom address” command
ll get the data (one Byte) from the actual custom address
this internal address pointer is
0x51). A direct write
a write command . See 2.3.2 Write
If the internal custom address is greater than 1Byte the High -Byte is
 increment internal

## Page 9

Specification E2-Interface_V4-1.docx Page 9/20
2.3.1.7 Read Status byte (control byte 0x71)
Reading the Status byte starts a new measurement (within the s lave). After the slave
specific measurement time the measurement values are ready for read out.
The status byte provides information about the validity of the last measurement.
Every bit represents a physical quantity (equal to the “ Available physical
measurements”):
Status byte
Bit 7 Bit 6 Bit 5 Bit 4 Bit 3 Bit 2 Bit 1 Bit 0
reserved reserved reserved reserved CO2 air velocity temperature humidity
Bit value:
1...error during measurement (see slave description)
0...measurement is OK
2.3.1.8 Read measurements (control byte 0x81 to 0xF1)
When reading out a 16 bit measured variable, it is necessary to read out the low byte
first and then the associated high byte. This ensures, that two associated bytes are
always read out (when reading the low by te together, the high byte is „captured“ in the
slave). For the data format of the measured values refer to the data sheets of the
corresponding modules.
Often used but not compulsory is the following assignment (similar to the bit order):
Measurement value 1 = humidity
Measurement value 2 = temperature
Measurement value 3 = air velocity
Measurement value 4= carbon dioxide (CO2)

## Page 10

Specification E2-Interface_V4-1.docx Page 10/20
2.3.2 Write Byte to Slave
The “write Byte to Slave” is used to configure the slave.
After a “write byte to slave” command it is required to verify that the data is at the right
address. The slave ACK only indicates only that the last command was transferred
correctly. The checksum is verified afterwards, meaning it is possible for the pre vious
command to be invalid. Command structure:
Start Control byte Address byte Data byte Checksum Stop
7 6 5 4 3 2 1 0 ACK 7 6 5 4 3 2 1 0 ACK 7 6 5 4 3 2 1 0 ACK 7 6 5 4 3 2 1 0 ACK
Start x x x x x x x 0 A x x x x x x x x A x x x x x x x x A x x x x x x x x A Stop
From Master to Slave
From Slave to Master
Attention: The Bit 0 of the Control byte during the write command is always ‘0’ (low).
For “Write Byte to Slave”, the main commands are organized in such a form, that
important bytes can be written with one single command. (at the expense of I²C address
space)
The main command is not implemented on all components using the E2 interface.
Therefore please check the written variables by an additional read command.
Defined main commands:
Control byte Bit number Command description
7 6 5 4
0x10 0 0 0 1 Direct write to custom area
0x50 0 1 0 1 Set internal custom pointer

## Page 11

Specification E2-Interface_V4-1.docx Seite 11/20
2.3.2.2 Direct write to custom area (0x10)
The main command 0x 10 can be used to write one byte directly into the custom area.
The address in the custom area and the data is stated directly in the command structure
(see command structure above).
2.3.2.5 Set internal custom (address) pointer (0x50)
When there is a custom area in a slave, there will also be a custom (address) pointer.
Internal custom (address) pointer low Byte = Data byte
Internal custom (address) pointer high Byte = Address byte
This pointer increments automatically after every read from an internal address.
This custom pointer can be used to read out data from the custom area:
1.) Set the custom (address) pointer to the desired value with the write main
command (Control Byte = 0x50)
2.) Read the custom area with the main command “read from internal custom
address” (Control Byte = 0x51, auto increment after reading)

## Page 12

Specification E2-Interface_V4-1.docx Seite 12/20
2.4 Memory areas
2.4.1 Custom memory
This page is also a functional overview. If in “supported functions” (bytes 0x03…0x3F) a bit is set, the function could be read (and written). See bytes 0x40 and following
Adr. R /W Group
description Description Detail Comment
0x00 R Firmware-Version Main version 1 = Version 1.xx FW-version = 0x55.0x55 means, that
no command is supported
0x01 R Firmware-Version Sub-Version 12 = Version x.12 FW-version = 0x55.0x55 means, that
no command is supported
0x02 R E2-Spec
Version of E2-
specification 4 = Version 4 Version of the E2 specification used
during product develop ment
7 6 5 4 3 2 1 0 OG = Offset and Gain
0x03 R Supported
functions
Custom
adjustment reserved reserved reserved reserved OG- CO2 OG-v OG-T OG-RH (each physical quantity)
MSB LSB 0= not supported
7 6 5 4 3 2 1 0 Availability of “save adjustment point
0x04 R Custom
adjustment reserved reserved reserved reserved APt- CO2 APt -v APt -T APt -RH value at custom Calibration” (each
physical quantity)
MSB LSB 0= not supported
7 6 5 4 3 2 1 0 Availability of the timestamp
0x05 R Custom
adjustment reserved reserved reserved reserved reserved reserved reserved ATime-gen (custom adjustment, general)
MSB LSB 0= not supported
7 6 5 4 3 2 1 0 Availability of the timestamp (custom
0x06 R Custom
adjustment reserved reserved reserved reserved ATime- CO2 ATime-v ATime-T ATime-RH Adjustment each physical quantity)
MSB LSB 0= not supported
7 6 5 4 3 2 1 0
0x07 R Operating
functions Error Code
Measureme
nt Value
Filter
Specific
Measuring
Interval
Global
Measuring
Interval
reserved E2 Bus
address
Custom
name
E+E serial
number Availability of several functions
MSB LSB 0= not supported
7 6 5 4 3 2 1 0
0x08 R Operating mode reserved reserved reserved reserved reserved reserved E2 priority Measure
mode Availability of several functions
MSB LSB 0= not supported
7 6 5 4 3 2 1 0
0x09 R Special features reserved reserved reserved reserved reserved reserved reserved Auto
adjustment Availability of several functions
MSB LSB 0= not supported
0x0A
to
0x3F
R 54 reserved

## Page 13

Specification E2-Interface_V4-1.docx Seite 13/20
Adr. R /W Group description Description Detail L/H-Byte Comment
0x40 R/W Custom adjustment Humidity Offset L-Byte Offset = signed Int [1/100 %RH]
0x41 R/W Offset H-Byte
0x42 R/W Gain L-Byte Gain = GainValue/32768 ( Example: GainValue = 32768 Gain of 1,0000000)
0x43 R/W Gain H-Byte
0x44 R/W Fpoint_L L-Byte [1/100 %RH] Level of last “lower” adjustment point
0x45 R/W Fpoint_L H-Byte
0x46 R/W Fpoint_U L-Byte [1/100 %RH] Level of last “upper” adjustment point
0x47 R/W Fpoint_U H-Byte
0x48 R/W Temp. Offset L-Byte Offset = signed Int [1/100 K]
0x49 R/W Offset H-Byte
0x4A R/W Gain L-Byte Gain = GainValue/32768 ( Example: GainValue = 32768 Gain of 1,0000000)
0x4B R/W Gain H-Byte
0x4C R/W Tpoint_L L-Byte [1/100 K] Level of last “lower” adjustment point
0x4D R/W Tpoint_L H-Byte
0x4E R/W Tpoint_U L-Byte [1/100 K] Level of last “upper” adjustment point
0x4F R/W Tpoint_U H-Byte
0x50 R/W Air velocity Offset L-Byte Offset = signed Int [1/100 m/s]
0x51 R/W Offset H-Byte
0x52 R/W Gain L-Byte Gain = GainValue/32768 ( Example: GainValue = 32768 Gain of 1,0000000)
0x53 R/W Gain H-Byte
0x54 R/W Vpoint_L L-Byte [1/100 m/s] Level of last “lower” adjustment point
0x55 R/W Vpoint_L H-Byte
0x56 R/W Vpoint_U L-Byte [1/100 m/s] Level of last “upper” adjustment point
0x57 R/W Vpoint_U H-Byte
0x58 R/W CO2 Offset L-Byte Offset = signed Int [ppm]
0x59 R/W Offset H-Byte
0x5A R/W Gain L-Byte Gain = GainValue/32768 ( Example: GainValue = 32768 Gain of 1,0000000)
0x5B R/W Gain H-Byte
0x5C R/W CO2point_L L-Byte [ppm] Level of last “lower” adjustment point
0x5D R/W CO2point_L H-Byte
0x5E R/W CO2point_U L-Byte [ppm] Level of last “upper” adjustment point
0x5F R/W CO2point_U H-Byte
0x60
to
0x7F
R/W
32 reserved
Reserved for other
physical quantities

## Page 14

Specification E2-Interface_V4-1.docx Seite 14/20
Adr. R /W Group description Description Detail
L/H-
Byte Comment
0x80 R/W Custom adjustment CA - global Year Last custom adjustment (date); Year = 6  2006
0x81 R/W CA - global Month 1 to 12
0x82 R/W CA - global Day
0x83 R/W CA- Humidity Year Last custom adjustment (date) RH; Year = 6  2006
0x84 R/W CA- Humidity Month 1 to 12
0x85 R/W CA- Humidity Day
0x86 R/W CA- Temperature Year Last custom adjustment (date)Temp.; Year = 6  2006
0x87 R/W CA- Temperature Month 1 to 12
0x88 R/W CA- Temperature Day
0x89 R/W CA-Velocity Year Last custom adjustment (date) V; Year = 6  2006
0x8A R/W CA-Velocity Month 1 to 12
0x8B R/W CA-Velocity Day
0x8C R/W CA-CO2 Year Last custom adjustment (date) CO2; Year = 6  2006
0x8D R/W CA-CO2 Month 1 to 12
0x8E R/W CA- CO2 Day
0x8F to 0x9A R/W 12 reserved Reserved for other physical quantities
0x9B to 0x9F R/W 5 reserved
0xA0 to 0xAF R Configuration Serial number Unique E+E serial number
0xB0 to 0xBF R/W Part name Free usable part name On delivery it is filled with the E+E sensor type. eg.EE871
0xC0 R/W Bus-address Configurable bus-address (0…7) On delivery the Bus-Address = 0
0xC1 R/W Error handling Error code Relevant if status byte marks an error Gives information about failure
0xC2 to 0xC5 R/W 4 reserved
0xC6 R/W Time interval measurement interval Global measurement interval L-Byte unsigned int // unit = 1/10 s
0xC7 R/W measurement interval Global measurement interval H-Byte unsigned int // unit = 1/10 s
0xC8 R/W measurement interval Specific interval moisture Positive = global Interval multiplier // Negative = global Interval divider
0xC9 R/W measurement interval Specific interval temperature Positive = global Interval multiplier // Negative = global Interval divider
0xCA R/W measurement interval Specific interval velocity Positive = global Interval multiplier // Negative = global Interval divider
0xCB R/W measurement interval Specific interval CO2 Positive = global Interval multiplier // Negative = global Interval divider
0xCC to 0xCF R/W 4 reserved Reserved for other physical quantities
0xD0 R/W Measurement filter Filter Humidity Details see product datasheet
0xD1 R/W Filter temperature Details see product datasheet
0xD2 R/W Filter velocity Details see product datasheet
0xD3 R/W Filter CO2 Details see product datasheet
0xD4 to 0xD7 R/W 4 reserved Reserved for other physical quantities
0xD8 R/W Operating mode See chapter operating mode
0xD9 R/W Special features See chapter special features
0xDA to 0xDF R/W 6 reserved Reserved for other configurations
0xE0 to 0xFD R/W 30 reserved
0xFE R Addresspointer Custom area L-Byte
0xFF R Custom area H-Byte

## Page 15

Specification E2-Interface_V4-1.docx Seite 15/20
2.4.1.1 Supported functions
Group description “supported functions” (see table above, column “group description”):
At this addresses you can “ask the slave” if the described functions are supported
See the column “Comment” and “Detail” in the table above.
Appropriate Bit = 0  function not supported
Appropriate Bit = 1  function supported
2.4.1.2 Custom adjustment
With this feature you can adjust the physical values in offset and gain.
The values you can set are (if supported):
1.) Offset (signed int)
2.) Gain (unsigned int  gain in “unsigned int” = gain in “float” *38 )
3.) Lower calibration point (unsigned int)
4.) Upper calibration point (unsigned int)
5.) Last custom adjustment (Calibration time) global (year, month, day)
6.) Last custom adjustment (Calibration time) every physical quantity (year, month, day)
2.4.1.3 Operating functions
R/W R/W R/W R/W R/W R/W R/W R/W
Adr.
0x07
Error
code
M.Value
Filter
Specific
M.Interval
Global
M.Interval reserved E2 bus
address
Custom
name
E+E serial
number
BIT7 BIT6 BIT5 BIT4 BIT3 BIT2 BIT1 BIT0
BIT0: E+E serial number
1: It is possible to read (only) the unique E+E serial number
0: Reading the E+E serial number isn’t possible.
BIT1: Custom namer
1: It is possible to read and write a free usable part name (16 Byte)
0: A free usable part name is unsupported
BIT2: E2 bus address
1: It is possible to change the device address. You can control up to 8 devices on the
same E2-lines (CLK & DATA). That means one master and up to 8 slaves
0: Only a single master to slave connection is possible
BIT3: reserved
1:
0:
BIT4: Global measurement interval
1: Changing the global measurement interval is possible
0: A fixed measurement interval is implemented (see product datasheet)
BIT5: Specific measurement interval
1: Changing the specific measurement interval is possible (see product datasheet)
0: no specific measurement intervals. Only a global measurement interval is possible.
(see product datasheet)
BIT6: Measurement value filter
1: Changing the measurement value filter is possible (see product datasheet)
0: There is a fixed measurement value filter implemented
BIT7: Error code
1: An erro r code can be requested if the s tatus byte marks an error (see product
datasheet)
0: No error code supported

## Page 16

Specification E2-Interface_V4-1.docx Seite 16/20
2.4.1.4 Operating mode
Supported function
R/W R/W R/W R/W R/W R/W R/W R/W
Adr.
0x08 reserved reserved reserved reserved reserved reserved E2
priority
Low
power
mode
BIT7 BIT6 BIT5 BIT4 BIT3 BIT2 BIT1 BIT0
BIT0: Low power mode
0: going into Low power mode is not possible
1: going into Low power mode is supported
BIT1: E2-priority (slave measurement and E2 communication at the same time)
0: Changing the E2-priority is not possible. Slave answers with NACK during
measurement
1: Changing the E2-priority is supported
BIT2-7: reserved
Documentation
R/W R/W R/W R/W R/W R/W R/W R/W
Adr.
0xD8 reserved reserved reserved reserved reserved reserved E2
priority
Low
power
mode
BIT7 BIT6 BIT5 BIT4 BIT3 BIT2 BIT1 BIT0
BIT0: Measure mode
0: Free running mode or trigger mode
1: Low power mode (measurement only after “read statusbyte”)
BIT1: E2-priority (slave measurement and E2 communication at the same time)
0: Priority to measurement. Slave answers with NACK during measurement
1: Priority to E2 communication
BIT2-7: reserved

## Page 17

Specification E2-Interface_V4-1.docx Seite 17/20
2.4.1.5 Special features
Supported function
R/W R/W R/W R/W R/W R/W R/W R/W
Adr.
0x09 reserved reserved reserved reserved reserved reserved reserved Auto
adjustment
BIT7 BIT6 BIT5 BIT4 BIT3 BIT2 BIT1 BIT0
BIT0: Auto adjustment
0: A manually triggered auto adjustment is not supported
1: A manually triggered auto adjustment is supported
BIT1-7: reserved
Documentation
R/W R/W R/W R/W R/W R/W R/W R/W
Adr.
0xD9 reserved reserved reserved reserved reserved reserved reserved Auto
adjustment
BIT7 BIT6 BIT5 BIT4 BIT3 BIT2 BIT1 BIT0
BIT0: Auto adjustment
read = 1  an auto adjustment is currently running
read = 0  currently normal operation (no auto adjustment)
set to 1  starts an auto adjustment. After the auto adjustment is finished, this bit will be
cleared automatically.
set to 0  interrupting the auto adjustment is not allowed and not possible
During the auto adjustment the measurement values are held on the last measured
value.
BIT1-7: reserved

## Page 18

Specification E2-Interface_V4-1.docx Seite 18/20
3 Appendix: Flow-charts
3.1 “Addressing + Read/Write distinction” flow chart
Start
Device address
matches?
Bit 0 in
“Control byte”
is HIGH
Read-Routine Write-Routine
End
NO
YES
YES NO
Control byte:
Bit 0: 0=Write / 1=Read
Bit 1: Device Address (Low-Byte)
Bit 2: Device Address
Bit 3: Device Address (High Byte)

## Page 19

Specification E2-Interface_V4-1.docx Seite 19/20
3.2 “Read from Memory” flow chart
3.2.1 “Read from Custom Memory” flow chart
Read-Routine Start
Read-Routine End
“Control byte” ==
0x51 ?
(read)
Data byte output from current
customer address
(from slave to master)
address pointer will be
incremented (regardless if ACK
or NACK received)
YES
In examples, all Control Byte values refer to bus address ‘0’.
Unsupported commands or address values will be answered with 0x55 or 0xFF
read byte from custom memory
“Control byte” ==
0x50 ?
(write)
send address byte value (is
always 0x00)
(from master to slave)
YES
set custom address pointer
send data byte value
(0x00…0xFD)
(from master to slave)
NO
NO

## Page 20

Specification E2-Interface_V4-1.docx Seite 20/20
3.3 “Write to Memory” flow chart
3.3.1 “Write to Custom Memory” flow chart
Write-Routine Start
Write-Routine End
In Examples, all Control Byte values refer to bus address ‘0’.
Unsupported commands or address values will be answered with 0x55 or 0xFF
“Control byte” ==
0x10 ?
(write)
send address byte value
(0x00…0xFD)
(from master to slave)
YES
write to custom memory
send databyte value
(0x00…0xFF)
(from master to slave)
address pointer will be
incremented (regardless if ACK
or NACK received)
NO
