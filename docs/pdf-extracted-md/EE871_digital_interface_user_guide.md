# EE871 digital interface user guide

- Source PDF: `docs/EE871_digital_interface_user_guide.pdf`
- Extraction date: 2026-05-09
- Page count: 4
- SHA256: `69af6269fa74ad128807080f773347bf920f148e72846fc5cd345ff77ab19283`

## Page 1

The E+E CO2 probe EE871 is designed for use in demanding OEM applications. It incorporates the dual wavelength NDIR CO 2
sensor, which compensates for ageing effects, is highly insensitive to pollution and stands for outstanding long term stability.
With a special filter cap, the probe can be employed in applications with periodical H 2O2 sterilization, see „Replacement Parts /
Accessories“.
A multiple point CO2 and temperature factory adjustment leads to excellent CO 2 measurement accuracy over the entire tempe-
rature working range.
The measured data range of up to 5 % CO 2 (50,000 ppm) is available on E2 digital interface and up to 1 % CO 2 (10.000 ppm) is
available on Modbus RTU interface.
For use in special applications do not hesitate to contact E+E Elektronik or a local distributor.
GENERAL
USER’S GUIDE
EE871 – CO2 Probe with Digital Interface
• The device shall not be exposed to extreme mechanical stress. The sensing head and mostly the sensing cell might not be
exposed to any mechanical stress.
Probe
Filter cap
CO2 Sensing cell
Sensing head

• The device must be operated with the filter cap on at all times. Do not touch the sensing cell or electronics inside the sensing
head.
• A long response time indicates a dirty filter cap, as it might happen in polluted applications. Do not attempt to clean the filter
cap; it would only cause its clogging. Replace the filter cap by an E+E original one, order no. HA010116.
• While replacing the filter cap take utmost care to not touch the sensing cell and the electronics.
• This device is not appropriate for safety, emergency stop or other critical applications where device malfunction or failure
could cause injury to human beings.
CAUTION
CONNECTION DIAGRAM DIMENSIONS
M12x1 flange coupling
M16x1.5
96 (3.78“)
18.5 (0.73“)
M12x1 Gewicht: 30g (1.06oz)

Modbus E2
1... +UB GND
2... B-RS485 +UB
3... A-RS485 DATA
4... GND CLOCK
Modbus E2
+UB GND brown
B-RS485 +UB white
A-RS485 DATA blue
GND CLOCK black
Shielding grey Weight: 30g (1.06oz)

## Page 2

(Modification rights reserved)
TECHNICAL DATA
For communication with EE871 with E2 interface please see the support literature at www.epluse.com/EE871 .
EE871 WITH E2 INTERFACE
MODBUS MAP
The measured values are saved as a 32Bit float value from 0x2D to 0x30. The factory setting for the Slave-ID is 246
as an integer 16Bit value. This ID can be customised in the register 0x00 (permitted values 1 - 247).
For Modbus protocol setting please see Application Note (www.epluse.com/EE871).
For communication with EE871 with Modbus RTU interface please see the Modbus Application Note AN0103 at
www.epluse.com/EE871.
EE871 WITH MODBUS INTERFACE
Measured values
 CO2
 Measuring principle Dual wavelength (non-dispersive infrared technology) NDIR
 Measurement range 0...2000 ppm: < ± (50 ppm + 2 % from the measured value)
 Accuracy at 25 °C and 0...5000 ppm: < ± (50 ppm + 3 % from the measured value)
 1013 mbar 1) (77 °F...14,69 psi) 0...10,000 ppm:  < ± (100 ppm + 5 % from the measured value)
   0...3 %: < ± (1,5 % from full scale + 2 % from the measured value)
   0...5 %:
 Response time t63 105 s with measured data averaging (smooth output)
   60 s without measured data averaging
 Temperature dependency 0...2000 ppm:
 (-20...45 °C) (-4...113 °F) 0...5000 ppm: typ. ± (1 + CO2 concentration [ppm] / 1000) ppm/°C
   0...10,000 ppm:
   0...3 %: typ. -0,3 % from the measured value/°C
   0...5 %:
 Measurement interval adjustable from 15 s to 1 h  (Factory setting: 15 s)
General
 Digital interface Modbus RTU or E2 (details: www.epluse.com)
Supply voltage  4.75 - 7.5 VDC
Average current consumption 2) 120 µA (at 1 h measurement interval)...4.3 mA (at 15 sec. measurement interval)
Current peak max. 350 mA for 0.05 s
 Housing / Protection class Plastic PC / Housing IP65
 Electrical connection  Connector M12 x 1
 Cable length E2 interface max. 10 m (32.8 ft)
 Electromagnetic compatibility EN61326-1

 (Industrial enviroment) EN61326-2-3
 Operating conditions -40...60 °C (-40...140 °F)  0...100 % RH (non-condensing)  85...110 kPa (12,33...15,95 psi)
 Storage conditions -40...60 °C (-40...140 °F)  0...100 % RH (non-condensing)  70...110 kPa (10,15...15,95 psi)
1) For averaging output
2) The average current consumption depends on the measurement interval
FLOAT (read register): INTEGER (write register):
Coil / Register Numbers Data-Addresses Parameter name
30046 0x2D CO2 Response time = 60s
30048 0x2F CO2 Response time = 105s
Coil / Register Numbers Data-Addresses Parameter name
60001 0x00 Slave-ID
60002 0x01 RS485 Setting
60003 0x02 Measuring time interval

## Page 3

For outdoor applications EE871 must be used with the radiation shield order no. HA010507, which protects the device
against rain, snow, ice, and solar radiation.
OPERATION OUTDOORS
The EE871 probe is ready to use and does not require any configuration by the user. The factory setup of EE871 corresponds
to the type number ordered. For ordering guide please see data sheet at www.epluse.com/EE871 .
If needed, the user can change the factory setup. One can set the Slave-ID and the Modbus parameter (baud rate, parity and
stop bits) and perform the adjustment/calibration of the CO
2 reading.
•  EE871 with Modbus Interface. Use the optional Modbus Configuration Adapter HA011012, see data sheet “Accessories”
at www.epluse.com/EE871) and the E+E Product Configuration Software EE-PCS.
• EE871 with E2 Interface. Use the optional E2 Test and Configuration Adapter HA011010, see data sheet “Accessories” at
www.epluse.com/EE871 and the E+E Product Configuration Software EE-PCS.
The E+E Product Configuration Software EE-PCS is available for free download at www.epluse.com/configurator .
SETUP AND ADJUSTMENT
SCOPE OF SUPPLY
Mounting flange HA010212
M12x1 flanged coupling with 50 mm (1,97”) stranded wire HA010705
Modbus configuration adapter HA011012
E2 Test and configuration adapter HA011010
E+E Product configuration software  EE-PCS
(Download: www.epluse.com/Configurator)
Connecting cable M12 - flying leads (1.5 m (59.06”) / 5 m (196.85”) / 10 m (393.70”)) HA0108 19/20/21
T-Coupler M12 - M12 HA030204
M12 Connector for self assembly HA010707
PTFE filter cap HA010116
H
2O2 filter cap  HA010122
Radiation shield HA010507
Protection cap for the M12 cable socket HA010781
Protection cap for the M12 plug of EE871 HA010782
REPLACEMENT PARTS / ACCESSORIES
For further information, see data sheet „Accessories“
- EE871 probe according to ordering guide
- Test report according to DIN EN10204 - 2.2
94
44
174
Ø105
122
W
20 Ø5

## Page 4

USA
FCC notice:
This equipment has been tested and found to comply with the limits for a Class B digital device, pursuant to part 15 of the FCC Rules. These
limits are designed to provide reasonable protection against harmful interference in a residential installation. This equipment generates, uses and
can radiate radio frequency energy and, if not installed and used in accordance with the installation manual, may cause harmful interference to
radio communications. However, there is no guarantee that interference will not occur in a particular installation. If this equipment does cause
harmful interference to radio or television reception, which can be determined by turning the equipment off and on, the user is encouraged to try
to correct the interference by one or more of the following measures:
- Reorient or relocate the receiving antenna.
- Increase the separation between the equipment and receiver.
- Connect the equipment into an outlet on a circuit different from that to which thereceiver is connected.
- Consult the dealer or an experienced radio/TV technician for help.
CANADIAN
ICES-003 Issue 5:
CAN ICES-3 B / NMB-3 B
INFORMATION  +43 7235 605 0 / info@epluse.com
Langwiesen 7 • A-4209 Engerwitzdorf, Austria
Tel: +43 7235 605-0 • Fax: +43 7235 605-8
info@epluse.com • www.epluse.com
LG Linz Fn 165761 t • UID-Nr. ATU44043101
Place of Jurisdiction: A-4020 Linz • DVR0962759
BA_EE871_e // v1.2 // technical data are subject to change
