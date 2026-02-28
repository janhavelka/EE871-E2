# EE871 E2 Protocol + Register Map (Implementation Reference)

**Target device:** EE871 CO2 probe, E2 interface, **0...5% CO2 (0...50,000 ppm)**.  
This document is a **complete implementation reference** for writing an E2 master library.

---

## 1) Quick mental model

E2 is a **2-wire synchronous, open-drain** bus (Clock + Data + GND), similar in electrical principles to I2C/SMBus but with:
- **Much slower clock** (500...5000 Hz)
- **Different "addressing"** embedded into the first byte ("Control Byte")
- **Checksum ("PEC")** appended to every transfer
- Slave **can clock-stretch** (hold clock low) up to 25 ms after any bit

E2 is master-driven: the **slave never initiates** communication.

---

## 2) Physical layer (wiring + electrical levels)

### 2.1 Signals
- **Clock (SCL/CLK)**: driven by master, open-drain with pull-up
- **Data (SDA/DATA)**: bidirectional, open-drain with pull-up
- **GND**: reference potential

From the E2 specification:
- Both lines are **open-drain** and pulled up to operating voltage via resistors; idle = HIGH.  
- Change of DATA level is allowed **only while clock is LOW**, except for START/STOP.  
- START = DATA falling edge while clock HIGH; STOP = DATA rising edge while clock HIGH.

### 2.2 EE871 connector / pins (M12x1)
From EE871 user guide ("Modbus / E2" wiring diagram):
- **Pin 1:** GND  
- **Pin 2:** +UB (supply)  
- **Pin 3:** DATA  
- **Pin 4:** CLOCK

### 2.3 Voltage levels and limits (E2 spec)
Let VDD be the E2 bus pull-up voltage.

**Electrical levels (E2 Interface Specification v4.1):**
- Input high threshold: **VIH >= 0.8*VDD** (max VIH = VDD + 0.3 V)
- Input low threshold: **VIL <= 0.2*VDD** (or <=0.8 V, whichever is smaller)
- Output low level: **VOL <= 0.7 V @ IOL = 0.5 mA**
- Line capacitance to ground: **Cline <= 1 nF** (example condition @ Rup = 22 kΩ)

**Bus-high voltage (CO2 E2 addendum):**
- VDD (bus-high voltage) **3.6...5.2 V**, recommended **4.5...5.0 V** for minimizing supply current.

> Practical note: because the bus is open-drain, the **pull-up voltage defines the bus HIGH level**.
> In many systems the pull-ups go to the same rail that powers the probe (within the allowed VDD range).
> If your MCU is 3.3 V, use a **bidirectional open-drain level shifter** compatible with clock stretching.

### 2.4 Pull-ups and bus constraints
- Pull-up resistor range:
  - E2 spec: **1 kΩ...100 kΩ**
  - CO2 addendum: **4.7 kΩ...100 kΩ**
- Clock frequency depends on line capacitance + pull-ups.
- EE871 E2 cable length guideline: **<= 10 m**.

---

## 3) Timing requirements (what the master must respect)

### 3.1 Clock frequency
- **fCLK = 500...5000 Hz**
- Minimum times:
  - **tCLKH >= 100 us** (clock high)
  - **tCLKL >= 100 us** (clock low)

### 3.2 START condition timing
From the E2 spec (bit transfer description):
- Create START by pulling **DATA HIGH->LOW while CLK is HIGH**
- Wait **>= 4 us**
- Then pull **CLK LOW** to begin shifting the first data bit (MSB first)

### 3.3 Bit validity / sampling
- Data must be **stable during the HIGH phase of CLK**
- DATA transitions are allowed only during **CLK LOW**, except START/STOP.

### 3.4 Clock stretching ("Clock Low Extension")
The slave may hold CLK LOW:
- Up to **25 ms after every transmitted data bit**
- Total transmission time for a complete byte must not exceed **35 ms**

Your master must therefore:
- When releasing CLK to HIGH, **read back CLK** (or wait until it actually becomes HIGH)
- Apply timeouts: per-bit stretch timeout <= 25 ms; per-byte timeout <= 35 ms

---

## 4) Layer 2: Byte framing, ACK/NACK, PEC

### 4.1 Byte transfer & ACK/NACK
- Transmission is in **bytes (8 bits)**, MSB first.
- Each byte is followed by an **ACK/NACK 9th bit**:
  - **ACK** = receiver pulls DATA LOW
  - **NACK** = receiver leaves DATA HIGH

### 4.2 Control Byte structure
The first transmitted byte is always the **Control Byte** sent **master -> slave**.

ControlByte bits (b7..b0):
- b7..b4: **MainCommand** (4 bits)
- b3..b1: **DeviceAddress** (3 bits, 0...7)
- b0: **R/W** (1 = Read, 0 = Write)

### 4.3 PEC / checksum ("Packet Error Code")
Checksum byte is always the **last byte** in every transmission.

From spec:
- Checksum is the **low byte of the sum** of all transmitted bytes.

Formulas:
- **Read:** `PEC = (ControlByte + DataByte) & 0xFF`
- **Write:** `PEC = (ControlByte + AddressByte + DataByte) & 0xFF`

---

## 5) Layer 3: Protocol commands (Main Commands)

### 5.1 Read Byte from Slave (generic read)
**Structure:**
`START -> ControlByte(read) -> (slave DATA byte) -> (slave PEC) -> master NACK -> STOP`

Notes:
- For READ commands, b0 (R/W) in ControlByte is always **1**
- Read terminates with master **NACK** then **STOP**

**Defined read main commands (ControlByte high nibble):**
- `0x1?` Sensor type (group L-byte)  (control byte base **0x11**)
- `0x2?` Sensor type (subgroup)      (base **0x21**)
- `0x3?` Available physical measurements (base **0x31**)
- `0x4?` Sensor type (group H-byte)  (base **0x41**)
- `0x5?` Read from internal custom address (base **0x51**)  <- reads data at internal pointer
- `0x7?` Status byte (base **0x71**) <- also triggers measurement (see Sec. 8)
- `0x8?`...`0xF?` Measurement value bytes:
  - 0x81 / 0x91 = MV1 low/high
  - 0xA1 / 0xB1 = MV2 low/high
  - 0xC1 / 0xD1 = MV3 low/high
  - 0xE1 / 0xF1 = MV4 low/high

If a main command is not implemented, slave may answer **0x55 or 0xFF**.

### 5.2 Write Byte to Slave
**Structure:**
`START -> ControlByte(write) -> AddressByte -> DataByte -> PEC -> STOP`

Notes:
- For WRITE commands, b0 (R/W) is always **0**.

Defined write main commands:
- `0x10` Direct write to custom area (one byte at explicit custom address)
- `0x50` Set internal custom pointer (sets pointer used by 0x51 reads)

### 5.3 Custom address pointer behavior
- Internal custom pointer is:
  - Set by **0x50** write: AddressByte = pointer high, DataByte = pointer low
  - Defaults to **0x0000 after power-up**
  - **Auto-increments after each read** via 0x51 (and after write commands per flowcharts)

---

## 6) EE871: Identification, supported measurements, measurement values

### 6.1 Sensor type (group) for EE871
EE871 group value is **0x0367** (decimal 871).  
Read sequence:
- Read group LOW byte with control byte **0x11** -> returns **0x67**
- Read group HIGH byte with control byte **0x41** -> returns **0x03**
Combine: `group = low + (high<<8)`.

### 6.2 Subgroup, available physical measurements
From CO2 E2 addendum:
- Subgroup = **0x09**
- Available physical measurement bits byte = **0x08** (bit 3 set -> CO2 supported)

### 6.3 Status byte (control byte 0x71)
Status byte bit meanings match "Available physical measurements".
- For EE871, only **bit 3 (CO2)** is relevant:
  - bit3 = 0 -> last CO2 measurement OK
  - bit3 = 1 -> error during last measurement; read error code from custom memory (see Sec. 7.4.5)

**Important behavior:** reading the status byte **starts a new measurement** in the slave (see Sec. 8).

### 6.4 Measurement values for EE871 CO2
From CO2 E2 addendum:
- Measurement values 1 and 2: **not defined**
- **Measurement value 3:** CO2 "fast response" (raw, no averaging), unsigned 16-bit, in ppm
- **Measurement value 4:** CO2 averaged value (moving average of last 11 measurements), unsigned 16-bit, in ppm

Read any 16-bit measured value as:
1) read LOW byte (control byte for low)  
2) read HIGH byte (control byte for high)  
This "captures" the paired bytes consistently in the slave.

---

## 7) Custom memory map (register map)

Custom memory is addressed by an **8-bit address 0x00...0xFF** (some are reserved).  
To read:
1) Set pointer with 0x50  
2) Read bytes with repeated 0x51 (pointer auto-increments)

To write:
- Use 0x10 to write (address, data) directly.

### 7.1 Overview / capabilities discovery
**0x03...0x3F** are "supported functions" bitfields (feature discovery).  
If a bit is set, the feature is supported and the relevant registers exist.

Key bytes:
- **0x00** Firmware main version
- **0x01** Firmware sub version
- **0x02** E2 spec version used by device
- **0x07** Operating functions supported bits (see Sec. 7.4.1)
- **0x08** Operating mode supported bits (see Sec. 7.4.2)
- **0x09** Special features supported bits (see Sec. 7.4.3)

### 7.2 Firmware / spec identification
- `0x00` Firmware-Version main
- `0x01` Firmware-Version sub
- `0x02` E2-Spec version (e.g., 4 = spec v4)

### 7.3 Calibration/adjustment registers (Offset/Gain and points)
These exist per physical quantity; for EE871 CO2, relevant block is:

**CO2 adjustment (ppm):**
- `0x58` CO2 Offset L (signed int, unit ppm)
- `0x59` CO2 Offset H
- `0x5A` CO2 Gain L (unsigned int; gain = GainValue / 32768)
- `0x5B` CO2 Gain H
- `0x5C` CO2 point_L L (unsigned int, ppm) last "lower" adjustment point
- `0x5D` CO2 point_L H
- `0x5E` CO2 point_U L (unsigned int, ppm) last "upper" adjustment point
- `0x5F` CO2 point_U H

### 7.4 Operating/configuration registers

#### 7.4.1 Feature support flags: address 0x07 (bitfield)
Meaning (bit set = supported):
- bit0: E+E serial number readable
- bit1: Part name (16 bytes) readable/writable
- bit2: E2 bus address configurable (0...7)
- bit4: Global measurement interval configurable
- bit5: Specific measurement interval configurable
- bit6: Measurement value filter configurable
- bit7: Error code available when status indicates error

#### 7.4.2 Operating mode support: address 0x08 (bitfield)
- bit0: Low power mode supported
- bit1: E2 priority supported (measurement vs comm priority)
(others reserved)

#### 7.4.3 Special features support: address 0x09 (bitfield)
- bit0: Manual "auto adjustment" supported

#### 7.4.4 Device identity strings
- `0xA0...0xAF` (R): **E+E serial number** (16 bytes)
- `0xB0...0xBF` (R/W): **Part name** (16 bytes; default filled e.g. "EE871")

#### 7.4.5 Bus address and error handling
- `0xC0` (R/W): **Bus-address** (0...7). Default 0.
- `0xC1` (R/W): **Error code** (meaning depends on product addendum; for CO2 see Sec. 9)

#### 7.4.6 Measurement interval
Global measurement interval (unsigned 16-bit, unit = **0.1 s**):
- `0xC6` low byte
- `0xC7` high byte

Specific interval factors (signed 8-bit convention described as):
- `0xCB` Specific interval CO2: Positive = global interval multiplier; Negative = global interval divider  
(Other quantities are at 0xC8..0xCA; usually irrelevant for EE871 CO2-only.)

#### 7.4.7 Measurement value filtering
- `0xD3` Filter CO2 (details product-specific; see device addendum / datasheet)

#### 7.4.8 Runtime operating mode register
- `0xD8` Operating mode (bitfield):
  - bit0 Measuremode: 0 = freerunning/trigger mode, 1 = low power mode (measure after status read)
  - bit1 E2 priority: 0 = measurement priority (NACK during measurement), 1 = communication priority

#### 7.4.9 Special features register
- `0xD9` Auto adjustment control/status:
  - read=1 -> auto adjustment running
  - set=1 -> start auto adjustment
  - set=0 -> cannot stop/interrupt
  - During auto adjustment, measurement values are held at last value.

#### 7.4.10 Address pointer visibility
- `0xFE` Address pointer low byte
- `0xFF` Address pointer high byte

---

## 8) Measurement timing (EE871 CO2 behavior)

From CO2 E2 addendum:

- Power-up warm-up:
  - t_pwrup: **5...10 s**
  - Warm-up stabilizes sensor; first measurement after ~4.3 s is mentioned in power-current section.

- Measurement time:
  - t_meas: **0.7 s**

- Measurement interval (global):
  - t_mti: **15...3600 s** (default factory often 15 s)

### 8.1 Triggering a measurement via status byte
If global measuring interval is **> 15 s**, a measurement can be triggered by reading **status byte (0x71)**,
but only if the last measurement is older than **10 s**.

Triggered measurement behavior:
- New measurement value available typically **5...10 s** after trigger.
- After each triggered measurement, the interval counter is reset to 0.

### 8.2 Selecting output value (MV3 vs MV4)
- MV4 (averaged, last 11 measurements) is standard, low-noise, suitable for control loops.
- MV3 is "fast response" but noisier (typ. +/-30 ppm noise mentioned) and reflects disturbances immediately.

---

## 9) Error codes (EE871 CO2 addendum)
Error codes are read via custom memory error handling register (commonly 0xC1), and validity is indicated by status byte bit3.

CO2 error list (addendum):
- 1: Supply Voltage Low detected
- 200: Sensor Counts Low (possible damage of electronic or sensor-cell)
- 201: Sensor Counts High (possible damage of electronic or sensor-cell)
- 202: Supply Voltage Breakdown at current peak for measurement (supply internal resistance too high)

---

## 10) Write timing and flash behavior (critical!)

From CO2 E2 addendum (write timing):
- Writing one byte with control byte **0x10** can take **<= 150 ms** (flash write).
- During flash write, E2 communication interrupts are disabled.
- If master tries to communicate during flash write, slave will **hold clock low** (clock stretching) until write finishes.

**Special case: writing measurement interval (0xC6 and 0xC7):**
- Both bytes are committed together.
- Writing starts **after both bytes are sent**.
- Total delay can be **<= 300 ms**.

---

## 11) Required transaction recipes (exact steps)

### 11.1 Read a "normal" byte (control-byte addressed read)
Use this for:
- group low/high (0x11 / 0x41)
- subgroup (0x21)
- available measurements (0x31)
- status (0x71)
- measurement bytes

Steps:
1) START
2) Send ControlByte (read: b0=1) and expect ACK
3) Read DataByte
4) Read PEC byte
5) Send NACK
6) STOP
7) Verify `PEC == (ControlByte + DataByte) & 0xFF`

### 11.2 Read from custom memory at address A (0x00...0xFF)
**Pointer + 0x51 reads**
1) Write "set internal pointer":
   - START
   - ControlByte = 0x50 with bus address (write)
   - AddressByte = 0x00 (pointer high for 8-bit addresses)
   - DataByte = A (pointer low)
   - PEC = (CB + Addr + Data) & 0xFF
   - STOP
2) Read with ControlByte 0x51:
   - START
   - ControlByte = 0x51 with bus address (read)
   - Slave returns DataByte = mem[A]
   - Slave returns PEC
   - Master NACK + STOP
3) Verify PEC = (CB + Data) & 0xFF

For reading multi-byte blocks (like 0xA0..0xAF), do step (1) once, then repeat (2) N times; pointer auto-increments.

### 11.3 Write custom memory byte at address A
1) START
2) ControlByte = 0x10 with bus address (write)
3) AddressByte = A
4) DataByte = value
5) PEC = (CB + Addr + Data) & 0xFF
6) STOP
7) **Wait** for flash write completion (up to 150 ms; up to 300 ms for the 0xC6/0xC7 pair)
8) Read back to verify.

---

## 12) Reference code from E+E Application Note AN0105 (verbatim)

These code snippets are provided by E+E as example implementations for an 8051 master.
They demonstrate: start/stop, send/read byte, ACK/NACK, and read-with-checksum.

### 12.1 Control byte constants (`fl_E2bus.h`)
```c
#define CB_TYPELO 0x11 // ControlByte for reading Sensortype Low-Byte
#define CB_TYPESUB 0x21 // ControlByte for reading Sensor-Subtype
#define CB_AVPHMES 0x31 // ControlByte for reading Available physical measurements
#define CB_TYPEHI 0x41 // ControlByte for reading Sensortype High-Byte
#define CB_STATUS 0x71 // ControlByte for reading Statusbyte
#define CB_MV1LO 0x81 // ControlByte for reading Measurement value 1 Low-Byte
#define CB_MV1HI 0x91 // ControlByte for reading Measurement value 1 High-Byte
#define CB_MV2LO 0xA1 // ControlByte for reading Measurement value 2 Low-Byte
#define CB_MV2HI 0xB1 // ControlByte for reading Measurement value 2 High-Byte
#define CB_MV3LO 0xC1 // ControlByte for reading Measurement value 3 Low-Byte
#define CB_MV3HI 0xD1 // ControlByte for reading Measurement value 3 High-Byte
#define CB_MV4LO 0xE1 // ControlByte for reading Measurement value 4 Low-Byte
#define CB_MV4HI 0xF1 // ControlByte for reading Measurement value 4 High-Byte
#define E2_DEVICE_ADR 0 // Address of E2-Slave-Device
```

### 12.2 Kernel interface (`knl_E2bus.h`)
```c
#define RETRYS 3 // number of read attempts
#define DELAY_FACTOR 2 // delay factor for configuration of interface speed

typedef struct st_E2_Return
{
  unsigned char DataByte;
  unsigned char Status;
} st_E2_Return;

st_E2_Return knl_E2bus_readByteFromSlave(unsigned char ControlByte);
void knl_E2bus_start(void);
void knl_E2bus_stop(void);
void knl_E2bus_sendByte(unsigned char);
unsigned char knl_E2bus_readByte(void);
void knl_E2bus_delay(unsigned int value);
char knl_E2bus_check_ack(void);
void knl_E2bus_send_ack(void);
void knl_E2bus_send_nak(void);
```

### 12.3 Read-byte routine with PEC check (`knl_E2bus.c`)
```c
st_E2_Return knl_E2bus_readByteFromSlave( unsigned char ControlByte )
{
  unsigned char Checksum;
  unsigned char counter=0;
  st_E2_Return xdata E2_Return;
  E2_Return.Status = 1;

  while (E2_Return.Status && counter<RETRYS)
  {
    knl_E2bus_start();                // send E2 start condition
    knl_E2bus_sendByte( ControlByte );// send control byte

    if ( knl_E2bus_check_ack() == 1 ) // ACK received?
    {
      E2_Return.DataByte = knl_E2bus_readByte();
      knl_E2bus_send_ack();
      Checksum = knl_E2bus_readByte();
      knl_E2bus_send_nak();

      if ( ( ( ControlByte + E2_Return.DataByte ) % 0x100 ) == Checksum )
        E2_Return.Status = 0;
    }

    knl_E2bus_stop();                 // stop condition
    counter++;
  }
  return E2_Return;
}
```

### 12.4 Start/stop primitives (AN0105)
```c
void knl_E2bus_start(void)
{
  knl_E2bus_set_SDA();
  knl_E2bus_set_SCL();
  knl_E2bus_delay(30);
  knl_E2bus_clear_SDA();
  knl_E2bus_delay(30);
}

void knl_E2bus_stop(void)
{
  knl_E2bus_clear_SCL();
  knl_E2bus_delay(20);
  knl_E2bus_clear_SDA();
  knl_E2bus_delay(20);
  knl_E2bus_set_SCL();
  knl_E2bus_delay(20);
  knl_E2bus_set_SDA();
}
```

### 12.5 Send/read byte + ACK/NACK (AN0105)
```c
void knl_E2bus_sendByte(unsigned char value)
{
  unsigned char mask;
  for ( mask = 0x80; mask > 0; mask >>= 1)
  {
    knl_E2bus_clear_SCL();
    knl_E2bus_delay(10);

    if ((value & mask) != 0) knl_E2bus_set_SDA();
    else knl_E2bus_clear_SDA();

    knl_E2bus_delay(20);
    knl_E2bus_set_SCL();
    knl_E2bus_delay(30);
    knl_E2bus_clear_SCL();
  }
  knl_E2bus_set_SDA();
}

unsigned char knl_E2bus_readByte(void)
{
  unsigned char data_in = 0x00;
  unsigned char mask = 0x80;
  for (mask=0x80;mask>0;mask >>=1)
  {
    knl_E2bus_clear_SCL();
    knl_E2bus_delay(30);
    knl_E2bus_set_SCL();
    knl_E2bus_delay(15);
    if (knl_E2bus_read_SDA()) data_in |= mask;
    knl_E2bus_delay(15);
    knl_E2bus_clear_SCL();
  }
  return data_in;
}

char knl_E2bus_check_ack(void)
{
  bit input;
  knl_E2bus_clear_SCL();
  knl_E2bus_delay(30);
  knl_E2bus_set_SCL();
  knl_E2bus_delay(15);
  input = knl_E2bus_read_SDA();
  knl_E2bus_delay(15);
  if(input == 1) return 0; // NAK
  else return 1;           // ACK
}

void knl_E2bus_send_ack(void)
{
  knl_E2bus_clear_SCL();
  knl_E2bus_delay(15);
  knl_E2bus_clear_SDA();
  knl_E2bus_delay(15);
  knl_E2bus_set_SCL();
  knl_E2bus_delay(28);
  knl_E2bus_clear_SCL();
  knl_E2bus_delay(2);
  knl_E2bus_set_SDA();
}

void knl_E2bus_send_nak(void)
{
  knl_E2bus_clear_SCL();
  knl_E2bus_delay(15);
  knl_E2bus_set_SDA();
  knl_E2bus_delay(15);
  knl_E2bus_set_SCL();
  knl_E2bus_delay(30);
  knl_E2bus_set_SCL();
}
```

---

## 13) Library design requirements (so it composes in a larger project)

### 13.1 Required HAL (platform abstraction)
Implement the driver using a tiny hardware abstraction with:
- `set_scl(level)` / `set_sda(level)` where **level=1 means release line** (open-drain high), level=0 means pull low
- `read_scl()` / `read_sda()` to support clock stretching checks
- `delay_us(t)` (or equivalent) to control bit timing

### 13.2 Suggested public API
Minimum API to implement:
- `bool e2_read_byte(uint8_t control_byte, uint8_t* out_data)`
- `bool e2_read_u16(uint8_t control_low, uint8_t control_high, uint16_t* out)`
- `bool e2_set_custom_ptr(uint16_t addr)`
- `bool e2_custom_read(uint16_t addr, uint8_t* out)`
- `bool e2_custom_read_n(uint16_t addr, uint8_t* buf, size_t n)`
- `bool e2_custom_write(uint8_t addr, uint8_t value)` (and verify)
- EE871 helpers:
  - `bool ee871_read_co2_fast(uint16_t* ppm)`  // MV3
  - `bool ee871_read_co2_avg(uint16_t* ppm)`   // MV4
  - `bool ee871_read_status(uint8_t* status)`  // 0x71
  - `bool ee871_read_error_code(uint8_t* code)`// custom 0xC1 (if supported)

### 13.3 Mandatory robustness rules
- Timeouts for clock stretching:
  - <= 25 ms per bit, <= 35 ms per byte
- Always verify PEC for every read/write transaction
- After any write (0x10/0x50), read back to verify
- After 0x10 writes, allow up to 150 ms (up to 300 ms for the 0xC6/0xC7 pair)

---

## 14) Source documents used
- **Specification_E2_Interface.pdf** (E2 Interface Specification v4.1)
- **AN0105.pdf** (E+E Application Note: E2 software examples)
- **E2-Interface-CO2.pdf** (CO2 module addendum: timing, MV3/MV4 meaning, write delays, power modes)
- **manual-9CvOog6A.pdf** (EE871 user guide: 50,000 ppm E2 range, pinout, measurement interval)
