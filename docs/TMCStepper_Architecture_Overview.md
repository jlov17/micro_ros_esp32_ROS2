# TMCStepper Library Architecture Overview
### How TMC5272 Dual-Motor Control Works in Your System

---

## 1. High-Level System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                         Your Application                          │
│                          (main.cpp)                               │
│                                                                    │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  TMC5272Stepper driver(CS_PIN, RSENSE);                    │  │
│  │                                                             │  │
│  │  driver.M0_VMAX(50000);      // Control Motor 0            │  │
│  │  driver.M1_XTARGET(10000);   // Control Motor 1            │  │
│  │  driver.GCONF(0x00000000);   // Global configuration       │  │
│  └────────────────────────────────────────────────────────────┘  │
└──────────────┬───────────────────────────────────────────────────┘
               │
               │ Function Calls
               ▼
┌──────────────────────────────────────────────────────────────────┐
│                      TMCStepper Library                           │
│                                                                    │
│  ┌────────────────────┐  ┌──────────────────┐  ┌──────────────┐ │
│  │  TMC5272Stepper    │  │  TMC2130Stepper  │  │  TMCStepper  │ │
│  │    (Specific)      │─>│   (SPI Layer)    │─>│   (Base)     │ │
│  └────────────────────┘  └──────────────────┘  └──────────────┘ │
│           │                      │                                │
│           │ Register Maps        │ SPI Communication              │
│           ▼                      ▼                                │
│  ┌────────────────────┐  ┌──────────────────┐                   │
│  │ TMC5272_bitfields  │  │  SW_SPI / HW_SPI │                   │
│  │      (.h)          │  │     (.cpp)       │                   │
│  └────────────────────┘  └──────────────────┘                   │
└──────────────┬───────────────────┬───────────────────────────────┘
               │                   │
               │ Register Data     │ SPI Protocol
               ▼                   ▼
┌──────────────────────────────────────────────────────────────────┐
│                       Hardware Layer                              │
│                                                                    │
│   ESP32-C3  ──SPI──>  TMC5272 IC  ──PWM/Current──>  Motors       │
│   GPIO7 (CS)          (Dual Axis)                    M0 & M1      │
│   GPIO3 (EN)                                                      │
└──────────────────────────────────────────────────────────────────┘
```

---

## 2. Class Hierarchy & Inheritance

The TMCStepper library uses object-oriented inheritance to share code between different TMC chip families:

```
                    ┌──────────────┐
                    │  TMCStepper  │  (Base Class)
                    └──────┬───────┘
                           │
                    • rms_current()
                    • microsteps()
                    • GSTAT() - basic
                    • test_connection()
                    • Rsense (0.075Ω)
                           │
                           │ Inherits
                           ▼
                    ┌──────────────────┐
                    │ TMC2130Stepper   │  (SPI Driver)
                    └──────┬───────────┘
                           │
                    • SPI read() / write()
                    • GCONF() register
                    • CHOPCONF()
                    • IHOLD_IRUN()
                    • begin() - initializes SPI
                    • defaults() - sets registers
                           │
                           │ Inherits & Extends
                           ▼
                    ┌──────────────────┐
                    │ TMC5272Stepper   │  (Dual Motor)
                    └──────────────────┘
                           │
                    • M0_XTARGET() - Motor 0 position
                    • M1_VMAX() - Motor 1 velocity
                    • M0_CHOPCONF() - Motor 0 chopper
                    • M1_IHOLD_IRUN() - Motor 1 current
                    • GCONF() - dual motor config
                    • DRV_CONF() - driver config
```

### Key Inheritance Benefits:
- **TMCStepper**: Common stepper motor math (current, microsteps)
- **TMC2130Stepper**: SPI communication reused by TMC5272
- **TMC5272Stepper**: Adds dual-motor register addressing

---

## 3. Register Access Flow

### Example: Writing to Motor 1's Target Position

```
Your Code:
    driver.M1_XTARGET(10000);
         │
         ▼
┌─────────────────────────────────────────────────────────────┐
│ TMC5272Stepper::M1_XTARGET(int32_t input)                   │
│ {                                                            │
│   M1_XTARGET_register = input;                              │
│   write(0x3D, M1_XTARGET_register);  ← Register 0x3D       │
│ }                                                            │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ TMC2130Stepper::write(uint8_t address, uint32_t data)       │
│ {                                                            │
│   addressByte |= 0x80;  // Set write bit                   │
│   beginTransaction();   // Start SPI                        │
│   switchCSpin(LOW);     // Enable chip select               │
│   transfer(addressByte);                                    │
│   transfer(data >> 24); // Send 4 bytes MSB first          │
│   transfer(data >> 16);                                     │
│   transfer(data >> 8);                                      │
│   transfer(data);                                           │
│   switchCSpin(HIGH);    // Disable chip select              │
│   endTransaction();                                         │
│ }                                                            │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ SPI.transfer() - Arduino/ESP32 Hardware SPI                  │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
                   ┌──────────┐
                   │ TMC5272  │ Physical chip
                   │ Register │ 0x3D ← 10000
                   │  0x3D    │
                   └──────────┘
```

---

## 4. Bitfield Structures (Register Mapping)

TMC5272 uses C++ structs with bitfields to map register contents:

```cpp
// From TMC5272_bitfields.h
struct GCONF_t {
  constexpr static uint8_t address = 0x00;
  union {
    uint32_t sr;  // 32-bit "shift register" raw value
    struct {
      // Motor 0 bits (0-15)
      bool M0_en_pwm_mode : 1,      // Bit 0
           M0_multistep_filt : 1,   // Bit 1
           M0_shaft : 1,            // Bit 2
           ...
           M0_drv_enn : 1,          // Bit 14 - Motor 0 Enable (active low)
           ...
      // Motor 1 bits (16-31)
      bool M1_en_pwm_mode : 1,      // Bit 16
           ...
           M1_drv_enn : 1,          // Bit 28 - Motor 1 Enable (active low)
           ...
    };
  };
};
```

### Usage in Code:

```cpp
// Method 1: Write entire register as 32-bit value
driver.GCONF(0x00000000);  // All bits = 0 (both motors enabled)

// Method 2: Read-modify-write individual bits (not exposed in TMC5272)
GCONF_t conf;
conf.sr = driver.GCONF();     // Read current value
conf.M1_drv_enn = 0;          // Enable M1
driver.GCONF(conf.sr);        // Write back
```

---

## 5. TMC5272 Register Memory Map

The TMC5272 has **three types** of registers:

### A. Global Registers (Shared by both motors)
```
0x00  GCONF        - Global configuration (enable M0/M1)
0x01  GSTAT        - Global status (errors, reset flags)
0x03  NODECONF     - Node address for multi-drop
0x08  DRV_CONF     - Driver configuration
```

### B. Motor 0 Registers (Prefix M0_)
```
0x20  M0_CHOPCONF     - Chopper configuration
0x30  M0_IHOLD_IRUN  - Hold/run current
0x33  M0_XTARGET     - Target position
0x21  M0_XACTUAL     - Actual position
0x24  M0_VMAX        - Maximum velocity
0x38  M0_X_ENC       - Encoder position
0x3A  M0_DRV_STATUS  - Driver status
```

### C. Motor 1 Registers (Prefix M1_)
```
0x40  M1_CHOPCONF     - Chopper configuration
0x50  M1_IHOLD_IRUN  - Hold/run current
0x53  M1_XTARGET     - Target position
0x41  M1_XACTUAL     - Actual position
0x44  M1_VMAX        - Maximum velocity
0x58  M1_X_ENC       - Encoder position
0x5A  M1_DRV_STATUS  - Driver status
```

### Register Organization:
```
┌──────────────────────────────────────────────┐
│ 0x00-0x09: Global (GCONF, GSTAT, DRV_CONF)  │
├──────────────────────────────────────────────┤
│ 0x20-0x3F: Motor 0 (M0_*)                   │
├──────────────────────────────────────────────┤
│ 0x40-0x5F: Motor 1 (M1_*)                   │
└──────────────────────────────────────────────┘
```

---

## 6. How Your main.cpp Uses the Library

### Initialization Sequence:

```cpp
// 1. Create driver object (calls constructor)
TMC5272Stepper driver(CS_PIN, RSENSE);  
   │
   ├─> TMC5272Stepper::TMC5272Stepper()
   ├─> calls TMC2130Stepper::TMC2130Stepper() (parent constructor)
   └─> calls defaults() - sets GCONF_register = 0

// 2. Setup SPI (in setup())
SPI.begin();
driver.begin();  // DON'T CALL - writes cached GCONF_register!
   │
   └─> TMC2130Stepper::begin()
       └─> Writes GCONF_register (bug: resets to 0)

// 3. Configure motors
driver.GCONF(0x00000000);  // Enable both motors
driver.M0_CHOPCONF(0x00000003);  // TOFF=3, 256 µsteps
driver.M0_IHOLD_IRUN(0x000A1008);  // IHOLD=8, IRUN=16, delay=10

driver.M1_CHOPCONF(0x00000003);
driver.M1_IHOLD_IRUN(0x000A1008);

// 4. Set motion parameters
driver.M0_VMAX(50000);
driver.M0_AMAX(50000);
driver.M0_XTARGET(0);  // Hold at position 0

driver.M1_VMAX(50000);
driver.M1_AMAX(50000);
driver.M1_XTARGET(0);

// 5. Runtime control (in timer_callback())
int32_t m0_pos = driver.M0_XACTUAL();  // Read position
int32_t m1_enc = driver.M1_X_ENC();    // Read encoder
driver.M0_XTARGET(new_target);         // Command movement
```

---

## 7. Common Patterns in Your Code

### Pattern 1: Dual Motor Control
```cpp
// Apply same configuration to both motors
for (int motor = 0; motor < 2; motor++) {
  if (motor == 0) {
    driver.M0_CHOPCONF(0x00000003);
    driver.M0_IHOLD_IRUN(0x000A1008);
  } else {
    driver.M1_CHOPCONF(0x00000003);
    driver.M1_IHOLD_IRUN(0x000A1008);
  }
}
```

### Pattern 2: Register Readback Verification
```cpp
driver.GCONF(0x00000000);
uint32_t readback = driver.GCONF();
if (readback != 0x00000000) {
  Serial.println("GCONF write failed!");
}
```

### Pattern 3: Motor Position Control
```cpp
// In ROS callback
void motor_cmd_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg = msgin;
  driver.M0_XTARGET(msg->data.data[0]);  // Motor 0
  driver.M1_XTARGET(msg->data.data[1]);  // Motor 1
}
```

---

## 8. Critical Bug Discovery: defaults() & begin()

### The Problem:

```cpp
// In TMC5272Stepper.cpp constructor
TMC5272Stepper::TMC5272Stepper(...) {
  defaults();  // ← Sets GCONF_register = 0
}

void TMC5272Stepper::defaults() {
  GCONF_register = 0;  // ← Cached value
  ...
}

// In TMC2130Stepper.cpp
void TMC2130Stepper::begin() {
  GCONF(GCONF_register.sr);  // ← Writes cached 0
  ...
}
```

### Why This Breaks TMC5272:

```
TMC5272 Reset State:
  GCONF = 0x50004000
  ├─> M0_drv_enn = 0 (enabled)
  └─> M1_drv_enn = 1 (disabled) ← Problem!

Library defaults():
  GCONF_register = 0 (cached in RAM)

Library begin():
  Writes GCONF_register = 0 to chip
  ├─> M0_drv_enn = 0 (enabled) ✓
  └─> M1_drv_enn = 0 (enabled) ✓

Your code:
  driver.GCONF(0x00000000);  ← Direct write
  
But begin() gets called by library, overwriting your setting!
```

### Solution:
```cpp
// DON'T call these - they reset GCONF
// driver.begin();
// driver.defaults();

// Instead, manually initialize
SPI.begin();
pinMode(CS_PIN, OUTPUT);
digitalWrite(CS_PIN, HIGH);

// Then configure directly
driver.GCONF(0x00000000);  // Your working value
```

---

## 9. SPI Communication Details

### SPI Transaction for WRITE:

```
Step 1: Assert CS (LOW)
   ESP32 GPIO7: HIGH → LOW

Step 2: Send 5 bytes over SPI
   Byte 0: Address | 0x80 (write bit)
           Example: 0x00 | 0x80 = 0x80 (write to GCONF)
   Byte 1: Data[31:24] (MSB)
   Byte 2: Data[23:16]
   Byte 3: Data[15:8]
   Byte 4: Data[7:0] (LSB)
           Example: 0x00000000 for GCONF

Step 3: Deassert CS (HIGH)
   ESP32 GPIO7: LOW → HIGH

Timing: ~3.3 MHz SPI clock (spi_speed = 16MHz/8)
```

### SPI Transaction for READ:

```
Step 1: Assert CS, send read request
   Send: Address (without write bit)
   Example: 0x00 (read GCONF)
   Recv: Previous status byte

Step 2: Deassert CS, reassert CS

Step 3: Send address again, receive data
   Send: Address
   Recv: Data byte 0 (MSB)
   
   Send: 0x00 (dummy)
   Recv: Data byte 1
   
   Send: 0x00 (dummy)
   Recv: Data byte 2
   
   Send: 0x00 (dummy)
   Recv: Data byte 3 (LSB)

Step 4: Deassert CS

Note: TMC chips require 2 transactions - first queues the read,
      second returns the data.
```

---

## 10. Key Takeaways for Your Team

### What the Library Does:
1. **Abstraction**: Hides SPI complexity behind simple function calls
2. **Type Safety**: Uses bitfield structs to prevent register corruption
3. **Reusability**: Inheritance shares code between TMC chip families
4. **Dual Motor**: TMC5272 extends TMC2130 with M0_/M1_ prefixed functions

### What You Need to Know:
1. **GCONF = 0x00000000** enables both motors (your working value)
2. **Don't call** `driver.begin()` or `driver.defaults()` - they reset GCONF
3. **M0_** functions control Motor 0 (registers 0x20-0x3F)
4. **M1_** functions control Motor 1 (registers 0x40-0x5F)
5. **Each write** triggers a 5-byte SPI transaction
6. **Each read** requires TWO SPI transactions

### Current System Status:
✅ Both motors move and respond to commands  
✅ M0 has holding torque  
✅ M1 can move but may lack holding torque at idle  
✅ GCONF correctly written as 0x00000000  
✅ SPI communication verified working  

### Debugging Tools:
```cpp
// Check register values
Serial.println(driver.GCONF(), HEX);
Serial.println(driver.M1_DRV_STATUS(), HEX);

// Check errors
uint32_t gstat = driver.GSTAT();
if (gstat & (1 << 5)) Serial.println("M1 driver error!");
```

---

## 11. File Structure Reference

```
.pio/libdeps/esp32-c3-devkitc-02/TMCStepper/src/
├── TMCStepper.h              Main header (class declarations)
├── TMCStepper_UTILITY.h      Helper macros
└── source/
    ├── TMCStepper.cpp        Base class (current calc, microsteps)
    ├── TMC2130Stepper.cpp    SPI layer (read/write functions)
    ├── TMC5272Stepper.cpp    TMC5272 specific (M0/M1 registers)
    ├── TMC5272_bitfields.h   Register bitfield definitions
    ├── GCONF.cpp             GCONF register handlers
    ├── CHOPCONF.cpp          CHOPCONF register handlers
    ├── IHOLD_IRUN.cpp        Current control handlers
    ├── DRV_STATUS.cpp        Status register handlers
    └── SW_SPI.cpp            Software SPI (if needed)
```

---

## Summary Diagram: Your System

```
┌─────────────────────────────────────────────────────────────┐
│  ESP32-C3 (main.cpp)                                        │
│                                                              │
│  driver.M0_XTARGET(pos0) ────┐                             │
│  driver.M1_XTARGET(pos1) ────┼─> TMC5272Stepper Library    │
│  driver.GCONF(0x00000000) ───┘                             │
│                               │                              │
│                               ├─> TMC2130Stepper (SPI)      │
│                               └─> TMCStepper (Base)         │
└───────────────────────────────┬─────────────────────────────┘
                                │ SPI (CS=GPIO7)
                                ▼
┌─────────────────────────────────────────────────────────────┐
│  TMC5272 Dual Stepper Driver IC                            │
│                                                              │
│  ┌─────────────────┐  ┌─────────────────┐                 │
│  │  Motor 0 Driver │  │  Motor 1 Driver │                 │
│  │  (M0_*)         │  │  (M1_*)         │                 │
│  └────────┬────────┘  └────────┬────────┘                 │
└───────────┼─────────────────────┼──────────────────────────┘
            │                     │
            ▼                     ▼
      ┌─────────┐           ┌─────────┐
      │ Motor 0 │           │ Motor 1 │
      │ + Encoder│          │ + Encoder│
      └─────────┘           └─────────┘
```

This architecture allows you to control two independent stepper motors through a single SPI interface, with the library handling all low-level register manipulation and communication protocol details.
