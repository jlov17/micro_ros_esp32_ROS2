# C++ Bitfields in TMCStepper: Deep Dive
### How Memory is Shared Between Integers and Individual Bits

---

## 1. What Are Bitfields?

Bitfields are a C++ language feature that let you specify how many bits each struct member occupies:

```cpp
struct Example {
  uint8_t field_a : 3;  // Uses 3 bits (values 0-7)
  uint8_t field_b : 5;  // Uses 5 bits (values 0-31)
  // Total: 8 bits = 1 byte
};
```

### Why Use Bitfields?
- **Hardware Registers**: Hardware chips pack multiple configuration values into single 32-bit registers
- **Memory Efficiency**: Pack many boolean flags into a single byte
- **Type Safety**: Access individual bits by name instead of bit-shifting
- **Readability**: `config.M1_drv_enn = 0` vs `config |= (1 << 28)`

---

## 2. The Union Trick: Two Ways to Access Same Memory

The TMCStepper library uses **unions** to access the same memory location in two different ways:

```cpp
union {
  uint32_t sr;      // Access as 32-bit integer (shift register)
  struct {
    bool bit0 : 1;  // Access individual bits by name
    bool bit1 : 1;
    bool bit2 : 1;
    // ... 29 more bits
  };
};
```

### Memory Layout:
```
Address 0x1000:  [bit31|bit30|...|bit2|bit1|bit0]  ← 32 bits
                        ↑                    ↑
                        |                    |
            Access as: sr = 0x50004000      bit0 = false
```

**Key Concept**: `sr` and the bitfield struct occupy **the exact same memory**. Changing one changes the other!

---

## 3. Real Example: GCONF Register (0x00)

### The Bitfield Definition:

```cpp
struct GCONF_t {
  constexpr static uint8_t address = 0x00;  // Register address
  union {
    uint32_t sr;  // Shift register - full 32-bit value
    struct {
      // Bits 0-15: Motor 0 configuration
      bool M0_en_pwm_mode : 1,       // Bit 0
           M0_multistep_filt : 1,    // Bit 1
           M0_shaft : 1,             // Bit 2
           M0_diag0_error : 1,       // Bit 3
           M0_diag0_ot_pwd : 1,      // Bit 4
           M0_diag0_stall_step : 1,  // Bit 5
           M0_diag1_stall_dir : 1,   // Bit 6
           M0_diag1_index : 1;       // Bit 7
      bool diag0_int_pushpull : 1,   // Bit 8
           diag1_poscomp_pushpull : 1, // Bit 9
           M0_small_hysteresis : 1,  // Bit 10
           M0_stop_enable : 1,       // Bit 11
           M0_direct_mode : 1,       // Bit 12
           M0_SD : 1,                // Bit 13
           M0_drv_enn : 1,           // Bit 14 ← Motor 0 Enable
           ramp_m1_dir : 1;          // Bit 15
      // Bits 16-31: Motor 1 configuration
      bool M1_en_pwm_mode : 1,       // Bit 16
           M1_multistep_filt : 1,    // Bit 17
           M1_shaft : 1,             // Bit 18
           diag0_intout_sel : 2,     // Bits 19-20 (2 bits!)
           diag1_xcomp_sel : 2,      // Bits 21-22 (2 bits!)
           M1_diag1_index : 1;       // Bit 23
      bool M1_diag1_index_2 : 1,     // Bit 24
           M1_small_hysteresis : 1,  // Bit 25
           M1_stop_enable : 1,       // Bit 26
           M1_direct_mode : 1,       // Bit 27
           M1_SD : 1,                // Bit 28
           M1_drv_enn : 1,           // Bit 29 ← Motor 1 Enable
           ramp_m1_dir_mode : 1,     // Bit 30
           diag0_sel_nstall_index : 1; // Bit 31
    };
  };
};
```

### How It Maps to Hardware:

```
GCONF Register (32 bits):

Bit:  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16
      ┌──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┐
      │ d│ r│M1│M1│M1│M1│M1│M1│M1│ d│ d│ d│ d│M1│M1│M1│
      │ i│ a│_d│_S│_d│_s│_s│_d│_d│ i│ i│ i│ i│_s│_m│_e│
      │ a│ m│rv│ D│ir│to│ma│ia│ia│ a│ a│ a│ a│ha│ul│n_│
      │ g│ p│_e│  │ec│p_│ll│g1│g1│ g│ g│ g│ g│ft│ti│pw│
      │ 0│ _│nn│  │t_│en│_h│_i│_i│ 1│ 1│ 0│ 0│  │st│m_│
      │  │ m│  │  │mo│ab│ys│nd│nd│ _│ _│ _│ _│  │ep│mo│
      │  │ 1│  │  │de│le│te│ex│ex│ x│ x│ i│ i│  │_f│de│
      │  │ _│  │  │  │  │re│_2│  │ c│ c│ n│ n│  │il│  │
      │  │ d│  │  │  │  │si│  │  │ o│ o│ t│ t│  │t │  │
      │  │ i│  │  │  │  │s │  │  │ m│ m│ o│ o│  │  │  │
      │  │ r│  │  │  │  │  │  │  │ p│ p│ u│ u│  │  │  │
      │  │  │  │  │  │  │  │  │  │ _│ _│ t│ t│  │  │  │
      │  │  │  │  │  │  │  │  │  │ s│ s│ _│ _│  │  │  │
      │  │  │  │  │  │  │  │  │  │ e│ e│ s│ s│  │  │  │
      │  │  │  │  │  │  │  │  │  │ l│ l│ e│ e│  │  │  │
      └──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┘

Bit:  15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
      ┌──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┐
      │ r│M0│M0│M0│M0│M0│ d│ d│M0│M0│M0│M0│M0│M0│M0│M0│
      │ a│_d│_S│_d│_s│_s│ i│ i│_d│_d│_d│_d│_d│_s│_m│_e│
      │ m│rv│ D│ir│to│ma│ a│ a│ia│ia│ia│ia│ia│ha│ul│n_│
      │ p│_e│  │ec│p_│ll│ g│ g│g1│g1│g0│g0│g0│ft│ti│pw│
      │ _│nn│  │t_│en│_h│ 1│ 0│_s│_i│_s│_o│_e│  │st│m_│
      │ m│  │  │mo│ab│ys│ _│ _│ta│nd│ta│t_│rr│  │ep│mo│
      │ 1│  │  │de│le│te│ p│ i│ll│ex│ll│pw│or│  │_f│de│
      │ _│  │  │  │  │re│ o│ n│_d│  │_s│d │  │  │il│  │
      │ d│  │  │  │  │si│ s│ t│ir│  │te│  │  │  │t │  │
      │ i│  │  │  │  │s │ c│ _│  │  │p │  │  │  │  │  │
      │ r│  │  │  │  │  │ o│ p│  │  │  │  │  │  │  │  │
      │  │  │  │  │  │  │ m│ u│  │  │  │  │  │  │  │  │
      │  │  │  │  │  │  │ p│ s│  │  │  │  │  │  │  │  │
      │  │  │  │  │  │  │ _│ h│  │  │  │  │  │  │  │  │
      │  │  │  │  │  │  │ p│ p│  │  │  │  │  │  │  │  │
      │  │  │  │  │  │  │ u│ u│  │  │  │  │  │  │  │  │
      │  │  │  │  │  │  │ l│ l│  │  │  │  │  │  │  │  │
      │  │  │  │  │  │  │ l│ l│  │  │  │  │  │  │  │  │
      └──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┘

Example Values:
  0x00000000 = All bits 0
    → M0_drv_enn = 0 (Motor 0 enabled)
    → M1_drv_enn = 0 (Motor 1 enabled)
    
  0x10004000 = Bits 14 and 28 set
    → M0_drv_enn = 0 (bit 14 = 0, enabled)
    → M1_drv_enn = 1 (bit 28 = 1, disabled)
```

---

## 4. Using Bitfields: Three Methods

### Method 1: Write Entire Register as Integer (Your Working Code)

```cpp
// Write all 32 bits at once
driver.GCONF(0x00000000);  
// Binary: 00000000 00000000 00000000 00000000
//         M1=disabled―┐   ┌―M0=disabled
//                     ↓   ↓
//         Bit 29=0 enables M1, Bit 14=0 enables M0
```

**Pros**: Fast, explicit, matches datasheet values  
**Cons**: Hard to see which bits are set

### Method 2: Read-Modify-Write Individual Bits

```cpp
GCONF_t config;
config.sr = driver.GCONF();   // Read current value (e.g., 0x10004000)
config.M1_drv_enn = 0;        // Change just bit 28
driver.GCONF(config.sr);      // Write back modified value
```

**Pros**: Only modifies intended bits, readable  
**Cons**: Requires read first, verbose

### Method 3: Build Register from Scratch (Library Pattern)

```cpp
GCONF_t config;
config.sr = 0;                 // Start with all zeros
config.M0_en_pwm_mode = 0;     // Bit 0 = 0
config.M0_multistep_filt = 0;  // Bit 1 = 0
config.M0_drv_enn = 0;         // Bit 14 = 0 (enable M0)
config.M1_drv_enn = 0;         // Bit 29 = 0 (enable M1)
driver.GCONF(config.sr);       // Write constructed value
```

**Pros**: Very readable, self-documenting  
**Cons**: Verbose for simple writes

---

## 5. Another Example: CHOPCONF Register

### The Structure:

```cpp
struct CHOPCONF_t {
  constexpr static uint8_t address = 0x38;  // Motor 0 CHOPCONF
  union {
    uint32_t sr;
    struct {
      uint8_t toff : 4,              // Bits 0-3 (4 bits = 0-15)
              hstrt_tfd210 : 3,      // Bits 4-6 (3 bits = 0-7)
              hend_offset : 4,       // Bits 7-10 (4 bits = 0-15)
              fd3 : 1,               // Bit 11
              disfdcc : 1,           // Bit 12
              chm : 1,               // Bit 13 (chopper mode)
              tbl : 2,               // Bits 14-15 (2 bits = 0-3)
              vhighfs : 1,           // Bit 16
              vhighchm : 1,          // Bit 17
              tpfd : 4,              // Bits 18-21 (4 bits)
              mres : 4,              // Bits 22-25 (4 bits = microsteps)
              intpol : 1,            // Bit 26 (interpolation)
              dedge : 1,             // Bit 27 (double edge)
              diss2g : 1,            // Bit 28
              diss2vs : 1;           // Bit 29
      // Bits 30-31 unused
    };
  };
};
```

### Your Code:

```cpp
driver.M0_CHOPCONF(0x00000003);
// Binary: 00000000 00000000 00000000 00000011
//                                      └─┬─┘
//                                        └─> toff = 3 (bits 0-3)
//         All other bits = 0:
//           mres = 0 (bits 22-25) → 256 microsteps
//           tbl = 0 (bits 14-15) → blank time = 16 clocks
```

### Breaking It Down:

```
Value: 0x00000003

Bit Layout:
31-30: (unused) = 00
   29: diss2vs = 0
   28: diss2g = 0
   27: dedge = 0
   26: intpol = 0
25-22: mres = 0000 ← 256 microsteps
21-18: tpfd = 0000
   17: vhighchm = 0
   16: vhighfs = 0
15-14: tbl = 00 ← Blank time
   13: chm = 0 ← SpreadCycle mode
   12: disfdcc = 0
   11: fd3 = 0
10-07: hend_offset = 0000
06-04: hstrt_tfd210 = 000
03-00: toff = 0011 ← OFF time = 3 clocks
```

---

## 6. The SET_REG Macro Pattern

The library uses macros to simplify read-modify-write operations:

```cpp
#define SET_REG(SETTING) \
  CHOPCONF_register.SETTING = B; \
  write(CHOPCONF_register.address, CHOPCONF_register.sr)
```

### Usage:

```cpp
// Setter function using macro
void TMC2130Stepper::toff(uint8_t B) {
  SET_REG(toff);  // Expands to:
                  // CHOPCONF_register.toff = B;
                  // write(0x38, CHOPCONF_register.sr);
}

// Getter function using temporary struct
uint8_t TMC2130Stepper::toff() {
  CHOPCONF_t r{0};         // Create temp struct, initialize to 0
  r.sr = CHOPCONF();       // Read register into .sr
  return r.toff;           // Extract toff bitfield (bits 0-3)
}
```

### Step-by-Step: Reading toff

```cpp
uint8_t value = driver.toff();

// Step 1: Create temporary struct
CHOPCONF_t r{0};  // r.sr = 0x00000000

// Step 2: Read register via SPI
r.sr = CHOPCONF();  // Returns 0x00000003 from hardware

// Step 3: Memory now looks like:
//   r.sr = 0x00000003
//   r.toff = 3 (bits 0-3 of 0x00000003)
//   r.mres = 0 (bits 22-25)
//   ...etc

// Step 4: Return just the toff field
return r.toff;  // Returns 3
```

---

## 7. IHOLD_IRUN: Multi-Bit Fields

Some bitfields use more than 1 bit for numeric values:

```cpp
struct IHOLD_IRUN_t {
  constexpr static uint8_t address = 0x12;
  union {
    uint32_t sr;
    struct {
      uint8_t ihold : 5,        // Bits 0-4 (5 bits = 0-31)
              : 3,              // Bits 5-7 (unused, anonymous)
              irun : 5,         // Bits 8-12 (5 bits = 0-31)
              : 3,              // Bits 13-15 (unused)
              iholddelay : 4,   // Bits 16-19 (4 bits = 0-15)
              : 4,              // Bits 20-23 (unused)
              irundelay : 4;    // Bits 24-27 (4 bits = 0-15)
      // Bits 28-31 unused
    };
  };
};
```

### Your Code:

```cpp
driver.M0_IHOLD_IRUN(0x000A1008);
// Binary: 00000000 00001010 00010000 00001000
//         └──┬──┘ └──┬──┘ └──┬──┘ └──┬──┘
//            │       │       │       └─> ihold = 0x08 = 8
//            │       │       └─────────> irun = 0x10 = 16
//            │       └─────────────────> iholddelay = 0x0A = 10
//            └─────────────────────────> irundelay = 0 (unused)
```

### Breakdown:

```
0x000A1008 in binary:
Bits 27-24: irundelay = 0000 = 0
Bits 23-20: (unused) = 0000
Bits 19-16: iholddelay = 1010 = 10 ← Delay before reducing current
Bits 15-13: (unused) = 000
Bits 12-8:  irun = 10000 = 16 ← Run current (0-31 scale)
Bits 7-5:   (unused) = 000
Bits 4-0:   ihold = 01000 = 8 ← Hold current (0-31 scale)
```

### Practical Use:

```cpp
IHOLD_IRUN_t current;
current.sr = 0;
current.ihold = 8;          // Holding torque (when stopped)
current.irun = 16;          // Run current (when moving)
current.iholddelay = 10;    // Delay before reducing to hold

driver.M0_IHOLD_IRUN(current.sr);  // Writes 0x000A1008
```

---

## 8. Anonymous Bitfields (Padding)

Notice the `: 3,` with no name? These are **padding bits**:

```cpp
struct {
  uint8_t ihold : 5,   // Bits 0-4
          : 3,         // Bits 5-7 ← Anonymous, can't access
          irun : 5;    // Bits 8-12
};
```

**Why?** The hardware has reserved/unused bits. Anonymous bitfields:
- Reserve space without allowing accidental access
- Keep bit positions aligned with hardware
- Document the register structure

---

## 9. DRV_STATUS: Read-Only Status Register

```cpp
struct DRV_STATUS_t {
  constexpr static uint8_t address = 0x3B;  // Motor 0 status
  union {
    uint32_t sr;
    struct {
      uint16_t sg_result : 10,    // Bits 0-9: StallGuard result
               : 2,               // Bits 10-11: unused
               s2vsa : 1,         // Bit 12: Short to supply A
               s2vsb : 1,         // Bit 13: Short to supply B
               stealth : 1,       // Bit 14: StealthChop active
               fsactive : 1;      // Bit 15: Full-step active
      uint8_t cs_actual : 5,      // Bits 16-20: Actual current
              : 3,                // Bits 21-23: unused
              ola : 1,            // Bit 24: Open load A
              olb : 1,            // Bit 25: Open load B
              s2ga : 1,           // Bit 26: Short to ground A
              s2gb : 1,           // Bit 27: Short to ground B
              ot : 1,             // Bit 28: Overtemperature
              otpw : 1,           // Bit 29: Overtemp pre-warning
              stallguard : 1,     // Bit 30: StallGuard event
              stst : 1;           // Bit 31: Standstill detected
    };
  };
};
```

### Reading Status:

```cpp
// Method 1: Check individual flags
DRV_STATUS_t status;
status.sr = driver.M0_DRV_STATUS();

if (status.ot) {
  Serial.println("⚠️  Overtemperature!");
}
if (status.s2ga || status.s2gb) {
  Serial.println("⚠️  Short to ground!");
}
Serial.print("Current: ");
Serial.println(status.cs_actual);  // 0-31

// Method 2: Check raw bits (your code)
uint32_t drv_status = driver.M1_DRV_STATUS();
if (drv_status & (1 << 28)) {  // Check bit 28 (ot)
  Serial.println("Overtemp!");
}
```

---

## 10. Memory Alignment & Packing

The `#pragma pack(push, 1)` directive ensures no padding:

```cpp
#pragma pack(push, 1)  // Pack tightly, no padding bytes

struct GCONF_t {
  union {
    uint32_t sr;    // 4 bytes
    struct {
      bool bit0 : 1;
      bool bit1 : 1;
      // ... 30 more bits
    };
  };
};

#pragma pack(pop)  // Restore default packing
```

**Without packing**: Compiler might add padding bytes  
**With packing**: Struct is exactly 4 bytes (32 bits)

This ensures the bitfield struct matches the hardware register layout perfectly.

---

## 11. Common Pitfalls & Best Practices

### ❌ Pitfall 1: Forgetting to Write Back

```cpp
// WRONG - reads register but doesn't save changes
GCONF_t config;
config.sr = driver.GCONF();
config.M1_drv_enn = 0;  // Changes local copy only!
// Forgot: driver.GCONF(config.sr);
```

### ✅ Solution:

```cpp
GCONF_t config;
config.sr = driver.GCONF();
config.M1_drv_enn = 0;
driver.GCONF(config.sr);  // Write back!
```

### ❌ Pitfall 2: Writing Partial Values

```cpp
// WRONG - doesn't set other required bits
driver.CHOPCONF(0x03);  // Sets toff=3, but leaves mres undefined!
```

### ✅ Solution:

```cpp
// Set all required fields explicitly
driver.CHOPCONF(0x00000003);  // toff=3, mres=0 (256 µsteps)
```

### ❌ Pitfall 3: Endianness Confusion

```cpp
// Bitfields are COMPILER-DEPENDENT!
// Order might reverse on different architectures
```

### ✅ Solution:

```cpp
// Use explicit integer values when in doubt
driver.GCONF(0x00000000);  // Always correct, portable
```

### ✅ Best Practice: Combine Methods

```cpp
// Start with known-good value from datasheet
uint32_t chopconf = 0x00000003;  // From your working code

// If you need to modify, use bitfields
CHOPCONF_t config;
config.sr = chopconf;
config.intpol = 1;  // Enable interpolation
driver.M0_CHOPCONF(config.sr);
```

---

## 12. Why This Matters for Your TMC5272 Bug

### The Problem:

```cpp
// TMC5272Stepper.cpp - Library code
void TMC5272Stepper::defaults() {
  GCONF_register = 0;  // Sets to 0x00000000
}

void TMC2130Stepper::begin() {
  GCONF(GCONF_register.sr);  // Writes cached 0
}
```

### The Fix:

```cpp
// Your code - DON'T call begin() or defaults()
driver.GCONF(0x00000000);  // Direct write bypasses cache
```

### Why It Works:

The bitfield union allows the library to:
1. Cache values in `GCONF_register.sr` (uint32_t)
2. Access individual bits like `GCONF_register.M1_drv_enn`
3. Write the entire register with `write(0x00, GCONF_register.sr)`

But if `GCONF_register.sr = 0` at initialization, calling `begin()` writes zeros and **overwrites your configuration**!

---

## 13. Debugging with Bitfields

### Print Individual Bits:

```cpp
GCONF_t config;
config.sr = driver.GCONF();

Serial.print("GCONF = 0x");
Serial.println(config.sr, HEX);
Serial.print("  M0_drv_enn (bit 14): ");
Serial.println(config.M0_drv_enn);
Serial.print("  M1_drv_enn (bit 29): ");
Serial.println(config.M1_drv_enn);
Serial.print("  M0_multistep_filt (bit 1): ");
Serial.println(config.M0_multistep_filt);
```

### Compare Expected vs Actual:

```cpp
uint32_t written = 0x00000000;
driver.GCONF(written);

GCONF_t readback;
readback.sr = driver.GCONF();

if (readback.sr != written) {
  Serial.println("❌ GCONF mismatch!");
  Serial.print("  Written:  0x");
  Serial.println(written, HEX);
  Serial.print("  Readback: 0x");
  Serial.println(readback.sr, HEX);
  
  if (readback.M1_drv_enn != 0) {
    Serial.println("  → M1_drv_enn stuck at 1 (disabled)");
  }
}
```

---

## Summary: Bitfields in Action

```
┌──────────────────────────────────────────────────────────┐
│  Hardware Register (TMC5272 chip)                        │
│  Address 0x00: [bit31|bit30|...|bit1|bit0]              │
│                      ↕ SPI Communication                 │
└──────────────────────┬───────────────────────────────────┘
                       │
┌──────────────────────┴───────────────────────────────────┐
│  C++ Union (TMCStepper library)                          │
│                                                           │
│  union {                                                  │
│    uint32_t sr;          ← Access as integer             │
│    struct {              ← Access as named bits          │
│      bool M0_en_pwm : 1;                                 │
│      bool M1_drv_enn : 1;                                │
│      ...                                                  │
│    };                                                     │
│  };                                                       │
└───────────────────────────────────────────────────────────┘
                       │
┌──────────────────────┴───────────────────────────────────┐
│  Your Code (main.cpp)                                    │
│                                                           │
│  driver.GCONF(0x00000000);  ← Write integer              │
│                                                           │
│  GCONF_t cfg;                                            │
│  cfg.sr = driver.GCONF();   ← Read integer               │
│  if (cfg.M1_drv_enn) {...}  ← Check named bit            │
└───────────────────────────────────────────────────────────┘
```

**Key Takeaway**: Bitfields let you think about hardware registers in terms of **named configuration options** instead of **raw bit manipulation**, making code more readable and less error-prone—but you must understand the underlying memory layout to debug issues!
