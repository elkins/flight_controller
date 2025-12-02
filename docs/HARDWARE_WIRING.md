# PyBoard Hexacopter Wiring Guide

Complete wiring diagram and instructions for connecting all components to the PyBoard v1.1 flight controller.

## Table of Contents
- [Component Overview](#component-overview)
- [Power Distribution](#power-distribution)
- [PyBoard Pinout](#pyboard-pinout)
- [Detailed Wiring](#detailed-wiring)
- [Wiring Diagrams](#wiring-diagrams)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)

---

## Component Overview

| Component | Quantity | Connection Type | Purpose |
|-----------|----------|-----------------|---------|
| PyBoard v1.1 | 1 | - | Flight controller (STM32F4) |
| MPU6050 (GY-521) | 1 | I2C | IMU (accel + gyro) |
| ESC (20-30A) | 6 | PWM Output | Motor speed controllers |
| RC Receiver | 1 | PWM Input | Radio control |
| Brushless Motor | 6 | 3-wire to ESC | Propulsion |
| LiPo Battery (3S) | 1 | XT60/Deans | Power source |
| Power Distribution Board | 1 (optional) | Solder pads | Distribute battery power |

---

## Power Distribution

### Power Flow Diagram

```
                    ┌─────────────────┐
                    │  3S LiPo Battery │
                    │   (11.1V nominal)│
                    └────────┬─────────┘
                             │
                    ┌────────┴─────────┐
                    │  Main Power Split │
                    └────┬─────────┬───┘
                         │         │
              ┌──────────┘         └──────────┐
              │                                │
    ┌─────────▼────────┐           ┌──────────▼──────────┐
    │  ESC x6 (11.1V)  │           │   5V BEC (optional)  │
    │  Power to Motors │           │   PyBoard Power      │
    └──────────────────┘           └─────────────────────┘
                                              │
                                   ┌──────────▼──────────┐
                                   │   PyBoard (USB or   │
                                   │   5V BEC during     │
                                   │   development)      │
                                   └─────────────────────┘
```

### Power Options

**Option 1: USB Power (Development/Testing)**
- Connect PyBoard via USB
- Powers PyBoard only
- ESCs powered separately from battery
- **⚠️ DO NOT connect battery to PyBoard during USB programming**

**Option 2: BEC Power (Flight)**
- Use ESC with 5V BEC output
- Connect BEC 5V to PyBoard VIN or 5V pin
- Powers PyBoard during flight
- Battery powers ESCs → one ESC BEC powers PyBoard

**Option 3: Separate BEC (Recommended for reliability)**
- Dedicated 5V regulator from battery
- Powers PyBoard independently
- More reliable than ESC BEC

---

## PyBoard Pinout

### PyBoard v1.1 Layout

```
                    ┌─────────────────────┐
                    │      MicroUSB       │
                    ├─────────────────────┤
    Y1  ○───────────┤ Y1          X1  ○───│──────────○ X1
    Y2  ○───────────┤ Y2          X2  ○───│──────────○ X2
    Y3  ○───────────┤ Y3          X3  ○───│──────────○ X3
    Y4  ○───────────┤ Y4          X4  ○───│──────────○ X4
    Y5  ○───────────┤ Y5          X5  ○───│──────────○ X5
    Y6  ○───────────┤ Y6          X6  ○───│──────────○ X6
    Y7  ○───────────┤ Y7          X7  ○───│──────────○ X7
    Y8  ○───────────┤ Y8          X8  ○───│──────────○ X8
    Y9  ○───────────┤ Y9          X9  ○───│──────────○ X9  (I2C SCL)
    Y10 ○───────────┤ Y10         X10 ○───│──────────○ X10 (I2C SDA)
    Y11 ○───────────┤ Y11         X11 ○───│──────────○ X11
    Y12 ○───────────┤ Y12         X12 ○───│──────────○ X12
    3V3 ○───────────┤ 3V3         VIN ○───│──────────○ VIN
    GND ○───────────┤ GND         GND ○───│──────────○ GND
                    └─────────────────────┘
```

### Pin Assignments (Per Your Code)

#### Motor Outputs (PWM, 50Hz)
| Motor | Pin | Timer | Position on Frame |
|-------|-----|-------|-------------------|
| Motor 0 | X1 | TIM5 CH1 | Front |
| Motor 1 | X2 | TIM5 CH2 | Front-Right |
| Motor 2 | X3 | TIM5 CH3 | Rear-Right |
| Motor 3 | X6 | TIM2 CH1 | Rear |
| Motor 4 | Y9 | TIM2 CH3 | Rear-Left |
| Motor 5 | Y10 | TIM2 CH4 | Front-Left |

#### RC Receiver Inputs (PWM Input Capture)
| Channel | Pin | Timer | Function |
|---------|-----|-------|----------|
| RC Ch1 (Throttle) | Y8 | TIM12 CH2 | Throttle |
| RC Ch2 (Roll) | Y7 | TIM12 CH1 | Roll |
| RC Ch3 (Pitch) | Y4 | TIM4 CH4 | Pitch |
| RC Ch4 (Yaw) | Y3 | TIM4 CH3 | Yaw |

#### IMU (I2C Bus 1)
| Signal | Pin | Function |
|--------|-----|----------|
| SCL | X9 | I2C Clock |
| SDA | X10 | I2C Data |
| VCC | 3V3 | Power (3.3V) |
| GND | GND | Ground |

---

## Detailed Wiring

### 1. IMU (MPU6050) Wiring

**GY-521 Breakout to PyBoard:**

```
┌──────────────┐           ┌──────────────┐
│   MPU6050    │           │   PyBoard    │
│   (GY-521)   │           │              │
├──────────────┤           ├──────────────┤
│ VCC      ○───┼───────────┼───○ 3V3      │
│ GND      ○───┼───────────┼───○ GND      │
│ SCL      ○───┼───────────┼───○ X9       │
│ SDA      ○───┼───────────┼───○ X10      │
│ XDA      ○   │ (not used)│              │
│ XCL      ○   │ (not used)│              │
│ AD0      ○   │ (leave    │              │
│ INT      ○   │  floating)│              │
└──────────────┘           └──────────────┘
```

**Important:**
- Use **3.3V**, not 5V (PyBoard is 3.3V logic)
- Keep I2C wires short (<15cm for reliability)
- No pull-up resistors needed (on-board on GY-521)

### 2. ESC Wiring

**Each ESC has:**
- 3 wires to motor (any order, swap 2 if motor spins wrong way)
- 2 thick wires to battery (red=+, black=-)
- 3-wire servo connector (signal, +5V, GND)

**ESC Signal Wire to PyBoard:**

```
Motor 0 ESC:  Signal (white/yellow) → X1,  GND (brown/black) → GND
Motor 1 ESC:  Signal → X2,  GND → GND
Motor 2 ESC:  Signal → X3,  GND → GND
Motor 3 ESC:  Signal → X6,  GND → GND
Motor 4 ESC:  Signal → Y9,  GND → GND (⚠️ conflicts with motor output)
Motor 5 ESC:  Signal → Y10, GND → GND (⚠️ conflicts with motor output)
```

**⚠️ Pin Conflict Resolution:**

Your code assigns:
- **Y9** to both Motor 4 output AND I2C SCL
- **Y10** to both Motor 5 output AND I2C SDA

**CORRECTED Pin Assignment (use these instead):**

| Motor | Pin | Timer | Notes |
|-------|-----|-------|-------|
| Motor 0 | X1 | TIM5 CH1 | ✓ OK |
| Motor 1 | X2 | TIM5 CH2 | ✓ OK |
| Motor 2 | X3 | TIM5 CH3 | ✓ OK |
| Motor 3 | X6 | TIM2 CH1 | ✓ OK |
| Motor 4 | **Y1** | **TIM8 CH1** | **Changed from Y9** |
| Motor 5 | **Y2** | **TIM8 CH2** | **Changed from Y10** |

This frees up X9/X10 for I2C exclusively.

### 3. RC Receiver Wiring

**FlySky FS-i6 Receiver (or similar PWM output receiver):**

```
┌─────────────────┐           ┌──────────────┐
│  RC Receiver    │           │   PyBoard    │
├─────────────────┤           ├──────────────┤
│ Ch1 (Throttle)  │           │              │
│   Signal    ○───┼───────────┼───○ Y8       │
│   +5V       ○───┼───────────┼───○ VIN/5V   │ (optional)
│   GND       ○───┼───────────┼───○ GND      │
│                 │           │              │
│ Ch2 (Roll)      │           │              │
│   Signal    ○───┼───────────┼───○ Y7       │
│   GND       ○───┼───────────┼───○ GND      │
│                 │           │              │
│ Ch3 (Pitch)     │           │              │
│   Signal    ○───┼───────────┼───○ Y4       │
│   GND       ○───┼───────────┼───○ GND      │
│                 │           │              │
│ Ch4 (Yaw)       │           │              │
│   Signal    ○───┼───────────┼───○ Y3       │
│   GND       ○───┼───────────┼───○ GND      │
└─────────────────┘           └──────────────┘
```

**Notes:**
- RC receiver usually powered by separate BEC or battery
- **Connect all GNDs together** (common ground)
- Some receivers have +5V output - do NOT connect if using USB power

### 4. Motor Layout

**View from top (front = up):**

```
              Front
               ↑
               
        5 ●         ● 0
     (CCW)         (CW)
        
    4 ●               ● 1
   (CW)               (CCW)
   
        3 ●         ● 2
      (CCW)         (CW)
      
              Rear
```

**Motor Rotation:**
- CW (Clockwise): Motors 0, 2, 4
- CCW (Counter-Clockwise): Motors 1, 3, 5

**If motor spins wrong direction:** Swap any 2 of the 3 motor wires

---

## Wiring Diagrams

### Complete System Diagram

```
                            ┌─────────────────────┐
                            │   3S LiPo Battery   │
                            │      11.1V          │
                            └──────────┬──────────┘
                                       │
                    ┌──────────────────┴──────────────────┐
                    │    Power Distribution Board (PDB)   │
                    │         (or solder joints)          │
                    └─┬──┬──┬──┬──┬──┬──────────────┬────┘
                      │  │  │  │  │  │              │
         ┌────────────┘  │  │  │  │  └────────┐     │
         │  ┌────────────┘  │  │  └────────┐  │     │
         │  │  ┌────────────┘  └────────┐  │  │     │
         │  │  │  ┌──────────────────┐  │  │  │     │
         │  │  │  │                  │  │  │  │     │ (5V BEC)
     ┌───▼──▼──▼──▼──▼──▼───┐        │  │  │  │     │
     │  ESC 0,1,2,3,4,5      │        │  │  │  │     │
     │  (20-30A each)        │        │  │  │  │     │
     └─┬──┬──┬──┬──┬──┬──────┘        │  │  │  │     │
       │  │  │  │  │  │               │  │  │  │     │
   ┌───┘  │  │  │  │  └───┐           │  │  │  │     │
   │  ┌───┘  │  │  └───┐  │           │  │  │  │     │
   │  │  ┌───┘  └───┐  │  │           │  │  │  │     │
   │  │  │          │  │  │           │  │  │  │     │
   ▼  ▼  ▼          ▼  ▼  ▼           │  │  │  │     │
  M0 M1 M2         M3 M4 M5           │  │  │  │     │
 (CW)(CCW)(CW)   (CCW)(CW)(CCW)       │  │  │  │     │
                                      │  │  │  │     │
ESC Signal Wires:                     │  │  │  │     │
  ESC0 signal ─────────────────────┐  │  │  │  │     │
  ESC1 signal ─────────────────┐   │  │  │  │  │     │
  ESC2 signal ─────────────┐   │   │  │  │  │  │     │
  ESC3 signal ─────────┐   │   │   │  │  │  │  │     │
  ESC4 signal ─────┐   │   │   │   │  │  │  │  │     │
  ESC5 signal ──┐  │   │   │   │   │  │  │  │  │     │
                │  │   │   │   │   │  │  │  │  │     │
            ┌───▼──▼───▼───▼───▼───▼──▼──▼──▼──▼─────▼────┐
            │           PyBoard v1.1                       │
            │                                              │
            │  X1  X2  X3  X6  Y1  Y2   ← Motor outputs   │
            │  Y3  Y4  Y7  Y8           ← RC inputs       │
            │  X9  X10                  ← I2C             │
            │  VIN GND 3V3              ← Power           │
            └────────────────┬──────────────────┬─────────┘
                             │                  │
                    ┌────────▼─────┐   ┌────────▼────────┐
                    │  MPU6050     │   │  RC Receiver    │
                    │  (GY-521)    │   │  (4+ channels)  │
                    └──────────────┘   └─────────────────┘
```

### Power Distribution Detail

```
Battery Positive (+11.1V)
    │
    ├─────→ ESC 0 red wire
    ├─────→ ESC 1 red wire
    ├─────→ ESC 2 red wire
    ├─────→ ESC 3 red wire
    ├─────→ ESC 4 red wire
    ├─────→ ESC 5 red wire
    └─────→ 5V BEC input (optional)

Battery Negative (GND)
    │
    ├─────→ ESC 0 black wire
    ├─────→ ESC 1 black wire
    ├─────→ ESC 2 black wire
    ├─────→ ESC 3 black wire
    ├─────→ ESC 4 black wire
    ├─────→ ESC 5 black wire
    ├─────→ PyBoard GND
    ├─────→ MPU6050 GND
    └─────→ RC Receiver GND
    
    ⚠️ ALL GROUNDS MUST BE CONNECTED TOGETHER
```

---

## Testing

### Pre-Flight Testing Sequence

#### 1. USB Power Test (No Battery, No Props)

```bash
# Connect PyBoard via USB only
# Flash your code to PyBoard
# Verify:
```

**Test IMU:**
```python
from hal_pyboard import PyBoardHAL
hal = PyBoardHAL()
imu = hal.read_imu()
print(imu)  # Should show accel/gyro data
```

**Test RC Input:**
```python
rc = hal.read_rc()
print(rc)  # Move transmitter sticks, values should change
```

**Test Motor Outputs (LOW SIGNAL ONLY):**
```python
# Set motors to minimum (1000µs)
hal.set_motors([1000, 1000, 1000, 1000, 1000, 1000])
# Should NOT spin without battery connected to ESCs
```

#### 2. ESC Calibration (No Props)

1. Disconnect USB power from PyBoard
2. Connect battery to ESCs (NOT to PyBoard yet)
3. Power PyBoard via separate 5V BEC
4. Run ESC calibration:

```python
from hal_pyboard import PyBoardHAL
import time

hal = PyBoardHAL()

# Set all throttles to MAXIMUM
hal.set_motors([2000, 2000, 2000, 2000, 2000, 2000])
print("Set throttle to max, plug in battery now...")
time.sleep(5)  # Wait for ESC to recognize max

# Set all throttles to MINIMUM
hal.set_motors([1000, 1000, 1000, 1000, 1000, 1000])
print("Calibration complete - ESCs should beep")
time.sleep(2)
```

ESCs should beep to confirm calibration.

#### 3. Motor Direction Test (No Props)

```python
# Test each motor individually at LOW speed
for i in range(6):
    print(f"Testing motor {i}...")
    motors = [1000, 1000, 1000, 1000, 1000, 1000]
    motors[i] = 1100  # Low speed
    hal.set_motors(motors)
    time.sleep(2)
    hal.set_motors([1000]*6)
    time.sleep(1)
```

**Verify rotation direction matches diagram above**
- If wrong: Swap any 2 motor wires

#### 4. PID Test (No Props)

```python
# Run main.py
# Tilt PyBoard - motors should respond
# Check serial output for PID values
```

#### 5. Prop Installation & First Hover

⚠️ **SAFETY CRITICAL:**
- Install propellers correctly (CW vs CCW)
- Check all screws tight
- Clear area - no people within 10m
- Arm transmitter kill switch
- Start at LOW throttle
- Be ready to disarm immediately

---

## Troubleshooting

### Common Issues

#### Motors Don't Spin

**Check:**
- [ ] Battery fully charged (11.1V nominal)
- [ ] All ESC power wires connected
- [ ] ESCs calibrated
- [ ] PWM signal wires connected to correct pins
- [ ] All GNDs connected together
- [ ] Throttle above minimum (>1100µs)

**Test:**
```python
# Force motor at medium throttle
hal.set_motors([1500, 1500, 1500, 1500, 1500, 1500])
```

#### IMU No Data

**Check:**
- [ ] MPU6050 powered (3.3V, NOT 5V)
- [ ] I2C wires connected (X9=SCL, X10=SDA)
- [ ] Short wires (<15cm)
- [ ] No motor wires on Y9/Y10 (I2C conflict)

**Test:**
```python
from machine import I2C
i2c = I2C(1)
devices = i2c.scan()
print([hex(d) for d in devices])  # Should show ['0x68']
```

#### RC Input Not Working

**Check:**
- [ ] Transmitter on and bound to receiver
- [ ] Receiver powered (red LED on)
- [ ] Correct pins (Y8, Y7, Y4, Y3)
- [ ] PWM mode (not SBUS/PPM)
- [ ] All GNDs connected

**Test:**
```python
while True:
    rc = hal.read_rc()
    print(f"Throttle: {rc['throttle']}, Roll: {rc['roll']}")
    time.sleep(0.1)
# Move sticks - values should change
```

#### One Motor Wrong Direction

**Fix:** Swap any 2 of the 3 motor wires to that motor

#### Drone Flips on Takeoff

**Likely causes:**
1. Motor rotation wrong direction
2. Props installed backwards
3. PID gains too aggressive
4. Motor/prop layout incorrect

#### Oscillations During Hover

**Tune PID:**
- Reduce P gain if oscillating fast
- Reduce I gain if oscillating slow
- Add D gain for damping
- See README.md PID tuning section

---

## Safety Checklist

Before EVERY flight:

- [ ] All wires secure (zip ties)
- [ ] Battery voltage >10.5V
- [ ] Props installed correctly (CW/CCW)
- [ ] Props balanced
- [ ] All screws tight
- [ ] Open area, no people
- [ ] RC transmitter on, good signal
- [ ] Emergency procedures reviewed
- [ ] Kill switch identified
- [ ] Test flight plan prepared

---

## Parts Shopping List

Ready to buy? Here's a complete list with quantities:

| Item | Qty | Notes |
|------|-----|-------|
| PyBoard v1.1 | 1 | Flight controller |
| MPU6050 (GY-521) | 1 | IMU sensor |
| F550 hexacopter frame | 1 | Frame kit |
| 920KV 2212 motors | 6 | Brushless outrunners |
| 20-30A ESCs | 6 | PWM input |
| 10x4.5 props CW | 6 | Spare sets recommended |
| 10x4.5 props CCW | 6 | Spare sets recommended |
| 3S 2200mAh LiPo | 1 | 25C+ discharge rating |
| XT60 connectors | 2 | Battery connector |
| Power distribution board | 1 | Optional but recommended |
| FlySky FS-i6 | 1 | Transmitter + receiver |
| LiPo charger | 1 | Balance charger |
| Jumper wires | 10 | Female-female for IMU |
| Heat shrink tubing | 1m | Various sizes |
| Zip ties | 20 | Cable management |
| Velcro straps | 2 | Battery mounting |

**Estimated Total:** $150-200

---

## Next Steps

1. **Order parts** using the shopping list above
2. **Review this wiring guide** while waiting for parts
3. **Test code in simulator** to understand behavior
4. **Follow testing sequence** when parts arrive
5. **Start with bench testing** (no props)
6. **Calibrate ESCs and check motors**
7. **First flight in open area**
8. **Tune PIDs based on flight performance**

---

**Last Updated:** December 2, 2025

**Questions?** Check the main README.md or TELEMETRY_TESTING.md for more information.
