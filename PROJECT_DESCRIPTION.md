# PPA-100: Pick and Place Arm

Karishma Premnath
MS Electrical and Computer Engineering, Northeastern University
October 2026

---

## One line

Firmware for a two joint robot arm that finds objects with a laser, picks them
up, and estimates their weight and its own contact force from motor current
alone, with no scale and no force sensor.

## One paragraph

PPA-100 is embedded firmware in C++ for a two joint planar manipulator that
performs a complete pick and place cycle: sweep a laser ranger across a table,
segment the height profile into objects, plan a collision free path through
configuration space, descend until contact, grip, carry, and place. An eight
state Extended Kalman Filter fuses an IMU, two magnetic encoders and two motor
drives across four buses, and estimates two quantities the hardware cannot
measure directly, payload mass and external contact torque, from motor current
and the arm's own dynamic model. A fixed priority scheduler runs five tasks at
1 kHz and measures its own deadline misses. A safety supervisor detects a lying
sensor, decides which of two channels is lying, and latches a stop. The same
source cross compiles for a Cortex-M4 and for a host simulation, where the four
bus interfaces are the only thing that changes.

---

## What it does, concretely

A single autonomous cycle, no operator input:

1. **Survey.** Sweep the laser ranger across the table and build a height
   profile. Segment it at height discontinuities into discrete objects.
   Result: three boxes located to 1 mm in centre position and 1.9 mm in height.

2. **Plan.** Build a 96 x 96 configuration space grid of the arm's reachable
   poses, mark the ones that collide with the table or an object, and run a
   breadth first distance field from the goal. 3,892 of 9,216 poses are blocked.
   The map builds in 0.3 ms.

3. **Approach and contact.** Descend in two stages, fast then creeping, and stop
   on an estimated contact force of 4 N. The estimator supplies that force; there
   is no force sensor in the system.

4. **Grip and weigh.** Close the jaws, then estimate the payload mass from the
   change in gravity torque seen at the joints. Settled error is 11 g mean.

5. **Carry and place.** Follow a jerk limited path to the destination, descend,
   release, and confirm the object actually left the gripper.

6. **Supervise throughout.** Cross check every sensor against the filter's
   prediction, catch a failure, isolate which channel caused it, and either
   degrade or stop.

---

## Architecture

```
app/              interactive console, recorder, 8 check suites
include/
  hal/            the four bus interfaces the firmware may see
  drivers/        IMU, encoder, motor drive, laser ranger
  est/            arm model, linear algebra, Extended Kalman Filter
  ctrl/           cascaded position and velocity control
  task/           sequence state machine, survey, planner, assembly
  diag/           sensor health arbitration, safety supervisor
  rtos/           fixed priority scheduler
src/              implementations
sim/              plant model, bus simulators (host only)
target/stm32f407/ linker script, startup, clock, bus drivers, main
renode/           Cortex-M4 emulation scripts and an LSM6DSL device model
tools/            plotting, 3D viewer generator, phone notification gateway
```

The firmware above the HAL only ever talks to `I2cBus`, `SpiBus`, `CanBus` and
`UartBus`. Simulation supplies one implementation of those four; the STM32F407
target supplies another. Nothing above that line knows which it is running on.

### The four buses, and why each part is on the one it is on

| Bus | Device | Why this bus |
|---|---|---|
| I2C | LSM6DSL IMU at the tool | Low rate, short distance, cheap pins |
| SPI | Two AS5047P magnetic encoders | Needs the throughput and a deterministic frame |
| CAN | Two motor drives | Multi drop, noise tolerant, long runs in a cell |
| UART | Laser ranger | Streams unsolicited frames with a sync byte and checksum |

### The eight filter states

```
q1, q2             joint angles
qd1, qd2           joint rates
tau_ext1, tau_ext2 external contact torque
payload            carried mass
friction_scale     joint friction multiplier
```

The last three are the point. None of them has a sensor. They are recovered
from the mismatch between commanded motor current and the torque the arm's
dynamic model says that motion should have required.

### A real design constraint, worked out rather than assumed

The motor winding time constant is L/R = 1 ms, so the current loop wants roughly
10 kHz. A CAN frame carrying two data bytes is about 76 bits after stuffing, so
76 us on the wire. One cycle of a two axis loop needs four frames, which is
302 us of bus traffic. At 10 kHz the entire cycle budget is 100 us.

So a 10 kHz current loop across CAN needs three times the bandwidth that exists.
The current loop therefore lives inside the drive and takes a torque setpoint
over the bus. Velocity and position stay on the main controller at 1 kHz, where
302 us of traffic is 30% bus load. This is why real drives are built this way,
and the number is what proves it rather than the convention.

---

## Measured results

| Measurement | Result |
|---|---|
| Payload mass, settled | 11 g mean, 51 g worst |
| Payload mass, controlled sweep over three boxes | 1 to 4 g |
| Contact force, steady 87 N case | 0.1 N |
| Joint friction scale | 2.96 against a true 3.00 |
| Tool position, held | 0.09 mm |
| Tool path following, 160 mm jerk limited path | 0.37 mm worst, 0.24 mm mean |
| Drive current limit | never exceeded 6.000 A |
| Anti windup reaction | 9 ms, against 766 ms with it disabled |
| Object location by laser sweep | 1 mm centres, 1.9 mm heights |
| Configuration space map | 3,892 of 9,216 poses blocked, 0.3 ms to build |
| False fault alarms | 0 over 7,493 control cycles of hard step moves |

### Fault detection

| Fault injected | Detected in | Response |
|---|---|---|
| Encoder error flag | 0 ms | degrade |
| Frozen encoder | 23 ms | degrade, channel dropped and isolated |
| Laser ranger silent | 43 ms | degrade |
| Collision in free motion | 160 ms | safe stop |
| Motor drive stops answering | 202 ms | safe stop |

All timings at the 1 kHz control rate.

### On the target

Cross compiled for STM32F407, Cortex-M4F, hard float, `-Os`.

| | |
|---|---|
| Flash | 29,120 bytes of 1,048,576 (2.8%) |
| Static RAM | 11,916 bytes of 131,072 (9.1%) |
| Deepest single stack frame | 2,424 bytes, in the filter's update |
| Dynamic allocation | none, enforced at link time |

The linker script, vector table, startup code, clock tree and register level bus
drivers are all hand written. `_sbrk` is stubbed to always fail, so any
accidental allocation is a link error rather than a silent heap. A 1 KB stack
guard band is filled with a known pattern at boot and can be checked for depth.

Verification is 127 automated checks across eight suites, all passing, run with
a single `make test`.

---

## Things that were wrong, and how they were found

These are the ones worth telling, because each one was silent.

**The filter estimated 19.9 kg for a 0.25 kg box.** External torque, payload and
friction all compete to explain the same residual, so the filter is free to blame
any of them. Fixed by running the estimator in two modes, free motion and
contact, with different process noise, and letting external torque decay toward
zero when nothing is being touched.

**An angle wrap function hung the entire firmware.** `while (a > kPi) a -= kTwoPi`
never terminates for infinity, because infinity minus two pi is still infinity.
One NaN anywhere upstream and the 1 kHz control loop stops forever, which is the
worst available failure mode. Rewritten with `fmod` and a finiteness guard.

**The fault arbitrator convicted the honest sensor.** Comparing two position
channels against each other fails once the liar has already contaminated the
estimate. Fixed by comparing each channel's *displacement* against the motion the
filter's own velocity says should have occurred.

**One box in three was silently never weighed.** The payload term returned early
for mass at or below zero, which made the numerical Jacobian for that state
exactly zero, so the filter could not learn it in that direction. Fixed by
keeping the function smooth through zero.

**The survey reported 89% coverage of a table it had never seen.** The ranger
driver returned its last value as valid after the device had gone silent. Fixed
by making a timed out sample report invalid.

**A 5 mm bias in one direction was quantisation, not noise.** The ranger reports
whole centimetres, so every distance read short and every surface height read
high. No amount of averaging removes it. Fixed in the driver by adding half a
count.

**I misdiagnosed one myself.** I claimed a traverse swung into the table at
2628 N, and patched three moves on that basis. Measured later, that straight line
sags the tool 14 mm and stays 96 mm clear. The 2628 N came from an unrelated bug
in how a held box was positioned. The patches were reverted and the correction is
recorded in the planner header and the README.

---

## Honest limitations

- **The register level bus code has never run on real silicon.** It compiles and
  links for the target, and the boot path has been exercised under Renode
  emulation, but no STM32F407 has executed it. Nothing here claims otherwise.
- **Renode is incomplete.** The firmware boots under emulation, the PLL locks at
  168 MHz, and SPI and UART configure correctly. Telemetry does not emerge,
  because Renode's I2C controller model does not support a repeated START, which
  is how any chip's register read works. The firmware requests one correctly and
  correctly times out instead of hanging. This is a gap in the emulator, not in
  the firmware. An LSM6DSL device model was written for Renode and does work; the
  controller side is still open.
- **The planner's grid is coarse.** 3.75 degrees per cell, roughly 30 mm at the
  tool.
- **Perception is one dimensional.** A laser sweep and a height profile. No
  camera, no image processing.
- **It is all simulation.** The plant is a model, not a robot.

## Not done

- A Modbus TCP interface so a SCADA system could supervise the cell
- A moving horizon estimator to compare against the EKF

---

## Keywords this project actually demonstrates

C++, Git, RTOS, ARM, I2C, SPI, CAN, UART, sensor fusion, Extended Kalman Filter,
state estimation, sensor drivers, bare metal firmware, cross compilation, linker
scripts, fixed priority scheduling, cascaded control, anti windup, fault
detection and isolation, configuration space planning, breadth first search.

## How to run it

```
make test        # 127 checks, eight suites
make plots       # five figures into docs/
make viewer      # a self contained 3D view of a recorded run
make -f Makefile.arm report   # cross compile and print the target budget
```
