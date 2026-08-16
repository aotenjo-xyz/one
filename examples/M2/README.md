# M2
M2 motor winds the wire around the stator.

Changes
- Increase `voltage_limit` and `velocity_limit` value to rotate the motor faster.
- Change the line1 in `src/CANProfile.h` to set the motor ID to `0x02` for M2 like the following:
```cpp
#define MOTOR_ID 0x02 // M2
```

e.g.
```
voltage_limit: 24.0
velocity_limit: 30.0
```