# Pathfinders APAC

Robot firmware using Grove Vision AI for Phi/Psi/Omega and Nicla Vision for
cognitive victim detection.

## Files

- `robot/pathfinders_apac/pathfinders_apac.ino` — robot firmware.
- `nicla/main.py` — existing Nicla program for cognitive targets.

## Grove classes

The Grove model's class IDs and provisional victim meanings are grouped at the
top of the robot sketch. Change these constants when the trained model's class
order and labels are known:

| Grove class | Current class ID | Provisional victim target | Robot response |
| --- | ---: | --- | --- |
| Phi | 4 | Harmed | Two kit drops |
| Psi | 5 | Stable | One kit drop |
| Omega | 6 | Unharmed | No kit drop |

`GROVE_CLASS_PHI`, `GROVE_CLASS_PSI`, and `GROVE_CLASS_OMEGA` are the model's
numeric class IDs. `GROVE_PHI_TARGET`, `GROVE_PSI_TARGET`, and
`GROVE_OMEGA_TARGET` assign their victim meanings. The class IDs 4, 5, and 6
retain the old Grove slots until the model's actual IDs are confirmed. Unknown
Grove classes are ignored.

## Nicla cognitive targets

The Nicla program sends `victim,stable_ms,x,y` at 115200 baud. The robot reads
it on Pico GPIO 9 (RX) through `Serial2` (UART1), waits for the classification to remain stable for 750 ms,
then maps it to the same victim actions:

| Nicla value | Meaning | Robot target |
| ---: | --- | ---: |
| 2 | Harmed | 0 |
| 1 | Stable | 1 |
| 0 | Unharmed | 2 |

Nicla TX connects to Pico GPIO 9 (physical pin 12), and the boards share ground.
Pico GPIO 8 (physical pin 11) is the optional Pico TX line. Move any existing
Nicla TX wire from Pico GPIO 7 (physical pin 10) to GPIO 9. GPIO 7 cannot be
used as this hardware UART's RX pin.

## Test status

The Grove class routing and Nicla protocol were checked in code. The final
class IDs, Phi/Psi/Omega meanings, and behavior need validation with the
trained Grove model and physical robot.
