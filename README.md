# Pathfinders APAC

Robot firmware using Grove Vision AI for Phi/Psi/Omega and Nicla Vision for
cognitive victim detection.

## Files

- `robot/pathfinders_apac/pathfinders_apac.ino` — robot firmware.
- `nicla/main.py` — Nicla Vision program for cognitive targets.

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

The Nicla program uses `UART("LP1", 115200)` on its J2-3 (D1/UART_TX) and
J2-4 (D2/UART_RX) pins. It sends `victim,stable_ms,x,y`. The robot reads this
on Pico 2 GPIO 9 (RX) through `Serial2` (UART1), waits for a classification
to remain stable for 750 ms, then maps it to the same victim actions:

| Nicla value | Meaning | Robot target |
| ---: | --- | ---: |
| 2 | Harmed | 0 |
| 1 | Stable | 1 |
| 0 | Unharmed | 2 |

Connect Nicla J2-3 TX to Pico 2 GPIO 9 (physical pin 12, RX). Connect Nicla
J2-6 GND to Pico GND (for example physical pin 13). Nicla J2-4 RX is optional:
the current program only sends data. If two-way UART is needed, connect Pico
GPIO 8 (physical pin 11, TX) to Nicla J2-4 RX. Both boards must share ground.
The Pico's old GPIO 7 connection is not the receiver used by this sketch.

## Test status

On 13 November 2026, the Pico 2 sketch was compiled, uploaded, and verified
against the board's flash. A zero-stability UART test message sent from the
Nicla's LP1 port reached the Pico, which printed its parsed target and zero
confidence. This confirms the one-way UART link without triggering a victim
action. Real victim recognition, motor and kit-drop behavior, and the Grove
class IDs and Phi/Psi/Omega meanings still need validation on the robot.
