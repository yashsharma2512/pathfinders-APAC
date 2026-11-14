# Pathfinders APAC

Robot firmware using Grove Vision AI for Phi/Psi/Omega and Nicla Vision for
cognitive victim detection.

## Files

- `robot/pathfinders_apac/pathfinders_apac.ino` — robot firmware.
- `nicla/main.py` — Nicla Vision program for cognitive targets.

## Grove classes

The installed model reports its labels in this order: `harmed`, `unharmed`,
`stable`, `nothing`. The robot sketch assumes zero-based IDs in that order:

| Model label | Printed symbol | Assumed class ID | Robot response |
| --- | --- | ---: | --- |
| Harmed | Phi (Φ) | 0 | Two kit drops |
| Unharmed | Omega (Ω) | 1 | No kit drop |
| Stable | Psi (Ψ) | 2 | One kit drop |
| Nothing | Wall/background | 3 | Ignored |

The old IDs 4, 5, and 6 were provisional values for an earlier setup; the
current sketch no longer uses them. The Pico has logged class 3 at score 99,
but IDs 0–2 still need live target checks to confirm their meanings. The
sketch uses the highest-scoring class and ignores unknown IDs.

## Silver tile stop

The Pico reads the AS7341 on GPIO 16/17 and learns its clear-channel level
from 12 ordinary-floor readings while the motors are stopped at startup. Start
with the sensor at its normal height over a non-black, non-blue floor tile. If
that baseline is unavailable, it keeps collecting ordinary-floor readings in
the main loop.

Three consecutive neutral-color readings with a clear-channel value at least
70% above the learned floor and 200 counts higher latch a silver stop. The
Pico stops both motors and blinks both NeoPixels white every 250 ms until the
board restarts. `SILVER_CLEAR_RATIO_PERCENT` and `SILVER_MIN_CLEAR_RISE` are
starting thresholds; they need tuning against the actual silver and ordinary
floor tiles. The silver behavior has been compiled but not tested on hardware.

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
confidence. A later 60-second test matched all 43 Nicla USB messages to 43
complete messages on the Pico UART listener.

The Grove Vision AI V2 acknowledges at I2C address `0x62` on Pico GPIO 18/19.
With the previous model, `AI.invoke(1, false, false)` returned code 3 over I2C;
the result-only `AI.invoke(1, true, false)` request returned classes. The robot
sketch logged class ID 2 with scores of 50–76. Its class-to-victim mapping is
still provisional.

The model installed later on 13 November reports labels `harmed`, `unharmed`,
`stable`, and `nothing`. During a subsequent check, both USB camera sampling and
USB inference returned event code 3 (timeout), even with the Pico not polling
the Grove. Reinitializing the sensor and fully power-cycling the Grove did not
clear the error. The same timeout persisted on Pico I2C after the power cycle.
The Pico sketch now assumes zero-based class IDs: 0 harmed, 1 unharmed,
2 stable, and 3 nothing (ignored). It uses the highest-scoring class.

In a no-target integrated test, the Nicla sent `0` (unharmed) packets with
long stability times, and the Pico accepted two as confidence 100. The Nicla
vision filter needs a separate false-positive fix before unattended operation.

On 14 November, the updated Grove class mapping compiled and was uploaded to
the Pico 2. A safe Nicla UART check sent zero-stability packets through LP1;
the Pico logged six parsed packets at confidence 0. In a later 30-second live
check, the Pico logged repeated Grove class ID 3 results at score 99 and parsed
Nicla UART packets. Grove inference is reaching the Pico again; the cause of
the earlier timeout is unknown. The harmed, unharmed, and stable class IDs and
motor/kit-drop responses remain unverified.
