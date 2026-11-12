# Pathfinders APAC

Maze-solver firmware using both Grove Vision AI and Nicla Vision.

## Files

- `robot/pathfinders_apac.ino` — the supplied base robot firmware, unchanged
  except for the Nicla Vision UART integration.
- `nicla/main.py` — the supplied working Nicla Vision program, unchanged.

## Nicla integration

The working Nicla program sends:

```text
victim,stable_ms,x,y
```

The robot now reads that exact format at 115200 baud on GPIO 7 (RX). It waits
until the classification has been stable for 750 ms, then maps the Nicla result
into the base firmware's existing victim cases:

| Nicla value | Meaning | Existing robot target |
| --- | --- | --- |
| 2 | Harmed | 0 |
| 1 | Stable | 1 |
| 0 | Unharmed | 2 |

Nicla TX connects to robot GPIO 7, and both boards must share ground. The Grove
Vision AI code and all maze, motor, sensor, LED, and kit-drop behavior remain as
provided.

## Test status

The protocol mismatch and UART pin mismatch were corrected by inspection. A
complete end-to-end test still requires the actual robot, Nicla camera, Grove
camera, sensors, and trained Grove model.
