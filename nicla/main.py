"""
Victim color classification for OpenMV / Nicla Vision.

Finds square-ish colored patches (blobs) in the camera frame, samples five
points across the patch's diameter, classifies each sample point's color,
and sums scores across the five points to decide a "victim" status:
    +2  -> "Harmed"
    +1  -> "Stable"
     0  -> "Unharmed"
  other -> "Fake"

A ToF sensor (VL53L1X) provides distance, which is used together with the
camera FOV to estimate the real-world width of a fixed-size (4 cm) patch
in pixels, for laying out the five sample points.
"""

import sensor
import time
import math
from machine import I2C
import vl53l1x
from pyb import LED, UART

red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

red_led.off()
green_led.off()
blue_led.off()

red_led.on()
time.sleep(0.2)
green_led.on()
time.sleep(0.2)
blue_led.on()
time.sleep(0.2)

red_led.off()
green_led.off()
blue_led.off()

uart = UART("LP1", 115200)  # Nicla J2-3 TX, J2-4 RX

# --------------------------------------------------------------------------
# Distance sensor (ToF)
# --------------------------------------------------------------------------
bus = I2C(2)  # internal bus (PMIC / fuel gauge / ToF)
tof = vl53l1x.VL53L1X(bus)

# --------------------------------------------------------------------------
# Camera / geometry configuration
# --------------------------------------------------------------------------
FOV = 35.0                 # camera field of view, degrees
PATCH_WIDTH_MM = 40        # real-world width of the target color patch
IMAGE_WIDTH = 160          # camera frame width, px
allowed_ratio_range = 0.8  # (reserved for patch-size sanity check, currently unused)

# --------------------------------------------------------------------------
# Blob-detection color thresholds (LAB), used by find_blobs()
# --------------------------------------------------------------------------
RED_THRESHOLD = (15, 100, 15, 127, -128, 127)
GREEN_THRESHOLD = (20, 100, -100, -10, -10, 100)
BLUE_THRESHOLD = (10, 100, -20, 50, -100, -10)
YELLOW_THRESHOLD = (30, 100, -30, 40, 20, 100)
BLACK_THRESHOLD = (0, 26, 127, -128, 127, -128)

COLOR_THRESHOLDS = [
    RED_THRESHOLD,
    GREEN_THRESHOLD,
    # BLUE_THRESHOLD,
    YELLOW_THRESHOLD,
    BLACK_THRESHOLD
]

last_changed = time.ticks_ms()
prev_victim = 0

# --------------------------------------------------------------------------
# Camera setup
# --------------------------------------------------------------------------
sensor.reset()
sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.B160X160)
sensor.snapshot(time=2000)
sensor.set_auto_exposure(True, exposure_us=1)
# sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_hmirror(True)
sensor.set_vflip(True)

clock = time.clock()

# --------------------------------------------------------------------------
# Main loop
# --------------------------------------------------------------------------
while True:
    clock.tick()

    img = sensor.snapshot()
    dist = tof.read()

    width_px = 0

    # Margin (px) kept clear on all sides of the frame when searching for blobs
    clip_window = 10

    blobs = img.find_blobs(
        COLOR_THRESHOLDS,
        roi=(
            clip_window + 1,
            clip_window + 1,
            img.width() - clip_window * 2 - 2,
            img.height() - clip_window * 2 - 2,
        ),
        x_stride=2,
        y_stride=2,
        area_threshold=100,
        pixels_threshold=80,
        merge=True,
        margin=2,
    )

    for blob in blobs:
        # ------------------------------------------------------------
        # Basic geometry / rejection filters
        # ------------------------------------------------------------
        w = blob.w
        h = blob.h
        bx = blob.x
        by = blob.y
        cx = bx + w // 2
        cy = by + h // 2

        stats = img.get_statistics(roi=blob.rect)

        # Estimate pixel width of a 4 cm real-world patch at this distance
        width_px = (IMAGE_WIDTH * PATCH_WIDTH_MM / (2.0 * dist * math.tan(math.radians(FOV / 2.0))))
        patch_size_estimate = width_px / w

        # Reject blobs touching the ROI edges (likely clipped)
        if blob.x <= 0 or blob.y <= 0:
            continue
        if blob.x + w >= img.width() - clip_window - 1:
            continue
        if blob.y + h >= img.height() - clip_window - 1:
            continue

        # Reject non-square blobs
        ratio = w / h
        if ratio < 0.85 or ratio > 1.15:
            continue

        # ------------------------------------------------------------
        # Lay out 5 sample points across the patch, from center outward
        # ------------------------------------------------------------
        x1 = int(cx)
        x2 = int(cx - width_px * 1.32 / 10)
        x3 = int(cx - width_px * 2.32 / 10)
        x4 = int(cx - width_px * 3.32 / 10)
        x5 = int(cx - width_px * 4.32 / 10)

        points = (x5, x4, x3, x2, x1)
        y = cy
        radius = min(w, h) // 2

        # ------------------------------------------------------------
        # Classify the color at each sample point and accumulate score
        # ------------------------------------------------------------
        victim = 0
        c = ""

        for i, xx in enumerate(points):
            # Average a 3x3 pixel neighborhood around the sample point
            r = 0
            g = 0
            b = 0
            for y_scan in range(y - 1, y + 2):
                for x_scan in range(xx - 1, xx + 2):
                    if img:
                        p = img.get_pixel((x_scan, y_scan))
                        if p and len(p) >= 3:
                            r += p[0]
                            g += p[1]
                            b += p[2]
            r //= 9
            g //= 9
            b //= 9

            # Normalize RGB to percentages of their sum, for color comparison
            _sum = 1
            if r > 0 and g > 0 and b > 0:
                _sum = r + g + b

            r_scaled = int((r / _sum) * 100)
            g_scaled = int((g / _sum) * 100)
            b_scaled = int((b / _sum) * 100)

            # Draw the sampled color at this point (mul boosts brightness for visibility)
            mul = 1.5
            img.draw_circle(
                (xx, int(y), 2),
                color=(min(int(r * mul), 255), min(int(g * mul), 255), min(int(b * mul), 255)),
                fill=True,
            )
            img.draw_circle((xx, int(y), 2), color=(0, 0, 0), fill=False)

            _size = 10
            max_val = max(r_scaled, g_scaled, b_scaled)
            min_val = min(r_scaled, g_scaled, b_scaled)
            spread = max_val - min_val

            isGreen = (g_scaled - r_scaled >= 5) and (g_scaled > b_scaled)
            isRed = ((r_scaled - (g_scaled + b_scaled) / 2) > 15) and (abs(g_scaled - b_scaled) < 15)
            isBlue = ((b_scaled - (g_scaled + r_scaled) / 2) > 5)
            isBlack = (_sum < 230) and (max_val - min_val < 10)

            if isRed:
                c = "red"
                victim -= 1
                img.draw_rectangle((int(i * _size), 0, _size, _size), color=(180, 0, 0), fill=True)
            elif isGreen:
                c = "green"
                victim += 1
                img.draw_rectangle((int(i * _size), 0, _size, _size), color=(0, 180, 0), fill=True)
            elif isBlue:
                c = "blue"
                victim += 2
                img.draw_rectangle((int(i * _size), 0, _size, _size), color=(100, 100, 255), fill=True)
            elif isBlack:
                c = "black"
                victim -= 2
                img.draw_rectangle((int(i * _size), 0, _size, _size), color=(0, 0, 0), fill=True)
            else:
                c = "none"
                img.draw_rectangle((int(i * _size), 0, _size, _size), color=(200, 200, 0), fill=True)

        # ------------------------------------------------------------
        # Report the final victim classification for this blob
        # ------------------------------------------------------------
        if victim != prev_victim:
            prev_victim = victim
            last_changed = time.ticks_ms()
        elapsed = time.ticks_diff(time.ticks_ms(), last_changed)

        img.draw_string((x1 - 10, y), str(victim), scale=3, color=(0, 0, 0))

        img.draw_string((60, 0), str(dist), scale=1, color=(0, 0, 0))
        img.draw_string((90, 0), str(elapsed), scale=1, color=(0, 0, 0))

        img.draw_rectangle(
            (clip_window,
             clip_window,
             img.width() - clip_window * 2,
             img.height() - clip_window * 2),
            thickness=1,
            color=(255, 255, 255),
        )

        victim_text = ""
        if victim == 2:
            victim_text = "Harmed"
        elif victim == 1:
            victim_text = "Stable"
        elif victim == 0:
            victim_text = "Unharmed"
        else:
            victim_text = "Fake"

        data = str(victim) + "," + str(elapsed) + "," + str(x1) + "," + str(y)
        print(data)
        data += "\n"
        uart.write(data)

        img.draw_string((12, 12), victim_text, scale=1, color=(0, 0, 0))
        img.draw_circle((x1, y, int(width_px / 2)), color=(255, 255, 255), thickness=1)
