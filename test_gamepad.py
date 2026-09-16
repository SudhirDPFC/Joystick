#!/usr/bin/env python3
"""
Test script to read live inputs from the STM32 Ci4UVS USB HID Gamepad
Uses pygame to read standard DirectInput / USB Gamepad events.
"""
import sys
import time

try:
    import pygame
except ImportError:
    print("[ERROR] pygame not found. Run: pip install pygame")
    sys.exit(1)

def main():
    pygame.init()
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("[WAITING] No gamepads detected. Make sure the STM32 USB cable is connected!")
        print("Waiting for device connection...")
        while pygame.joystick.get_count() == 0:
            pygame.event.pump()
            pygame.joystick.quit()
            pygame.joystick.init()
            time.sleep(0.5)

    js = pygame.joystick.Joystick(0)
    js.init()

    print("=" * 60)
    print(f" Detected Controller: {js.get_name()}")
    print(f" Axes Count         : {js.get_numaxes()}")
    print(f" Buttons Count      : {js.get_numbuttons()}")
    print("=" * 60)
    print("Move Joystick (X, Y, Z) and press Buttons (PB12..PB15, PA8, PA9)...")
    print("Press Ctrl+C to exit.\n")

    BUTTON_LABELS = {
        0: "BTN 1 (PB12)",
        1: "BTN 2 (PB13)",
        2: "BTN 3 (PB14)",
        3: "BTN 4 (PB15)",
        4: "BTN 5 (PA8)",
        5: "BTN 6 (PA9)",
    }

    last_print = 0

    try:
        while True:
            pygame.event.pump()

            # Read axes (-1.0 to 1.0)
            axes = [round(js.get_axis(i), 3) for i in range(js.get_numaxes())]

            # Read buttons (0 or 1)
            buttons_down = []
            for b in range(js.get_numbuttons()):
                if js.get_button(b):
                    label = BUTTON_LABELS.get(b, f"BTN {b+1}")
                    buttons_down.append(label)

            now = time.time()
            if now - last_print >= 0.1:  # 10Hz print rate
                last_print = now
                axis_str = " | ".join([f"Axis {i}: {val:+.3f}" for i, val in enumerate(axes)])
                btn_str = ", ".join(buttons_down) if buttons_down else "None"
                sys.stdout.write(f"\r[LIVE] {axis_str} | Active Buttons: [{btn_str}]     ")
                sys.stdout.flush()

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nTest stopped by user.")
        pygame.quit()

if __name__ == "__main__":
    main()
