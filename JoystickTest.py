import pygame
import time

# Initialize pygame
pygame.init()
pygame.joystick.init()

# Check for joystick
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joystick detected.")
    exit(1)

# Open joystick 0
js = pygame.joystick.Joystick(0)
js.init()

print(f"Joystick connected: {js.get_name()}")
print(f"Axes: {js.get_numaxes()}, Buttons: {js.get_numbuttons()}")

print("\nMove joystick or press buttons...\n")

while True:
    pygame.event.pump()  # read hardware state

    # Print axes
    for i in range(js.get_numaxes()):
        val = js.get_axis(i)
        print(f"Axis {i}: {val:.3f}", end=" | ")

    print()  # newline

    # Print buttons
    for i in range(js.get_numbuttons()):
        if js.get_button(i):
            print(f"Button {i} pressed")

    time.sleep(0.1)
