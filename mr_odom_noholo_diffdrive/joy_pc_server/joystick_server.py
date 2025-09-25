import pygame
import socket
import json
import time

# Init
pygame.init()
pygame.joystick.init()
joy = pygame.joystick.Joystick(0)
joy.init()

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("127.0.0.1", 5005))

print(f"Streaming from joystick: {joy.get_name()}")

while True:
    pygame.event.pump()
    axes = [joy.get_axis(i) for i in range(joy.get_numaxes())]
    buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]
    hats = list(joy.get_hat(0)) if joy.get_numhats() > 0 else []

    data = {
        "axes": axes,
        "buttons": buttons,
        "hats": hats,
        "timestamp": time.time()
    }

    sock.sendall((json.dumps(data) + "\n").encode())
    time.sleep(0.02)  # ~50 Hz
