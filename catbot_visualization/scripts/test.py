"""
# Sample Python/Pygame Programs
# Simpson College Computer Science
# http://programarcadegames.com/
# http://simpson.edu/computer-science/
"""

import pygame
import time
# Basic colors
NEGRO = (0, 0, 0)
BLANCO = (255, 255, 255)


pygame.init()
pygame.joystick.init()

# -------- Main loop -----------
while True:
    # Event processing
    for evento in pygame.event.get():
        if evento.type == pygame.QUIT:
            hecho = True

        # Possible joystick events: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
        if evento.type == pygame.JOYBUTTONDOWN:
            print('Joystick button pressed.')
        if evento.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")


    joystick_count = pygame.joystick.get_count()

    print("Joystick count: {}".format(joystick_count))

    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()

        print("Joystick {}".format(i))

        # Controller name as reported by the OS
        nombre = joystick.get_name()
        print("Joystick name: {}".format(nombre))

        # Axes are typically paired: up/down on one, left/right on the other.
        ejes = joystick.get_numaxes()
        print("Axis count: {}".format(ejes))

        for i in range(ejes):
            eje = joystick.get_axis(i)
            print("Axis {} value: {:>6.3f}".format(i, eje))

        botones = joystick.get_numbuttons()
        print("Button count: {}".format(botones))

        for i in range(botones):
            boton = joystick.get_button(i)
            print("Button {:>2} value: {}".format(i, boton))

        # Hat values are returned as tuples.
        hats = joystick.get_numhats()
        print("Hat count: {}".format(hats))

        for i in range(hats):
            hat = joystick.get_hat(i)
            print("Hat {} value: {}".format(i, str(hat)))

    print("\n\n\n\n")
    time.sleep(0.1)

pygame.quit()
