import numpy as np
import math
import matplotlib.pyplot as plt
import pygame
import socket, struct

#
dt = 0.01 # intergration step timedt = 0.01 # integration step time

pygame.init() # start pygame
window = pygame.display.set_mode((800, 600)) # create a window (size in pixels)
window.fill((255,255,255)) # white background
xc, yc = window.get_rect().center # window center
pygame.display.set_caption('robot arm')

font = pygame.font.Font('freesansbold.ttf', 12) # printing text font and font size
text = font.render('robot arm', True, (0, 0, 0), (255, 255, 255)) # printing text object
textRect = text.get_rect()
textRect.topleft = (10, 10) # printing text position with respect to the top-left corner of the window
EE_width, EE_height = 15, 15

clock = pygame.time.Clock() # initialise clock
FPS = int(1/dt) # refresh rate


# Initial conditions
t = 0.0 # time

pr = np.zeros(2) # reference endpoint position
p = np.array([500,500]) # actual endpoint position
p_prev = np.zeros(2) # previous endpoint position
dp = np.zeros(2) # actual endpoint velocity
F = np.zeros(2) # endpoint force

m = 0.5 # endpoint mass
i = 0 # loop counter
state = [] # state vector
orientation = 0  # Stiffness frame orientation
er = np.zeros(2) # Error

# IMPEDANCE CONTROLLER PARAMETERS
Ks = np.diag([300,100]) # stiffness in the endpoint stiffness frame [N/m] 30
Kd = 2*np.sqrt(Ks * m) # damping in the endpoint stiffness frame [N/ms-1]

object_dict = {
            'skin': {'color': (186, 154, 127), 'rect': pygame.Rect(0, 100, 600, 50), 'force': 0},
            'bone': {'color': (255, 5, 127), 'rect': pygame.Rect(0, 200, 600, 50), 'force': 0},
            'heart': {'color': (255, 0, 0), 'rect': pygame.Rect(300, 350, 50, 50), 'force': 0},
        }

object_dict = {
    'skin_0': {'color': (186, 154, 127), 'rect': pygame.Rect(0, 300, 600, 50), 'force': 0},
    'skin_1': {'color': (186, 154, 127), 'rect': pygame.Rect(0, 350, 600, 50), 'force': 0},
    'skin_2': {'color': (186, 154, 127), 'rect': pygame.Rect(0, 400, 600, 50), 'force': 0},
    'skin_3': {'color': (186, 154, 127), 'rect': pygame.Rect(0, 450, 600, 50), 'force': 0},
    'skin_4': {'color': (186, 154, 127), 'rect': pygame.Rect(0, 500, 600, 50), 'force': 0}
}

occupancy_grid = np.zeros((800, 600))
for key in object_dict:
    pg_rect = object_dict[key]['rect']
    tl_x, tl_y = pg_rect.topleft
    br_x, br_y = pg_rect.bottomright
    occupancy_grid[tl_x:br_x, tl_y:br_y] = 1


print(occupancy_grid)

def in_collision_with_grid(pr):
    if not (occupancy_grid[pr[0], pr[1]] or occupancy_grid[pr[0] + EE_width, pr[1]] or occupancy_grid[pr[0], pr[1] + EE_height] or occupancy_grid[pr[0] + EE_width, pr[1] + EE_height]):
        return False
    else:
        return True


def pygame_controls():
    for event in pygame.event.get(): # interrupt function
        if event.type == pygame.QUIT: # force quit with closing the window
            run = False
        elif event.type == pygame.KEYUP:
            if event.key == ord('q'): # force quit with q button
                run = False
            '''*********** Student should fill in ***********'''
            # tele-impedance interface / control mode switch
            if event.key == ord('s'): # switch mode
                pass
            # global orientation, Ks, Kd

            if event.key == ord('x'):
                Ks[0,0] += 100  # Increase x stiffness
                Ks[0,0] = np.clip(Ks[0,0],0, 1000)
            elif event.key == ord('z'):
                Ks[0,0] -= 100  # Increase x stiffness
                Ks[0, 0] = np.clip(Ks[0, 0], 0, 1000)
            elif event.key == ord('d'):
                Ks[1,1] += 100  # Increase y stiffness
                Ks[1, 1] = np.clip(Ks[1, 1], 0, 1000)
            elif event.key == ord('c'):
                Ks[1,1] -= 100  # Decrease y stiffness
                Ks[1, 1] = np.clip(Ks[1, 1], 0, 1000)
            elif event.key == ord('a'):
                orientation += 10  # Increase orientation
                orientation = np.clip(orientation, -90, 90)
            elif event.key == ord('s'):
                orientation -= 10  # Decrease orientation
                orientation = np.clip(orientation, -90, 90)

            Kd = 2 * np.sqrt(Ks * m)  # damping in the endpoint stiffness frame [N/m-1]

# Set up sockets
send_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # create a send socket
recv_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # create a receive socket
recv_sock.bind(("localhost", 40002)) # bind the socket to port 40002

# Send dummy data
force = np.array([0.0, 0.0])
send_data = bytearray(struct.pack("=%sf" % force.size, *force))  # convert array of 3 floats to bytes
send_sock.sendto(send_data, ("localhost", 40001))  # send to IP address 192.168.0.3 and port 40001

def render():
    window.fill((255, 255, 255))  # clear window

    for key in object_dict:
        pygame.draw.rect(window, object_dict[key]['color'], object_dict[key]['rect'])

    pygame.draw.rect(window, (0, 0, 0), (p[0], p[1], EE_width, EE_height))

    pygame.display.flip()

def receive_udp():
    pass

def send_udp():
    pass

run = True
while run:
    pygame_controls()

    # Receive Position from UDP
    recv_data, address = recv_sock.recvfrom(12)  # receive data with buffer size of 12 bytes
    position = struct.unpack("2f", recv_data)  # convert the received data from bytes to an array of 3 floats (assuming force in 3 axes

    # Set position if no collision
    prev_p = p
    pr = np.asarray(position)
    # Transform coordinates
    pr[0] *= 800/600
    pr[1] *= 600/400
    pr = np.int32(pr)

    if not in_collision_with_grid(pr):
        p = pr
        pass
    else:
        err = pr - p
        p_pot = p + np.int32(err/10)
        p_pot_dx = np.array([p_pot[0], p[1]])
        p_pot_dy = np.array([p[0], p_pot[1]])
        print(p_pot_dx)
        if not in_collision_with_grid(p_pot):
            p = p_pot
        elif not in_collision_with_grid(p_pot_dx):
            p = p_pot_dx
        elif not in_collision_with_grid(p_pot_dy):
            p = p_pot_dy
        else:
            p = prev_p


    # if position.ndim == 1:
    #     position = np.expand_dims(position, axis=1)
    # position = np.vstack((position, np.ones((1, position.shape[1]))))

    # Send Force over UDP
    force = np.array([0, 0])
    send_data = bytearray(struct.pack("=%sf" % force.size, *force))  # convert array of 3 floats to bytes
    send_sock.sendto(send_data, ("localhost", 40001))  # send to IP address 192.168.0.3 and port 40001



    render()

    if run == False:
        break

pygame.quit() # stop pygame

