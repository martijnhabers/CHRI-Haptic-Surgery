import random
import numpy as np
import math
import matplotlib.pyplot as plt
import pygame
import socket, struct
import os

#
dt = 0.01 # intergration step timedt = 0.01 # integration step time
os.environ['SDL_VIDEO_WINDOW_POS'] = "1000,400"
pygame.init() # start pygame
window = pygame.display.set_mode((800, 600)) # create a window (size in pixels)
window.fill((255,255,255)) # white background
xc, yc = window.get_rect().center # window center
pygame.display.set_caption('robot arm')

font = pygame.font.Font('freesansbold.ttf', 12) # printing text font and font size
text = font.render('robot arm', True, (0, 0, 0), (255, 255, 255)) # printing text object
textRect = text.get_rect()
textRect.topleft = (10, 10) # printing text position with respect to the top-left corner of the window
EE_width, EE_height = 20, 20

clock = pygame.time.Clock() # initialise clock
FPS = int(1/dt) # refresh rate


# Initial conditions
t = 0.0 # time
n = 0 # plot

pr = np.zeros(2) # reference endpoint position
p = np.array([50.0,50.0]) # actual endpoint position
p_prev = np.zeros(2) # previous endpoint position
dp = np.zeros(2) # actual endpoint velocity
F = np.zeros(2) # endpoint force

m = 0.5 # endpoint mass
i = 0 # loop counter
state = [] # state vector
orientation = 0  # Stiffness frame orientation
er = np.zeros(2) # Error

# IMPEDANCE CONTROLLER PARAMETERS
Ks = np.diag([1000.0,1000.0])*20 # stiffness in the endpoint stiffness frame [N/m] 30
Kd = 2*np.sqrt(Ks * m)*10# damping in the endpoint stiffness frame [N/ms-1]

# Kd = np.diag([0.005,0.005])

window_width = 800
window_height = 600

offset = 200
object_dict = {
    'air': {'color': (255, 255, 0), 'rect': pygame.Rect(0, 0+offset, window_width/2, 100), 'force': 0},
    'skin1': {'color': (186, 154, 127), 'rect': pygame.Rect(0, 100+offset, window_width/2, 50), 'force': 0},
    'skin2': {'color': (186, 154, 127), 'rect': pygame.Rect(0, 150+offset, 800/2, 50), 'force': 0},
    'skin3': {'color': (186, 154, 127), 'rect': pygame.Rect(0, 200+offset, 800, 50), 'force': 0},
    'bone': {'color': (240, 240, 240), 'rect': pygame.Rect(0, 250+offset, 800, 50), 'force': 0},
    'tissue': {'color': (255, 182, 193), 'rect': pygame.Rect(0, 300+offset, 800, 50), 'force': 0},
    'tumor': {'color': (255, 0, 255), 'rect': pygame.Rect(0, 350+offset, 800, 50), 'force': 0},
    'heart': {'color': (255, 0, 0), 'rect': pygame.Rect(300, 300+offset, 68, 123), 'force': 0},
}

# Resolution in X
x_steps = 25
y_steps = 25


def generate_objects(object_dict, x_res, y_res):
    dict = {}
    i = 0
    for key in object_dict:
        rect = object_dict[key]['rect']
        color = object_dict[key]['color']
        width = rect.width
        height = rect.height
        
        offset_y = rect.y
        offset_x = rect.x
        for x in range(0, width, x_res):
            for y in range(0, height, y_res):
                rect = pygame.Rect(x + offset_x, y + offset_y, x_res, y_res)
                random_color = tuple(random.randint(0, 255) for _ in range(3))
                dict[str(int(i))] = {'color': color, 'rect': rect, 'force': 5000}
                i += 1
    return dict

split_object_dict = generate_objects(object_dict, x_steps, y_steps)

def generate_occupancy_grid(split_object_dict):
    occupancy_grid = np.zeros((800, 600))
    for key in split_object_dict:
        pg_rect = split_object_dict[key]['rect']
        tl_x, tl_y = pg_rect.topleft
        br_x, br_y = pg_rect.bottomright
        occupancy_grid[tl_x:br_x, tl_y:br_y] = int(key)
    return occupancy_grid

occupancy_grid = generate_occupancy_grid(split_object_dict)


print(occupancy_grid)

def in_collision_with_grid(pr):
    if pr[0] < 0 or pr[0] > 800- EE_width or pr[1] < 0 or pr[1] > 600 - EE_height:
        return False
    buffer = 0
    # Check if there is a collision from the top


    if not (occupancy_grid[pr[0], pr[1]] or occupancy_grid[pr[0] + EE_width, pr[1]] or occupancy_grid[pr[0], pr[1] + EE_height] or occupancy_grid[pr[0] + EE_width, pr[1] + EE_height]):
        return False
    else:
        return True


#TODO: new possible approach: once a block collides with an object, keep it "on rails", allowing it to move only in certain ways along the edges of the object/objecs


def check_collision(p, occupancy_grid, buffer=0):
    p = np.int32(p)
    collisions = {}

    # Define the 8 critical points with buffer applied
    critical_points = {
        "bottom-left": (p[0] - buffer, p[1] - buffer),
        "bottom-right": (p[0] + EE_width + buffer, p[1] - buffer),
        "top-left": (p[0] - buffer, p[1] + EE_height + buffer),
        "top-right": (p[0] + EE_width + buffer, p[1] + EE_height + buffer),
        "mid-bottom": (p[0] + EE_width // 2, p[1] - buffer),
        "mid-top": (p[0] + EE_width // 2, p[1] + EE_height + buffer),
        "mid-left": (p[0] - buffer, p[1] + EE_height // 2),
        "mid-right": (p[0] + EE_width + buffer, p[1] + EE_height // 2),
    }

    critical_points = {
        "top-left": (p[0] - buffer, p[1] - buffer),
        "top-right": (p[0] + EE_width + buffer, p[1] - buffer),
        "bottom-left": (p[0] - buffer, p[1] + EE_height + buffer),
        "bottom-right": (p[0] + EE_width + buffer, p[1] + EE_height + buffer),
        "mid-top": (p[0] + EE_width // 2, p[1] - buffer),
        "mid-bottom": (p[0] + EE_width // 2, p[1] + EE_height + buffer),
        "mid-left": (p[0] - buffer, p[1] + EE_height // 2),
        "mid-right": (p[0] + EE_width + buffer, p[1] + EE_height // 2),
    }

    # Check for collisions and store them in a dictionary
    for alias, (x, y) in critical_points.items():
        if occupancy_grid[x, y]:  # If occupied
            collisions[alias] = (x, y, occupancy_grid[x, y])

    return collisions  # Dictionary of alias -> coordinate



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

    for key in split_object_dict:
        pygame.draw.rect(window, split_object_dict[key]['color'], split_object_dict[key]['rect'])

    pygame.draw.rect(window, (0, 0, 0), (p[0], p[1], EE_width, EE_height))
    pygame.draw.rect(window, (255, 0, 0), (pr[0], pr[1], int(EE_width/2), int(EE_height/2)))

    pygame.display.flip()

def should_pop(idd):
    key_to_pop = str(int(idd))
    # check if key exists in split_object_dict
    if key_to_pop in split_object_dict:
        pg_rect = split_object_dict[key_to_pop]['rect']
        breaking_force = split_object_dict[key_to_pop]['force']

        print("Breaking force: ", breaking_force)
        print("Force: ", np.linalg.norm(F))

        if np.linalg.norm(F) > breaking_force:
            tl_x, tl_y = pg_rect.topleft
            br_x, br_y = pg_rect.bottomright
            occupancy_grid[tl_x:br_x, tl_y:br_y] = 0
            split_object_dict.pop(key_to_pop)
def receive_udp():
    pass

def send_udp():
    pass

run = True
while run:
    pygame_controls()


    # Receive Reference Position from UDP
    recv_data, address = recv_sock.recvfrom(12)  # receive data with buffer size of 12 bytes
    position = struct.unpack("2f", recv_data)  # convert the received data from bytes to an array of 3 floats (assuming force in 3 axes

    # Set position if no collision

    pr = np.asarray(position)
    # Transform coordinates
    pr[0] *= 800/600
    pr[1] *= 600/400


    er_prev = er
    er = np.subtract(pr, p)  # Error
    derr_dt = (er - er_prev) / dt  # derr/dt

    dp_dt = (p - p_prev)/dt
    p_prev = p.copy()

    F_spring = -Ks @ er/100
    # F_damper = -(Kd @ derr_dt/100)
    F_damper = (Kd @ dp_dt / 100)
    F = F_spring + F_damper
    F = -F
    if n > 100:
        print("velocity: ", derr_dt)
        print("F_spring: ", F_spring)
        print("F_damper: ", F_damper)
        print("Force: ", F)
        n = 0
    n += 1


    # integration
    ddp = F / m
    dp += ddp * dt
    p += dp * dt
    t += dt


    collisions = check_collision(p, occupancy_grid, buffer=1)


    for direction, (x, y, idd) in collisions.items():
        pg_rect = split_object_dict[str(int(idd))]['rect']
        tl_x, tl_y = pg_rect.topleft
        br_x, br_y = pg_rect.bottomright
        if direction in ["mid-left"]: #, "top-left", "bottom-left"
            p[0] = np.clip(p[0], br_x, window_width)
            dp[0] = 0
            should_pop(idd)

        elif direction in ["mid-right"]: #, "top-right", "bottom-right"
            p[0] = np.clip(p[0], 0, tl_x-EE_width)
            dp[0] = 0
            should_pop(idd)
            # print("clipping right", x)
        if direction in ["mid-top"]: #, "top-left", "top-right"
            p[1] = np.clip(p[1], br_y, window_height)
            dp[1] = 0
            should_pop(idd)
            # print("clipping top", y)
        elif direction in ["mid-bottom"]: #, "bottom-left", "bottom-right"
            p[1] = np.clip(p[1], 0, tl_y-EE_height)
            dp[1] = 0
            should_pop(idd)
            # print("clipping bot", y)


    # if position.ndim == 1:
    #     position = np.expand_dims(position, axis=1)
    # position = np.vstack((position, np.ones((1, position.shape[1]))))

    # Send Force over UDP
    force = np.array(F)
    send_data = bytearray(struct.pack("=%sf" % force.size, *force))  # convert array of 3 floats to bytes
    send_sock.sendto(send_data, ("localhost", 40001))  # send to IP address 192.168.0.3 and port 40001



    render()

    if run == False:
        break

pygame.quit() # stop pygame

