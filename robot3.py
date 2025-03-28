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
Ks_initial = np.diag([1000.0,1000.0])*10
Ks = Ks_initial.copy() # stiffness in the endpoint stiffness frame [N/m] 30
Kd = 2*np.sqrt(Ks * m)*10# damping in the endpoint stiffness frame [N/ms-1]

# Kd = np.diag([0.005,0.005])

window_width = 800
window_height = 600

offset = 300
#TODO: add stiffnesses if you use the original object_dict
object_dict = {
    # format: 'name': {'color': (R, G, B), 'rect': pygame.Rect(x_tl, y_tl, width, height), 'force': breaking force}
    'heart': {'color': (255, 0, 0), 'rect': pygame.Rect(300, 300 + offset, 75, 125), 'force': 25*100},
    'heart2': {'color': (255, 0, 0), 'rect': pygame.Rect(450, 300 + offset, 75, 125), 'force': 25*100},
    'tumor': {'color': (255, 120, 255), 'rect': pygame.Rect(425, 350 + offset, 50, 50), 'force': 5*100},
    'skin': {'color': (186, 154, 127), 'rect': pygame.Rect(0, offset, window_width, 250), 'force': 150*100},
    'bone': {'color': (240, 240, 240), 'rect': pygame.Rect(0, 250 + offset, window_width, 50), 'force': 300*100},
    'tissue': {'color': (255, 182, 193), 'rect': pygame.Rect(0, 300 + offset, window_width, 50), 'force': 0},
    'muscle': {'color': (96, 5, 33), 'rect': pygame.Rect(0, 350 + offset, window_width, 50), 'force': 0},
}

materials = {
    "heart" : {"color": (255, 0, 0), "force": 25*50, "stiffness": 3000},
    "tumor" : {"color": (255, 120, 255), "force": 50*50, "stiffness": 6000},
    "skin" : {"color": (186, 154, 127), "force": 150*50, "stiffness": 4500},
    "bone" : {"color": (240, 240, 240), "force": 400*50, "stiffness": 15000},
    # "tissue" : {"color": (255, 182, 193), "force": 0},
    # "muscle" : {"color": (96, 5, 33), "force": 0},
    # "lung" : {"color": (255, 255, 0), "force": 0},
    # "fat" : {"color": (255, 140, 0), "force": 0},
    # "nerve" : {"color": (0, 255, 0), "force": 0},
}

def generate_random_object_configurations(num_layers, materials=materials):
    object_dict = {}
    y_height = 0
    material_keys = list(materials.keys())  # Get the keys of the materials dictionary
    for i in range(num_layers):
        layer_name = f'layer_{i}'
        selected_material = random.choice(material_keys)  # Randomly select a material
        material = materials[selected_material]  # Get the material properties
        color = material["color"]
        force = material["force"]
        stiffness = material["stiffness"]
        height = random.randrange(50, 150, 25)  # Random height for the layer
        rect = pygame.Rect(0, y_height, window_width, height)
        y_height += height
        object_dict[layer_name] = {'color': color, 'rect': rect, 'force': force, 'stiffness': stiffness}
    return object_dict

# Resolution in X
x_steps = 25
y_steps = 25

# Generate random object configurations, for training
object_dict = generate_random_object_configurations(8)

def generate_objects(object_dict, x_res, y_res):
    dict = {}
    i = 0
    occupied_pixels = set()
    for key in object_dict:
        rect = object_dict[key]['rect']
        color = object_dict[key]['color']
        force = object_dict[key]['force']
        stiffness = object_dict[key]['stiffness']
        width = rect.width
        height = rect.height

        offset_y = rect.y
        offset_x = rect.x
        for x in range(0, width, x_res):
            for y in range(0, height, y_res):
                rect = pygame.Rect(x + offset_x, y + offset_y, x_res, y_res)
                rect_pixels = {(rect.x + dx, rect.y + dy) for dx in range(rect.width) for dy in range(rect.height)}

                if not rect_pixels & occupied_pixels:  # Check for overlap
                    occupied_pixels.update(rect_pixels)
                    dict[str(int(i))] = {'color': color, 'rect': rect, 'force': force, 'stiffness': stiffness}
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

def check_collision(p, occupancy_grid, buffer=0):
    p = np.int32(p)
    collisions = {}

    # Define the 8 critical points with buffer applied
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
        collision_id = occupancy_grid[x, y]
        if collision_id:  # If occupied
            collisions[alias] = (x, y, int(collision_id))

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

def render():
    window.fill((255, 255, 255))  # clear window

    for key in split_object_dict:
        pygame.draw.rect(window, split_object_dict[key]['color'], split_object_dict[key]['rect'])

    pygame.draw.rect(window, (0, 0, 0), (p[0], p[1], EE_width, EE_height))
    pygame.draw.rect(window, (255, 0, 0), (pr[0], pr[1], int(EE_width/2), int(EE_height/2)))

    pygame.display.flip()

def should_pop(idd, direction):
    key_to_pop = str(int(idd))
    # check if key exists in split_object_dict
    if key_to_pop in split_object_dict:
        pg_rect = split_object_dict[key_to_pop]['rect']
        breaking_force = split_object_dict[key_to_pop]['force']

        print("Breaking force: ", breaking_force)
        print("Force: ", np.linalg.norm(F))
        Fx, Fy = F
        if abs(Fx) > breaking_force and direction in ["mid-left", "mid-right"]:
            tl_x, tl_y = pg_rect.topleft
            br_x, br_y = pg_rect.bottomright
            occupancy_grid[tl_x:br_x, tl_y:br_y] = 0
            split_object_dict.pop(key_to_pop)

        if abs(Fy) > breaking_force and direction in ["mid-top", "mid-bottom"]:
            tl_x, tl_y = pg_rect.topleft
            br_x, br_y = pg_rect.bottomright
            occupancy_grid[tl_x:br_x, tl_y:br_y] = 0
            split_object_dict.pop(key_to_pop)

    else:
        print(f"Key {key_to_pop} not found in split_object_dict.")

def adjust_stifness(object_dict, idd):
    global Ks
    material_stifness = object_dict[str(int(idd))]['stiffness']
    Ks = np.diag([material_stifness, material_stifness])


# Set up sockets
send_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # create a send socket
recv_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # create a receive socket
recv_sock.bind(("localhost", 40002)) # bind the socket to port 40002

# Send dummy data
force = F
send_data = bytearray(struct.pack("=%sf" % force.size, *force))  # convert array of 3 floats to bytes
send_sock.sendto(send_data, ("localhost", 40001))  # send to IP address 192.168.0.3 and port 40001

def receive_udp():
    recv_data, address = recv_sock.recvfrom(12)  # receive data with buffer size of 12 bytes
    position = struct.unpack("2f", recv_data)  # convert the received data from bytes to an array of 3 floats (assuming force in 3 axes
    return position

def send_udp(force):
    # Send Force over UDP
    force = np.array(F)
    send_data = bytearray(struct.pack("=%sf" % force.size, *force))  # convert array of 3 floats to bytes
    send_sock.sendto(send_data, ("localhost", 40001))  # send to IP address 192.168.0.3 and port 40001

run = True
while run:
    pygame_controls()
    position = receive_udp()  # Receive position from UDP
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
    if len(collisions) == 0:
        Ks = Ks_initial.copy()

    processed_ids = set()
    for direction, (x, y, idd) in collisions.items():
        if str(int(idd)) in processed_ids:
            continue

        pg_rect = split_object_dict[str(int(idd))]['rect']
        tl_x, tl_y = pg_rect.topleft
        br_x, br_y = pg_rect.bottomright
        if direction in ["mid-left"]: #,"top-left", "bottom-left"
            p[0] = np.clip(p[0], br_x, window_width)
            dp[0] = 0
            print("clipping left", x)
            adjust_stifness(split_object_dict, idd)
            should_pop(idd, "mid-left")
            processed_ids.add(str(int(idd)))

        elif direction in ["mid-right"]: #, "top-right", "bottom-right"
            p[0] = np.clip(p[0], 0, tl_x-EE_width)
            dp[0] = 0
            print("clipping right", x)
            adjust_stifness(split_object_dict, idd)
            should_pop(idd, "mid-right")
            processed_ids.add(str(int(idd)))

        if direction in ["mid-top"]: #, "top-left", "top-right"
            p[1] = np.clip(p[1], br_y, window_height)
            dp[1] = 0
            print("clipping top", y)
            adjust_stifness(split_object_dict, idd)
            should_pop(idd, "mid-top")
            processed_ids.add(str(int(idd)))

        elif direction in ["mid-bottom"]: #, "bottom-left", "bottom-right"
            p[1] = np.clip(p[1], 0, tl_y-EE_height)
            dp[1] = 0
            print("clipping bot", y)
            adjust_stifness(split_object_dict, idd)
            should_pop(idd, "mid-bottom")
            processed_ids.add(str(int(idd)))
        

    send_udp(F)
    render()

    if run == False:
        break

pygame.quit() # stop pygame

