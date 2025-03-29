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
EE_width, EE_height = 16, 16

clock = pygame.time.Clock() # initialise clock
FPS = int(1/dt) # refresh rate


# Initial conditions
t = 0.0 # time
n = 0 # plot

pr = np.zeros(2) # reference endpoint position
p = np.array([586.0,138.0]) # actual endpoint position
p_prev = np.zeros(2) # previous endpoint position
dp = np.zeros(2) # actual endpoint velocity
F = np.zeros(2) # endpoint force
vel_ema = np.zeros(2)
alpha = 0.1

m = 0.5 # endpoint mass
i = 0 # loop counter
state = [] # state vector
orientation = 0  # Stiffness frame orientation
er = np.zeros(2) # Error

# IMPEDANCE CONTROLLER PARAMETERS
Kd_scale = 25
Ks_initial = np.diag([1000.0,1000.0])*10
Ks = Ks_initial.copy() # stiffness in the endpoint stiffness frame [N/m] 30
Kd_initial = 2*np.sqrt(Ks * m)*Kd_scale# damping in the endpoint stiffness frame [N/ms-1]
Kd = Kd_initial.copy()

# Kd = np.diag([0.005,0.005])

window_width = 800
window_height = 600

x_blocks = int(window_width/EE_width)
y_blocks = int(window_height/EE_height)

np.set_printoptions(threshold=np.inf)
print(x_blocks)
print(y_blocks)

def generate_maze(y_blocks, x_blocks):
    # Start with a maze full of flesh (2)
    maze = np.full((y_blocks, x_blocks), 2, dtype=int)

    for dir in [np.array([1, 0]), np.array([0, 1])]:
        start = [5, x_blocks//2]
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                ny, nx = start[0] + dy, start[1] + dx
                if 0 <= ny < y_blocks and 0 <= nx < x_blocks:  # Ensure within bounds
                    maze[ny, nx] = 0

        # Random initial direction

        pos = np.array([start[0], start[1]])

        # Create initial tunnel (3 steps forward)
        for _ in range(3):
            maze[pos[0], pos[1]] = 0
            pos = pos + dir
            pos[0] = np.clip(pos[0], 2, y_blocks - 3)  # Keep inside bounds
            pos[1] = np.clip(pos[1], 2, x_blocks - 3)

        # Create more tunnels
        for _ in range(300):
            if random.random() < 0.1:  # 5% chance to change direction
                if tuple(dir) in [(1, 0), (-1, 0)]:
                    dir = random.choice([np.array([0, 1]), np.array([0, -1])])
                else:
                    dir = random.choice([np.array([1, 0]), np.array([-1, 0])])

            maze[pos[0], pos[1]] = 0
            pos = pos + dir
            pos[0] = np.clip(pos[0], 2, y_blocks - 3)
            pos[1] = np.clip(pos[1], 2, x_blocks - 3)

    # Step 1: Add first layer of vein walls (1-block thick)
    for y in range(1, y_blocks - 1):
        for x in range(1, x_blocks - 1):
            if maze[y, x] == 2:  # Flesh cells only
                neighbors = [(y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)]
                if any(maze[ny, nx] == 0 for ny, nx in neighbors):
                    maze[y, x] = 1  # Convert flesh to vein wall if next to free space

    # Step 2: Expand vein walls to be 2 blocks thick **without touching free space (0)**
    new_vein_walls = []
    for y in range(1, y_blocks - 1):
        for x in range(1, x_blocks - 1):
            if maze[y, x] == 2:  # Flesh cells only
                neighbors = [(y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)]
                if any(maze[ny, nx] == 1 for ny, nx in neighbors):
                    new_vein_walls.append((y, x))  # Store these to update later

    # Apply the new vein walls **after scanning** to avoid overwriting during iteration
    for y, x in new_vein_walls:
        maze[y, x] = 1

    return maze



def create_object_dict(maze, cell_size=25):
    object_dict = {}
    wall_count = 1  # Counter for naming vein walls
    flesh_count = 1  # Counter for naming flesh blocks

    for y in range(maze.shape[0]):
        for x in range(maze.shape[1]):
            if maze[y, x] == 1:  # Vein Wall
                obj_name = f'vein_wall{wall_count}'
                object_dict[obj_name] = {
                    'color': (139, 0, 0),  # Bright red for veins
                    'rect': pygame.Rect(x * cell_size, y * cell_size, cell_size, cell_size),
                    'force': 100 * 100,
                    'stiffness': 10000
                }
                wall_count += 1

            elif maze[y, x] == 2:  # Flesh
                obj_name = f'flesh{flesh_count}'
                object_dict[obj_name] = {
                    'color': (255,192,203),  # Pink
                    'rect': pygame.Rect(x * cell_size, y * cell_size, cell_size, cell_size),
                    'force': 10 * 100,  # Flesh has different properties
                    'stiffness': 2000
                }
                flesh_count += 1

    return object_dict

maze = generate_maze(y_blocks, x_blocks)
object_dict = create_object_dict(maze)



# Resolution in X
x_steps = 25
y_steps = 25

# Generate random object configurations, for training
# object_dict = generate_random_object_configurations(8)

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
        Fx, Fy = F_spring
        print(Fx, Fy)
        if abs(Fx) > breaking_force and direction in ["mid-left", "mid-right"]:
            tl_x, tl_y = pg_rect.topleft
            br_x, br_y = pg_rect.bottomright
            occupancy_grid[tl_x:br_x, tl_y:br_y] = 0
            split_object_dict.pop(key_to_pop)
            print("Popped ID: ", key_to_pop)
            print("Breaking force (x,y): ", Fx, Fy)
            print("Damping force: ", F_damper)

        if abs(Fy) > breaking_force and direction in ["mid-top", "mid-bottom"]:
            tl_x, tl_y = pg_rect.topleft
            br_x, br_y = pg_rect.bottomright
            occupancy_grid[tl_x:br_x, tl_y:br_y] = 0
            split_object_dict.pop(key_to_pop)
            print("Popped ID: ", key_to_pop)
            print("Breaking force (x,y): ", Fx, Fy)
            print("Damping force: ", F_damper)

    else:
        print(f"Key {key_to_pop} not found in split_object_dict.")

def adjust_stifness(object_dict, idd):
    global Ks, Kd
    material_stifness = object_dict[str(int(idd))]['stiffness']
    Ks = np.diag([material_stifness, material_stifness])
    Kd = 2 * np.sqrt(Ks * m) * Kd_scale
    print("New stiffness: ", Ks)
    print("New damping: ", Kd)


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

time_list, vel_list = [], []
run = True
while run:
    pygame_controls()
    position = receive_udp()  # Receive position from UDP
    pr = np.asarray(position)

    # Transform coordinates
    pr[0] = pr[0]*800/600*1.4 -200
    pr[1] = pr[1]*600/400*1.4 -200
    # pr[0] = np.clip(pr[0], 0, window_width-EE_width)
    # pr[1] = np.clip(pr[1], 0, window_height-EE_height)

    er_prev = er
    er = np.subtract(pr, p)  # Error
    vel = (er - er_prev) / dt  # derr/dt



    dp_dt = (p - p_prev)/dt
    vel_ema = alpha * dp_dt + (1 - alpha) * vel_ema
    # dp_dt = np.clip(dp_dt, -1000, 1000)
    p_prev = p.copy()


    F_spring = -Ks @ er/100
    # F_damper = -(Kd @ derr_dt/100)
    F_damper = (Kd @ vel_ema / 100)
    F = F_spring + F_damper
    F = -F
    if n > 100:
        # print("velocity: ", derr_dt)
        # print("F_spring: ", F_spring)
        # print("F_damper: ", F_damper)
        # print("Force: ", F)
        n = 0
    n += 1


    # integration
    ddp = F / m
    dp += ddp * dt
    p += dp * dt
    t += dt

    p[0] = np.clip(p[0], 0, window_width-EE_width-5)
    p[1] = np.clip(p[1], 0, window_height-EE_height-5)

    collisions = check_collision(p, occupancy_grid, buffer=1)
    if len(collisions) == 0:
        Ks = Ks_initial.copy()
        Kd = Kd_initial.copy()


    processed_ids = set()
    for direction, (x, y, idd) in collisions.items():
        if str(int(idd)) in processed_ids:
            continue

        pg_rect = split_object_dict[str(int(idd))]['rect']
        tl_x, tl_y = pg_rect.topleft
        br_x, br_y = pg_rect.bottomright
        if direction in ["mid-left"]:
            p[0] = np.clip(p[0], br_x, window_width)
            dp[0] = 0
            print("clipping left", x)
            adjust_stifness(split_object_dict, idd)
            should_pop(idd, "mid-left")
            processed_ids.add(str(int(idd)))
            # p_prev = p.copy()

        elif direction in ["mid-right"]: #, "top-right", "bottom-right"
            p[0] = np.clip(p[0], 0, tl_x-EE_width)
            dp[0] = 0
            print("clipping right", x)
            adjust_stifness(split_object_dict, idd)
            should_pop(idd, "mid-right")
            processed_ids.add(str(int(idd)))
            # p_prev = p.copy()

        if direction in ["mid-top"]: #, "top-left", "top-right"
            p[1] = np.clip(p[1], br_y, window_height)
            dp[1] = 0
            print("clipping top", y)
            adjust_stifness(split_object_dict, idd)
            should_pop(idd, "mid-top")
            processed_ids.add(str(int(idd)))
            # p_prev = p.copy()

        elif direction in ["mid-bottom"]:
            p[1] = np.clip(p[1], 0, tl_y-EE_height)
            dp[1] = 0
            print("clipping bot", y)
            adjust_stifness(split_object_dict, idd)
            should_pop(idd, "mid-bottom")
            processed_ids.add(str(int(idd)))
            # p_prev = p.copy()
        
    vel_list.append(dp_dt)
    time_list.append(t)
    send_udp(F)
    render()
    pygame_controls()

    if run == False:
        plt.plot(time_list, vel_list)
        plt.show()
        break

pygame.quit() # stop pygame

