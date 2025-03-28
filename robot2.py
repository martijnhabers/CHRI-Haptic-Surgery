import random
import numpy as np
import math
import matplotlib.pyplot as plt
import pygame
import socket, struct

#
dt = 0.01  # intergration step timedt = 0.01 # integration step time

pygame.init()  # start pygame
window = pygame.display.set_mode((800, 600))  # create a window (size in pixels)
window.fill((255, 255, 255))  # white background
xc, yc = window.get_rect().center  # window center
pygame.display.set_caption('robot arm')

font = pygame.font.Font('freesansbold.ttf', 12)  # printing text font and font size
text = font.render('robot arm', True, (0, 0, 0), (255, 255, 255))  # printing text object
textRect = text.get_rect()
textRect.topleft = (10, 10)  # printing text position with respect to the top-left corner of the window
EE_width, EE_height = 5, 5

clock = pygame.time.Clock()  # initialise clock
FPS = int(1 / dt)  # refresh rate

# Initial conditions
t = 0.0  # time

pr = np.zeros(2)  # reference endpoint position
p = np.array([500, 500])  # actual endpoint position
p_potential = None
p_prev = np.zeros(2)  # previous endpoint position
dp = np.zeros(2)  # actual endpoint velocity
F = np.zeros(2)  # endpoint force

m = 0.5  # endpoint mass
i = 0  # loop counter
state = []  # state vector
orientation = 0  # Stiffness frame orientation
er = np.zeros(2)  # Error

# IMPEDANCE CONTROLLER PARAMETERS
Ks = np.diag([5000, 5000])  # stiffness in the endpoint stiffness frame [N/m] 30
Kd = 2 * np.sqrt(Ks * m)  # damping in the endpoint stiffness frame [N/ms-1]

window_width = 800
window_height = 600

offset = 200



object_dict = {
    # format: 'name': {'color': (R, G, B), 'rect': pygame.Rect(x_tl, y_tl, width, height), 'force': breaking force}
    'heart': {'color': (255, 0, 0), 'rect': pygame.Rect(300, 300 + offset, 75, 125), 'force': 25},
    'heart2': {'color': (255, 0, 0), 'rect': pygame.Rect(450, 300 + offset, 75, 125), 'force': 25},
    'tumor': {'color': (255, 120, 255), 'rect': pygame.Rect(425, 350 + offset, 50, 50), 'force': 5},

    'skin': {'color': (186, 154, 127), 'rect': pygame.Rect(0, offset, window_width, 250), 'force': 150},
    'bone': {'color': (240, 240, 240), 'rect': pygame.Rect(0, 250 + offset, window_width, 50), 'force': 300},
    'tissue': {'color': (255, 182, 193), 'rect': pygame.Rect(0, 300 + offset, window_width, 50), 'force': 0},
    'muscle': {'color': (96, 5, 33), 'rect': pygame.Rect(0, 350 + offset, window_width, 50), 'force': 0},
}

materials = {
    "heart" : {"color": (255, 0, 0), "force": 25},
    "tumor" : {"color": (255, 120, 255), "force": 5},
    "skin" : {"color": (186, 154, 127), "force": 150},
    "bone" : {"color": (240, 240, 240), "force": 400},
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
        height = random.randrange(50, 150, 25)  # Random height for the layer
        rect = pygame.Rect(0, y_height, window_width, height)
        y_height += height
        object_dict[layer_name] = {'color': color, 'rect': rect, 'force': force}
    return object_dict

# Generate random object configurations, for training
object_dict = generate_random_object_configurations(8)

# object dict one big square in the middle
# object_dict = {
#     'heart': {'color': (255, 0, 0), 'rect': pygame.Rect(300, 300, 100, 100), 'force': 500},
# }
# Resolution in X
x_steps = 25
y_steps = 25


def generate_objects(object_dict, x_res, y_res):
    dict = {}
    i = 0
    occupied_pixels = set()
    for key in object_dict:
        rect = object_dict[key]['rect']
        color = object_dict[key]['color']
        force = object_dict[key]['force']
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
                    dict[str(int(i))] = {'color': color, 'rect': rect, 'force': force}
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

def in_collision_with_grid(pr):
    if pr[0] < 0 or pr[0] > 800 - EE_width - 1 or pr[1] < 0 or pr[1] > 600 - EE_height - 1:
        return False
    buffer = 0
    if not (occupancy_grid[pr[0], pr[1]] or occupancy_grid[pr[0] + EE_width - buffer, pr[1]] or occupancy_grid[
        pr[0], pr[1] + EE_height - buffer] or occupancy_grid[pr[0] + EE_width - buffer, pr[1] + EE_height - buffer]):
        return False
    else:
        return True

def pygame_controls():
    for event in pygame.event.get():  # interrupt function
        if event.type == pygame.QUIT:  # force quit with closing the window
            run = False
        elif event.type == pygame.KEYUP:
            if event.key == ord('q'):  # force quit with q button
                run = False
            '''*********** Student should fill in ***********'''
            # tele-impedance interface / control mode switch
            if event.key == ord('s'):  # switch mode
                pass
            # global orientation, Ks, Kd

            if event.key == ord('x'):
                Ks[0, 0] += 100  # Increase x stiffness
                Ks[0, 0] = np.clip(Ks[0, 0], 0, 1000)
            elif event.key == ord('z'):
                Ks[0, 0] -= 100  # Increase x stiffness
                Ks[0, 0] = np.clip(Ks[0, 0], 0, 1000)
            elif event.key == ord('d'):
                Ks[1, 1] += 100  # Increase y stiffness
                Ks[1, 1] = np.clip(Ks[1, 1], 0, 1000)
            elif event.key == ord('c'):
                Ks[1, 1] -= 100  # Decrease y stiffness
                Ks[1, 1] = np.clip(Ks[1, 1], 0, 1000)
            elif event.key == ord('a'):
                orientation += 10  # Increase orientation
                orientation = np.clip(orientation, -90, 90)
            elif event.key == ord('s'):
                orientation -= 10  # Decrease orientation
                orientation = np.clip(orientation, -90, 90)
            elif event.key == ord('q'):
                # quit the program and send a message to the robot to stop
                run = False

            Kd = 2 * np.sqrt(Ks * m)  # damping in the endpoint stiffness frame [N/m-1]

def render():
    window.fill((255, 255, 255))  # clear window

    for key in split_object_dict:
        pygame.draw.rect(window, split_object_dict[key]['color'], split_object_dict[key]['rect'])

    pygame.draw.rect(window, (0, 0, 0), (p[0], p[1], EE_width, EE_height))

    if p_potential is not None:
        pygame.draw.rect(window, (0, 255, 0), (p_potential[0], p_potential[1], EE_width, EE_height))

    if force_breaking_vector is not None:
        pygame.draw.rect(window, (255, 0, 0), (p[0] + force_breaking_vector[0], p[1] + force_breaking_vector[1], EE_width, EE_height))

    # draw a small square at position "pr"
    pygame.draw.rect(window, (0, 0, 255), (pr[0], pr[1], EE_width, EE_height))

    pygame.display.flip()


def receive_udp():
    recv_data, address = recv_sock.recvfrom(12)  # receive data with buffer size of 12 bytes
    position = struct.unpack("2f", recv_data)  # convert the received data from bytes to an array of 3 floats (assuming force in 3 axes
    return position

def send_udp(force):
    # Send Force over UDP
    force = np.array(F)
    send_data = bytearray(struct.pack("=%sf" % force.size, *force))  # convert array of 3 floats to bytes
    send_sock.sendto(send_data, ("localhost", 40001))  # send to IP address 192.168.0.3 and port 40001

def bresenham_line(x0, y0, x1, y1):
    """Bresenham's Line Algorithm to generate points on a line."""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    return points


def has_line_of_sight(p, pr, occupancy_grid):
    """Check if there is a line of sight between p and pr."""
    line_points = bresenham_line(p[0], p[1], pr[0], pr[1])
    for (x, y) in line_points:
        if occupancy_grid[x, y] != 0:
            return False
    return True

def calculate_forces(pr,p,dp,Ks,Kd):
    spring_force = - np.dot(Ks, (pr - p))
    damping_force = - np.dot(Kd, (np.zeros(2) - dp))
    total_force = spring_force + damping_force
    total_force[0] = - total_force[0]  # Invert x force
    return total_force

# Set up sockets, UDP communication
send_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # create a send socket
recv_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # create a receive socket
recv_sock.bind(("localhost", 40002)) # bind the socket to port 40002

# Send dummy data, to initialize the connection
force = F
send_data = bytearray(struct.pack("=%sf" % force.size, *force))  # convert array of 3 floats to bytes
send_sock.sendto(send_data, ("localhost", 40001))  # send to IP address 192.168.0.3 and port 40001

run = True
while run:
    pygame_controls()
    position = receive_udp()

    # Set position if no collision
    prev_p = p
    pr = np.asarray(position)
    # Transform coordinates
    pr[0] *= 800 / 600
    pr[1] *= 600 / 400
    pr = np.int32(pr)

    # Find spring and damping forces
    F = calculate_forces(pr, p, dp, Ks, Kd) / 1000

    # boolean is first collision

    if not in_collision_with_grid(pr) and has_line_of_sight(p, pr, occupancy_grid):
        #### IF NO COLLISION ####
        p = pr
        p_potential = None
        force_breaking_vector = None
    else:
        #### IF COLLISION ####
        err = pr - p
        p_pot = p + np.int32(err / 10)
        p_pot_dx = np.array([p_pot[0], p[1]])
        p_pot_dy = np.array([p[0], p_pot[1]])
        # print(p_pot_dx)
        if not in_collision_with_grid(p_pot):
            p = p_pot
        elif not in_collision_with_grid(p_pot_dx):
            p = p_pot_dx
        elif not in_collision_with_grid(p_pot_dy):
            p = p_pot_dy
        else:
            p = prev_p

        # normalized direction between pr and p
        direction = err / np.linalg.norm(err)

        # check with added buffer in direction of collision
        p_potential = p + np.int32(1.5*(x_steps/EE_width)*(direction / np.linalg.norm(direction)))

        key_to_pop = str(int(occupancy_grid[p_potential[0], p_potential[1]]))
        # check if key exists in split_object_dict
        if key_to_pop in split_object_dict:
            pg_rect = split_object_dict[key_to_pop]['rect']
            breaking_force = split_object_dict[key_to_pop]['force']

            # distance_to_break
            distance_to_break = breaking_force / Ks[0,0]
            force_breaking_vector = np.array([distance_to_break, distance_to_break])
            force_breaking_vector = np.diag(force_breaking_vector) @ direction

            print("Breaking force: ", breaking_force)
            print("Force: ", round(np.linalg.norm(Ks @ direction)))

            if np.linalg.norm(F) > breaking_force:
                tl_x, tl_y = pg_rect.topleft
                br_x, br_y = pg_rect.bottomright
                occupancy_grid[tl_x:br_x, tl_y:br_y] = 0
                split_object_dict.pop(key_to_pop)

    send_udp(F)
    render()

    if run == False:
        break

pygame.quit()  # stop pygame
