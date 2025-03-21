#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Control in Human-Robot Interaction Assignment 2a: teleoperation & tele-impedance
-------------------------------------------------------------------------------
DESCRIPTION:
2-DOF planar robot arm model with shoulder and elbow joints. The code includes
simulation environment and visualisation of the robot.

Assume that the robot is a torque-controlled robot and its dynamics is
compensated internally, therefore the program operates at the endpoint in
Cartesian space. The input to the robot is the desired endpoint force and
the output from the robot is the measured endpoint position.

Important variables:
pm[0] -> mouse x position
pm[1] -> mouse y position
pr[0] -> reference endpoint x position
pr[1] -> reference endpoint y position
p[0] -> actual endpoint x position
p[1] -> actual endpoint y position
dp[0] -> endpoint x velocity
dp[1] -> endpoint y velocity
F[0] -> endpoint x force
F[1] -> endpoint y force

NOTE: Keep the mouse position inside the robot workspace before pressing 'e'
to start and then maintain it within the workspace during the operation,
in order for the inverse kinematics calculation to work properly.
-------------------------------------------------------------------------------


INSTURCTOR: Luka Peternel
e-mail: l.peternel@tudelft.nl

"""



import numpy as np
import math
import matplotlib.pyplot as plt
import pygame
import socket, struct





'''ROBOT MODEL'''

class robot_arm_2dof:
    def __init__(self, l):
        self.l = l # link length
    
    
    
    # arm Jacobian matrix
    def Jacobian(self, q):
        J = np.array([[-self.l[0]*np.sin(q[0]) - self.l[1]*np.sin(q[0] + q[1]),
                     -self.l[1]*np.sin(q[0] + q[1])],
                    [self.l[0]*np.cos(q[0]) + self.l[1]*np.cos(q[0] + q[1]),
                     self.l[1]*np.cos(q[0] + q[1])]])
        return J
    
    
    
    # inverse kinematics
    def IK(self, p):
        q = np.zeros([2])
        r = np.sqrt(p[0]**2+p[1]**2)
        q[1] = np.pi - math.acos((self.l[0]**2+self.l[1]**2-r**2)/(2*self.l[0]*self.l[1]))
        q[0] = math.atan2(p[1],p[0]) - math.acos((self.l[0]**2-self.l[1]**2+r**2)/(2*self.l[0]*r))
        
        return q






'''SIMULATION'''

# SIMULATION PARAMETERS
dt = 0.01 # intergration step timedt = 0.01 # integration step time
dts = dt*1 # desired simulation step time (NOTE: it may not be achieved)



# ROBOT PARAMETERS
x0 = 0.0 # base x position
y0 = 0.0 # base y position
l1 = 0.33 # link 1 length 0.33
l2 = 0.33 # link 2 length (includes hand) 0.33
l = [l1, l2] # link length


# SIMULATOR
# initialise robot model class
model = robot_arm_2dof(l)

# initialise real-time plot with pygame
pygame.init() # start pygame
window = pygame.display.set_mode((800, 600)) # create a window (size in pixels)
window.fill((255,255,255)) # white background
xc, yc = window.get_rect().center # window center
pygame.display.set_caption('robot arm')

font = pygame.font.Font('freesansbold.ttf', 12) # printing text font and font size
text = font.render('robot arm', True, (0, 0, 0), (255, 255, 255)) # printing text object
textRect = text.get_rect()
textRect.topleft = (10, 10) # printing text position with respect to the top-left corner of the window

clock = pygame.time.Clock() # initialise clock
FPS = int(1/dts) # refresh rate

# initial conditions
t = 0.0 # time
pm = np.zeros(2) # mouse position
pr = np.zeros(2) # reference endpoint position
p = np.array([0.1,0.1]) # actual endpoint position
dp = np.zeros(2) # actual endpoint velocity
F = np.zeros(2) # endpoint force
q = np.ones(2)*0.05 # joint position
p_prev = np.zeros(2) # previous endpoint position
m = 0.5 # endpoint mass
i = 0 # loop counter
state = [] # state vector
orientation = 0     # Stiffness frame orientation
er = np.zeros(2) # Error

# IMPEDANCE CONTROLLER PARAMETERS
Ks = np.diag([300,100]) # stiffness in the endpoint stiffness frame [N/m] 30
Kd = 2*np.sqrt(Ks * m) # damping in the endpoint stiffness frame [N/ms-1]

object_dict = {
            'skin': {'color': (186, 154, 127), 'rect': pygame.Rect(0, 100, 600, 50), 'force': 0},
            'bone': {'color': (255, 5, 127), 'rect': pygame.Rect(0, 200, 600, 50), 'force': 0},
            'heart': {'color': (255, 0, 0), 'rect': pygame.Rect(300, 350, 50, 50), 'force': 0},
        }

theta = 0.0 # roation of the endpoint stiffness frame wrt the robot base frame [rad]
stiffness_value_increment = 100 # for tele-impedance [N/m]
stiffness_angle_increment = 10 # for tele-impedance [rad]

# scaling
window_scale = 800 # conversion from meters to pixles

location_wall_robot = np.array([[0.1666, 0, 1]])
width_wall = 300
height_wall = 600

force_perturbation = 1 #set to 1 to turn on and to 0 to turn off

# Set up sockets
send_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # create a send socket
recv_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # create a receive socket
recv_sock.bind(("localhost", 40002)) # bind the socket to port 40002

# Send dummy data
force = np.array([0.0, 0.0])
send_data = bytearray(struct.pack("=%sf" % force.size, *force))  # convert array of 3 floats to bytes
send_sock.sendto(send_data, ("localhost", 40001))  # send to IP address 192.168.0.3 and port 40001


# wait until the start button is pressed
run = True
while run:
    for event in pygame.event.get(): # interrupt function
        if event.type == pygame.KEYUP:
            if event.key == ord('e'): # enter the main loop after 'e' is pressed
                run = False

# Ratios between the haptic screen and the
wd_ratio_x = 800/600
wd_ratio_y = 600/400
T = np.array([[1, 0, -xc],
              [0, -1, yc],
              [0, 0, window_scale]])/window_scale

T_haptic_to_robot = np.array([[wd_ratio_x,0, -xc],
                              [0, -wd_ratio_y, yc],
                              [0, 0, window_scale]])/window_scale

def transform_haptic_to_robot(pm):
    global T_haptic_to_robot
    if pm.ndim == 1:
        pm = np.expand_dims(pm, axis=1)
    # pm = np.vstack((pm, np.ones((1, pm.shape[1]))))
    pr = T_haptic_to_robot @ pm
    return pr

def transform_pygame_to_robot(pm):
    global T
    if pm.ndim == 1:
        pm = np.expand_dims(pm, axis=1)
    # pm = np.vstack((pm, np.ones((1, pm.shape[1]))))
    pr = T @ pm
    return pr

def transform_robot_to_pygame(pr):
    global T
    if pr.ndim == 1:
        pr = np.expand_dims(pr, axis=1)
    # pr = np.vstack((pr, np.ones((1, pr.shape[1]))))
    pm = np.linalg.inv(T) @ pr
    return pm

test = np.array([[5, 15, 1]]).T
tf = transform_pygame_to_robot(test)



def make_ellipse(ax0, ax1, s0, s1, center, color):
    angles = np.linspace(0, 2*np.pi, 50)
    points_circle = np.array([np.cos(angles)*0.02, np.sin(angles)*0.02, np.ones(50)])

    T_ellipse = np.array([[ax0[0]*s0, ax1[0]*s1, center[0]],
                  [ax0[1]*s0, -ax1[1]*s1, center[1]],
                  [0, 0, 1]])

    points_ellipse = np.matmul(T_ellipse, points_circle)

    points_ellipse_pygame = (transform_robot_to_pygame(points_ellipse)).astype(dtype=int)
    points_ellipse_pygame = points_ellipse_pygame.T[:, :2]

    # print("points", points_ellipse_pygame)

    pygame.draw.lines(window, color, True, points_ellipse_pygame, 2)

def make_stiffness_ellipse(R, center, scale):
    angles = np.linspace(0, 2 * np.pi, 50)
    points_circle = np.array([np.cos(angles) * 0.02, np.sin(angles) * 0.02, np.ones(50)])

    T_ellipse = np.array([[R[0,0] * scale, R[0,1] * scale, center[0]],
                          [R[1,0] * scale, R[1,1] * scale, center[1]],
                          [0, 0, 1]])

    points_ellipse = np.matmul(T_ellipse, points_circle)
    points_ellipse_pygame = (transform_robot_to_pygame(points_ellipse)).astype(dtype=int)
    points_ellipse_pygame = points_ellipse_pygame.T[:, :2]

    pygame.draw.lines(window, (200, 200, 50), True, points_ellipse_pygame, 2)


# MAIN LOOP
i = 0
n = 0
run = True
while run:
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



            '''*********** Student should fill in ***********'''

    # Receive Force from UDP
    recv_data, address = recv_sock.recvfrom(12)  # receive data with buffer size of 12 bytes
    position = struct.unpack("2f", recv_data)  # convert the received data from bytes to an array of 3 floats (assuming force in 3 axes)
    print("Received from address: ", address)
    print("Received position: ", position)
    position = np.asarray(position)
    if position.ndim == 1:
        position = np.expand_dims(position, axis=1)
    position = np.vstack((position, np.ones((1, position.shape[1]))))
    pr = transform_haptic_to_robot(position)


    '''*********** Student should fill in ***********'''
    # main control code
    pm = np.array(pygame.mouse.get_pos())

    if pm.ndim == 1:
        pm = np.expand_dims(pm, axis=1)
    pm = np.vstack((pm, np.ones((1, pm.shape[1]))))

    # pr = transform_pygame_to_robot(pm)

    pr = pr[:2,0].T
    pm = pm[:2,0].T

    pr = pr.T
    print("Transformed to: ", pr)
    np.clip(pr, np.array([-0.2, -0.2]), np.array([0.2,0.2]))
    print("Transformed to: ", pr)
    pm = pr
    print("----")
    print("p ", p)
    print("pr ", pr)


    er_prev = er
    er = np.subtract(pr, p) # Error
    derr_dt = (er-er_prev)/dts #derr/dt
    # derr_dt = dp

    orientation_rad = np.copy(np.radians(orientation))
    R = np.array([[np.cos(orientation_rad), -np.sin(orientation_rad)],
                  [np.sin(orientation_rad), np.cos(orientation_rad)]])

    Ks_rot = R @ np.copy(Ks) @ R.T
    Kd_rot = R @ np.copy(Kd) @ R.T


    # print("Rotaded Ks", Ks)

    F_spring = Ks_rot @ er
    F_damper = Kd_rot @ derr_dt
    F_perturbation = np.array([0, 10*np.sin(t)])
    # F_perturbation = np.array([0,0]) #TODO:Remove







    # F = F_spring + F_damper
    F = np.sum([F_spring, F_damper, F_perturbation], axis=0)

    print(F_spring)

    force = F
    send_data = bytearray(struct.pack("=%sf" % force.size, *force))  # convert array of 3 floats to bytes
    send_sock.sendto(send_data, ("localhost", 40001))  # send to IP address 192.168.0.3 and port 40001



    # print("Orientation", orientation)

    J = model.Jacobian(q)
    J_transpose_inverse = np.linalg.inv(J).T

    U, S, Vh = np.linalg.svd(J_transpose_inverse)


    scale = 10
    ax0 = U[:,0]
    ax1 = U[:,1]
    s0 = S[0]
    s1 = S[1]
    scaled_axes = (int(ax0[0] * scale), int(ax0[1] * scale))
    '''*********** Student should fill in ***********'''



	# previous endpoint position for velocity calculation
    p_prev = p.copy()

    # log states for analysis
    state.append([t, pr[0], pr[1], p[0], p[1], dp[0], dp[1], F[0], F[1], Ks[0,0], Ks[1,1]])  #Ks was K in the initial template
    
    # integration
    ddp = F/m
    dp += ddp*dt
    p += dp*dt
    t += dt

    if p[0] > 0.1666:
        dp[0] = 0
        p[0] = np.clip(p[0], -5, 0.1666)
    
    '''*********** Student should fill in ***********'''
    # simulate a wall
    wall_location_pygame = transform_robot_to_pygame(location_wall_robot.T).astype(dtype=int)
    '''*********** Student should fill in ***********'''

    # increase loop counter
    i = i + 1
    
    
    
    # update individual link position
    q = model.IK(p)
    x1 = l1*np.cos(q[0])
    y1 = l1*np.sin(q[0])
    x2 = x1+l2*np.cos(q[0]+q[1])
    y2 = y1+l2*np.sin(q[0]+q[1])
    
    # real-time plotting
    window.fill((255,255,255)) # clear window
    '''*********** Student should fill in ***********'''
    # draw a wall
    pygame.draw.rect(window, (100, 100, 100), (wall_location_pygame[0, 0], wall_location_pygame[1, 0]-300, width_wall, height_wall))

    for key in object_dict:
        pygame.draw.rect(window, object_dict[key]['color'], object_dict[key]['rect'])
        
    '''*********** Student should fill in ***********'''
    pygame.draw.circle(window, (0, 255, 0), (pm[0], pm[1]), 5) # draw reference position
    pygame.draw.lines(window, (0, 0, 255), False, [(window_scale*x0+xc,-window_scale*y0+yc), (window_scale*x1+xc,-window_scale*y1+yc), (window_scale*x2+xc,-window_scale*y2+yc)], 6) # draw links
    pygame.draw.circle(window, (0, 0, 0), (window_scale*x0+xc,-window_scale*y0+yc), 9) # draw shoulder / base
    pygame.draw.circle(window, (0, 0, 0), (window_scale*x1+xc,-window_scale*y1+yc), 9) # draw elbow
    pygame.draw.circle(window, (255, 0, 0), (window_scale*x2+xc,-window_scale*y2+yc), 5) # draw hand / endpoint
    
    force_scale = 50/(window_scale*(l1*l1)) # scale for displaying force vector
    pygame.draw.line(window, (0, 255, 255), (window_scale*x2+xc,-window_scale*y2+yc), ((window_scale*x2+xc)+F[0]*force_scale,(-window_scale*y2+yc-F[1]*force_scale)), 2) # draw endpoint force vector
    
    '''*********** Student should fill in ***********'''
    # Make and plot an ellipse showing the force manipulability
    pc = p.copy()
    if pc.ndim == 1:
        pc = np.expand_dims(pc, axis=1)
    pc = np.vstack((pc, np.ones((1, pc.shape[1]))))
    make_ellipse(ax0, ax1, s0, s1, pc[:2, 0], (255, 0, 0))

    # Make and plot an ellipse showing stiffness
    # s0 = R @ Ks @ np.array([0,1])
    # s1 = R @ Ks @ np.array([1, 0])
    # print(s0)
    # print(s1)
    # sc = np.array([s0,s1])
    # sc = np.sum(abs(sc), axis=0)
    # print(sc)
    ax0_rot = np.dot(R, np.array([0,1]))
    ax1_rot = np.dot(R, np.array([1,0]))
    # print(ax0_rot, ax1_rot)


    if not np.allclose(np.diag([1,1]),R.T@R) and np.isclose(1.0, np.linalg.det(R)):
        print("Rotation matrix fucked")

    if not (np.isclose(np.linalg.norm(ax0_rot), 1.0) and np.isclose(np.linalg.norm(ax1_rot), 1.0)):
        print("non unit vector")

    # make_ellipse(ax0_rot, ax1_rot, 5, 5, pc[:2, 0], (0, 255, 0))


    # pygame.draw.line(window, (255,0,0), (300, 300), [400,400]*ax0_rot, 3)

    make_stiffness_ellipse(Ks_rot, pc[:2, 0], 0.02)

    '''*********** Student should fill in ***********'''

    # print data
    text = font.render("FPS = " + str( round( clock.get_fps() ) ) + "   K = " + str( [np.round(Ks[0,0],3),np.round(Ks[1,1],3)] ) + " N/m" + "   x = " + str( np.round(p,3) ) + " m" + "   x_r = " + str(np.round(pr,3) ) + " m" +"   F = " + str( np.round(F,3) ) + " N", True, (0, 0, 0), (255, 255, 255)) #Ks was k in original template
    window.blit(text, textRect)
    
    pygame.display.flip() # update display
    
    
    
    # try to keep it real time with the desired step time
    clock.tick(FPS)
    
    if run == False:
        break

pygame.quit() # stop pygame












'''ANALYSIS'''

state = np.array(state)


plt.figure(3)
plt.subplot(411)
plt.title("VARIABLES")
plt.plot(state[:,0],state[:,1],"b",label="x")
plt.plot(state[:,0],state[:,2],"r",label="y")
plt.legend()
plt.ylabel("pr [m]")

plt.subplot(412)
plt.plot(state[:,0],state[:,3],"b")
plt.plot(state[:,0],state[:,4],"r")
plt.ylabel("p [m]")

plt.subplot(413)
plt.plot(state[:,0],state[:,7],"b")
plt.plot(state[:,0],state[:,8],"r")
plt.ylabel("F [N]")

plt.subplot(414)
plt.plot(state[:,0],state[:,9],"c")
plt.plot(state[:,0],state[:,10],"m")
plt.ylabel("K [N/m]")
plt.xlabel("t [s]")

plt.tight_layout()




plt.figure(4)
plt.title("ENDPOINT BEHAVIOUR")
plt.plot(0,0,"ok",label="shoulder")
plt.plot(state[:,1],state[:,2],"lime",label="reference")
plt.plot(state[:,3],state[:,4],"r",label="actual")
plt.axis('equal')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend()

plt.tight_layout()




