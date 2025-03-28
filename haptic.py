# -*- coding: utf-8 -*-
import sys
import math
import time
import numpy as np
import pygame
from matplotlib import pyplot as plt
from Physics import Physics
from Graphics import Graphics
import socket, struct
import os



#Hints:
#1: You can create your own persistant variables in the __init__() or run() functions by prefacing them with "self".
#     - For example: "self.prev_xh = xh"
#2: Change the values of "fe" and those forces will be sent to the device or simulated if no device is connected
#     - Note that this occurs AT THE END OF THE RUN FUNCTION
#     - fe is in SI units
#     - The Haply device can only exert a limited amount of force, so at some point setting very high forces will be useless
#3: Graphical components are all encapsulated in self.graphics (variable "g" in the run() function)
#     - These include the screenHaptics and screenVR surfaces which are the right and left display screens respectively
#     - Thus drawing a line on the right screen would be:
#            "pygame.draw.lines(g.screenHaptics, (0,255,0), False,[(0,0),(400,400)],1)"
#     - The orange proxy object is "g.haptic" and has an initial size of 48x48
#           - g.haptic has several parameters that can be read or updated such as:
#                 - haptic.w and haptic.h: haptic proxy rectangle width and height
#                 - haptic.center: haptic proxy center position
#4: Graphics contains two conversion functions to convert from and to SI units
#     - "g.convert_pos( (x,y) )" converts from the haply's physical coordinates to the graphical coordinates. Remember that the axes may not be aligned the same way!
#     - "g.inv_convert_pos( (x,y) )" converts from the graphical coordinates to the haply's physical coordinates
#     - Both functions can take in a single or multiple point written as a tuple or list with two elements
#           - point = g.convert_pos( (x,y) )
#           - p0,p1 = g.convert_pos( (x0,y0),(x1,y1) )
#     - For simple scale conversion, use the pixels-per-meter value, "g.window_scale"
#5: Other useful parameters of graphics
#     - The framerate of the graphics rendering can be accessed (and changed if you really want) via the "g.FPS" parameter
#     - "g.debug_text" is the debug string that will be drawn in the upper left. This string can be added to for debug purposes
#6: If you want to use pygame's collision detection ( for example, colliderect() ) while also changing the shape of the haptic proxy to represent impedance,
#       consider using the equivalent haptic proxy on the screenHaptic side (g.effort_cursor), as it is the same size as g.haptic and is at the same position.

class PA:
    def __init__(self):
        self.physics = Physics(hardware_version=3) #setup physics class. Returns a boolean indicating if a device is connected
        self.device_connected = self.physics.is_device_connected() #returns True if a connected haply device was found
        self.graphics = Graphics(self.device_connected) #setup class for drawing and graphics.
        #  - Pass along if a device is connected so that the graphics class knows if it needs to simulate the pantograph
        # xc,yc = self.graphics.screenVR.get_rect().center
        ##############################################
        #ADD things here that you want to run at the start of the program!
        xc, yc = 0,0
        self.k_virt = 0.05  #0.1
        self.b_virt = 0.001 #0.001
        self.screen_center = np.array([xc, yc])
        self.previous_xh = np.array([0, 0])
        self.previous_t = time.time()
        self.previous_v = 0
        self.n = 0
        self.x_list = []
        self.v_list = []
        self.t_list = []
        self.f_damper_list = []
        self.t0 = time.time()
        self.t = 0


        # Initialize Sockets
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # create a receive socket
        self.recv_sock.bind(("localhost", 40001))  # bind the socket to port 40002
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # create a send socket

        # Send dummy data
        position = np.array([100, 100])
        send_data = bytearray(struct.pack("=%sf" % position.size, *position))  # convert array of 3 floats to bytes
        self.send_sock.sendto(send_data, ("localhost", 40002))  # send to IP address 192.168.0.3 and port 40001


        self.object_dict = {
            'skin': {'color': (186, 154, 127), 'rect': pygame.Rect(0, 100, 600, 50), 'force': 0},
            'bone': {'color': (255, 5, 127), 'rect': pygame.Rect(0, 200, 600, 50), 'force': 0},
            'heart': {'color': (255, 0, 0), 'rect': pygame.Rect(300, 350, 50, 50), 'force': 0},
        }

        ##############################################
    
    def run(self):
        p = self.physics #assign these to shorthand variables for easier use in this function
        g = self.graphics
        #get input events for both keyboard and mouse
        keyups,xm = g.get_events()
        #  - keyups: list of unicode numbers for keys on the keyboard that were released this cycle
        #  - pm: coordinates of the mouse on the graphics screen this cycle (x,y)      
        #get the state of the device, or otherwise simulate it if no device is connected (using the mouse position)
        if self.device_connected:
            pA0,pB0,pA,pB,pE = p.get_device_pos() #positions of the various points of the pantograph
            pA0,pB0,pA,pB,xh = g.convert_pos(pA0,pB0,pA,pB,pE) #convert the physical positions to screen coordinates
        else:
            xh = g.haptic.center
            #set xh to the current haptic position, which is from the last frame.
            #This previous position will be compared to the mouse position to pull the endpoint towards the mouse

        fe = np.array([0.0,0.0]) #fx,fy
        xh = np.array(xh) #make sure fe is a numpy array
        # xc,yc = g.screenVR.get_rect().center
        xc, yc = 0,0
        g.erase_screen()
        ##############################################
        #ADD things here that run every frame at ~100fps!
        for key in keyups:
            if key==ord("q"): #q for quit, ord() gets the unicode of the given character
                sys.exit(0) #raises a system exit exception so the "PA.close()" function will still execute
            if key == ord('m'): #Change the visibility of the mouse
                pygame.mouse.set_visible(not pygame.mouse.get_visible())
            if key == ord('r'): #Change the visibility of the linkages
                g.show_linkages = not g.show_linkages
            if key == ord('d'): #Change the visibility of the debug text
                g.show_debug = not g.show_debug
            #you can add more if statements to handle additional key characters


        position = xh

        xh = xh  # / g.window_scale
        dt = time.time() - self.previous_t
        dt = 1/100
        vel = ((xh - self.previous_xh)) / dt
        self.t += dt

        self.previous_xh = xh
        self.previous_t = time.time()
        self.previous_v = vel

        self.x_list.append([xh[0], xh[1]])
        self.v_list.append([vel[0], vel[1]])
        self.t_list.append(time.time() - self.t0)

        f_damper = vel * self.b_virt


        # Send position data and receive Force data
        recv_data, address = self.recv_sock.recvfrom(12)  # receive data with buffer size of 12 bytes
        force = struct.unpack("2f", recv_data)  # convert the received data from bytes to an array of 3 floats (assuming force in 3 axes)
        print("Received data from address: ", address)
        print("Force: ", force)
        send_data = bytearray(struct.pack("=%sf" % position.size, *position))  # convert array of 3 floats to bytes
        self.send_sock.sendto(send_data, ("localhost", 40002))  # send to IP address 192.168.0.3 and port 40001



        # pygame.draw.line(self.graphics.screenHaptics, (0,0,0), [0,0], [50,50]*int(fe/30), 5)



        fe = np.array(force)/3000
        fe = np.clip(fe, -3, 3)

        ##############################################
        if self.device_connected: #set forces only if the device is connected
            p.update_force(fe)
        else:
            xh = g.sim_forces(xh,fe,xm,mouse_k=0.5,mouse_b=0.8) #simulate forces with mouse haptics
            pos_phys = g.inv_convert_pos(xh)
            pA0,pB0,pA,pB,pE = p.derive_device_pos(pos_phys) #derive the pantograph joint positions given some endpoint position
            pA0,pB0,pA,pB,xh = g.convert_pos(pA0,pB0,pA,pB,pE) #convert the physical positions to screen coordinates
        g.render(pA0,pB0,pA,pB,xh,fe,xm, self.object_dict)
        
    def close(self):
        ##############################################
        #ADD things here that you want to run right before the program ends!
        
        ##############################################
        self.graphics.close()
        self.physics.close()

if __name__=="__main__":
    os.environ['SDL_VIDEO_WINDOW_POS'] = "50,50"
    pa = PA()
    try:
        while True:
            pa.run()
    finally:
        pa.close()





