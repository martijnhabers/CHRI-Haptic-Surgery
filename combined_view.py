# -*- coding: utf-8 -*-
import sys
import math
import time
import numpy as np
import pygame

from Physics import Physics
from Graphics import Graphics

import matplotlib.pyplot as plt

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
        xc,yc = self.graphics.screenVR.get_rect().center
        ##############################################
        #ADD things here that you want to run at the start of the program!
        self.prev_xh = np.array([0,0])
        self.ema_vel = np.array([0,0])
        self.alpha = 0.1
        self.time = 0

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
        xc,yc = g.screenVR.get_rect().center
        g.erase_screen()
        ##############################################
        #ADD things here that run every frame at ~100 fps!
    


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
        k = 0.1 #spring constant
        b = 0.008 #damping constant

        # critical damping
        # b = 2 * math.sqrt(k)

        dt = 1.0 / g.FPS
        self.time+=dt

        vel = (xh - self.prev_xh) / dt

        self.ema_vel = self.alpha * vel + (1 - self.alpha) * self.ema_vel

        g.debug_text += str(np.round(self.ema_vel, 2))

        # only use damping if velocity is high enough
        # if np.linalg.norm(self.ema_vel) < 500:
        #     b = 0

        damp_x = self.ema_vel[0] * b
        damp_y = self.ema_vel[1] * b

        spring_x = (xh[0] - xc) * 0
        spring_y = (xh[1] - yc) * 0

        fx = spring_x + damp_x
        fy = spring_y + damp_y
        fe = np.array([fx, fy]) #fx,fy

        # print("damp_x: ", damp_x)
        # print("damp_y: ", damp_y)
        # print("spring_x: ", spring_x)
        # print("spring_y: ", spring_y)

        self.prev_xh = xh

        ##############################################
        if self.device_connected: #set forces only if the device is connected
            p.update_force(fe)
        else:
            xh = g.sim_forces(xh,fe,xm,mouse_k=0.5,mouse_b=0.8) #simulate forces with mouse haptics
            pos_phys = g.inv_convert_pos(xh)
            pA0,pB0,pA,pB,pE = p.derive_device_pos(pos_phys) #derive the pantograph joint positions given some endpoint position
            pA0,pB0,pA,pB,xh = g.convert_pos(pA0,pB0,pA,pB,pE) #convert the physical positions to screen coordinates
        g.render(pA0,pB0,pA,pB,xh,fe,xm, self.object_dict) #render the graphics
        
    def close(self):
        ##############################################
        #ADD things here that you want to run right before the program ends!
        ##############################################
        self.graphics.close()
        self.physics.close()

if __name__=="__main__":
    pa = PA()
    try:
        while True:
            pa.run()
    finally:
        pa.close()