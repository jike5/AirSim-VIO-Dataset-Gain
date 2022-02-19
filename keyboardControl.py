import airsim
import numpy as np
import time
import keyboard  # using module keyboard (install using pip3 install keyboard)
 
vx=0
vy=0
vz=0
 
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
 
 
while True:  # making a loop
    try:  # used try so that if user pressed other than the given key error will not be shown
        if keyboard.is_pressed('w'):  # if key 'q' is pressed 
            vx=5
        elif keyboard.is_pressed('s'):
            vx=-5
        else:
            vx=0
        if keyboard.is_pressed('d'):  # if key 'q' is pressed 
            vy=5
        elif keyboard.is_pressed('a'):
            vy=-5
        else:
            vy=0
        if keyboard.is_pressed('space'):
            vz=-5
        else:
            vz=5            
        print("vx" + str(vx) + " vy " + str(vy) + " vz " + str(vz))
        client.moveByVelocityAsync(vx,vy,vz,12)
    except:
        break  # if user pressed a key other than the given key the loop will break