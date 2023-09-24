import numpy as np
from matplotlib import pyplot as plt
import sys
import odrive as od
import time




## Communications params:
def checkCommSettings(odrv):
    print(f'CAN enabled: {odrv.config.enable_can_a}')
    print(f'CAN node ID: {odrv.axis0.config.can.node_id}')
    print(f'Baud-rate (same as bit rate): {odrv.can.config.baud_rate}')

def updateCommSettings(odrv, 
                       CAN_enabled=True,
                       CAN_node_ID=None,
                       CAN_baud_bit_rate=None):
    
    if CAN_enabled: odrv.config.enable_can_a = True
    else: odrv.config.enable_can_a = False
    
    if CAN_node_ID: odrv.axis0.config.can.node_id = CAN_node_ID
    
    if CAN_baud_bit_rate: odrv.can.config.baud_rate = CAN_baud_bit_rate
    

    odrv.save_configuration()           #save configuration
    time.sleep(3.0)                     #wait for reboot
    odrv = od.find(odrv)                #re-aquire
    checkCommSettings(odrv)             #verify results

def pingCAN():
    return


if __name__ == "__main__":
    odrv0 = od.find_any()
    checkCommSettings(odrv0)




'''
print(odrv0.config.dc_max_positive_current)
#odrv0.axis0.config.motor.current_soft_max = 21.5
#odrv0.axis0.config.motor.current_hard_max = 22.0
print(odrv0.axis0.config.motor.current_soft_max)
print(odrv0.axis0.config.motor.current_hard_max)

#odrv0.axis0.controller.config.pos_gain = 180.0
#odrv0.axis0.controller.config.vel_gain = 0.25
#odrv0.axis0.controller.config.vel_integrator_gain = 0.3
print(odrv0.axis0.controller.config.vel_gain)
print(odrv0.axis0.controller.config.vel_integrator_gain)
#odrv0.save_configuration()
'''


'''
odrv0.config.dc_max_positive_current = 25.0

odrv0.axis0.config.motor.current_soft_max = 20.0
odrv0.axis0.config.motor.current_hard_max = 21.0
odrv0.save_configuration()


def mouse_event(event):
    print('x: {} and y: {}'.format(event.xdata, event.ydata))
    region.append(event.xdata)

fig = plt.figure()

region = []


cid = fig.canvas.mpl_connect('button_press_event', mouse_event)




x = np.linspace(-10, 10, 100)
y = np.sin(x)

plt.plot(x, y)

plt.show()
fig.canvas.mpl_disconnect(cid)
'''