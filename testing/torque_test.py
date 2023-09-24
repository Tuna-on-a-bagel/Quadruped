import numpy as np
from matplotlib import pyplot as plt
import sys
import odrive as od







###NOTE: MUST RECALIBRATE ENCODER OFFSET!!!!!!!!!!                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
# Done @ 12:29



region = []
def mouse_event(event):

    x_idx = nearestIdx(time, event.xdata) #find index of nearest value in x-axis list
    region.append(x_idx)
    print(f'stored point: {len(region)}')
   

def nearestIdx(lis, value):

    diff = [abs(x - value) for x in lis]    #list of differences

    idx = diff.index(min(diff))

    return idx




current = np.linspace(1, 21, 200)
torque = 8.27*current / 90




#plt.plot(current, torque)
#plt.legend(['analytical output'])
#plt.show()


odrv0 = od.find_any()

#Notes:
# AxisState.IDLE = 1
# AxisState.MOTOR_CALIBRATION = 4
# AxisiSate.CLOSED_LOOP_CONTROL = 8

print(odrv0.axis0.current_state)

#
print(od.utils.dump_errors(odrv0))

odrv0.clear_errors()
#odrv0.config.brake_resistor0.enable = True
#odrv0.save_configuration()
#odrv0.axis0.config.motor.enable_brake_resistor0 = True

print(dir(odrv0.axis0.motor))
print(odrv0.axis0.motor.input_iq)

odrv0.axis0.requested_state = 8

power = []
current = []
T_est = []
for i in range(12000):
    power.append(round(odrv0.axis0.motor.electrical_power, 4))
    #current.append(round(odrv0.axis0.motor.input_id, 4))
    T_est.append(round(odrv0.axis0.motor.torque_estimate, 4))
    I_input = T_est[i] * 90/8.27
    current.append(I_input)

odrv0.axis0.requested_state = 1


time = np.linspace(1, 12, len(power))
p = np.array(power)
I = np.array(current)
T = np.array(T_est)

fig = plt.figure()
cid = fig.canvas.mpl_connect('button_press_event', mouse_event)

plt.plot(time, p)
plt.plot(time, I)
plt.plot(time, T)
plt.grid(True)
plt.legend(['El power [W]', 'I [A]', 'Torque Est. [Nm]'])
plt.xlabel('time')
plt.ylabel('P, I, T')
plt.show()


avg_I = sum(current[region[0]:region[1]])/(region[1] - region[0])
avg_P = sum(power[region[0]:region[1]])/(region[1] - region[0])
avg_est_T = sum(T_est[region[0]:region[1]])/(region[1] - region[0])

print(f'avg_I: {abs(avg_I)}')
print(f'avg estimated torque:; {abs(avg_est_T)}')
print(f'avg electrical power: {abs(avg_P)}')





'''
#print(str(odrv0.vbus_voltage))

#print(dir(odrv0))

#print(odrv0.axis0.foc.Iq_setpoint)

'''