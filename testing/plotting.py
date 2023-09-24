import numpy as np
from matplotlib import pyplot as plt
import csv
import pandas as pd
from matplotlib.ticker import MultipleLocator



def plot_T_vs_I():
    data = pd.read_csv('/home/tuna/Documents/Quad/testing/Motor_testing_T_vs_I.csv')

    #print(data)

    T_real = list(pd.to_numeric(data['Unnamed: 0'][9:21]))
    print(f't_real: {T_real}')

    I1 = list(pd.to_numeric(data['Unnamed: 5'][9:21]))
    I2 = list(pd.to_numeric(data['Unnamed: 6'][9:21]))
    I3 = list(pd.to_numeric(data['Unnamed: 7'][9:21]))
    I4 = list(pd.to_numeric(data['Unnamed: 8'][9:21]))

    T_pred1 = list(pd.to_numeric(data['Unnamed: 11'][9:21]))
    T_pred2 = list(pd.to_numeric(data['Unnamed: 12'][9:21]))
    T_pred3 = list(pd.to_numeric(data['Unnamed: 13'][9:21]))
    T_pred4 = list(pd.to_numeric(data['Unnamed: 14'][9:21]))

    Pow1 = list(pd.to_numeric(data['Unnamed: 17'][9:21]))
    Pow2 = list(pd.to_numeric(data['Unnamed: 18'][9:21]))
    Pow3 = list(pd.to_numeric(data['Unnamed: 19'][9:21]))
    Pow4 = list(pd.to_numeric(data['Unnamed: 12'][9:21]))

    fig, ax = plt.subplots()

    plotting_current = np.linspace(0, 22, 100)
    predicted_torque = 8.27/90 * plotting_current

    I_avg = []

    for i in range(len(I1)):
        I_avg.append((I1[i] + I2[i] + I3[i] + I4[i])/4)

    I_avg_np = np.array(I_avg)
    T_real_np = np.array(T_real) 




    coeffs = np.polyfit(I_avg_np, T_real_np, 1)
    print(f'coeffs: {coeffs}')
    polynom = np.poly1d(coeffs)

    fit = polynom(I_avg_np)




    plt.plot(plotting_current, predicted_torque, color='#f00a02', linestyle='--')
    plt.plot(I_avg, fit, color='blue', linestyle='-')

    for i in range(len(I1)):
        plt.plot(I1[i], T_real[i], color='#2179fc', marker='x', markersize=4)
        plt.plot(I2[i], T_real[i], color='#2179fc', marker='x', markersize=4)
        plt.plot(I3[i], T_real[i], color='#2179fc', marker='x', markersize=4)
        plt.plot(I4[i], T_real[i], color='#2179fc', marker='x', markersize=4)


    print(f'I1: {I1}')
    print(I_avg)




    x_ticks = np.linspace(0, 22, 22)



    # Major ticks and grid
    ax.xaxis.set_major_locator(MultipleLocator(1))  # Major x-ticks spacing of 1
    ax.yaxis.set_major_locator(MultipleLocator(0.2))  # Major y-ticks spacing of 0.5

    # Minor ticks and grid
    ax.xaxis.set_minor_locator(MultipleLocator(0.5))  # Minor x-ticks spacing of 0.2 (creates additional grid lines between major ticks)
    ax.yaxis.set_minor_locator(MultipleLocator(0.1))  # Minor y-ticks spacing of 0.1

    # Display grid for both major and minor ticks
    ax.grid(which='major', linestyle='-', linewidth='0.5', alpha=0.9)
    ax.grid(which='minor', linestyle='--', linewidth='0.5', alpha=0.6)


    #plt.grid(True, which='both', axis='both')
    #fig.add_subplot(1, 1, 1).grid(which='major')
    #ax.set_xticks(x_ticks)
    plt.title('8308 Torque Output vs 3-Phase Current Input')
    plt.xlabel('Current [A]')
    plt.ylabel('Torque [Nm]')
    plt.legend(['"Ideal" T vs. I', 'Experimental T vs. I'])
    plt.show()


def plot_idle_cur_vs_mass_vs_gear():

    mass = np.linspace(10, 15, 100)
    gearRedux = np.linspace(5, 7, 5)
    print(gearRedux)

    leverArm = 0.11 #meter

    forcePerLeg = mass*9.81/4
    fig, ax = plt.subplots()
    for i in range(5):
        torquePerMotor = (forcePerLeg*leverArm)/gearRedux[i]
        currentPerMotor = (torquePerMotor - 0.28)/0.0555 #experimentally derived

        plt.plot(mass, currentPerMotor)

    plt.title('Current per motor vs mass of body given different gear ratios')
    plt.ylabel('current per motor [A]')
    plt.xlabel('mass of body [kg]')
    plt.legend(['5:1', '5.5:1', '6:1', '6.5:1', '7:1'])
    plt.grid()
    plt.show()



if __name__ == "__main__":
    plot_idle_cur_vs_mass_vs_gear()


