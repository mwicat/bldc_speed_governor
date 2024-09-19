
#
# Ported to Tkinter toolkit from Jupyter Notebook code found at
# https://apmonitor.com/pdc/index.php/Main/ProportionalIntegralDerivative
#

import tkinter as tk

import numpy as np

from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg)

from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure


from scipy.integrate import odeint


n = 100 # time points to plot
tf = 20.0 # final time
SP_start = 2.0 # time of set point change

def process(y,t,u):
    Kp = 4.0
    taup = 3.0
    thetap = 1.0
    if t<(thetap+SP_start):
        dydt = 0.0  # time delay
    else:
        dydt = (1.0/taup) * (-y + Kp * u)
    return dydt

def pidPlot(Kc,tauI,tauD):
    t = np.linspace(0,tf,n) # create time vector
    P= np.zeros(n)          # initialize proportional term
    I = np.zeros(n)         # initialize integral term
    D = np.zeros(n)         # initialize derivative term
    e = np.zeros(n)         # initialize error
    OP = np.zeros(n)        # initialize controller output
    PV = np.zeros(n)        # initialize process variable
    SP = np.zeros(n)        # initialize setpoint
    SP_step = int(SP_start/(tf/(n-1))+1) # setpoint start
    SP[0:SP_step] = 0.0     # define setpoint
    SP[SP_step:n] = 4.0     # step up
    y0 = 0.0                # initial condition
    # loop through all time steps
    for i in range(1,n):
        # simulate process for one time step
        ts = [t[i-1],t[i]]         # time interval
        y = odeint(process,y0,ts,args=(OP[i-1],))  # compute next step
        y0 = y[1]                  # record new initial condition
        # calculate new OP with PID
        PV[i] = y[1]               # record PV
        e[i] = SP[i] - PV[i]       # calculate error = SP - PV
        dt = t[i] - t[i-1]         # calculate time step
        P[i] = Kc * e[i]           # calculate proportional term
        I[i] = (I[i-1] + (Kc/tauI) * e[i] * dt) if tauI != 0 else 0  # calculate integral term
        D[i] = -Kc * tauD * (PV[i]-PV[i-1])/dt # calculate derivative term
        OP[i] = P[i] + I[i] + D[i] # calculate new controller output
        
    # plot PID response
    plt0.clear()
    #plt0.autoscale(False)
    plt0.set_ylim([-10, 10])
    plt0.plot(t,SP,'k-',linewidth=2,label='Setpoint (SP)')
    plt0.plot(t,PV,'r:',linewidth=2,label='Process Variable (PV)')
    plt0.legend(loc='best')
    plt0.grid()

    plt1.clear()
    plt1.plot(t,P,'g.-',linewidth=2,label=r'Proportional = $K_c \; e(t)$')
    plt1.plot(t,I,'b-',linewidth=2,label=r'Integral = $\frac{K_c}{\tau_I} \int_{i=0}^{n_t} e(t) \; dt $')
    plt1.plot(t,D,'r--',linewidth=2,label=r'Derivative = $-K_c \tau_D \frac{d(PV)}{dt}$')    
    plt1.legend(loc='best')
    plt1.grid()

    plt2.clear()
    plt2.plot(t,e,'m--',linewidth=2,label='Error (e=SP-PV)')
    plt2.legend(loc='best')
    plt2.grid()

    plt3.clear()
    plt3.plot(t,OP,'b--',linewidth=2,label='Controller Output (OP)')
    plt3.legend(loc='best')
    plt3.set_xlabel('time')
    plt3.grid()

    fig.canvas.draw()
    fig.canvas.flush_events()


def redraw_plot(value=None):
    Kc = Kc_scale.get()
    tauI = tauI_scale.get()
    tauD = tauD_scale.get()

    pidPlot(Kc,tauI,tauD)


root = tk.Tk()

innerframe = tk.Frame(root)

innerframe.grid_rowconfigure(0, weight=1)
innerframe.grid_rowconfigure(1, weight=1)
innerframe.grid_rowconfigure(2, weight=1)

innerframe.grid_columnconfigure(1, weight=1)


tk.Label(innerframe, text='P').grid(row=0)
tk.Label(innerframe, text='I').grid(row=1)
tk.Label(innerframe, text='D').grid(row=2)



Kc_scale = tk.Scale(innerframe, from_=-0, to=1.0, orient=tk.HORIZONTAL, resolution=0.01, command=redraw_plot)
Kc_scale.set(0.1)
Kc_scale.grid(row=0, column=1, sticky="nsew")

tauI_scale = tk.Scale(innerframe, from_=0.0, to=5.0, orient=tk.HORIZONTAL, resolution=0.01, command=redraw_plot)
tauI_scale.set(4.0)
tauI_scale.grid(row=1, column=1, sticky="nsew")

tauD_scale = tk.Scale(innerframe, from_=0.0, to=1.0, orient=tk.HORIZONTAL, resolution=0.01, command=redraw_plot)
tauD_scale.set(0.0)
tauD_scale.grid(row=2, column=1, sticky="nsew")


innerframe.pack(fill="both", expand=True)

fig = Figure()

plt0 = fig.add_subplot(2,2,1)
plt1 = fig.add_subplot(2,2,2)
plt2 = fig.add_subplot(2,2,3)
plt3 = fig.add_subplot(2,2,4)


canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

tk.mainloop()
