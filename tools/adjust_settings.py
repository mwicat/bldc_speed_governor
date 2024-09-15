import threading
import tkinter
from tkinter import *
from queue import Queue, Empty

from matplotlib.animation import FuncAnimation

from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg)

from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure

import serial
from serial.tools import list_ports


class SerialPortManager:
    # A class for management of serial port data in a separate thread
    # Source: https://github.com/m3y54m/python-serial-port-gui

    def __init__(self, serialPortBaud=9600):
        self.isRunning = False
        self.serialPortName = None
        self.serialPortBaud = serialPortBaud
        self.serialPort = serial.Serial()
        # Create a byte array to store incoming data
        self.serialPortBuffer = bytearray()
        self.write_queue = Queue()
        self.receive_callback = None

    def write(self, msg):
        self.write_queue.put_nowait(msg)

    def set_name(self, serialPortName):
        self.serialPortName = serialPortName

    def set_baud(self, serialPortBaud):
        self.serialPortBaud = serialPortBaud

    def start(self):
        self.isRunning = True
        self.serialPortThread = threading.Thread(target=self.thread_handler)
        self.serialPortThread.daemon = True
        self.serialPortThread.start()

    def stop(self):
        self.isRunning = False

    def thread_handler(self):
        while self.isRunning:
            try:
                msg = self.write_queue.get_nowait()
            except Empty:
                msg = None

            if not self.serialPort.isOpen():

                self.serialPort = serial.Serial(
                    port=self.serialPortName,
                    baudrate=self.serialPortBaud,
                    bytesize=8,
                    timeout=2,
                    stopbits=serial.STOPBITS_ONE,
                )
            else:
                if msg is not None:
                    self.serialPort.write(msg.encode('utf-8'))

                while self.serialPort.in_waiting > 0:
                    line =  self.serialPort.readline().decode('ascii')
                    if self.receive_callback is not None:
                        self.receive_callback(line)

        if self.serialPort.isOpen():
            self.serialPort.close()

    def read_buffer(self):
        buffer = self.serialPortBuffer
        self.serialPortBuffer = bytearray()
        return buffer

    def __del__(self):
        if self.serialPort.isOpen():
            self.serialPort.close()


class App(tkinter.Tk):

    def handle_serial_data(self, msg):
        fields = msg.strip().split(',')
        timestamp = int(fields[0])
        rpm = float(fields[1])
        pwm_width = float(fields[2])

        self.x.append(timestamp)
        self.y.append(rpm)

        print(timestamp, rpm, pwm_width)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        def send_settings():
            Kp = float(entry_p.get())
            Ki = float(entry_i.get())
            Kd = float(entry_d.get())
            print(Kp, Ki, Kd)
            serialPortManager.write(f'{Kp},{Ki},{Kd}')

        serialPortManager = SerialPortManager(115200)
        serialPortManager.receive_callback = self.handle_serial_data

        self.wm_title("BLDC Speed Governor settings")
        self.bind('<Return>', lambda event: send_settings())

        innerframe = Frame(self)

        Label(innerframe, text='P').grid(row=0)
        Label(innerframe, text='I').grid(row=1)
        Label(innerframe, text='D').grid(row=2)

        entry_p = Entry(innerframe)
        entry_p.insert(0, '0.1')

        entry_i = Entry(innerframe)
        entry_i.insert(0, '5')

        entry_d = Entry(innerframe)
        entry_d.insert(0, '0')

        entry_p.grid(row=0, column=1)
        entry_i.grid(row=1, column=1)
        entry_d.grid(row=2, column=1)

        button = Button(innerframe, text="Send", command=send_settings)
        button.grid(row=3, columnspan=2)

        innerframe.pack(padx=40, pady=5)

        print('Ports:')
        for port in list_ports.comports():
            print(port)

        serialPortManager.set_name('COM3')
        serialPortManager.start()

        def animate(i):
            plt.clear()
            plt.grid()
            plt.plot(self.x[-100:], self.y[-100:])
            plt.set_ylim([0, 1000])

            return plt

        fig = Figure()

        plt = fig.add_subplot()
        plt.autoscale(False)

        canvas = FigureCanvasTkAgg(fig, master=self)
        canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)

        self.x = []
        self.y = []

        self.anim = FuncAnimation(fig, animate, interval=100)



App().mainloop()

#serialPortManager.stop()
