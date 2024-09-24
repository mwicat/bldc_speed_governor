import multiprocessing as mp
import queue
from subprocess import PIPE
import subprocess
import sys

import serial
import serial.threaded

from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6 import QtWidgets, QtGui
from PyQt6.QtWidgets import QApplication, QMainWindow, QGridLayout, QWidget, QGroupBox

from pglive.sources.data_connector import DataConnector
from pglive.sources.live_plot import LiveLinePlot
from pglive.sources.live_plot_widget import LivePlotWidget
from pglive.sources.live_axis_range import LiveAxisRange
from pglive.kwargs import Crosshair

from pyqtgraph import InfiniteLine
import pyqtgraph as pg  # type: ignore


sigrok_path = r'C:\Program Files\sigrok\sigrok-cli\sigrok-cli.exe'

args = [
    sigrok_path,
    '-d',
    'fx2lafw',
    '--config',
    'samplerate=1MHz',
    '--continuous',
    '-C',
    'D1=fg,D0=pwm',
    '-P',
    'timing:data=fg:edge=rising:avg_period=5',
    '-P',
    'pwm:data=pwm',
    '-A',
    'pwm=duty-cycle,timing=averages',
    '--protocol-decoder-samplenum'
]

pwm_maximum = 2047


class SerialPortManager(QThread):
    
    line_received = pyqtSignal(str)

    def __init__(self, serialPortName, serialPortBaud=9600, parent=None):
        super(SerialPortManager, self).__init__(parent)

        self.serialPortName = serialPortName
        self.serialPortBaud = serialPortBaud

        self.serialPort = serial.Serial(
            port=self.serialPortName,
            baudrate=self.serialPortBaud,
            bytesize=8,
            timeout=2,
            stopbits=serial.STOPBITS_ONE,            
        )

        self.write_queue = queue.Queue()
        self.running = False

    def write(self, msg):
        self.write_queue.put_nowait(msg)

    def stop(self):
        self.running = False

    def run(self):
        self.running = True
        manager = self

        class LineReader(serial.threaded.LineReader):
            def connection_made(self, transport):
                super(LineReader, self).connection_made(transport)
                print('serial port opened')
                self.write_line('r 0')

            def handle_line(self, data):
                manager.line_received.emit(data)

            def connection_lost(self, exc):
                if exc is not None:
                    print(exc)
                print('serial port closed')
            

        with serial.threaded.ReaderThread(self.serialPort, LineReader) as protocol:
            while self.running:
                try:
                    msg = self.write_queue.get(timeout=1)
                except queue.Empty:
                    pass
                else:    
                    protocol.write_line(msg)
        
        print('serial port manager exiting')


class SigrokProcess(mp.Process):

    def __init__(self, queue, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.queue = queue
        self.stop_event = mp.Event()

    def _parse_data(self, data):
        data = data.decode('utf-8').strip()
        fields = data.split()

        category = fields[1].split('-')[0]

        x = int(fields[0].split('-')[0])
        
        if category == 'pwm':
            y = float(fields[2][:-1]) / 100 * pwm_maximum
        elif category == 'timing':
            y = float(1 / float(fields[2]) * 1000 * 60 / 4)
            #y = int(1 / float(fields[2]) * 1000 * 60 / 4)
            #y = int(float((fields[2])))

        return category, (x, y)

    def stop(self):
        self.stop_event.set()

    def run(self):
        self.proc = subprocess.Popen(
            args, shell=False, stdin=None, stdout=PIPE, stderr=None)

        avg_cnt = 0
        avg_limit = 1000
        avg_sum = 0
        
        while not self.stop_event.is_set():
            l = self.proc.stdout.readline()
            
            if not l:
                print('Process exited, restarting...')
                self.queue.put_nowait(False)
                self.proc = subprocess.Popen(
                    args, shell=False, stdin=None, stdout=PIPE, stderr=None)
                continue
            
            category, (x, y) = self._parse_data(l)       

            if category == 'timing':
                self.queue.put_nowait((category, (y, x)))
            elif category == 'pwm':
                avg_sum += y
                avg_cnt += 1
                if avg_cnt == avg_limit-1:
                    val = avg_sum/avg_limit
                    self.queue.put_nowait((category, (val, x)))
                    avg_cnt = 0
                    avg_sum = 0

        print('sigrok process killing sigrok child')
        self.proc.kill()                    
        print('sigrok process exiting')


class SigrokWorker(QThread):
    
    timing_received = pyqtSignal(tuple)
    pwm_received = pyqtSignal(tuple)
    crashed = pyqtSignal()

    def __init__(self, queue, parent=None):
        super(SigrokWorker, self).__init__(parent)
        self.queue = queue
        self.running = False
        #self.daemon = True

    def stop(self):
        self.running = False

    def run(self):
        self.running = True
        
        while self.running:
            try:
                item = self.queue.get(timeout=1)
            except queue.Empty:
                pass
            else:
                if item == False:
                    self.crashed.emit()
                else:           
                    (cat, (y, x)) = item
                    if cat == 'timing':
                        self.timing_received.emit((y, x))
                    elif cat == 'pwm':
                        self.pwm_received.emit((y, x))
        
        print('sigrok worker exiting')


def main():
    app = QApplication(sys.argv)

    crosshair_pen = pg.mkPen(
        color="green", width=1, style=Qt.PenStyle.DashDotLine)

    plot_widget_opts = {Crosshair.ENABLED: True,
            Crosshair.LINE_PEN: crosshair_pen,
            Crosshair.TEXT_KWARGS: {"color": "green"}}

    plot_widget = LivePlotWidget(
        title='Speed [rpm]',
        y_range_controller=LiveAxisRange(fixed_range=[0, 1000]),
        **plot_widget_opts
        )

    plot_curve = LiveLinePlot()
    plot_widget.addItem(plot_curve)

    mark_line = InfiniteLine(angle=0, pos=1000, label='target')
    plot_widget.addItem(mark_line)

    pwm_y_range = [0, pwm_maximum]

    plot_widget2 = LivePlotWidget(
        title='PWM [%]',
        x_range_controller=LiveAxisRange(fixed_range=[-1, 1]),
        y_range_controller=LiveAxisRange(fixed_range=pwm_y_range),
        **plot_widget_opts
        )

    plot_curve2 = LiveLinePlot()
    plot_widget2.addItem(plot_curve2)

    data_connector = DataConnector(plot_curve, max_points=60, update_rate=250)
    data_connector2 = DataConnector(plot_curve2, max_points=60, update_rate=250)

    process_queue = mp.Queue()
    sigrok_process = SigrokProcess(process_queue)

    sigrok_worker = SigrokWorker(process_queue)
    
    def on_timing_probe(item):
        data_connector.cb_append_data_point(*item)

    def on_pwm_probe(item):
        data_connector2.cb_append_data_point(*item)

    def on_probes_crashed():
        data_connector.clear()
        data_connector2.clear()
    
    sigrok_worker.timing_received.connect(on_timing_probe)
    sigrok_worker.pwm_received.connect(on_pwm_probe)
    sigrok_worker.crashed.connect(on_probes_crashed)

    sigrok_worker.start()

    sigrok_process.start()


    layout = QGridLayout()

    class MainWindow(QMainWindow):

        def keyPressEvent(self, event):
            if event.key() == Qt.Key.Key_Space:
                plot_widget.auto_btn_clicked()
                plot_widget2.auto_btn_clicked()
            elif event.key() == Qt.Key.Key_A:
                plot_widget.y_range_controller.fixed_range = None
                plot_widget2.y_range_controller.fixed_range = None
            elif event.key() == Qt.Key.Key_B:
                plot_widget.y_range_controller.fixed_range = [0, 1000]
                plot_widget2.y_range_controller.fixed_range = [0, pwm_maximum]
        

    window = MainWindow()

    layout.addWidget(plot_widget, 0, 0)
    layout.addWidget(plot_widget2, 1, 0)

    layout.addWidget(plot_widget2, 1, 0)

    def addSpinBox(form, title, *args, **kwargs):
        kwargs.setdefault('compactHeight', False)
        spin_box = pg.SpinBox(
            *args, **kwargs)
        form.addRow(
            QtWidgets.QLabel(title),
            spin_box
        )
        return spin_box

    # Parent settings
    settings_group_box = QGroupBox('Settings')
    settings_vbox_layout = QtWidgets.QVBoxLayout()
    settings_group_box.setLayout(settings_vbox_layout)    

    layout.addWidget(settings_group_box, 0, 1, 1, 1)
    
    # PID settings
    pid_settings_group_box = QGroupBox('PID')
    pid_settings_form = QtWidgets.QFormLayout()
    pid_settings_group_box.setLayout(pid_settings_form)

    settings_vbox_layout.addWidget(pid_settings_group_box)

    cmd_to_editbox = {}
    

    def addCommandInput(title, cmd_prefix, *args, **kwargs):
        spin_box = addSpinBox(pid_settings_form, title, *args, **kwargs)

        def send_cmd():
            cmd_val = spin_box.text()
            cmd = f'{cmd_prefix} {cmd_val}'
            print('send', cmd)
            serialPortManager.write(cmd)

            if cmd_prefix == 's':
                mark_line.setValue(cmd_val)

        spin_box.lineEdit().returnPressed.connect(send_cmd)
        spin_box.sigValueChanged.connect(send_cmd)
        
        cmd_to_editbox[cmd_prefix] = spin_box    

    addCommandInput(
        'Enabled',
        'e',
        bounds=[0, 1],
        int=True,
        value=0
        )

    addCommandInput(
        'Run frequency [Hz]',
        'f',
        bounds=[0, 20000],
        compactHeight=False,
        value=10
        )


    addCommandInput(
        'Setpoint',
        's',
        bounds=[0, 1000],
        int=True,
        value=400
        )

    addCommandInput(
        'P',
        'p',
        step=0.1,
        bounds=[0.0, 1000.0],
        value=1.0
        )

    addCommandInput(
        'I',
        'i',
        bounds=[0.0, 1000.0],
        value=1.0
        )

    addCommandInput(
        'D',
        'd',
        bounds=[0, 1000.0],
        compactHeight=False,
        value=5
        )

    addCommandInput(
        'Pulse width (manual)',
        'a',
        bounds=[0, pwm_maximum],
        int=True,
        value=250
        )

    # UI Settings
    ui_settings_group_box = QGroupBox('UI')
    ui_settings_form = QtWidgets.QFormLayout()
    ui_settings_group_box.setLayout(ui_settings_form)

    settings_vbox_layout.addWidget(ui_settings_group_box)

    # speed min max
    def set_speed_y_max(src):
        ymax = int(src.text())
        plot_widget.y_range_controller.fixed_range[1] = ymax

    speed_y_max_spinbox = addSpinBox(
        ui_settings_form,
        'Speed Y Max',
        int=True,
        bounds=[0, 10000],
        step=10,
        value=1000
        )
    speed_y_max_spinbox.sigValueChanged.connect(set_speed_y_max)

    def set_speed_y_min(src):
        ymin = int(src.text())
        plot_widget.y_range_controller.fixed_range[0] = ymin

    speed_y_min_spinbox = addSpinBox(
        ui_settings_form,
        'Speed Y Min',
        int=True,
        bounds=[0, 10000],
        step=10,
        value=0
        )
    speed_y_min_spinbox.sigValueChanged.connect(set_speed_y_min)

    # PWM min max

    def set_pwm_y_max(src):
        ymax = int(src.text())
        pwm_y_range[1] = ymax

    pwm_y_max_spinbox = addSpinBox(
        ui_settings_form,
        'PWM Y Max',
        int=True,
        bounds=[0, 10000],
        step=10,
        value=1000
        )
    pwm_y_max_spinbox.sigValueChanged.connect(set_pwm_y_max)

    def set_pwm_y_min(src):
        ymin = int(src.text())
        pwm_y_range[0] = ymin

    pwm_y_min_spinbox = addSpinBox(
        ui_settings_form,
        'PWM Y Min',
        int=True,
        bounds=[0, 10000],
        step=10,
        value=0
        )
    pwm_y_min_spinbox.sigValueChanged.connect(set_pwm_y_min)

    # Output Box
    output_group_box = QGroupBox('Output')
    output_vbox = QtWidgets.QVBoxLayout()
    
    serial_output = QtWidgets.QPlainTextEdit()
    output_vbox.addWidget(serial_output)
    output_group_box.setLayout(output_vbox)
    
    layout.addWidget(output_group_box, 1, 1, 1, 1)

    widget = QWidget()
    widget.setLayout(layout)

    window.setCentralWidget(widget)

    def serial_line_received(msg):
        serial_output.insertPlainText(msg + '\n')
        cursor = serial_output.textCursor()
        cursor.movePosition(QtGui.QTextCursor.MoveOperation.End)
        serial_output.setTextCursor(cursor)
        
        print(repr(msg))
        if msg.startswith('r '):
            field_strs = msg.split(' ')[1].split(',')
            for field_str in field_strs:
                cmd_prefix, cmd_val = field_str.split('=')
                cmd_val = float(cmd_val)
                cmd_to_editbox[cmd_prefix].setValue(cmd_val)
                if cmd_prefix == 's':
                    mark_line.setValue(cmd_val)
                    
            

    serialPortManager = SerialPortManager('COM8', 115200)
    serialPortManager.line_received.connect(serial_line_received)
    serialPortManager.start()


    window.show()

    def onSigRangeChanged(r):
        plot_widget2.setRange(
            yRange=pwm_y_range,
            xRange=r.getAxis('bottom').range
            )

    plot_widget.sigRangeChanged.connect(onSigRangeChanged)

    app.exec()
    
    sigrok_worker.stop()
    serialPortManager.stop()

    sigrok_process.stop()
    sigrok_process.join()
    

if __name__ == '__main__':
    mp.freeze_support()
    main()