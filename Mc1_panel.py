# img_viewer.py
#https://pysimplegui.readthedocs.io/en/latest/call%20reference/

from datetime import datetime
#from fcntl import DN_ACCESS
import struct
from time import sleep
import PySimpleGUI as sg
import os.path
from PySimpleGUI.PySimpleGUI import T, Multiline, popup
from serial import *
import serial.tools.list_ports
import glob
import numpy as np
from struct import *
from struct import Struct
from tkinter.font import Font

#libraries for the DACs Calibration
import statsmodels.api as sm
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from collections import namedtuple
import time 

READ_CONFIG   = b'\x08'
END_COMMAND   = b'\x0a'

MONITOR_DATA    = b'\x01'
START_RESPONSE  = b'\xAA'
READ_CONFIG     = b'\x08'
R_START         = b'x'  #Start monitoring
R_STOP          = b'z'  #Stop  monitoring
WRITE_CONFIG    = b'\x0b'  #start send data to Write config to EEPROM
R_WPOINT        = b'\x0c'  #start send data to Write point to flash
R_RPOINT        = b'\x0e'  #request read for point from flash
R_EPAGE         = b'\x12'  #start send data to write point in a page in  flash
R_ERASE_4K      = b'\x14'  #
P_ERASE_32K     = b'\x16'
R_ERASE_CHIP    = b'\x1a'
FLASH_INFO      = b'\x1c'
STATUS_REGISTER = b'\x1e'

R_ANGLE_TO_MC2 = b'A'
R_TEST_PWM     = b'\x20'
EMPTY_BUFFER   = b'\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n'

global serialPort
global serialChannel 
global baudRate 
global connectionEnabled 
global config
global config_format
global config_keys
global flash_table_data



serialPort = "/dev/ttyUSB0"
baudRate = 115200
connectionEnabled = False

pages=list(range(0,19))
angles=list(range(0,300))
tables=list(range(0,13))
dac = list(range(1,3))
channel = list(range(0,16))
flash_table_data=[]
serialChannel=Serial()

config_format='<8h8h8h8h6h13h4h' 
Config = namedtuple('Config',
           ['HeatEn0', 'HeatEn1','HeatEn2','HeatEn3','HeatEn4','HeatEn5','HeatEn6','HeatEn7',
           'consKp0', 'consKp1', 'consKp2', 'consKp3', 'consKp4', 'consKp5', 'consKp6', 'consKp7',
           'consKi0', 'consKi1', 'consKi2', 'consKi3', 'consKi4', 'consKi5', 'consKi6', 'consKi7',
           'consKd0', 'consKd1', 'consKd2', 'consKd3', 'consKd4', 'consKd5', 'consKd6', 'consKd7',
           'fan0','fan1','fan2','fan3', 'fan4', 'fan5',
           'temperature_table0','temperature_table1','temperature_table2','temperature_table3','temperature_table4',
           'temperature_table5','temperature_table6','temperature_table7','temperature_table8','temperature_table9',
           'temperature_table10','temperature_table11','temperature_table12',
           'adc_conv_paramenter','threshold_tolerance','pid_sample_time','max_pid_value'])

config=Config(   0,    0,    0,    0,    0,     0,    0,    0,
              50,    50,    50,    50,     50,    50,    50,    50,
               0,  0, 0,  0,  0, 0, 0, 0,   
               0,  0, 0,  0,  0, 0, 0, 0,
               0,    0,    0,     0,     0,    0,
              2500,   2700,   2900,    3100,    3300,
              3500,   3700,   3900,    4000,    4100,
              4200,   4300,   4400,
              1175,   100,    500,   127)

text_debug = 'In debug mode the voltages for the DACs need to be a decimal number\nThe equation for convert voltage to decimal is\nVoltage = Decimal x 0.0125\nExample: 100 V --> 8000\n'

Flash = namedtuple("Flash",["ManID","Capacity", "MaxPages"])

flash_format='<hll' #<--------controllare cosa è e a che cosa serve

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def log(data , name="DEBUG"):
    print(datetime.now(), "|", name, "|", data)


def decode_input_data(window,data):
    cmd = data[:1] #first char define the type of data

    if False:#if cmd == FLASH_INFO:
        flash = Flash._make( unpack_from(flash_format, data)) # ._make() permette di clonare una namedtuple
        update_config_panel(window, flash)
        return 0
    else:
        log(data, "*** OUT from MC1 not valid ***" )
    return 1

def receiveBuffer(serialChannel):
    commands={
        MONITOR_DATA    : {"size":15, "start":START_RESPONSE}, # le parentesi graffe sono i "set" che insieme a liste e tuple rapprensentano le strutture dati di python, i set servono per mettere dentro una variabile più oggetti
        READ_CONFIG     : {"size":112, "start":START_RESPONSE},
        R_RPOINT        : {"size":230, "start":START_RESPONSE},
        FLASH_INFO      : {"size":5, "start":START_RESPONSE},
        R_ANGLE_TO_MC2  : {"size":9, "start":START_RESPONSE}, #cambiato da 7 a 9 a causa dei due byte per il tempo di inversione
        R_EPAGE         :  {"size":4, "start":START_RESPONSE},
        R_WPOINT        :  {"size":1, "start":START_RESPONSE},
    }
    buffer=b''
    skipped=b''
    opcode=b''
    max_size=0
    while (serialChannel and serialChannel.in_waiting > 0): #if incoming bytes are waiting to be read from the serial input buffer
        current_char = serialChannel.read()
        if (opcode == b'' and current_char in commands): #first char of command
            opcode   = current_char
            max_size = commands[opcode]["size"]
            #skipped  = b''
            buffer  += current_char
            log(max_size, "scanSerial_char maxsize")
        elif (opcode == b'' and not(current_char in commands)):   #skip    
            skipped += current_char
        elif max_size > 1 and len(buffer) > max_size:  #opcode is not empty and maxsize valid
            return (buffer)
        elif max_size == 0 and current_char == '\x0a':  #opcode is not empty and maxsize = 0
            return (buffer)
        else:
            buffer+= current_char
       
    if (max_size >0 and len(buffer) >= max_size): return (buffer)
    if skipped != b'' : log(    skipped , "scanSerial_char skipped")  
    if buffer  != b'' : log(len(buffer) , "undecoded before size"  ); #decode_input_data(window,buffer)
    buffer=b''
    skipped=b''
    opcode = b''
    max_size = 0

def update_point_panel(window, data):
    #raw_data=window["-flash-data-"]
    flash_table_data = raw_data.get()
    #old_data=raw_data.get()
    #if len(old_data)<2: old_data=old_data.rstrip()
    new_data=",".join(str(x) for x in data).strip()
    flash_table_data.append([0,new_data])
    raw_data.update(values=flash_table_data)



def update_sensor_panel(window, data):
        #1 ->> PT1K_ADC[i];
        #2 ->> PT1K_OHM[i];
        #ohm_to_celsius(PT1K_OHM[i]);
        #3 --> T_TARGET[i];
        #4 --> T_HEATER[i];
    sensors_data=data
    sensor_num=sensors_data[0]
    if len(sensors_data) == 7 and int(sensor_num) < 8:
        temprature=window["Temperature"  +str(sensor_num)]
        adc=window["ADC"  +str(sensor_num)]
        target=window["Target"+str(sensor_num)]
        delta=window["Delta"  +str(sensor_num)]
        heater=window["Heat"  +str(sensor_num)]
        heater_status=window["HeatStatus"  +str(sensor_num)]

        adc.update(          "{0}".format(sensors_data[1]))
        temprature.update(   "{0} °C".format(sensors_data[2]/100))
        target.update(       "{0} °C".format(sensors_data[3]/100))
        delta.update(        "{0} °C".format(sensors_data[4]/100))
        heater_status.update("{0}".format(sensors_data[5]))
        heater.update(       "{0}".format(sensors_data[6]))

def update_monitor_panel(window, data):
    monitor = window["-monitor-"]
    monitor.update(monitor.get()+ "\r\n" + str(data))
 

def update_config_panel(window, config):
    for k in config._fields: #._fields serve per recuperare i parametri di una named tuple
        try:
            window[k].update(getattr(config,k)) #getattr prende il valore di un attributo di un oggetto, in questo caso di 'config' preleva l'attributo k che potrebbe essere l'indice di una lista
        except:
            pass
    table = window["-table-"].get()
    #window["-working-temp-"].update(window["temperature_table{}".format(table)].get())
    
def build_config_from_ui(window):
    cc={}
    for k in config._fields:
        if k.startswith("consK"):
          cc[k] = int(window[k].get())
        elif k in ['none']: #['R1', 'R2', 'R3','VCOPAMP', 'VREF', 'interval', 'initial_temp', 'heater_enabled','OPAPM_GAIN',]:
          cc[k] = window[k].get()
        else:    
          cc[k] = int(window[k].get())
    return Config(**cc)

def generate_point(index):
    tmp_layout=[
                [sg.T('DAC1 00-15'),
                      sg.In(key='dac1_{0}_0'.format(index), size =  (5, 1)),
                      sg.In(key='dac1_{0}_1'.format(index), size =  (5, 1)),
                      sg.In(key='dac1_{0}_2'.format(index), size =  (5, 1)),
                      sg.In(key='dac1_{0}_3'.format(index), size =  (5, 1)),
                      sg.In(key='dac1_{0}_4'.format(index), size =  (5, 1)),
                      sg.In(key='dac1_{0}_5'.format(index), size =  (5, 1)),
                      sg.In(key='dac1_{0}_6'.format(index), size =  (5, 1)),
                      sg.In(key='dac1_{0}_7'.format(index), size =  (5, 1)),
                      sg.In(key='dac1_{0}_8'.format(index), size =  (5, 1)),
                      sg.In(key='dac1_{0}_9'.format(index), size =  (5, 1)),
                      sg.In(key='dac1_{0}_10'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_11'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_12'.format(index), size = (5, 1)), 
                      sg.In(key='dac1_{0}_13'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_14'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_15'.format(index), size = (5, 1)),
                      ],
                [sg.T('DAC1 16-31'),
                      sg.In(key='dac1_{0}_16'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_17'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_18'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_19'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_20'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_21'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_22'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_23'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_24'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_25'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_26'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_27'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_28'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_29'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_30'.format(index), size = (5, 1)),
                      sg.In(key='dac1_{0}_31'.format(index), size = (5, 1)),

                      ],
        [sg.T('DAC2 00-15'),
                      sg.In(key='dac2_{0}_0'.format(index), size =  (5, 1)),
                      sg.In(key='dac2_{0}_1'.format(index), size =  (5, 1)),
                      sg.In(key='dac2_{0}_2'.format(index), size =  (5, 1)),
                      sg.In(key='dac2_{0}_3'.format(index), size =  (5, 1)),
                      sg.In(key='dac2_{0}_4'.format(index), size =  (5, 1)),
                      sg.In(key='dac2_{0}_5'.format(index), size =  (5, 1)),
                      sg.In(key='dac2_{0}_6'.format(index), size =  (5, 1)),
                      sg.In(key='dac2_{0}_7'.format(index), size =  (5, 1)),
                      sg.In(key='dac2_{0}_8'.format(index), size =  (5, 1)),
                      sg.In(key='dac2_{0}_9'.format(index), size =  (5, 1)),
                      sg.In(key='dac2_{0}_10'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_11'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_12'.format(index), size = (5, 1)), 
                      sg.In(key='dac2_{0}_13'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_14'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_15'.format(index), size = (5, 1)),
                      ],
                [sg.T('DAC2 16-31'),
                      sg.In(key='dac2_{0}_16'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_17'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_18'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_19'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_20'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_21'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_22'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_23'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_24'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_25'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_26'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_27'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_28'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_29'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_30'.format(index), size = (5, 1)),
                      sg.In(key='dac2_{0}_31'.format(index), size = (5, 1)),
                      ],
        [sg.T('RF data for all the 8 boards is 100 bytes'),],
        [sg.T('RF byte 1-25  '),sg.In(key='rf_{0}_0'.format(index), size = (105, 1)),],
        [sg.T('RF byte 26-50 '),sg.In(key='rf_{0}_1'.format(index), size = (105, 1)),],
        [sg.T('RF byte 51-75 '),sg.In(key='rf_{0}_2'.format(index), size = (105, 1)),],
        [sg.T('RF byte 76-100'),sg.In(key='rf_{0}_3'.format(index), size = (105, 1)),]                           
    ]
    return tmp_layout

#define a canvas for the graph
def draw_figure(canvas, figure):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg

def main():
    global config

    font = "Arial 13"
    font_family, font_size = font = ('Courier New', 10)
    sg.set_options(font=font)

    # rendere queste due grandezze configurabili da file
    # vBias = 100
    # v_lsb = 0.0125

    tab1_layout =  [
                        [sg.T('Configuration parameters')],
                        [
                            sg.T('A/D Parameter',size = (20, 1),     font=font),
                            sg.T('m(ADC)'),sg.In(key='adc_conv_paramenter',     size = (10, 1)),
                            sg.T('Tolerance'),sg.In(key='threshold_tolerance',  size = (10, 1)),
                            sg.T('PID Sample Time'),sg.In(key='pid_sample_time',size = (10, 1)),
                            sg.T('PID max output'),sg.In(key='max_pid_value',   size = (10, 1)),
                            
                        ],
                        [
                            sg.T('PID Paramenter',size = (20, 1)),
                            sg.T('Prop.', size = (5, 1),font=font),
                            sg.In(key='consKp0',size = (6, 1)),
                            sg.In(key='consKp1',size = (6, 1)),
                            sg.In(key='consKp2',size = (6, 1)),
                            sg.In(key='consKp3',size = (6, 1)),
                            sg.In(key='consKp4',size = (6, 1)),
                            sg.In(key='consKp5',size = (6, 1)),
                            sg.In(key='consKp6',size = (6, 1)),
                            sg.In(key='consKp7',size = (6, 1)),
                        ],
                        [
                            sg.T(' ',size = (20, 1)),
                            sg.T('Deriv.', size = (5, 1)),
                            sg.In(key='consKi0',size = (6, 1)),
                            sg.In(key='consKi1',size = (6, 1)),
                            sg.In(key='consKi2',size = (6, 1)),
                            sg.In(key='consKi3',size = (6, 1)),
                            sg.In(key='consKi4',size = (6, 1)),
                            sg.In(key='consKi5',size = (6, 1)),
                            sg.In(key='consKi6',size = (6, 1)),
                            sg.In(key='consKi7',size = (6, 1)),
                        ],
                        [
                            sg.T(' ',size = (20, 1)),
                            sg.T('Integ.', size = (5, 1)),
                            sg.In(key='consKd0',size = (6, 1)),
                            sg.In(key='consKd1',size = (6, 1)),
                            sg.In(key='consKd2',size = (6, 1)),
                            sg.In(key='consKd3',size = (6, 1)),
                            sg.In(key='consKd4',size = (6, 1)),
                            sg.In(key='consKd5',size = (6, 1)),
                            sg.In(key='consKd6',size = (6, 1)),
                            sg.In(key='consKd7',size = (6, 1)),
                        ],
                        [ 
                          sg.T('Temperature Table',size = (26, 1) ),
                          sg.In(key='temperature_table0', visible=True, size=(5,1)),
                          sg.In(key='temperature_table1', visible=True, size=(5,1)),
                          sg.In(key='temperature_table2', visible=True, size=(5,1)),
                          sg.In(key='temperature_table3', visible=True, size=(5,1)),
                          sg.In(key='temperature_table4', visible=True, size=(5,1)),
                          sg.In(key='temperature_table5', visible=True, size=(5,1)),
                          sg.In(key='temperature_table6', visible=True, size=(5,1)),
                          sg.In(key='temperature_table7', visible=True, size=(5,1)),
                          sg.In(key='temperature_table8', visible=True, size=(5,1)),
                          sg.In(key='temperature_table9', visible=True, size=(5,1)),
                          sg.In(key='temperature_table10',visible=True, size=(5,1)),
                          sg.In(key='temperature_table11',visible=True, size=(5,1)),
                          sg.In(key='temperature_table12',visible=True, size=(5,1)),],                        
                        
                        [
                            sg.Button('Read Config', key="READ-CONFIG", disabled=True),
                            sg.Button('Write Config', key="WRITE-CONFIG", disabled=True)
                        ]
                    ]

    tab2_layout =   [
                        [
                            sg.T('FAN Control', size = (20, 1),font=font),
                        
                            sg.Checkbox("Fan  1", default = True, key = "fan0"), sg.Checkbox("Fan  2", default = True, key = "fan1"),
                            sg.Checkbox("Fan  3", default = True, key = "fan2"), sg.Checkbox("Fan  4", default = True, key = "fan3"),
                            sg.Checkbox("Fan  5", default = True, key = "fan4"), sg.Checkbox("Fan  6", default = True, key = "fan5")
                        ],

                        [
                            sg.T('Heaters Control',size = (20, 1)),
                            sg.Checkbox("Heat 1",default=False, key='HeatEn0'),
                            sg.Checkbox("Heat 2",default=False, key='HeatEn1'),
                            sg.Checkbox("Heat 3",default=False, key='HeatEn2'),
                            sg.Checkbox("Heat 4",default=False, key='HeatEn3'),
                            sg.Checkbox("Heat 5",default=False, key='HeatEn4'),
                            sg.Checkbox("Heat 6",default=False, key='HeatEn5'),
                            sg.Checkbox("Heat 7",default=False, key='HeatEn6'),
                            sg.Checkbox("Heat 8",default=False, key='HeatEn7'),
                        ],
                        [sg.T('Sensors'),
                         sg.Button('Enable', key="EnableMonitoring", disabled=False) ,
                         sg.Button('Disable', key="DisableMonitoring", disabled=False) ,
                        ],
                        [
                            sg.T('Temperature:',size = (20, 1)),
                            sg.In(key='Temperature0',size = (10, 1),readonly=True),
                            sg.In(key='Temperature1',size = (10, 1),readonly=True),
                            sg.In(key='Temperature2',size = (10, 1),readonly=True),
                            sg.In(key='Temperature3',size = (10, 1),readonly=True),
                            sg.In(key='Temperature4',size = (10, 1),readonly=True),
                            sg.In(key='Temperature5',size = (10, 1),readonly=True),
                            sg.In(key='Temperature6',size = (10, 1),readonly=True),
                            sg.In(key='Temperature7',size = (10, 1),readonly=True),
                        ],               
                        [
                            sg.T('ADC:',size = (20, 1)),
                            sg.In(key='ADC0',size = (10, 1),readonly=True),
                            sg.In(key='ADC1',size = (10, 1),readonly=True),
                            sg.In(key='ADC2',size = (10, 1),readonly=True),
                            sg.In(key='ADC3',size = (10, 1),readonly=True),
                            sg.In(key='ADC4',size = (10, 1),readonly=True),
                            sg.In(key='ADC5',size = (10, 1),readonly=True),
                            sg.In(key='ADC6',size = (10, 1),readonly=True),
                            sg.In(key='ADC7',size = (10, 1),readonly=True),
                        ],               
                        [
                            sg.T('Target:',size = (20, 1)),
                            sg.In(key='Target0',size = (10, 1),readonly=True),
                            sg.In(key='Target1',size = (10, 1),readonly=True),
                            sg.In(key='Target2',size = (10, 1),readonly=True),
                            sg.In(key='Target3',size = (10, 1),readonly=True),
                            sg.In(key='Target4',size = (10, 1),readonly=True),
                            sg.In(key='Target5',size = (10, 1),readonly=True),
                            sg.In(key='Target6',size = (10, 1),readonly=True),
                            sg.In(key='Target7',size = (10, 1),readonly=True),
                            
                        ],
                        [
                            sg.T('Delta:',size = (20, 1)),
                            sg.In(key='Delta0',size = (10, 1),readonly=True),
                            sg.In(key='Delta1',size = (10, 1),readonly=True),
                            sg.In(key='Delta2',size = (10, 1),readonly=True),
                            sg.In(key='Delta3',size = (10, 1),readonly=True),
                            sg.In(key='Delta4',size = (10, 1),readonly=True),
                            sg.In(key='Delta5',size = (10, 1),readonly=True),
                            sg.In(key='Delta6',size = (10, 1),readonly=True),
                            sg.In(key='Delta7',size = (10, 1),readonly=True),
                            
                        ],
                        [
                            sg.T('Heater:',size = (20, 1)),
                            sg.In(key='Heat0',size = (10, 1),readonly=True),
                            sg.In(key='Heat1',size = (10, 1),readonly=True),
                            sg.In(key='Heat2',size = (10, 1),readonly=True),
                            sg.In(key='Heat3',size = (10, 1),readonly=True),
                            sg.In(key='Heat4',size = (10, 1),readonly=True),
                            sg.In(key='Heat5',size = (10, 1),readonly=True),
                            sg.In(key='Heat6',size = (10, 1),readonly=True),
                            sg.In(key='Heat7',size = (10, 1),readonly=True),
                            
                        ],
                        [
                            sg.T('HeatStatus:',size = (20, 1)),
                            sg.In(key='HeatStatus0',size = (10, 1),readonly=True),
                            sg.In(key='HeatStatus1',size = (10, 1),readonly=True),
                            sg.In(key='HeatStatus2',size = (10, 1),readonly=True),
                            sg.In(key='HeatStatus3',size = (10, 1),readonly=True),
                            sg.In(key='HeatStatus4',size = (10, 1),readonly=True),
                            sg.In(key='HeatStatus5',size = (10, 1),readonly=True),
                            sg.In(key='HeatStatus6',size = (10, 1),readonly=True),
                            sg.In(key='HeatStatus7',size = (10, 1),readonly=True),
                        ]       
                    ] 
    
    tab3_layout = [
                [sg.T('V BIAS'),sg.In(key='vbias',size=(5,1),default_text='100',readonly=True)],
                [sg.T('DAC1 00-15')],
                
                [sg.T('Slope    '),
                sg.In(key='slope_1_0',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_1',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_2',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_3',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_4',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_5',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_6',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_7',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_8',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_9',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_10',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_11',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_12',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_13',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_14',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_15',size=(5,1),readonly=True,default_text='80')],

                [sg.T('Intercept'),
                sg.In(key='intercept_1_0',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_1',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_2',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_3',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_4',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_5',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_6',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_7',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_8',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_9',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_10',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_11',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_12',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_13',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_14',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_15',size=(5,1),readonly=True,default_text='0')],

                [sg.T('DAC1 16-31')],
                
                [sg.T('Slope    '),
                sg.In(key='slope_1_16',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_17',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_18',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_19',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_20',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_21',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_22',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_23',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_24',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_25',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_26',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_27',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_28',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_29',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_30',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_1_31',size=(5,1),readonly=True,default_text='80')],

                [sg.T('Intercept'),
                sg.In(key='intercept_1_16',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_17',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_18',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_19',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_20',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_21',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_22',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_23',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_24',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_25',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_26',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_27',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_28',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_29',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_30',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_1_31',size=(5,1),readonly=True,default_text='0')],

                [sg.T('DAC2 00-15')],
                
                [sg.T('Slope    '),
                sg.In(key='slope_2_0',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_1',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_2',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_3',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_4',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_5',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_6',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_7',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_8',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_9',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_10',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_11',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_12',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_13',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_14',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_15',size=(5,1),readonly=True,default_text='80')],

                [sg.T('Intercept'),
                sg.In(key='intercept_2_0',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_1',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_2',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_3',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_4',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_5',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_6',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_7',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_8',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_9',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_10',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_11',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_12',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_13',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_14',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_15',size=(5,1),readonly=True,default_text='0')],

                [sg.T('DAC2 16-31')],
                
                [sg.T('Slope    '),
                sg.In(key='slope_2_16',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_17',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_18',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_19',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_20',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_21',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_22',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_23',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_24',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_25',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_26',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_27',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_28',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_29',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_30',size=(5,1),readonly=True,default_text='80'),
                sg.In(key='slope_2_31',size=(5,1),readonly=True,default_text='80')],

                [sg.T('Intercept'),
                sg.In(key='intercept_2_16',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_17',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_18',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_19',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_20',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_21',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_22',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_23',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_24',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_25',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_26',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_27',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_28',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_29',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_30',size=(5,1),readonly=True,default_text='0'),
                sg.In(key='intercept_2_31',size=(5,1),readonly=True,default_text='0')],

                [sg.Canvas(key='figCanvas')],
                [sg.T('File'), sg.In(key = "flash-data-filepath", readonly=True),sg.Button('Open File',key='-OPCalibration-')],
                [sg.T('Plot calibration of DAC'),sg.Combo(dac,key='-DAC-Calib-',default_value=1),sg.T('Channel'),sg.Combo(channel,key='-CHAN-Calib',default_value=0),sg.Button('Enter')]
              
            ]      
    #flash management tab
    tab4_layout = [

                [sg.T('Flash Management') ],
                [sg.T('FLASH ID'),
                    sg.In('ID1',size=(4,1),readonly=True, key="ManID"),
                    sg.In('ID2',size=(4,1),readonly=True, key="Capacity"),
                    sg.In('ID3',size=(4,1),readonly=True, key="MaxPages"),
                    sg.Button('QueryFlash')],
            
                [sg.T('Table'),sg.Combo(tables,key='-table-',default_value=0),
                    sg.T('Page '),   sg.Combo(pages, key='-page-' ,default_value=0),
                    sg.Button('ReadPoints'),sg.Button('WritePoints', disabled=True),
                    sg.Button('ErasePage',  font=font, button_color='red'),sg.T('...', key="elapsed", size=(10,1)),],
                
                [sg.T('Table'),  sg.Combo(tables,key='-table-mc2-',default_value=0),
                    sg.T('Angle'),  sg.Combo(angles, key='-angles-mc2-',size=(3,1),default_value=0),
                    sg.T("Inversion Time"), sg.In(key='inversionTime',size = (6,1),default_text='500'),sg.T("us"),
                    sg.Button("Send Angle", key = "angle-to-mc2") ],
                        
                [sg.T('File'), sg.In(key = "flash-data-filepath", readonly=True),
                    sg.Button("Open", key = "read-flash-file"),
                    sg.Button("Save", key = "write-flash-file"),
                    sg.Checkbox('Use calibration from this file',key="-overwrite-",default=False,size=(30,1))],

                [sg.Checkbox("Enable Debug mode",key="debug", default=False, size=(17,1)),sg.Button("Instruction")],
                        
                [sg.TabGroup([
                            [
                    sg.Tab('point0',  generate_point(0) , key='point0'), #per ogni tab devo generare un layout , ma è meglio usare una funzione così è più ordinato e modulare
                    sg.Tab('point1',  generate_point(1) , key='point1'),
                    sg.Tab('point2',  generate_point(2) , key='point2'),
                    sg.Tab('point3',  generate_point(3) , key='point3'),
                    sg.Tab('point4',  generate_point(4) , key='point4'),
                    sg.Tab('point5',  generate_point(5) , key='point5'),
                    sg.Tab('point6',  generate_point(6) , key='point6'),
                    sg.Tab('point7',  generate_point(7) , key='point7'),  
                    sg.Tab('point8',  generate_point(8) , key='point8'),  
                    sg.Tab('point9',  generate_point(9) , key='point9'),  
                    sg.Tab('point10', generate_point(10), key='point10'),  
                    sg.Tab('point11', generate_point(11), key='point11'),  
                    sg.Tab('point12', generate_point(12), key='point12'),  
                    sg.Tab('point13', generate_point(13), key='point13'),  
                    sg.Tab('point14', generate_point(14), key='point14'),  
                    sg.Tab('point15', generate_point(15), key='point15'), 
                            ]])
                        ],
            ]

    
              

    layout = [
                [
                        sg.T('Comm Port'),sg.Combo(serial_ports(), default_value='/dev/pts/2',key='-port-'), 
                        sg.T('Baud'),sg.Combo([9600,115200], default_value='115200',key='-baudrate-'),
                        sg.Button('Connect'), sg.Button('Disconnect'), sg.T('----', key="connection_status"),
                        sg.Exit(button_text = "Exit")
                ],
                
                [
                        sg.TabGroup([[ sg.Tab('1.Configuration', tab1_layout),
                            sg.Tab('2.Sensors', tab2_layout),
                            sg.Tab('3.DAC Calibration', tab3_layout),
                            sg.Tab('4.Flash Programming',tab4_layout)
                        ]])
                ],
            ]     
    
    window = sg.Window("Control Panel",
                        layout,
                        finalize=True,
                        resizable=True,)

    update_config_panel(window, config)

    serialChannel=Serial()

    # Run the Event Loop
    while True:
        event, values = window.read(500)
        if event in (sg.WIN_CLOSED, 'Quit', "Exit"):
            break

        if event == sg.TIMEOUT_EVENT:
            if serialChannel.is_open:
                buffer = receiveBuffer(serialChannel)
                if buffer: decode_input_data(window,buffer)
    
        elif event == "Instruction":
            sg.popup(text_debug,title = 'Debug Mode')
        
        elif event == "Connect":
            serialPort = window["-port-"].get() #mette dentro la variabile serialPort il valore contenuto in quel momento dalla chiave "-port-"
            baudRate = window["-baudrate-"].get()
            serialChannel = Serial( serialPort, 
                                    baudRate,
                                    timeout=None,
                                    xonxoff=True,
                                    stopbits=STOPBITS_ONE,
                                    bytesize = EIGHTBITS, 
                                    writeTimeout=0) #ensure non-blocking
            if serialChannel.is_open :
               for i in range(255):
                   serialChannel.write(b'\n')

               window["WRITE-CONFIG"].update(disabled=False)
               window["READ-CONFIG"].update(disabled=False)
               window["Connect"].update(disabled=True, button_color="red") 
               window["connection_status"].update("Connected")
               connectionEnabled = True
        elif event == "Disconnect": 
            window["WRITE-CONFIG"].update(disabled=True)
            window["READ-CONFIG"].update(disabled=True)
            window["connection_status"].update("---")
            #window["MonitorBtn"].update(disabled=True, text="Start")
            window["Connect"].update(disabled=False, button_color="green") 
            connectionEnabled = False
            serialChannel.close

        elif event == "READ-CONFIG":
            serialChannel.write(READ_CONFIG)
            serialChannel.write(END_COMMAND)
            serialChannel.flush()
            buffer = receiveBuffer(serialChannel)
            if buffer: decode_input_data(window,buffer)

        elif event == "QueryFlash":
            serialChannel.write(FLASH_INFO)
            serialChannel.flush()
            serialChannel.write(END_COMMAND)
            sleep(0.2)
            buffer = receiveBuffer(serialChannel)
            
            flash_id = unpack_from(">5B",buffer)
            f_id=[]
            for fid in range(len(flash_id)):
                f_id.append(hex(flash_id[fid]))
            
            window["ManID"].update(f_id[1])
            window["Capacity"].update(f_id[2])
            window["MaxPages"].update(f_id[3])
            window.Refresh()

        elif event == "WRITE-CONFIG":
            config=build_config_from_ui(window)
            data = pack(config_format, *config._asdict().values())
            serialChannel.write(WRITE_CONFIG + data[0:60])
            sleep(0.3)
            serialChannel.write(data[60:])
            serialChannel.flush()
            sleep(1)
            buffer = receiveBuffer(serialChannel)
            if buffer: decode_input_data(window,buffer)
        elif event=="EnableMonitoring":
            serialChannel.write(R_START)
            serialChannel.write(END_COMMAND)
            serialChannel.flush()
            sleep(1)
        elif event=="DisableMonitoring":
            serialChannel.write(R_STOP)
            serialChannel.write(END_COMMAND)
            serialChannel.flush()
            sleep(1)

        elif event =="-OPCalibration-":
            filename = sg.popup_get_file("Select file to read", save_as=False, default_path = window["flash-data-filepath"].get())

            if filename:
                try:
                    window["flash-data-filepath"].update(filename)
                    ctable = pd.read_csv(filename)
                except NotImplementedError:
                    pass
                
                window["vbias"].update(str(ctable.iat[0,0])) # takes the element in row 0, column 0 and put it in the field "bias" iat is a pandas method that take the value in a row, column
                
                for i in range(32):
                    X = ctable.iloc[:,i+3] #with iloc i select all the rows ":" and the index of column 'i+3'
                    y = ctable.DAC_decimal
                    X  = sm.add_constant(X)
                    model = sm.OLS(y,X).fit() # OLS is the linear regression function
                    window["slope_1_{}".format(i)].update(str(model.params[1]))
                    window["intercept_1_{}".format(i)].update(str(model.params[0]))

                for j in range(32):
                    X = ctable.iloc[:,j+35] #with iloc i select all the rows ":" and the index of column 'j+35'
                    y = ctable.DAC_decimal
                    X  = sm.add_constant(X)
                    model = sm.OLS(y,X).fit()
                    window["slope_2_{}".format(j)].update(str(model.params[1]))
                    window["intercept_2_{}".format(j)].update(str(model.params[0]))



        elif event == "angle-to-mc2":
    
            table=window['-table-mc2-'].get()
            point=window['-angles-mc2-'].get()
            inv_time=round((int(window["inversionTime"].get()))) #inv_time=round((int(window["inversionTime"].get()))/100)
            print("table {0}, point {1}".format(table,point))  # {0},{1} indicano dei simboli sostituibili dai valori posti come anrgomento dnetro format print("When you multiply {0} and {1} or {0} and {2}, the result is {0}".format(0,1,2))
            print(inv_time)
            data=R_ANGLE_TO_MC2 + pack('>2H', table,point)+ pack('>1H',inv_time)
            
            for y in range(len(data)):
                print(hex(data[y]), end=" ")
            
            serialChannel.write(data)
            serialChannel.write(b'\n')
            serialChannel.flush() # svuoto il buffer
            window.Refresh()
            #struct.pack(format, v1, v2, ...)
            #Return a bytes object containing the values v1, v2, … packed according to the format string format.
            #The arguments must match the values required by the format exactly.
            #'<2h' significa little endian e tipo short
        
        elif event == "-SERIAL-":
            #text = window['-monitor-']
            try:
                log(values[event],"RECV SERIAL")
            except NotImplementedError:
                pass

        elif event == "read-flash-file":
            filename = sg.popup_get_file("Select file to read", save_as=False, default_path=window["flash-data-filepath"].get())
            #raw_data = window['-flash-data-']
            if filename:
                try:
                    window["flash-data-filepath"].update(filename)
                    flash_table_data=[];
                    file = open(filename,'r')
                    lines= file.readlines()
                    file.close()
                    index=0
                     
                    if values['-overwrite-'] == True:
                        window["vbias"].update(lines[0].strip())

                        tmp_data=lines[1].strip().split(",")
                        for i in range(32):
                            window["slope_{0}_{1}".format(1,i)].update(tmp_data[i])
                        for i in range(32):
                            window["slope_{0}_{1}".format(2,i)].update(tmp_data[i+32])

                        tmp_data=lines[2].strip().split(",")
                        for i in range(32):
                            window["intercept_{0}_{1}".format(1,i)].update(tmp_data[i])
                        for i in range(32):
                            window["intercept_{0}_{1}".format(2,i)].update(tmp_data[i+32])
                    
                    for next_line in lines[3:]:
                        tmp_data=next_line.strip().split(",")
                        for i in range(16):
                            window["dac1_{0}_{1}".format(index,i)].update(tmp_data[i])
                        for i in range(16,32):
                            window["dac1_{0}_{1}".format(index,i)].update(tmp_data[i])
                        for i in range(16):
                            window["dac2_{0}_{1}".format(index,i)].update(tmp_data[i+32])
                        for i in range(16,32):
                            window["dac2_{0}_{1}".format(index,i)].update(tmp_data[i+32])
                        for i in range(4): #i pezzi del file sono stati divisi dalle virgole, quindi l'ultimo blocco ho messo 4 virgole ogni 25 byte in modo tale da spezzare le 4 righe che ho messo nell'interfaccia
                            window["rf_{0}_{1}".format(index,i)].update(tmp_data[64+i])
                            #window["rf_{0}_{1}".format(index,0)].update(tmp_data[64])
                        index+=1
                except NotImplementedError:
                    pass
        elif event == "write-flash-file":
            filename = sg.popup_get_file("Select file to store data", save_as=True)
            file=open(filename,"w")
            #         for line in raw_data.get():
            #             file.write(line[1])
            #             file.write("\r\n")
            #         file.close()
            
            # write bias
            file.write(str(window["vbias"].get()))
            #write slope
            file.write("\n")
            for slope_index in range(32):
                file.write(str(window["slope_{0}_{1}".format(1,slope_index)].get()))
                file.write(",")
            for slope_index in range(32):
                file.write(str(window["slope_{0}_{1}".format(2,slope_index)].get()))
                if(slope_index!=31):
                    file.write(",")
            file.write("\n")
            
            #write intercept
            for intercept_index in range(32):
                file.write(str(window["intercept_{0}_{1}".format(1,intercept_index)].get()))
                file.write(",")
            for intercept_index in range(32):
                file.write(str(window["intercept_{0}_{1}".format(2,intercept_index)].get()))
                if(intercept_index!=31):
                    file.write(",")
            file.write("\n")

            for i in range(16):
                point_data = []
                for index in range(32):
                    file.write(str(window["dac1_{0}_{1}".format(i,index)].get()))
                    file.write(",")
                for index in range(32):
                    file.write(str(window["dac2_{0}_{1}".format(i,index)].get()))
                    file.write(",")
                for index in range(4):
                    file.write(str(window["rf_{0}_{1}".format(i,index)].get()))
                    if(index!=3):
                        file.write(",")
                if (i!=15):
                    file.write("\n")
            file.close()    
        
        elif event == "WritePoints":
            try:
                table = int(window['-table-'].get())
                page = int(window['-page-'].get())
                vbias = float(window['vbias'].get())
                
                #metto tutti i coefficienti di slope dentro una lista
                slope = []
                for dac_slope in range(2):
                    for slope_index in range(32):
                        slope.append(float(window["slope_{0}_{1}".format(dac_slope+1,slope_index)].get()))
                
                #metto tutti i termini di intercept dentro una lista
                intercept = []
                for dac_intercept in range(2):
                    for intercept_index in range(32):
                        intercept.append(float(window["intercept_{0}_{1}".format(dac_intercept+1,intercept_index)].get()))
                
                for i in range(16): #16 sono i punti
                    
                    point_data = []
                    for index in range(32):

                        if values['debug'] == True:

                            dac_decimal  = int(window["dac1_{0}_{1}".format(i,index)].get()) #prendo il valore dentro la finestra e lo trasformo in intero
                            if dac_decimal < 0 or dac_decimal > 15999: #16384
                                raise ValueError("Point {}: DAC1 channel {} out of range".format(i,index))
                            point_data.append(dac_decimal)
                        
                        else:
                            voltage     = float(window["dac1_{0}_{1}".format(i,index)].get())
                            dac_decimal = round((voltage + vbias)*slope[index]+intercept[index])
                            point_data.append(dac_decimal) 
                    
                    for index in range(32):

                        if values['debug'] == True:
                            
                            dac_decimal   = int(window["dac2_{0}_{1}".format(i,index)].get())
                            if dac_decimal < 0 or dac_decimal > 15999:
                                raise ValueError("Point {}: DAC2 channel {} out of range".format(i,index))
                            point_data.append(int(dac_decimal))
                        
                        else:
                            voltage     = float(window["dac2_{0}_{1}".format(i,index)].get())
                            dac_decimal = round((voltage + vbias)*slope[index]+intercept[index])
                            point_data.append(dac_decimal) 

                    rf_data=[]
                    for rf_index in range(4): # 4 index sono le righe delle schede rf, ricordo che 'i' sono i punti del for precedente
                        tmp_rf=window["rf_{0}_{1}".format(i,rf_index)].get()
                        if len(tmp_rf) != 50: # prima era 25, non ricordo perché, ora metto 50 perché i byte di ogni riga sono 50 
                            raise ValueError("Point {}: RF channel {} invalid data".format(i,index))
                        rf_data += tmp_rf

                    point = (page*16)+i
                    log("write point {} of table {} ".format(point,table))
                    log("|".join(map(str,point_data)))

                    data = R_WPOINT + pack('>2H', table,point) +  pack('>64H', *point_data) #metto dentro data la richesta di scrittura della flash, l'indirzzo da leggere e i punti formattati
                    
                    q = 0
                    for q in range(100):
                        hexvalue="".join(rf_data[(q*2):(q*2+2)]) #prendo le coppie di caratteri e li metto insieme come byte
                        data += pack('c', int(hexvalue,base=16).to_bytes(1, byteorder='big')) #aggiungo dentro data i dati delle schede rf, ho meso big al posto di little perché il micro2 si aspetta un big endian
                    
                    data += b'\n'
                    
                    print(data)
                    
                    # for sw in range(len(data)):
                    #     serialChannel.write(data[sw])
                    #     sleep(0.1)
                    # serialChannel.flush()
                    #prima di cancellare questi dati vedere se col for va meglio                    
                    serialChannel.write(data[0:30])
                    serialChannel.flush()
                    sleep(0.3)
                    serialChannel.write(data[30:60])
                    serialChannel.flush()
                    sleep(0.3)
                    serialChannel.write(data[60:90])
                    serialChannel.flush()
                    sleep(0.3)
                    serialChannel.write(data[90:120])
                    serialChannel.flush()
                    sleep(0.3)
                    serialChannel.write(data[120:150])
                    serialChannel.flush()
                    sleep(0.3)
                    serialChannel.write(data[150:180])
                    serialChannel.flush()
                    sleep(0.3)
                    serialChannel.write(data[180:])
                    serialChannel.flush()
                    sleep(0.3)
                    serialChannel.flush()

                    buffer = receiveBuffer(serialChannel)
                    
                    if buffer :
                        log([len(buffer),buffer]) 
                        elapsed = unpack_from(">1c",buffer, offset=1)
                        window['point{0}'.format(i)].update(title='*{0}*'.format(i))
                    window.Refresh()

                window['WritePoints'].update(disabled=True)
            except ValueError as e: 
                 sg.popup(e)
                 log(e, "envet {} ERROR".format(event))
                 pass     
        elif event == "ReadPoints":
            table = int(window['-table-'].get())
            page = int(window['-page-'].get())
            vbias = float(window['vbias'].get())

            #metto tutti i coefficienti di slope dentro una lista
            slope = []
            for dac_slope in range(2):
                for slope_index in range(32):
                    slope.append(float(window["slope_{0}_{1}".format(dac_slope+1,slope_index)].get()))
                
            #metto tutti i termini di intercept dentro una lista
            intercept = []
            for dac_intercept in range(2):
                for intercept_index in range(32):
                    intercept.append(float(window["intercept_{0}_{1}".format(dac_intercept+1,intercept_index)].get()))

            flash_table_data=[]
            #raw_data=window["-flash-data-"]
            for i in range(16):
                serialChannel.write(EMPTY_BUFFER)
                serialChannel.flush()
                sleep(0.3)
                point = (page*16)+i
                log("ask point  {} of table {}".format(point,table))
                data=R_RPOINT + pack('>2H',table,point) #metto in data la richiesta di lettura della flash e l'indirizzo --> da controllare, forse sta qua il bug sulla scheda
                
                print(data) # PER TEST!!!!!
                
                serialChannel.write(data) #mando la richiesta al micro 1
                serialChannel.flush() #svuoto il buffer
                sleep(0.3) #aspetto 300 ms
                buffer = receiveBuffer(serialChannel) #metto nel buffer la risposta
                sg.Print(buffer)
                #visualize the buffer in hex numer
               
                buffer_int = unpack_from(">230B",buffer)
                b_int=[]
                for z in range(len(buffer_int)):
                    b_int.append(hex(buffer_int[z]))

                log([len(b_int),b_int])
                
                #visualize the buffer in decimal numbers
                point_data_DAC=unpack_from(">68H",buffer, offset=1) #spacchetto i dati in unsegned short ordnati big endian e li metto nella lista point_data
                print(point_data_DAC) #stampo a monitor i dati dei DAC in valore decimale (es. 8000 --> 100 volt)

                # questo for gestisce i dati in modalità debug, quindi dac_decimal
                if values['debug'] == True:
                    for p1 in range(32):
                        window["dac1_{0}_{1}".format(i,p1)].update(point_data_DAC[p1])
                        window.Refresh()
                    for p2 in range(32):
                        window["dac2_{0}_{1}".format(i,p2)].update(point_data_DAC[p2+32])
                        window.Refresh()

                    window["rf_{0}_{1}".format(i,0)].update(str(buffer[129:154]))
                    window["rf_{0}_{1}".format(i,1)].update(str(buffer[154:179]))
                    window["rf_{0}_{1}".format(i,2)].update(str(buffer[179:204]))
                    window["rf_{0}_{1}".format(i,3)].update(str(buffer[204:229]))
                
                else:
                    for p3 in range(32):
                        dac_decimal = float(point_data_DAC[p3])
                        voltage = round((dac_decimal - intercept[p3])*(1/slope[p3])-vbias,2)
                        window["dac1_{0}_{1}".format(i,p3)].update(voltage)
                        window.Refresh()
                    for p4 in range(32):
                        dac_decimal = float(point_data_DAC[p4+32])
                        voltage = round((dac_decimal - intercept[p4+32])*(1/slope[p4+32])-vbias,2)
                        window["dac2_{0}_{1}".format(i,p4)].update(voltage)
                        window.Refresh()

                    #print the RF strings without the binary hex format
                    rf_flow_1 = str(buffer[129:154]).replace('\\x','').replace('\'','').replace('b','',1)
                    window["rf_{0}_{1}".format(i,0)].update(rf_flow_1)
                    rf_flow_2 = str(buffer[154:179]).replace('\\x','').replace('\'','').replace('b','',1)
                    window["rf_{0}_{1}".format(i,1)].update(rf_flow_2)
                    rf_flow_3 = str(buffer[179:204]).replace('\\x','').replace('\'','').replace('b','',1)
                    window["rf_{0}_{1}".format(i,2)].update(rf_flow_3)
                    rf_flow_4 = str(buffer[204:229]).replace('\\x','').replace('\'','').replace('b','',1)
                    window["rf_{0}_{1}".format(i,3)].update(rf_flow_4)
                
                
                window['point{0}'.format(i)].update(title='point{0}'.format(i))

        elif event == "ErasePage":
            try:
                table = int(window['-table-'].get())    
                page = int(window['-page-'].get())
                window['elapsed'].update("")
                #window['-flash-data-'].update("".rstrip())
                log("erasing page  {} of table {}".format(page,table))
                data=R_ERASE_4K + pack('>2h', table,page)
                print(data)                                                           # per test
                serialChannel.write(data)
                serialChannel.write(END_COMMAND)
                serialChannel.flush()
                sleep(0.5)
                buffer  = receiveBuffer(serialChannel)
                if buffer:
                    log([len(buffer),buffer])
                    elapsed = unpack_from("<1h",buffer, offset=1)
                    window['elapsed'].update("{0} ms".format(elapsed[0]))
                    window['WritePoints'].update(disabled=False)
                
            except ValueError as e:
                sg.popup("wrong data")
                log(e, "envet {} ERROR".format(event))
                pass   
            
            window.refresh()

        elif event == "eraseChip":
            serialChannel.write(R_ERASE_CHIP)
            serialChannel.flush()
            pass
        
        else:
            print("event was:", event)
            pass
            # A timeout signals that all buttons have been released so clear the status display
            #window['-monitor-'].update('')

    window.close()

if __name__ == '__main__':
    sg.theme('DefaultNoMoreNagging') 
    main()    

#esempio pack
# >>> from struct import *
# >>> pack('<2h',0,1)
# b'\x00\x00\x01\x00'
# >>> pack('<2h',3,1)  #usato nel pannello, i byte sono invertiti come little endian
# b'\x03\x00\x01\x00'
# >>> pack('>2h',3,1)
# b'\x00\x03\x00\x01'

#esempio unpack
# struct.unpack_from(fmt, buffer[, offset=0])
# Unpack the buffer according to the given format. The result is a tuple even if it contains exactly one item.
# The buffer must contain at least the amount of data required by the format (len(buffer[offset:]) must be at least calcsize(fmt)).


#Join all items in a tuple into a string, using a hash character as separator:
# >>> tupla = ("john","peter","vicky")
# >>> x = "x".join(tupla)
# >>> x
# 'john#peter#vicky'

#map
# map applica la funzione del primo argomento a tutti gli elementi messi come secondo argomento
# numbers = [2, 4, 6, 8, 10]

# # returns square of a number
# def square(number):
#   return number * number

# # apply square() function to each item of the numbers list
# squared_numbers_iterator = map(square, numbers)

#window["rf_{0}_{1}".format(i,0)].update("".join(map(str,point_data[64:])))



# >>> rf_data_1
# b'\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff'
# >>> rf_data_2=buffer[180:230]
# >>> len(rf_data_2)
# 50
# >>> rf_data_2
# b'\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff'
# >>> rf_data_2=buffer[180:231]
# >>> rf_data_2
# b'\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\n'
# >>> len(rf_data_2)
# 51
# >>> dac_data=buffer[2:130]
# >>> len(dac_data)
# 128


# >>> a.encode('utf-8').hex()
# '421f'
# >>> a='\x1f\x40'
# >>> a.encode('utf-8').hex()
# '1f40'
# >>> a=b'\x42\x1f'
# >>> a.encode('utf-8').hex()
# Traceback (most recent call last):
#   File "<stdin>", line 1, in <module>
# AttributeError: 'bytes' object has no attribute 'encode'
# >>> str(a)
# "b'B\\x1f'"
# >>>
# >>> chr(a)
# Traceback (most recent call last):
#   File "<stdin>", line 1, in <module>
# TypeError: an integer is required (got type bytes)
# >>> a = b'B\x1f
# >>> a.decode('utf-8')
# 'B\x1f'
# >>> b=a.decode('utf-8')
# >>> b.encode('utf-8').hex()
# '421f'

#per stampare tutto in linea con uno spazio
# print(hex(buffer_int[i]),end=" ")