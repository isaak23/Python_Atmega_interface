# img_viewer.py
#https://pysimplegui.readthedocs.io/en/latest/call%20reference/

from datetime import datetime
from os import *
import struct
from time import sleep
import PySimpleGUI as sg
import os.path
#import threading #il vecchio pannello usava un thread per aggiornare lo stato, poi rimosso
from PySimpleGUI.PySimpleGUI import T, Multiline, popup
from serial import *
import serial.tools.list_ports
import glob
import numpy as np
from struct import *
from struct import Struct
from tkinter.font import Font

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
flash_table_data=[]
serialChannel=Serial()
#config_format='<8h8h8f8f8f6h13h' 
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


Flash = namedtuple("Flash",["ManID","Capacity", "MaxPages"])

flash_format='<hll'

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


# def init_connection(window):
    
#     #global serialPort,baudRate ,connectionEnabled, serialChannel
#     serialPort = window["-port-"].get()
#     baudRate = window["-baudrate-"].get()
    
#     #apro un thread per parlare con la seriale
#     serialChannel = Serial(serialPort , baudRate, timeout=0, writeTimeout=0) #ensure non-blocking
#     if serialChannel.open:
#         connectionEnabled=True
#         serial_thread = threading.Thread(target=scanSerial_char, args=(window,connectionEnabled), daemon=True)
#         serial_thread.start()
#         window["Connect"].update(disabled=True, button_color="red") 
#         window["connection_status"].update("Connected")
#         return True
#     else:
#         sg.popup('Communication Error')
#         log('Communication Error')

def decode_input_data(window,data):
    cmd = data[:1] #first char define the type of data
    if cmd == READ_CONFIG:
        config = Config._make( unpack_from(config_format, data, offset=1))
        print(config)
        update_config_panel(window, config)
        return 0
    if cmd == FLASH_INFO:
        flash = Flash._make( unpack_from(flash_format, data, offset=2))
        update_config_panel(window, flash)
        return 0
    elif cmd == MONITOR_DATA:
        sensor_data=unpack_from('<7h', data, offset=1)
        print(sensor_data)
        update_sensor_panel(window, sensor_data)
        return 0
    # elif cmd == R_RPOINT:
    #     point_data=unpack_from("<114h",data, offset=2)
    #     print(point_data)
    #     update_point_panel(window, point_data)
    #     return 0
    else:
        log(data, "*** OUT from MC1 not valid ***" )
    return 1



# vecchia funzione che usava il thread, va cancellata
# def scanSerial_char( window, connectionEnabled):
#     commands={
#         MONITOR_DATA    : {"size":15},
#         READ_CONFIG     : {"size":112},
#         R_RPOINT        : {"size":230},
#         FLASH_INFO      : {"size":12},
#         R_ANGLE_TO_MC2  : {"size":7},
#     }
#     buffer=b''
#     skipped=b''
#     opcode=b''
#     max_size=1
#     time.sleep(0.1)
#     try:
#         while True:
#             while (serialChannel and serialChannel.in_waiting > 0): #if incoming bytes are waiting to be read from the serial input buffer
#                 current_char = serialChannel.read()
#                 if (opcode == b'' and current_char in commands): #first char of command
#                     opcode   = current_char
#                     max_size = commands[opcode]["size"]
#                     #skipped  = b''
#                     buffer  += current_char
#                     log(max_size, "scanSerial_char maxsize")
#                 elif (opcode == b'' and not(current_char in commands)):   #skip    
#                     skipped += current_char
#                 elif len(buffer) >= max_size+1:  #opcode is not empty
#                     log(buffer, "scanSerial_char")
#                     try:
#                         decode_input_data(window,buffer)
#                         buffer = b''   
#                     except struct.error as e:
#                         x=True
#                         pass
#                     finally:
#                         buffer = b''
#                         opcode = b''
#                         max_size = 0
#                 else:
#                     buffer+= current_char
#                     #log(buffer, "scanSerial_char buffer")
#                     #time.sleep(0.1)        
#             if (max_size >0 and len(buffer) >= max_size): decode_input_data(window,buffer)
#             if skipped != b'' : log(    skipped , "scanSerial_char skipped")  
#             if buffer  != b'' : log(len(buffer) , "undecoded before size"  ); #decode_input_data(window,buffer)
#             buffer=b''
#             skipped=b''
#             opcode = b''
#             max_size = 0

#             time.sleep(0.5) # Optional: sleep 10 ms (0.01 sec) once per loop to let other threads on your PC run during this time. 
#             if window["connection_status"].get()=="---":
#                 serialChannel.close()
#                 return
           
            
#     except NotImplementedError as e:
#         print(e)
#         serialChannel.close()
#         window["connection_status"]="Error"
        

def receiveBuffer(serialChannel):
    commands={
        MONITOR_DATA    : {"size":15, "start":START_RESPONSE},
        READ_CONFIG     : {"size":112, "start":START_RESPONSE},
        R_RPOINT        : {"size":230, "start":START_RESPONSE},
        FLASH_INFO      : {"size":12, "start":START_RESPONSE},
        R_ANGLE_TO_MC2  : {"size":7, "start":START_RESPONSE},
#       START_RESPONSE  : {"size":255},
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
    for k in config._fields:
        try:
            window[k].update(getattr(config,k))
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
    volate_values = [x/20 for x in range(20000)]
    tmp_layout=[
                [sg.T('DAC Voltage resolution is 10 mV (0.01V)'),],
                [sg.T('DAC1 00-15'),
                      sg.Spin(volate_values,key='dac1_{0}_0'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_1'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_2'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_3'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_4'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_5'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_6'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_7'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_8'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_9'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_10'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_11'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_12'.format(index), size = (6, 1)), 
                      sg.Spin(volate_values,key='dac1_{0}_13'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_14'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_15'.format(index), size = (6, 1)),
                      ],
                [sg.T('DAC1 16-31'),
                      sg.Spin(volate_values,key='dac1_{0}_16'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_17'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_18'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_19'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_20'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_21'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_22'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_23'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_24'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_25'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_26'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_27'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_28'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_29'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_30'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac1_{0}_31'.format(index), size = (6, 1)),

                      ],
        [sg.T('DAC2 00-15'),
                      sg.Spin(volate_values,key='dac2_{0}_0'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_1'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_2'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_3'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_4'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_5'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_6'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_7'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_8'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_9'.format(index), size =  (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_10'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_11'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_12'.format(index), size = (6, 1)), 
                      sg.Spin(volate_values,key='dac2_{0}_13'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_14'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_15'.format(index), size = (6, 1)),
                      ],
                [sg.T('DAC2 16-31'),
                      sg.Spin(volate_values,key='dac2_{0}_16'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_17'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_18'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_19'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_20'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_21'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_22'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_23'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_24'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_25'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_26'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_27'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_28'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_29'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_30'.format(index), size = (6, 1)),
                      sg.Spin(volate_values,key='dac2_{0}_31'.format(index), size = (6, 1)),
                      ],
        [sg.T('RF data must contains 25 nibble [0-F]'),],
        [sg.T('RF 0-3'),
                    sg.In(key='rf_{0}_0'.format(index), size = (30, 1)),
                    sg.In(key='rf_{0}_1'.format(index), size = (30, 1)),
                    sg.In(key='rf_{0}_2'.format(index), size = (30, 1)),
                    sg.In(key='rf_{0}_3'.format(index), size = (30, 1)),
        ],
        [sg.T('RF 4-7'),
                    sg.In(key='rf_{0}_4'.format(index), size = (30, 1)),
                    sg.In(key='rf_{0}_5'.format(index), size = (30, 1)),
                    sg.In(key='rf_{0}_6'.format(index), size = (30, 1)),
                    sg.In(key='rf_{0}_7'.format(index), size = (30, 1)),
                                        
                    ],
        # [sg.T('RF '),
        #               sg.In(key='rf_{0}_0'.format(index), size = (50, 1), disabled=True),
        # #             sg.In(key='rf_{0}_5'.format(index), size = (25, 1)),
        # #             sg.In(key='rf_{0}_6'.format(index), size = (25, 1)),
        # #             sg.In(key='rf_{0}_7'.format(index), size = (25, 1)),
                                        
        #              ],
    ]
    return tmp_layout

def main():
    global config

    font = "Arial 13"
    font_family, font_size = font = ('Courier New', 10)
    sg.set_options(font=font)

    vBias = 100
    v_lsb = 0.0125


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
    tab3_layout =  [
                        [sg.T('Flash Management')],
                        [sg.T('FLASH ID'),
                            sg.T('ManID', key="ManID"),
                            sg.T('Capacity', key="Capacity"),
                            sg.T('MaxPages', key="MaxPages"),
                            sg.Button('QueryFlash'),],
                        [ sg.T('Table'),  sg.Combo(tables,key='-table-',default_value=0),
                          sg.T('Page'),   sg.Combo(pages, key='-page-' ,default_value=0),
                          sg.Button('ReadPoints'),sg.Button('WritePoints', disabled=True),
                          sg.Button('ErasePage',  font=font, button_color='red'),sg.T('...', key="elapsed", size=(10,1)),
                        ],
                        
                        [sg.T('File'), sg.In(key = "flash-data-filepath", readonly=True),
                            sg.Button("Open", key = "read-flash-file"),
                            sg.Button("Save", key = "write-flash-file"),
                        ],
                        [   sg.TabGroup([
                            [
                            sg.Tab('point0',  generate_point(0) , key='point0'),
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
                                                       ]

                            ])
                          ]
                        ]

    tab5_layout =  [
                        [sg.T('Monitor')],
                        [sg.Multiline(default_text = "",autoscroll = True,size = (80, 10),
                                      auto_size_text = True, key = "-monitor-", auto_refresh = False,
                                      echo_stdout_stderr=False)]
                    ]
    tab6_layout =  [
                        [sg.T('Microcontroller2')],
                        [sg.T('Table'),  sg.Combo(tables,key='-table-mc2-',default_value=0),
                         sg.T('Angle'),  sg.Combo(angles, key='-angles-mc2-' ,default_value=0),
                         sg.Button("Send Angle", key = "angle-to-mc2"),
                        ]
    ]
    layout = [
                    [
                            sg.T('Comm Port'),sg.Combo(serial_ports(), default_value='/dev/pts/2',key='-port-'), 
                            sg.T('Baud'),sg.Combo([9600,115200], default_value='115200',key='-baudrate-'),
                            sg.Button('Connect'), sg.Button('Disconnect'), sg.T('----', key="connection_status"),
                            sg.Exit(button_text = "Exit")
                    ],
                    [
                        sg.TabGroup([[ sg.Tab('Configuration', tab1_layout),
                            sg.Tab('Flash Programming', tab3_layout),
                            sg.Tab('Sensors', tab2_layout),
                            sg.Tab('Microcontroller2', tab6_layout),
                        ]])
                    ],
                     tab5_layout
                    ]     

    
    window = sg.Window("Control Panel", layout, finalize=True)

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
        elif event == "Connect":
            serialPort = window["-port-"].get()
            baudRate = window["-baudrate-"].get()
            serialChannel = Serial(serialPort , baudRate, timeout=0, writeTimeout=0) #ensure non-blocking
            if serialChannel.is_open :
               for i in range(255):
                   serialChannel.write(b'\n')
               window["WRITE-CONFIG"].update(disabled=False)
               window["READ-CONFIG"].update(disabled=False)
               #window["MonitorBtn"].update(disabled=False, text="Start")
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
            buffer = receiveBuffer(serialChannel)
            if buffer: decode_input_data(window,buffer)

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
        elif event == "angle-to-mc2":

                table=window['-table-mc2-'].get()
                point=window['-angles-mc2-'].get()
                print("table {0}, point {1}".format(table,point))
                data=R_ANGLE_TO_MC2 + pack('<2h', table,point)
                print(data)
                serialChannel.write(data)
                serialChannel.write(b'\n')
                serialChannel.flush()

                
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
                    file = open(filename,"r")
                    lines= file.readlines()
                    file.close()
                    index=0
                    for next_line in lines:
                        tmp_data=next_line.strip().split(",")
                        for i in range(16):
                            window["dac1_{0}_{1}".format(index,i)].update(tmp_data[i])
                        for i in range(16,32):
                            window["dac1_{0}_{1}".format(index,i)].update(tmp_data[i])
                        for i in range(16):
                            window["dac2_{0}_{1}".format(index,i)].update(tmp_data[i+32])
                        for i in range(16,32):
                            window["dac2_{0}_{1}".format(index,i)].update(tmp_data[i+32])
                        for i in range(8):
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
            for i in range(16):
                point_data = []
                for index in range(32):
                    file.write(str(window["dac1_{0}_{1}".format(i,index)].get()))
                    file.write(",")
                for index in range(32):
                    file.write(str(window["dac2_{0}_{1}".format(i,index)].get()))
                    file.write(",")
                for index in range(8):
                    file.write(str(window["rf_{0}_{1}".format(i,index)].get()))
                    file.write(",")
                file.write("\r\n")
            file.close()    
        elif event == "WritePoints":
            try:
                table = int(window['-table-'].get())
                page = int(window['-page-'].get())

                for i in range(16):
                    point_data = []
                    for index in range(32):
                        #(vBias-voltage)  / v_lsb --> decimal value for DAC
                        voltage     = window["dac1_{0}_{1}".format(i,index)].get()
                        voltage     = float(voltage)
                        dac_decimal = round((vBias - voltage) / v_lsb,2)
                        if dac_decimal < 0 or dac_decimal > 16000:
                            raise ValueError("Point {}: DAC1 channel {} out of range".format(i,index))
                        point_data.append(int(dac_decimal))

                    for index in range(32):
                        #(vBias-voltage)  / v_lsb --> decimal value for DAC
                        voltage     = float(window["dac2_{0}_{1}".format(i,index)].get())
                        dac_decimal = round((vBias - voltage) / v_lsb,2)
                        if dac_decimal < 0 or dac_decimal > 16000:
                            raise ValueError("Point {}: DAC2 channel {} out of range".format(i,index))
                        point_data.append(int(dac_decimal))

                    rf_data=[]
                    for index in range(8):
                        tmp_rf=window["rf_{0}_{1}".format(i,index)].get()
                        if len(tmp_rf) != 25:
                            raise ValueError("Point {}: RF channel {} invalid data".format(i,index))
                        rf_data += tmp_rf

                    point = (page*16)+i
                    log("write point {} of table {} ".format(point,table))
                    log("|".join(map(str,point_data)))

                    data = R_WPOINT + pack('<2H', table,point) +  pack('<64H', *point_data) 

                    for i in range(100):
                        hexvalue="".join(rf_data[(i*2):(i*2+2)])
                        data += pack('c', int(hexvalue,base=16).to_bytes(1, byteorder='little'))
                    
                    data += b'\n'

                    #print (data)
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
                        elapsed = unpack_from("<1c",buffer, offset=1)
                        window['point{0}'.format(i)].update(title='*{0}*'.format(i))

                window['WritePoints'].update(disabled=True)
            except ValueError as e: 
                 sg.popup(e)
                 log(e, "envet {} ERROR".format(event))
                 pass     
        elif event == "ReadPoints":
            table = int(window['-table-'].get())
            page = int(window['-page-'].get())
            flash_table_data=[]
            #raw_data=window["-flash-data-"]
            for i in range(16):
                serialChannel.write(EMPTY_BUFFER)
                serialChannel.flush()
                sleep(0.3)
                point = (page*16)+i
                log("ask point  {} of table {}".format(point,table))
                data=R_RPOINT + pack('<2H', table,point)
                serialChannel.write(data)
                serialChannel.flush()
                sleep(0.3)
                buffer = receiveBuffer(serialChannel)
                log([len(buffer),buffer])
                point_data=unpack_from("<114H",buffer, offset=2)
                print(point_data)
                for p in range(32):
                    dac_decimal = float(point_data[p])
                    dac_decimal *= v_lsb
                    voltage     = round(vBias - dac_decimal,2)
                    window["dac1_{0}_{1}".format(i,p)].update(voltage)
                    dac_decimal = point_data[p+32]
                    dac_decimal *= v_lsb
                    voltage     = round(vBias - dac_decimal,2)
                    window["dac2_{0}_{1}".format(i,p)].update(voltage)
                window["rf_{0}_{1}".format(i,0)].update("".join(map(str,point_data[64:])))
                window['point{0}'.format(i)].update(title='point{0}'.format(i))

        elif event == "ErasePage":
            try:
                table = int(window['-table-'].get())
                page = int(window['-page-'].get())
                window['elapsed'].update("")
                #window['-flash-data-'].update("".rstrip())
                log("erasing page  {} of table {}".format(page,table))
                data=R_ERASE_4K + pack('<2h', table,page)
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



