# img_viewer.py
#https://pysimplegui.readthedocs.io/en/latest/call%20reference/

from datetime import datetime
#from os import * #comando pericoloso in quanto import * importa un altro metodo per aprire i file che richide un integer
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


def update_config_panel(window, config):
    for k in config._fields:
        try:
            window[k].update(getattr(config,k))
        except:
            pass
    table = window["-table-"].get()

def decode_input_data(window,data):
    cmd = data[:1] #first char define the type of data

    if cmd == FLASH_INFO:
        flash = Flash._make( unpack_from(flash_format, data, offset=2))
        update_config_panel(window, flash)
        return 0
    else:
        log(data, "*** OUT from MC1 not valid ***" )
    return 1

def receiveBuffer(serialChannel):
    commands={
        MONITOR_DATA    : {"size":15, "start":START_RESPONSE},
        READ_CONFIG     : {"size":112, "start":START_RESPONSE},
        R_RPOINT        : {"size":230, "start":START_RESPONSE},
        FLASH_INFO      : {"size":12, "start":START_RESPONSE},
        R_ANGLE_TO_MC2  : {"size":7, "start":START_RESPONSE},
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

def main():
    global config

    font = "Arial 13"
    font_family, font_size = font = ('Courier New', 10)
    sg.set_options(font=font)

    # vBias = 100
    # v_lsb = 0.0125

    vBias = 1
    v_lsb = 1


    
    layout = [
                [sg.T('Comm Port'),sg.Combo(serial_ports(), default_value='/dev/pts/2',size=(10,1),key='-port-'), 
                    sg.T('Baud'),sg.Combo([9600,115200], default_value='115200',key='-baudrate-'),
                    sg.Button('Connect'), sg.Button('Disconnect'), sg.T('----', key="connection_status"),
                    sg.Exit(button_text = "Exit")],

                [sg.T('Flash Management')],
                [sg.T('FLASH ID'),
                    sg.T('ManID', key="ManID"),
                    sg.T('Capacity', key="Capacity"),
                    sg.T('MaxPages', key="MaxPages"),
                    sg.Button('QueryFlash'),],
            
                [sg.T('Table'),sg.Combo(tables,key='-table-',default_value=0),
                    sg.T('Page '),   sg.Combo(pages, key='-page-' ,default_value=0),
                    sg.Button('ReadPoints'),sg.Button('WritePoints', disabled=True),
                    sg.Button('ErasePage',  font=font, button_color='red'),sg.T('...', key="elapsed", size=(10,1)),],
                
                [sg.T('Table'),  sg.Combo(tables,key='-table-mc2-',default_value=0),
                    sg.T('Angle'),  sg.Combo(angles, key='-angles-mc2-',size=(3,1),default_value=0),
                    sg.Button("Send Angle", key = "angle-to-mc2"),],
                        
                    [sg.T('File'), sg.In(key = "flash-data-filepath", readonly=True),
                        sg.Button("Open", key = "read-flash-file"),
                        sg.Button("Save", key = "write-flash-file"),],
                        
                        [sg.TabGroup([
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
                            ]])
                        ]
                ]

    
    window = sg.Window("Control Panel", layout, finalize=True)

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
            serialPort = window["-port-"].get() #mette dentro la variabile serialPort il valore contenuto in quel momento dalla chiave "-port-"
            baudRate = window["-baudrate-"].get()
            serialChannel = Serial(serialPort , baudRate, timeout=0, writeTimeout=0) #ensure non-blocking
            if serialChannel.is_open :
               for i in range(255):
                   serialChannel.write(b'\n')
               window["Connect"].update(disabled=True, button_color="red") 
               window["connection_status"].update("Connected")
               connectionEnabled = True
        elif event == "Disconnect": 

            window["connection_status"].update("---")
            #window["MonitorBtn"].update(disabled=True, text="Start")
            window["Connect"].update(disabled=False, button_color="green") 
            connectionEnabled = False
            serialChannel.close

        elif event == "QueryFlash":
            serialChannel.write(FLASH_INFO)
            serialChannel.flush()
            serialChannel.write(END_COMMAND)
            buffer = receiveBuffer(serialChannel)
            if buffer: decode_input_data(window,buffer)

        elif event == "angle-to-mc2":
    
            table=window['-table-mc2-'].get()
            point=window['-angles-mc2-'].get()
            print("table {0}, point {1}".format(table,point))  # {0},{1} indicano dei simboli sostituibili dai valori posti come anrgomento dnetro format print("When you multiply {0} and {1} or {0} and {2}, the result is {0}".format(0,1,2))
            data=R_ANGLE_TO_MC2 + pack('<2h', table,point)
            print(data)
            serialChannel.write(data)
            serialChannel.write(b'\n')
            serialChannel.flush() # svuoto il buffer

            #struct.pack(format, v1, v2, ...)
            #Return a bytes object containing the values v1, v2, … packed according to the format string format.
            #The arguments must match the values required by the format exactly.
            #'<2h' significa little endian e tipo short


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
                        # voltage     = window["dac1_{0}_{1}".format(i,index)].get() #prendo il valore contenuto nella cella e lo metto in voltage
                        # voltage     = float(voltage) #converto voltage in float
                        # dac_decimal = round((vBias - voltage) / v_lsb,2) #
                        dac_decimal   = int(window["dac1_{0}_{1}".format(i,index)].get())
                        if dac_decimal < 0 or dac_decimal > 16384:
                            raise ValueError("Point {}: DAC1 channel {} out of range".format(i,index))
                        point_data.append(int(dac_decimal))
                    
                    for index in range(32):
                        #(vBias-voltage)  / v_lsb --> decimal value for DAC
                        # voltage     = float(window["dac2_{0}_{1}".format(i,index)].get())
                        # dac_decimal = round((vBias - voltage) / v_lsb,2)
                        dac_decimal   = int(window["dac2_{0}_{1}".format(i,index)].get())
                        if dac_decimal < 0 or dac_decimal > 16384:
                            raise ValueError("Point {}: DAC2 channel {} out of range".format(i,index))
                        point_data.append(int(dac_decimal))

                    rf_data=[]
                    for index in range(4):
                        tmp_rf=window["rf_{0}_{1}".format(i,index)].get()
                        if len(tmp_rf) != 50: # prima era 25, non ricordo perché, ora metto 50 perché i byte di ogni riga sono 
                            raise ValueError("Point {}: RF channel {} invalid data".format(i,index))
                        rf_data += tmp_rf

                    point = (page*16)+i
                    log("write point {} of table {} ".format(point,table))
                    log("|".join(map(str,point_data)))

                    data = R_WPOINT + pack('<2H', table,point) +  pack('<64H', *point_data) #metto dentro data la richesta di lettura della flash, l'indirzzo da leggere e i punti formattati
                    
                    q = 0
                    for q in range(100):
                        hexvalue="".join(rf_data[(q*2):(q*2+2)]) #prendo le coppie di caratteri e li metto insieme come byte
                        data += pack('c', int(hexvalue,base=16).to_bytes(1, byteorder='big')) #aggiungo dentro data i dati delle schede rf, ho meso big al posto di little perché il micro2 si aspetta un big endian
                    
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
                data=R_RPOINT + pack('<2H', table,point) #metto in data la richiesta di lettura della flash e l'indirizzo
                serialChannel.write(data) #mando la richiesta al micro 1
                serialChannel.flush() #svuoto il buffer
                sleep(0.3) #aspetto 300 ms
                buffer = receiveBuffer(serialChannel) #metto nel buffer la risposta
                log([len(buffer),buffer])
                point_data_DAC=unpack_from("<68H",buffer, offset=2) #spacchetto i dati e li metto nella lista point_data
                print(point_data_DAC) #stampo a monitor i dati dei DAC (es. 8000 --> 100 volt)


                for p in range(32):
                    # dac_decimal = float(point_data[p])
                    # dac_decimal *= v_lsb
                    # voltage     = round(vBias - dac_decimal,2)
                    window["dac1_{0}_{1}".format(i,p)].update(point_data_DAC[p]) 
                    # dac_decimal = point_data[p+32]
                    # dac_decimal *= v_lsb
                    # voltage     = round(vBias - dac_decimal,2)
                    window["dac2_{0}_{1}".format(i,p)].update(point_data_DAC[p+32])  # al posto di point_data[p+32] c'era voltage
                
                window["rf_{0}_{1}".format(i,0)].update(str(buffer[130:155]))
                window["rf_{0}_{1}".format(i,1)].update(str(buffer[155:180]))
                window["rf_{0}_{1}".format(i,2)].update(str(buffer[180:205]))
                window["rf_{0}_{1}".format(i,3)].update(str(buffer[205:230]))
                
                
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