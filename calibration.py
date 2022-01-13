import statsmodels.api as sm
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import io

from datetime import datetime
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

from collections import namedtuple
import time 


# VARS CONSTS:
_VARS = {'window': False}




def draw_figure(canvas, figure):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg

#nel main c'è il layout e il ciclo infinito
def main():
    
    #AppFont = 'Any 16'
    #font = "Arial 13"
    font_family, font_size = font = ('Courier New', 10)
    sg.set_options(font=font)

    layout =    [

                [sg.Canvas(key='figCanvas')],
                [sg.T('File'), sg.In(key = "flash-data-filepath", readonly=True),sg.Button('Open File',key='-OPEN-'),sg.Button('Exit')]
              
                ]
    
    _VARS['window'] = sg.Window('Calibration',
                            layout,
                            finalize=True,
                            resizable=True,
                            element_justification="right")

    # Run the Event Loop
    while True:
        event, values = _VARS['window'].read(500)
        if event in (sg.WIN_CLOSED, 'Quit', "Exit"):
            break

        if event == '-OPEN-':
            filename = sg.popup_get_file("Select file to read", save_as=False, default_path=_VARS['window']["flash-data-filepath"].get())
            #raw_data = window['-flash-data-']

            if filename:
                try:
                    _VARS['window']["flash-data-filepath"].update(filename)
                    # file = open(filename,'r')
                    # lines= file.readlines()
                    # file.close()
                    ctable = pd.read_csv(filename)
                except NotImplementedError:
                    pass
                fig = plt.figure()
                #plt.plot(ctable["DAC1_2"],ctable["Voltage_Dec"], linewidth=1,color='r') #stampo la linea della retta interpolata
                
                X = ctable.DAC1_2
                y = ctable.Voltage_Dec
                X  = sm.add_constant(X)
                model = sm.OLS(y,X).fit()
                sg.Print(model.summary())
                sg.Print(model.params)

                plt.plot(ctable["DAC1_2"],ctable["Voltage_Dec"],'ro:',label="measured values") #display the values of the measured points bo: vuol dire r --> red color,o--> round points, : -->linea 
                
                #for see the values of the measured numbers on the points
                for x,y in zip(ctable["DAC1_2"],ctable["Voltage_Dec"]):
                    label = "{:.2f}".format(x) #se metto x mi appare il vaore delle x, se metto y mi appare il valore delle y
                    plt.annotate(label, # this is the text
                    (x,y), # these are the coordinates to position the label
                    textcoords="offset points", # how to position the text
                    xytext=(0,10), # distance from text to points (x,y)
                    ha='center') # horizontal alignment can be left, right or center
                
                y_predict = model.params[0] + model.params[1]*ctable['DAC1_2'] #metto dentro y_predict i valori con i coefficienti calcolati dal modello
                plt.plot(ctable["DAC1_2"],y_predict, linewidth=1,color='r',label="fitted values") #stampo la linea della retta interpolata
                plt.title("calibration")
                plt.xlabel("measured Values")
                plt.ylabel("decimal voltage")
                plt.legend() #add a legend, thje names are in the label parameters of each graph
                draw_figure(_VARS['window']['figCanvas'].TKCanvas, fig)


if __name__ == '__main__':
    sg.theme('DefaultNoMoreNagging') 
    main()    