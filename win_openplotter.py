#                Anemometro de Hilo Caliente 20/12/15  (Hot wire anemometer)
# 		 This project was developed to use with Openplotter
#                Cuatro sensores MD0550 dxf de Modern Device(Four sensors  MD0550 dxf from Modern Device)   
#                Raspberry Pi2 B 1.1 and  A/D converter MCP 3008 with 10 bits/ 3,3 Voltios
#                Conexion via GPIO por SPI (Serial Paralel Interface)  Simulado por Sofware
#		 First part: read analog data from MCP3008 or MCP3208
#		 second part: Calculation of Win vector and win direcction
#		 third part: send data tu Openplotter via localhost
# Win_Opeplotter is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# any later version.
# Win_openplotter is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Openplotter. If not, see <http://www.gnu.org/licenses/>.


import time;
import math
import RPi.GPIO as GPIO
import socket, pynmea2

try:
    from configparser import ConfigParser
except ImportError:
    from ConfigParser import ConfigParser  # ver. < 3.0

# instantiate
config = ConfigParser()

# parse existing file
config.read('win_openplotter.ini')

# Beginn script to data read of the ADC MCP3008 or MCP3208 - from Erik Bartmann -www.erik-bartmann.de
GPIO.setmode(GPIO.BCM)  # GPIO-Pin Bezeichnungen verwenden
GPIO.setwarnings(False) # Warnungen deaktivieren
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def readAnalogData(adcChannel, SCLKPin, MOSIPin, MISOPin, CSPin, delay):
    """ Funktionsdefinition """
    # Negative Flanke des CS-Signals generieren
    GPIO.output(CSPin,   GPIO.HIGH)
    GPIO.output(CSPin,   GPIO.LOW)
    GPIO.output(SCLKPin, GPIO.LOW)   
    sendCMD = adcChannel
    sendCMD |= 0b00011000 # Entspricht 0x18 (1: Startbit, 1: Single/ended)
    # Senden der Bitkombination (Es finden nur 5 Bits Beruecksichtigung)
    for i in range(5):
        if(sendCMD & 0x10): # Bit an Position 4 pruefen.
            GPIO.output(MOSIPin, GPIO.HIGH)
        else:
            GPIO.output(MOSIPin, GPIO.LOW)
        # Negative Flanke des Clock-Signals generieren
        GPIO.output(SCLKPin, GPIO.HIGH)
        GPIO.output(SCLKPin, GPIO.LOW)
        sendCMD <<= 1 # Bitfolge eine Position nach links schieben
    # Empfangen der Daten des AD-Wandlers
    adcValue = 0 # Reset des gelesenen Wertes
    for i in range(11): # 13 bei MCP3208
        # Negative Flanke des Clock-Signals generieren
        GPIO.output(SCLKPin, GPIO.HIGH)
        GPIO.output(SCLKPin, GPIO.LOW)
        adcValue <<= 1 # Bitfolge 1 Position nach links schieben
        if(GPIO.input(MISOPin)):
            adcValue |=0x01
    time.sleep(delay) # Kurze Pause
    return adcValue

def setupGPIO(SCLKPin, MOSIPin, MISOPin, CSPin):
    """ GPIO-Pin Setup """
    GPIO.setup(SCLKPin, GPIO.OUT)
    GPIO.setup(MOSIPin, GPIO.OUT)
    GPIO.setup(MISOPin, GPIO.IN)
    GPIO.setup(CSPin,   GPIO.OUT)

# Variablendefinition ( read values from a section)
# Variable to read Analog data
ADCChannel = config.get('section_a', 'ADCChannel') # AD-Kanal
SCLK = config.getint('section_a', 'SCLK') # Serial-Clock
MOSI = config.getint('section_a', 'MOSI')  # Master-Out-Slave-In
MISO = config.getint('section_a', 'MISO') # Master-In-Slave-Out
CS = config.getint('section_a', 'CS') # Chip-Select

PAUSE = config.getfloat('section_a', 'PAUSE')
Ref_volt = config.getfloat('section_a', 'Ref_volt')
# End of variable to read analog data

# Begin of variable to use in Anemometer

Temp_cali = config.get('section_a', 'Temp_cali')

con_temp0_a = config.getfloat('section_a', 'con_temp0_a')
con_temp0_b = config.getfloat('section_a', 'con_temp0_b')
con_temp0_c = config.getfloat('section_a', 'con_temp0_b')

con_win_cero_a = config.getfloat('section_a', 'con_win_cero_a')
con_win_cero_b = config.getfloat('section_a', 'con_win_cero_b')
con_win_cero_c = config.getfloat('section_a', 'con_win_cero_c')

con_win_mph0_a = config.getfloat('section_a', 'con_win_mph0_a')
con_win_mph1_a = config.getfloat('section_a', 'con_win_mph1_a')
con_win_mph2_a = config.getfloat('section_a', 'con_win_mph2_a')
con_win_mph3_a = config.getfloat('section_a', 'con_win_mph3_a')

con_win_mph0_b = config.getfloat('section_a', 'con_win_mph0_b')
con_win_mph1_b = config.getfloat('section_a', 'con_win_mph1_b')
con_win_mph2_b = config.getfloat('section_a', 'con_win_mph2_b')
con_win_mph3_b = config.getfloat('section_a', 'con_win_mph3_b')
 # end of variable to use in Anemometer

setupGPIO(SCLK, MOSI, MISO, CS) # GPIO-Pin Setup and end of script to data  read
# Use of the funktion to read Analog data " readAnalogData(0, SCLK, MOSI, MISO, CS, PAUSE)"

# Begin script to Anemometer from Gypsylyon
# Calibration in customice Wincanal. Reference Anemometer VOLTCRAFT BL-30 AN error 0,2m/s

while True:
   Cont = 1
   Win_D_0 = 0
   Temp_D_0 = 0
   Win_D_1 = 0
   Temp_D_1 = 0
   Win_D_2 = 0
   Temp_D_2 = 0
   Win_D_3 = 0
   Temp_D_3 = 0

   for Cont in range(50): # 50 Averages to compensate air turbulences
	# Canal 0
        Win_D_0 = Win_D_0 + readAnalogData(0, SCLK, MOSI, MISO, CS, PAUSE)
        Temp_D_0 = Temp_D_0 + readAnalogData(1, SCLK, MOSI, MISO, CS, PAUSE)
   
	# Canal 1
        Win_D_1 = Win_D_1 + readAnalogData(2, SCLK, MOSI, MISO, CS, PAUSE)
        Temp_D_1 = Temp_D_1 + readAnalogData(3, SCLK, MOSI, MISO, CS, PAUSE)

        # Canal 2
        Win_D_2 = Win_D_2 + readAnalogData(4, SCLK, MOSI, MISO, CS, PAUSE)
        Temp_D_2 = Temp_D_2 + readAnalogData(5, SCLK, MOSI, MISO, CS, PAUSE)

        # Canal 3
        Win_D_3 = Win_D_3 + readAnalogData(6, SCLK, MOSI, MISO, CS, PAUSE)
        Temp_D_3 = Temp_D_3 + readAnalogData(7, SCLK, MOSI, MISO, CS, PAUSE)



   Win_D_0 = Win_D_0 / Cont
   Temp_D_0 = Temp_D_0 / Cont
   Win_D_1 = Win_D_1 / Cont
   Temp_D_1 = Temp_D_1 / Cont
   Win_D_2 = Win_D_2 / Cont
   Temp_D_2 = Temp_D_2 / Cont
   Win_D_3 = Win_D_3 / Cont
   Temp_D_3 = Temp_D_3 / Cont


# Calculos canal 0
   Win_V_0 = Win_D_0 * Ref_volt
   Temp_V_0 = Temp_D_0 * Ref_volt
   Temp_0 = con_temp0_a * math.pow(Temp_V_0, 2) - con_temp0_b * Temp_V_0 + con_temp0_c
   
   Win_cero_V_0 = con_win_cero_a * math.pow(Temp_V_0, 2)- con_win_cero_b * Temp_V_0 + con_win_cero_c

   Win_Inter = Win_V_0 + Win_cero_V_0 - 0.058

   Win_MPH_0 = con_win_mph0_a * math.pow(Win_Inter, con_win_mph0_b) 

# Calculos Canal 1
   Win_V_1 = Win_D_1 * Ref_volt
   Temp_V_1 = Temp_D_1 * Ref_volt
   Temp_1 = con_temp0_a * math.pow(Temp_V_1, 2) - con_temp0_b * Temp_V_1 + con_temp0_c
   Win_cero_V_1 = con_win_cero_a * math.pow(Temp_V_1, 2) - con_win_cero_b * Temp_V_1  + con_win_cero_c
 
   Win_Inter_1 = Win_V_1 + Win_cero_V_1 - 0.0676 

   Win_MPH_1 = con_win_mph1_a *  math.pow(Win_Inter_1, con_win_mph1_b)

# Calculos Canal 2

   Win_V_2 = Win_D_2 * Ref_volt
   Temp_V_2 = Temp_D_2 * Ref_volt
   Temp_2 = con_temp0_a * math.pow(Temp_V_2, 2) - con_temp0_b * Temp_V_2 + con_temp0_c
   Win_cero_V_2 = con_win_cero_a * math.pow(Temp_V_2, 2) - con_win_cero_b * Temp_V_2 + con_win_cero_c
 
   Win_Inter_2 = Win_V_2 + Win_cero_V_2 - 0.0676

   Win_MPH_2 = con_win_mph2_a *  math.pow(Win_Inter_2, con_win_mph2_b)


# Calculos canal 3


   Win_V_3 = Win_D_3 * Ref_volt
   Temp_V_3 = Temp_D_3 * Ref_volt
   Temp_3 = con_temp0_a * math.pow(Temp_V_3, 2) - con_temp0_b * Temp_V_3 + con_temp0_c
   Win_cero_V_3 = con_win_cero_a * math.pow(Temp_V_3, 2) - con_win_cero_b * Temp_V_3  + con_win_cero_c
 
   Win_Inter_3 = Win_V_3 + Win_cero_V_3 - 0.0676

   Win_MPH_3 = con_win_mph3_a *  math.pow(Win_Inter_3, con_win_mph3_b)



# Calculos Vector y direccion viento
# Detection of the cuadrant

   if (Win_MPH_0 > Win_MPH_1 and Win_MPH_0 > Win_MPH_2 and Win_MPH_0 > Win_MPH_3 ): 

       if (Win_MPH_1 > Win_MPH_2 and Win_MPH_1 > Win_MPH_3): 

           Win_A = Win_MPH_0
           Win_B = Win_MPH_1
           Cuadrante = 1

       elif (Win_MPH_3 > Win_MPH_2 and Win_MPH_3 > Win_MPH_1):

           Win_A = Win_MPH_0
           Win_B = Win_MPH_3
           Cuadrante = 8

   elif (Win_MPH_1 > Win_MPH_0 and Win_MPH_1 > Win_MPH_2 and Win_MPH_1 > Win_MPH_3): 
 
       if (Win_MPH_0 > Win_MPH_3 and Win_MPH_0 > Win_MPH_2): 
           
           Win_A = Win_MPH_1
           Win_B = Win_MPH_0
           Cuadrante = 2

       elif (Win_MPH_2 > Win_MPH_0 and Win_MPH_2 > Win_MPH_3):  


           Win_A = Win_MPH_1
           Win_B = Win_MPH_2
           Cuadrante = 3


   elif  (Win_MPH_2 > Win_MPH_1 and Win_MPH_2 > Win_MPH_0 and Win_MPH_2 > Win_MPH_3):
 
       if (Win_MPH_1 > Win_MPH_0 and Win_MPH_1 > Win_MPH_3):  

           Win_A = Win_MPH_2
           Win_B = Win_MPH_1
           Cuadrante = 4

       elif (Win_MPH_3 > Win_MPH_0 and Win_MPH_3 > Win_MPH_1):

           Win_A = Win_MPH_2
           Win_B = Win_MPH_3
           Cuadrante = 5


   elif  (Win_MPH_3 > Win_MPH_2 and Win_MPH_3 > Win_MPH_0 and Win_MPH_3 > Win_MPH_1):
 
       if (Win_MPH_2 > Win_MPH_1 and Win_MPH_2 > Win_MPH_0): 

           Win_A = Win_MPH_3
           Win_B = Win_MPH_2
           Cuadrante = 6

       elif (Win_MPH_0 > Win_MPH_2 and Win_MPH_0 > Win_MPH_1):  

           Win_A = Win_MPH_3
           Win_B = Win_MPH_0
           Cuadrante = 7

# Calculation of the angle in cuadrant
   Rel1 = Win_A / Win_B

   if (Rel1 < 1.05):
      Rel1 = 1.05

   c = 1.0441 - Rel1
   d = 4 * 0.0003 * c
   bcuadrado = 0.00000000000000007 * 0.00000000000000007
   Raiz_bac = math.sqrt(bcuadrado - d)
   
   Direccion = (-0.00000000000000007 + Raiz_bac) / (2 * 0.0003)

# Calculation of the angle refered to 360 degres
   if (Cuadrante == 1):
       W_Direccion = 360 - Direccion

   elif (Cuadrante == 2):

       W_Direccion = 0 + Direccion

   elif (Cuadrante == 3):

       W_Direccion = 90 - Direccion
  
   elif (Cuadrante == 4):

       W_Direccion = 90 + Direccion

   elif (Cuadrante == 5):

       W_Direccion = 180 - Direccion

   elif (Cuadrante == 6):

       W_Direccion = 180 + Direccion

   elif (Cuadrante == 7):

       W_Direccion = 270 - Direccion

   elif (Cuadrante == 8):

       W_Direccion = 270 + Direccion


# Calculation of the value of the win vector

   V_1 = math.cos(45) * math.sqrt((1 + (0.04 * 0.04)) * Win_B * Win_B - (0.04 * 0.04) * Win_A * Win_A) 
   V_2 = math.cos(45) * math.sqrt((1 + (0.04 * 0.04)) * Win_A * Win_A - (0.04 * 0.04) * Win_B * Win_B)


   Win_MPH_Vector =  ((V_1 + V_2) * 1.18) * 0.868976

   T_Media = (Temp_0 + Temp_1 + Temp_2 + Temp_3) / 4   # Midle temparature of 4 sensors
   
   # Cnversion tu NMEA0183
   mwv = pynmea2.MWV('OS', 'MWV', (str(round(W_Direccion,1)),'R',str(round(Win_MPH_Vector,1)),'N','A'))
   mwv1=str(mwv)
   mwv2=mwv1+"\r\n"
   sock.sendto(mwv2, ('localhost', 10110)) # Send datatu localhost 10110 to allow data read with Openplotter
   
   # Control show data  (not necessry to used with Openplotter
 
   print '                      Anemometro de Hilo Caliente 20/12/15      '
   print '                      Cuatro sensores MD0550 dxf de Modern Device        '       
   print '                      Raspberrs Pi2 B 1.1 con Convertidor A/D MCP 3008 de 10 bits 3,3 Voltios '
   print '                      Conexion via GPIO por SPI (Serial Paralel Interface)  Simulado por Sofware'
   print '                  '
   print '                ____________________________________________________________  '
   print '                                                    '
   print '     Win_Digital    : Canal 0: ' + str(Win_D_0) + ' - Canal 1: ' + str(Win_D_1) + ' - Canal 2: ' + str(Win_D_2) + ' - Canal 3: ' + str(Win_D_3)
   print '     Win_Voltios    : Canal 0:' + str(Win_V_0) + ' - Canal 1: ' + str(Win_V_1) + ' - Canal 2: ' + str(Win_V_2) + ' - Canal 3: ' + str(Win_V_3)
   print '     Win MPH        : Canal 0:' + str(int(Win_MPH_0)) + ' - Canal 1: ' + str(int(Win_MPH_1)) + ' - Canal 2: ' + str(int(Win_MPH_2)) + ' - Canal 3: ' + str(int(Win_MPH_3))
   print '                  '
   print '                _____________________________________________________________  '
   print '                  '

   print '     Temp Sensor    : Canal 0: ' + str(int(Temp_0)) + ' DC - Canal 1: ' + str(int(Temp_1)) + ' DC - Canal 2: ' + str(int(Temp_2)) + ' DC - Canal 3: ' + str(int(Temp_3)) + ' DC'
   print '                  '
   print '                ______________________________________________________________ '
   print '                  '

   print '     Win Vector: ' + str(int(Win_MPH_Vector)) + ' MPH - Win Direccion: ' + str(int(W_Direccion)) + ' Grados - Temperatura: ' + str(int(T_Media)) + ' DC'
   print '                _______________________________________________________________  '
   print '                  '
   print '                  '
 
