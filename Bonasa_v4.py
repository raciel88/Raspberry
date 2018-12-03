#Declaración de Librerias
import paho.mqtt.client as mqtt #import the client1
import sys
import os
import threading
import time
import RPi.GPIO as GPIO
from guizero import Window, Picture, App, Combo, Text, CheckBox, ButtonGroup, PushButton, info
import board
import busio
import digitalio
import adafruit_mcp230xx
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import serial

#Declaración de GPIOS
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Configuración pull_up para cada uno de los pines
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(19, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#Declaración de Variables de Interrupciones

# Initialize the I2C bus:
i2c = busio.I2C(board.SCL, board.SDA)

# Hardware SPI configuration:
SPI_PORT   = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# Create an instance of either the MCP23008 or MCP23017 class depending on
# which chip you're using:
mcp = adafruit_mcp230xx.MCP23017(i2c)  # MCP23017
mcp2 = adafruit_mcp230xx.MCP23017(i2c)  # MCP23017-2
 
# Optionally change the address of the device if you set any of the A0, A1, A2
# pins.  Specify the new address with a keyword parameter:
mcp2 = adafruit_mcp230xx.MCP23017(i2c, address=0x21)  # MCP23017 w/ A0 set
 
# Now call the get_pin function to get an instance of a pin on the chip.
# This instance will act just like a digitalio.DigitalInOut class instance
# and has all the same properties and methods (except you can't set pull-down
# resistors, only pull-up!).  For the MCP23008 you specify a pin number from 0
# to 7 for the GP0...GP7 pins.  For the MCP23017 you specify a pin number from
# 0 to 15 for the GPIOA0...GPIOA7, GPIOB0...GPIOB7 pins (i.e. pin 12 is GPIOB4).
pin0 = mcp.get_pin(0)
pin1 = mcp.get_pin(1)
pin2 = mcp.get_pin(2)
pin3 = mcp.get_pin(3)
pin4 = mcp.get_pin(4)
pin5 = mcp.get_pin(5)
pin6 = mcp.get_pin(6)
pin7 = mcp.get_pin(7)
pin8 = mcp.get_pin(8)
pin9 = mcp.get_pin(9)
pin10 = mcp.get_pin(10)
pin11 = mcp.get_pin(11)
pin12 = mcp.get_pin(12)
pin13 = mcp.get_pin(13)
pin14 = mcp.get_pin(14)
pin15 = mcp.get_pin(15)
pin0A = mcp2.get_pin(0)

# Setup pin0 as an output that's at a high logic level.
pin0.switch_to_output(value=False)
pin1.switch_to_output(value=False)
pin2.switch_to_output(value=False)
pin3.switch_to_output(value=False)
pin4.switch_to_output(value=False)
pin5.switch_to_output(value=False)
pin6.switch_to_output(value=False)
pin7.switch_to_output(value=False)
pin8.switch_to_output(value=False)
pin9.switch_to_output(value=False)
pin10.switch_to_output(value=False)
pin11.switch_to_output(value=False)
pin12.switch_to_output(value=False)
pin13.switch_to_output(value=False)
pin14.switch_to_output(value=False)
pin15.switch_to_output(value=False)
pin0A.switch_to_output(value=False)

S0=0
S1=0
S2=0
S3=0
S4=0
S5=0
S6=0
S7=0
S8=0
S9=0
S10=0

#Declaración de Variables del Cronómetro
Cron_M=0
Cron_Seg=0
Cron_S=0
T = 0
tiempo_inicial = 0
tiempo_actual = 0

ser = serial.Serial(
        port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)

#Información proveniente del servidor
# Tipo de Usuario (A = Nuevo B = Medio C = Master)
# Modelo de Bomba (A,B,C,D,E,F,G,H,I,J,K) 1 ó 0 dependiendo si se utilizará
broker_address="iot.eclipse.org"
print("creating new instance")
client = mqtt.Client("P1") #create new instance

Server = "A"
Server_flag = 0

#Función que reinicia la aplicación cuando se activa el último sensor
def restart_program():
    """Restarts the current program.
    Note: this function does not return. Any cleanup action (like
    saving data) must be done before calling this function."""
    python = sys.executable
    os.execl(python, python, * sys.argv)

def on_message(client, userdata, message):
    global server, Server_flag
    print("message received " ,str(message.payload.decode("utf-8")))
    server = str(message.payload.decode("utf-8"))
    Server_flag = 1

def counter():
    global Cron_M, Cron_Seg, reloj, Cron_S, Tiempo_I, Tiempo_F, tiempo_inicial, tiempo_actual
    if (T == 1):
        inicio = Text(app, text="Inicio del programa", grid=[0,1], width=50, align="left", size=15, font="Times New Roman")
        #Tiempo_F = time.time()
        #ron_Seg = int(Tiempo_F) - int(Tiempo_I)
        tiempo_actual = time.time()
        Cron_Seg =  int(tiempo_actual) - int(tiempo_inicial)
        Cron_M = int(Cron_Seg/60)
        Cron_S = Cron_Seg - Cron_M*60
        if Cron_S < 10:
            text.value=("%d" %Cron_M + ":0%d" %Cron_S)
        else:
            text.value=("%d" %Cron_M + ":%d" %Cron_S)
        reloj = text.value
        
def cronometro():
    global Cron_M, Cron_Seg, reloj, Cron_S, tiempo_inicial, tiempo_actual
    while True:
        if (T == 1):
            inicio = Text(app, text="Inicio del programa", grid=[0,1], width=50, align="left", size=15, font="Times New Roman")
            tiempo_actual = time.time()
            Cron_Seg =  int(tiempo_actual) - int(tiempo_inicial)
            Cron_M = int(Cron_Seg/60)
            Cron_S = Cron_Seg - Cron_M*60
            if Cron_S < 10:
                reloj=("%d" %Cron_M + ":0%d" %Cron_S)
            else:
                reloj=("%d" %Cron_M + ":%d" %Cron_S)
            #reloj = text.value
            text = Text(app, text=reloj, grid=[1,0], size=15, font="Times New Roman")
            
            

def ciclo():
    global server, Server_flag, T, reloj, S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11
    while True:
        if Server_flag == 1 & T ==1:
            S0 = 1
            def leds():
                pin0.value = False
                pin1.value = False
                pin2.value = False
                pin3.value = False
                pin4.value = False
                pin6.value = False
                pin7.value = False
            for i in range(len(server)):
                #print(i, server[i])
                #Server_flag = 0
                if (server[i]== "0"):
                    print ("0")
                    InsA = Text(app, text=("Instrucción"), grid=[1,2], width=50, align="left", size=15, font="Times New Roman")
                    pin0.value = True
                    while True:
                        if GPIO.input(4) == GPIO.LOW:
                            tiempo1 = Text(app, text=reloj, grid=[0,2], width=50, align="left", size=15, font="Times New Roman")
                            pin0.value = True
                            pin1.value = False
                            a="ate1=(0,0, 32, LIDITEK)"
                            b ="atd5=(1)"
                            c ="atd0=()"
                            ser.write(str.encode(a))
                            time.sleep(.3)
                            ser.write(str.encode(b))
                            #time.sleep(5)
                            time.sleep(2)
                            ser.write(str.encode(c))
                            time.sleep(4)
                            leds()
                            S1 = 1
                            break
                if (server[i]== "1"):
                    print ("1")
                    InsB = Text(app, text=("Instrucción"), grid=[1,3], width=50, align="left", size=15, font="Times New Roman")
                    pin1.value = True
                    while True:
                        if GPIO.input(17) == GPIO.LOW:
                            tiempo2 = Text(app, text=reloj, grid=[0,3], width=50, align="left", size=15, font="Times New Roman")
                            pin1.value = True
                            time.sleep(1.5)
                            pin1.value = False
                            time.sleep(.5)
                            pin1.value = True
                            time.sleep(1.5)
                            pin1.value = False
                            time.sleep(.5)
                            pin1.value = True
                            time.sleep(1.5)
                            leds()
                            S2 = 1
                            break
                if (server[i]== "2"):
                    print ("2")
                    InsC = Text(app, text=("Instrucción"), grid=[1,4], width=50, align="left", size=15, font="Times New Roman")
                    pin2.value = True
                    while True:
                        if GPIO.input(27) == GPIO.LOW:
                            tiempo3 = Text(app, text=reloj, grid=[0,4], width=50, align="left", size=15, font="Times New Roman")
                            pin2.value = True
                            time.sleep(1.5)
                            pin2.value = False
                            time.sleep(.5)
                            pin2.value = True
                            time.sleep(1.5)
                            pin2.value = False
                            time.sleep(.5)
                            pin2.value = True
                            time.sleep(1.5)
                            leds()
                            S3 = 1
                            break
                if (server[i]== "3"):
                    print ("3")
                    InsD = Text(app, text=("Instrucción"), grid=[1,5], width=50, align="left", size=15, font="Times New Roman")
                    pin3.value = True
                    while True:
                        if GPIO.input(22) == GPIO.LOW:
                            tiempo4 = Text(app, text=reloj, grid=[0,5], width=50, align="left", size=15, font="Times New Roman")
                            pin3.value = True
                            time.sleep(1.5)
                            pin3.value = False
                            time.sleep(.5)
                            pin3.value = True
                            time.sleep(1.5)
                            pin3.value = False
                            time.sleep(.5)
                            pin3.value = True
                            time.sleep(1.5)
                            leds()
                            S4 = 1
                            break
                if (server[i]== "4"):
                    print ("4")
                    InsE = Text(app, text=("Instrucción"), grid=[1,6], width=50, align="left", size=15, font="Times New Roman")
                    pin4.value = True
                    while True:
                        if GPIO.input(5) == GPIO.LOW:
                            tiempo5 = Text(app, text=reloj, grid=[0,6], width=50, align="left", size=15, font="Times New Roman")
                            pin4.value = True
                            time.sleep(1.5)
                            pin4.value = False
                            time.sleep(.5)
                            pin4.value = True
                            time.sleep(1.5)
                            pin4.value = False
                            time.sleep(.5)
                            pin4.value = True
                            time.sleep(1.5)
                            leds()
                            S5 = 1
                            break
                if (server[i]== "5"):
                    print ("5")
                    InsF = Text(app, text=("Instrucción"), grid=[1,7], width=50, align="left", size=15, font="Times New Roman")
                    pin5.value = True
                    while True:
                        if GPIO.input(6) == GPIO.LOW:
                            tiempo6 = Text(app, text=reloj, grid=[0,7], width=50, align="left", size=15, font="Times New Roman")
                            pin5.value = True
                            time.sleep(1.5)
                            pin5.value = False
                            time.sleep(.5)
                            pin5.value = True
                            time.sleep(1.5)
                            pin5.value = False
                            time.sleep(.5)
                            pin5.value = True
                            time.sleep(1.5)
                            leds()
                            S6 = 1
                            break
                if (server[i]== "6"):
                    print ("6")
                    InsG = Text(app, text=("Instrucción"), grid=[1,8], width=50, align="left", size=15, font="Times New Roman")
                    pin6.value = True
                    while True:
                        if GPIO.input(13) == GPIO.LOW:
                            tiempo7 = Text(app, text=reloj, grid=[0,8], width=50, align="left", size=15, font="Times New Roman")
                            pin6.value = True
                            time.sleep(1.5)
                            pin6.value = False
                            time.sleep(.5)
                            pin6.value = True
                            time.sleep(1.5)
                            pin6.value = False
                            time.sleep(.5)
                            pin6.value = True
                            time.sleep(1.5)
                            leds()
                            S7 = 1
                            break
                if (server[i]== "7"):
                    print ("7")
                    InsH = Text(app, text=("Instrucción"), grid=[1,9], width=50, align="left", size=15, font="Times New Roman")
                    pin7.value = True
                    while True:
                        if GPIO.input(19) == GPIO.LOW:
                            tiempo8 = Text(app, text=reloj, grid=[0,9], width=50, align="left", size=15, font="Times New Roman")
                            pin7.value = True
                            time.sleep(1.5)
                            pin7.value = False
                            time.sleep(.5)
                            pin7.value = True
                            time.sleep(1.5)
                            pin7.value = False
                            time.sleep(.5)
                            pin7.value = True
                            time.sleep(1.5)
                            leds()
                            S8 = 1
                            break
                if (server[i]== "8"):
                    print ("8")
                    InsI = Text(app, text=("Instrucción"), grid=[1,10], width=50, align="left", size=15, font="Times New Roman")
                    pin8.value = True
                    while True:
                        if GPIO.input(26) == GPIO.LOW:
                            tiempo9 = Text(app, text=reloj, grid=[0,10], width=50, align="left", size=15, font="Times New Roman")
                            pin8.value = True
                            time.sleep(1.5)
                            pin8.value = False
                            time.sleep(.5)
                            pin8.value = True
                            time.sleep(1.5)
                            pin8.value = False
                            time.sleep(.5)
                            pin8.value = True
                            time.sleep(1.5)
                            leds()
                            S9 = 1
                            break
                if (server[i]== "9"):
                    print ("9")
                    InsJ = Text(app, text=("Instrucción"), grid=[1,11], width=50, align="left", size=15, font="Times New Roman")
                    pin9.value = True
                    while True:
                        if GPIO.input(21) == GPIO.LOW:
                            tiempo10 = Text(app, text=reloj, grid=[0,11], width=50, align="left", size=15, font="Times New Roman")
                            pin9.value = True
                            time.sleep(1.5)
                            pin9.value = False
                            time.sleep(.5)
                            pin9.value = True
                            time.sleep(1.5)
                            pin9.value = False
                            time.sleep(.5)
                            pin9.value = True
                            time.sleep(1.5)
                            leds()
                            S10 = 1
                            break
            Server_flag = 0
                       
   
#Función que se activa cuando se activa el sensor del conveyor de entrada
#Activa el tiempo inicial y el hilo del reloj
def inicio(pin):
    global S0, reloj, text, T, tiempo_inicial
    if (T==0):
        tiempo_inicial = time.time()        
    T = 1
    #Se llama a una o más funciones declaradas en el programa principal
    if S0 == 1:
        error_inicio = Text(app, text=("error de inicio"), grid=[0,12], color="red", width=50, align="left", size=15, font="Times New Roman")
        #Instrucción que se ejecuta si una interrupción accedió dos veces

def handle1(pin):
    global S1, reloj
    if S1 == 1:
        #client.publish("bonasa","Sensor A")
        error1 = Text(app, text=("error en el Contenedor A"), grid=[0,13], color="red", width=50, align="left", size=15, font="Times New Roman")
    
def handle2(pin):
    global S2, reloj
    if S2 == 1:   
        error2 = Text(app, text=("error en el Contenedor B"), grid=[0,14], color="red", width=50, align="left", size=15, font="Times New Roman")
    
def handle3(pin):
    global S3, reloj
    if S3 == 1:
        error3 = Text(app, text=("error en el Contenedor C"), grid=[0,15], color="red", width=50, align="left", size=15, font="Times New Roman")
          
def handle4(pin):
    global S4, reloj
    if S4 == 1:
        error4 = Text(app, text=("error en el Contenedor D"), grid=[0,16], color="red", width=50, align="left", size=15, font="Times New Roman")

def handle5(pin):
    global S5, reloj
    if S5 == 1:
        error5 = Text(app, text=("error en el Contenedor E"), grid=[0,17], color="red", width=50, align="left", size=15, font="Times New Roman")

def handle6(pin):
    global S6, reloj
    if S6 == 1:
        error6 = Text(app, text=("error en el Contenedor F"), grid=[0,18], color="red", width=50, align="left", size=15, font="Times New Roman")
        
def handle6(pin):
    global S7, reloj
    if S7 == 1:
        error7 = Text(app, text=("error en el Contenedor G"), grid=[0,19], color="red", width=50, align="left", size=15, font="Times New Roman")

def handle6(pin):
    global S8, reloj
    if S8 == 1:
        error8 = Text(app, text=("error en el Contenedor H"), grid=[0,20], color="red", width=50, align="left", size=15, font="Times New Roman")
        
def handle6(pin):
    global S9, reloj
    if S9 == 1:
        error9 = Text(app, text=("error en el Contenedor I"), grid=[0,21], color="red", width=50, align="left", size=15, font="Times New Roman")

def handle6(pin):
    global S10, reloj
    if S10 == 1:
        error10 = Text(app, text=("error en el Contenedor J"), grid=[0,22], color="red", width=50, align="left", size=15, font="Times New Roman")
        

def fin(pin):
    client.publish("bonasa","Reset")
    fin = Text(app, text=("Fin del Programa"), grid=[0,23], width=50, align="left", size=15, font="Times New Roman")
    restart_program()
    # Instrucción que llama a la función que reinicia el código
        
#Parámetros del widget App
app = App(title="Bonasa", width=1280, height=720, layout="grid")
#Color de Fondo
#app.bg = "white"
#Color de Letras
#app.text_color = "black"
#El Modelo de la Bomba, que se obtendrá del Servidor
Bomba = "Bomba A"
#Impresión del modelo de la bomba
Modelo_Bomba = Text(app, text=("Modelo de Bomba: %s " %Bomba), grid=[0,0], width=50, height=5, align="top", size=17, font="Times New Roman")
picture = Picture(app, image="solid.gif", grid=[2,1,2,7], width=128, height=128, align="right")

#Declaración de Interrupciones, pin, rising falling or both, función, bouncetime=tiempo en el cual se bloquean interrupciones
GPIO.add_event_detect(20, GPIO.FALLING, callback=inicio, bouncetime=7000)
GPIO.add_event_detect(4, GPIO.FALLING, callback=handle1, bouncetime=7000)
GPIO.add_event_detect(17, GPIO.FALLING, callback=handle2, bouncetime=7000)
GPIO.add_event_detect(27, GPIO.FALLING, callback=handle3, bouncetime=7000)
GPIO.add_event_detect(22, GPIO.FALLING, callback=handle4, bouncetime=7000)
GPIO.add_event_detect(5, GPIO.FALLING, callback=handle5, bouncetime=7000)
GPIO.add_event_detect(6, GPIO.FALLING, callback=handle6, bouncetime=7000)
#GPIO.add_event_detect(13, GPIO.FALLING, callback=handle7, bouncetime=7000)
#GPIO.add_event_detect(19, GPIO.FALLING, callback=handle8, bouncetime=7000)
GPIO.add_event_detect(26, GPIO.FALLING, callback=fin, bouncetime=7000)

#Intrucción que identifica el inicio del programa en terminal
print("Hilo Principal")

#m = threading.Thread(target=mqtt)
#Indicación del objetivo del hilo
#Inicio del Hilo llamado t
#v = threading.Thread(target=cronometro)
#v.setDaemon(False)
t = threading.Thread(target=ciclo)

#Indicación del objetivo del hilo


t.start()

InsF = Text(app, text=("Instrucciones"), grid=[1,1], width=50, align="left", size=15, font="Times New Roman")        
cron = Text(app, text=("Cronómetro:"), grid=[1,0], align="left", size=15, font="Times New Roman")
text = Text(app, text="0", grid=[1,0], size=15, font="Times New Roman")
text.repeat(500, counter)  # Schedule call to counter() every 1000ms
print("connecting to broker")
client.connect(broker_address) #connect to broker
client.loop_start() #start the loop
print("Subscribing to topic","bonasa")
client.subscribe("bonasa1")
client.on_message=on_message
c ="atd0=()"
ser.write(str.encode(c))
time.sleep(.3)
app.display()

