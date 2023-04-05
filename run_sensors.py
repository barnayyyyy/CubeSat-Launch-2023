# By: John Barney & Lucas Nichols

###################- SETUP -####################
# to run as backgroun process not linked to putty: nohup python3 run_sensors.py &
# view all processes: htop

########## Rasberry Pi 4 Model B ##########
# IP: 10.129.142.104 (UTC) --- 192.168.135.223 (Hotspot)
# Login as: pi
# Password rasbperry

########## Initalzie New SD Card ##########
# On laptop download rufus: https://rufus.ie/en/
# On laptop download raspium: https://www.raspberrypi.com/software/operating-systems/ (Raspberry Pi OS with desktop)
# Open rufus -> device selection: the inserted SD card
#               boot selection: the raspium download
#               start -> ok -> ok (warning messages are if drive contains information to wipe it)
# When completed, file manager -> right click SD -> eject safely -> can now remove SD card from laptop
# Do the basic intro for the Raspberry Pi: insert SD card, connect peripherals, power Pi, configure Raspian
# Update the system: sudo apt update
#                    sudo apt upgrade

########## Prerequisites for our script ##########
# Python comes with raspbian but to verify latet version: sudo apt-get intall python3 --upgrade
# pip installation: sudo apt install python3-pip
# install circuit python (for adafruit I2C): cd ~
#                                            sudo pip install --upgrade adafruit-python-shell
#                                            wget https://raw.githubusercontent.com/adafruit/Raspberry-Pi-Installer-Scripts/master/raspi-blinka.py
#                                            sudo python raspi-blinka.py
# pip install adafruit-blinka                                                               # I2C connection
# pip install adafruit-circuitpython-bme680                                                 # temp/gas/humidity/pressure/altitude library
# pip install adafruit-circuitpython-lsm303-accel                                           # accelerometer library
# pip install adafruit-circuitpython-lis2mdl                                                # magnetometer library
# pip install adafruit-circuitpython-ds3231                                                 # real-time clock library
# enable i2c on pi: https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c
#   use this command instead of the smbus one: sudo apt install python3-smbus
# pip install os                                                                            # file management
# pip install warnings                                                                      # remove warnings
# pip install pandas                                                                        # dataframes
# sudo chmod 666 /dev/ttyUSB*                                                               # change permissions for USB port
# pip instal picamera
# in /boot/config.txt, verify:
#   start_x=1, gpu_mem=128, 
#       add (if not there):
#       enable_uart=1, dtoverlay=w1-gpio

########## Postrequisite for the data ##########
# Quickly download files from SD card to laptop using ext2read: https://sourceforge.net/projects/ext2read/
# Run application as administrator with the SD card in
# If SD card isn't showing up: file -> rescan system
# If wanting to navigate to desktop: /home/pi/desktop
# On desired folder: right click save -> select destination on laptop -> ok
# When completed, file manager -> right click SD -> eject safely -> can now remove SD card from laptop

# Combine multiple .h264 files into one .mp4 file using ffmpeg: https://www.youtube.com/watch?v=IECI72XEox0
# Open windows command promt and navigate to directory where the .h264 files are located: cd C:\Users\tvr861\OneDrive - University of Tennessee\Spring 2023\4 - Design Project - 4850\CubeSat Pi\results_overnight\videos - Copy
# Run command to concatenate all the '.h264' files into a single '.mp4' file: copy /b *.h264 output.mp4
# Upload to YouTube or open with prefered program that opens .mp4 files

########## Reduce Power Consumption ##########
# speed up sudo reboot: sudo nano /etc/systemd/system.conf -> uncomment DefaultTimeoutStopSec and make =5s
# see frequency in real time: watch -n1 vcgencmd measure_clock arm
# see temperature in real time: watch -n1 vcgencmd measure_temp
# see voltage in real time: watch -n1 vcgencmd measure_volts

##### CPU/GPU #####
# check cpu: lscpu
# add to bottom of file (remove "# " at beginning of every line)
### sudo nano /boot/config.txt ###
# [pi4]
# # CPU clock to 1200 MHz (default 1500 MHz)
# arm_freq=1200
# # CPU clock minimum to 100 MHz (default 250 MHz)
# arm_freq_min=100
# # Core clock to 150 MHz (default 500 MHz)
# core_freq=150
# # Core clock minimum to 0 (default 0)
# core_freq_min=0
# # GPU clock to 200 MHz (default 500 MHz)
# gpu_freq=200
# # GPU clock minimum to 100 MHz (default 300 MHz)
# gpu_freq_min=100
# # SDRAM clock to 400 MHz (default 400 MHz)
# sdram_freq=400
# # SDRAM clock minimum to 200 MHz (default 320 MHz)
# sdram_freq_min=200
# # Reduce voltage supplied to processor by 0.05 V
# over_voltage=-5
# # Reduce minimum voltage supplied to processotr by 0.1 V
# over_voltage_min=-10
# # Disable to allow processor to dnamically adjust its frequency based on load
# force_turbo=0ff
# # Avoid use of PWM
# avoid_pwm_pll=1
# # Disable audio output
# dtparam=audio=off
# # H.264 video decoder clock minimum to 100 MHz (default 400 MHz)
# h264_freq_min=100
# # Minimum frequency of V3D graphics engine clock to 100 MHz (default 500 MHz)
# v3d_freq_min=100

##### USB #####
# check usb: lsusb -v --- "bcdUSB" shows USB transfer rate and "MaxPower" shows power consumption
# add to bottom of file (remove "# " at beginning of every line)
### sudo nano /boot/config.txt ###
# [pi4]
# # Maximum current USB ports can provide (default 1200 mA)
# max_usb_current=500

##### HDMI (not permanent) #####
# check hdmi: tvservice -s
# turn off hdmi: sudo tvservice -o

##### LED/Bluetooth #####
# check bluetooth: sudo systemctl status bluetooth
# add to bottom of file (remove "# " at beginning of every line)
### sudo nano /boot/config.txt ###
# [all]
# # disable bluetooth
# dtoverlay=disable-bt
# # disable red power led
# dtparam=pwr_led_trigger=default-on
# dtparam=pwr_led_activelow=off
# # disable green activity led
# dtparam=act_led_trigger=none
# dtparam=act_led_activelow=off
# # disable ethernet activity led/
# dtparam=eth_led0=4
# # disable ethernet link led
# dtparam=eth_led1=4

########## Debugging ##########
# error: library board is installed
# solution: uninstall board and adafruit-blinka then reinstall adafrui-blinka (cannot have board installed)

###################- PROGRAM -####################

import os
import time                                                                                 # for sleep timer
import warnings                                                                             # for removing warnings
import pandas as pd                                                                         # for dataframes
import csv                                                                                  # for saving data into csv from serial communation port
import serial                                                                               # for modifying ports
import board                                                                                # for use in i2c
import busio                                                                                # for use in i2c
import adafruit_bme680                                                                      # for connecting bme680
import adafruit_lsm303_accel                                                                # for connecting lsm303 accelerometer
import adafruit_lis2mdl                                                                     # for connecting lsm303 magnetometer
import adafruit_ds3231                                                                      # for connecting ds3231
import signal                                                                               # for keyboard iunterrupt
from threading import Thread                                                                # for threading while loops
from picamera import PiCamera, Color                                                        # for Pi camera
warnings.simplefilter(action='ignore', category=FutureWarning)                              # removing append warning

########## Inputs ##########
sleeptime_bme = 10                                                                          # setting how long to sleep while loop for
sleeptime_lsm = 10                                                                          # setting how long to sleep while loop for
recordtime_cam = 60                                                                         # setting how long to record video (seconds)
reconnect_time = 10                                                                         # setting how long to wait before reconnecting
temperature_offset = -9                                                                     # calibrating thermometer
pressure_sealevel = 1018.3                                                                  # calibrating pressure
HOMEPATH = '/home/pi/Desktop/'                                                              # working directory
path_data = HOMEPATH+'data/'                                                                # creating a savefile for data
path_video = HOMEPATH+'videos/'                                                             # creating a savefile for video
bmename = "BME680"                                                                          # file name for bme data
lsmname = "LSM303AGR"                                                                       # file name for lsm data
muonname = "MUON"                                                                           # file name for muon detector data
videoname = "VIDEO_"                                                                        # file name for video

########## Setup ##########
# Checking for existing savefiles
bmesave = bmename + "_0"                                                                    # initalize final savefile for bme
lsmsave = lsmname + "_0"                                                                    # initalize final savefile for lsm
muonsave = muonname + "_0"                                                                  # initalize final savefile for muon
videosave = "{}{:03d}".format(videoname,0)                                                  # initalize final savefile for video
bmei = 0                                                                                    # counter for bme
lsmi = 0                                                                                    # counter for lsm
muoni = 0                                                                                   # counter for muon
videoi = 0                                                                                  # counter for video
while(os.path.exists(path_data + bmesave + ".csv"))==True:                                  # loop until new savefile is made
    bmei += 1                                                                               # increment counter
    bmesave = bmename + '_' + str(bmei)                                                     # increment savefile
while(os.path.exists(path_data + lsmsave + ".csv"))==True:                                  # loop until new savefile is made
    lsmi += 1                                                                               # increment counter
    lsmsave = lsmname + '_' + str(lsmi)                                                     # increment savefile
while(os.path.exists(path_data + muonsave + ".csv"))==True:                                 # loop until new savefile is made
    muoni += 1                                                                              # increment counter
    muonsave = muonname + '_' + str(muoni)                                                  # increment savefile
while(os.path.exists(path_video + videosave + ".h264"))==True:                              # loop until new savefile is made
    videoi += 1                                                                             # increment counter
    videosave = "{}{:03d}".format(videoname,videoi)                                         # initalize final savefile for video

# Other
i2c = busio.I2C(board.SCL, board.SDA)                                                       # create sensor object, communicating over the board's default I2C bus 

########## MUON DETECTOR SETUP ##########
def signal_handler(signal, frame):                                                          # Define actions to terminate program gracefully
    print('You pressed Ctrl+C! Exiting gracefully...')                              
    det.close()			     	 									                        # close connection
    csv_file.close()			 									                        # close file
    exit()					 								     	                        # exits the program

signal.signal(signal.SIGINT, signal_handler)                                                # Sets up signal handler for the SIGINT (interrupt from keyboard = Ctrl+C) signal

csv_file = open(path_data+muonsave+'.csv', 'w', newline='') 	                            # create a file object to write data to the CSV file
csv_writer = csv.writer(csv_file)						                                    # create a CSV writer object to write data to the CSV file
csv_writer.writerow(['Event','Time_CPU','Time_RTC','Ardn_time[ms]','ADC[0-0123]','SiPM[mV]','Deadtime[ms]','Temp[C]']) # column titles

class BME680(Thread):                                                                       # class for threading bme
    def __init__(self):                                                                     # initialization for threading
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):                                                                          # run file for threading
        df1 = pd.DataFrame(columns=['Time_CPU','Time_RTC','Temperature[C]','Gas[ohm]','Humidity[%RS]','Pressure[hPa]','Altitude[m]']) # initializing dataframe for bme680
        while True:
            ########## Time ##########
            try:
                rtc = adafruit_ds3231.DS3231(i2c)                                           # extract rtc from i2c
                t = rtc.datetime                                                            # read time from rtc
                Time_RTC = str(t.tm_hour)+':'+str(t.tm_min)+':'+str(t.tm_sec)               # setting rtc time
                print("The time is {}:{:02}:{:02}".format(t.tm_hour, t.tm_min, t.tm_sec))
            except:                                                                         # if an error was thron
                print('DS3231 DISCONNECTED')                                                # displaying ds3231 is disconnected
                Time_RTC = float("NaN")                                                     # setting to Nan value for unmeasured
            try:
                Time_CPU = time.strftime('%H:%M:%S', time.localtime())                      # setting cpu time
            except:
                Time_CPU = float("NaN")                                                     # if cpu time fails
            ########## BME680 ##########
            try:                                                                            # for avoiding errors
                bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)                           # connecting bme680 to i2c bus
                bme680.sea_level_pressure = pressure_sealevel                               # setting the sea level pressure
                Temperature = bme680.temperature + temperature_offset                       # capturing temperature
                Gas         = bme680.gas                                                    # capturing gas (atmosphere)
                Humidity    = bme680.relative_humidity                                      # capturing humidity
                Pressure    = bme680.pressure                                               # capturing pressure
                Altitude    = bme680.altitude                                               # capturing altitude
                print("Temperature: %0.1f C" % Temperature)                                 # displaying Temperature in C
                print("Gas: %d ohm" % Gas)                                                  # displaying resistance in ohms of gas sensor
                print("Humidity: %0.1f %%" % Humidity)                                      # displaying % humidity from 0-100
                print("Pressure: %0.3f hPa" % Pressure)                                     # displaying pressure in hPa
                print("Altitude = %0.2f meters" % Altitude)                                 # displaying altitude in m
            except:                                                                         # if an error was thrown
                print('BME680 DISCONNECTED')                                                # displaying bme680 is disconnected
                Temperature = float("NaN")                                                  # setting to Nan value for unmeasured 
                Gas         = float("NaN")                                                  # setting to Nan value for unmeasured
                Humidity    = float("NaN")                                                  # setting to Nan value for unmeasured
                Pressure    = float("NaN")                                                  # setting to Nan value for unmeasured
                Altitude    = float("NaN")                                                  # setting to Nan value for unmeasured
            df1 = df1.append({"Time_CPU":Time_CPU,"Time_RTC": Time_RTC, "Temperature[C]": Temperature, "Gas[ohm]": Gas, "Humidity[%RS]": Humidity,"Pressure[hPa]": Pressure, "Altitude[m]": Altitude},ignore_index=True) # adding bme680 data to df1
            df1.to_csv(path_data+bmesave+'.csv', float_format='%.15f')                      # saving df1 to .csv format
            time.sleep(sleeptime_bme)                                                       # pausing loop

class LSM303AGR(Thread):                                                                    # class for threading lsm
    def __init__(self):                                                                     # initialization for threading
        Thread.__init__(self)
        self.daemon = True
        self.start()
    def run(self):                                                                          # run file for threading
        df2 = pd.DataFrame(columns=['Time_CPU','Time_RTC','Acceleration[(x,y,z),(m/s^2)]','Magnetometer[(x,y,z),(uT)]']) # initializing dataframe for lsm303agr
        while True:
            ########## Time ##########
            try:
                rtc = adafruit_ds3231.DS3231(i2c)                                           # extract rtc from i2c
                t = rtc.datetime                                                            # read time from rtc
                Time_RTC = str(t.tm_hour)+':'+str(t.tm_min)+':'+str(t.tm_sec)               # setting rtc time
                print("The time is {}:{:02}:{:02}".format(t.tm_hour, t.tm_min, t.tm_sec))
            except:                                                                         # if an error was thron
                print('DS3231 DISCONNECTED')                                                # displaying ds3231 is disconnected
                Time_RTC = float("NaN")                                                     # setting to Nan value for unmeasured
            try:
                Time_CPU = time.strftime('%H:%M:%S', time.localtime())                      # setting cpu time
            except:
                Time_CPU = float("NaN")                                                     # if cpu time fails
            ########## LSM303AGR ##########
            try:                                                                            # for avoiding errors
                mag = adafruit_lis2mdl.LIS2MDL(i2c)                                         # connecting magnetometer to i2c bus
                accel = adafruit_lsm303_accel.LSM303_Accel(i2c)                             # connecting accelerometer to i2c bus
                Acceleration = accel.acceleration                                           # capturing acceleration
                Magnetometer = mag.magnetic                                                 # capturing orientation
                print("Acceleration[(x,y,z),(m/s^2)]: X=%0.3f Y=%0.3f Z=%0.3f" % Acceleration) # displaying acceleration
                print("Magnetometer[(x,y,z),(uT)]: X=%0.3f Y=%0.3f Z=%0.3f" % mag.magnetic) # displaying orientation
            except:                                                                         # if an error was thrown
                print('LSM303AGR DISCONNECTED')                                             # displaying lsm303agr is disconnected
                Acceleration = float("NaN")                                                 # setting to Nan value for unmeasured
                Magnetometer = float("NaN")                                                 # setting to Nan value for unmeasured
            df2 = df2.append({"Time_CPU": Time_CPU,"Time_RTC":Time_RTC, "Acceleration[(x,y,z),(m/s^2)]": Acceleration, "Magnetometer[(x,y,z),(uT)]": Magnetometer},ignore_index=True) # adding lsm303agr data to df2
            df2.to_csv(path_data+lsmsave+'.csv', float_format='%.15f')                      # saving df2 to .csv format
            time.sleep(sleeptime_lsm)                                                       # pausing loop

class MUON_DETECTOR(Thread):                                                                # class for threading muon detector
    def __init__(self):                                                                     # initialization for threading
        Thread.__init__(self)
        self.daemon = True
        self.start()
    
    def run(self):                                                                          # run file for threading
        i=0																	                # counter for transmissions
        c=0																	                # variable for connection status
        while True:
            ########## MUON DETECTOR ##########
            if c==0:														                # if is it disconnected
                try:
                    for porti in range(0,100):                                              # looping through possible port names
                        portsave = '/dev/ttyUSB' + str(porti)                               # finding savefile location of port
                        if(os.path.exists(portsave))==True:                                 # loop until new savefile is made
                            break                                                           # breaking loop when port is found
                        else:
                            porti += 1 														# incrementing port file name

                    global det                                  
                    #					  USB	     bit/second		bits/byte     none      one stop bit for end of a byte of data
                    det = serial.Serial(portsave, baudrate=9600, bytesize=8, parity='N', stopbits=1) # open the serial port and configure its parameters
                    time.sleep(1) 											                # wait for 1 second for the serial port to stabilize

                    c = 1													                # variable for USB is connected
                    i = 0                                                                   # counter variable
                except:
                    print('MUON DETECTOR DISCONNECTED')
                    time.sleep(reconnect_time)											    # try to reconnect every x seconds

            if c==1:														                # is it connected, wait to recieve data
                try:
                    if det.inWaiting():												        # if there is data waiting in the serial port buffer, read it and decode it
                        if i<9:														        # for the first 9 transmissions
                            data = det.readline().decode().strip()					        # read input
                            det.write(str.encode('got-it'))							        # let serial communication port know transmission is recieved
                            i += 1													        # don't do anything with it
                            continue												        # do it again
                        data = det.readline().decode().strip()						        # read all muon hits
                        data_list = data.split()									        # split the data into string array
                        
                        ########## Time ##########
                        try:
                            rtc = adafruit_ds3231.DS3231(i2c)                               # extract rtc from i2c
                            t = rtc.datetime                                                # read time from rtc
                            Time_RTC = str(t.tm_hour)+':'+str(t.tm_min)+':'+str(t.tm_sec)
                            print("The time is {}:{:02}:{:02}".format(t.tm_hour, t.tm_min, t.tm_sec))
                        except:                                                             # if an error was thron
                            print('DS3231 DISCONNECTED')                                    # displaying ds3231 is disconnected
                            Time_RTC = float("NaN")                                         # setting to Nan value for unmeasured
                        try:
                            Time_CPU = time.strftime('%H:%M:%S', time.localtime())          # setting cpu time
                        except:
                            Time_CPU = float("NaN")                                         # if cpu time fails
                        data_list.insert(1, Time_RTC)										# add the RTC time to the data list
                        data_list.insert(1, Time_CPU)										# add the CPU time to the data list
                        csv_writer.writerow(data_list)								        # write the data to the CSV file
                        det.write(str.encode('got-it'))								        # let serial communication port know transmission is recieved
                        print("HIT!")
                except:
                    c = 0															        # variable for if USB is disconnected		

class CAMERA(Thread):                                                                       # class for threading camera
    def __init__(self):                                                                     # initialization for threading
        Thread.__init__(self)
        self.daemon = True
        self.start()

    def run(self):                                                                          # run file for threading
        videosave = "{}{:03d}".format(videoname,0)                                          # initalize final savefile for video
        videoi = 0                                                                          # initializing new video counter 
        while True:
            ########## RASPBERRY PI CAMERA V2.1 ##########
            try:
                while(os.path.exists(path_video + videosave + ".h264"))==True:              # loop until new savefile is made
                    videoi += 1                                                             # increment counter
                    videosave = "{}{:03d}".format(videoname,videoi)                         # initalize final savefile for video
                cam = PiCamera(resolution=(1920,1080), framerate=15)                        # initialize camera object with resolution and FPS
                cam.video_stabilization = True                                              # setting video stabilization (default: False)
                cam.start_recording(path_video+videosave+'.h264')                           # begin recording and save as a .h264 file

                ########## Time ##########
                try:
                    Timestamp = time.strftime('%H:%M:%S', time.localtime())+'_CPU'          # setting CPU time
                except:                                                                     # if an error was thrown
                    print('CPU DISCONNECTED')
                    try:    
                        rtc = adafruit_ds3231.DS3231(i2c)                                   # extract rtc from i2c
                        t = rtc.datetime                                                    # read time from rtc
                        Timestamp = str(t.tm_hour)+':'+str(t.tm_min)+':'+str(t.tm_sec)+'_RTC' # setting RTC time                    
                    except:
                        Timestamp = 'CPU_RTC_DISCONNECTED'                                  # tertiary time

                cam.annotate_text = Timestamp                                               # add time to video
                cam.wait_recording(recordtime_cam)                                          # defalt back to set record time
                cam.stop_recording()                                                        # stop recording
                cam.close()                                                                 # close connection
                print('VIDEO TAKEN')                                                        # dispaly successful interaction
            except Exception as e:
                print(e)
                print('VIDEO DISCONNECTED')
                time.sleep(reconnect_time)                                                  # sleep time for x seconds between trying to reconnecting
                videoi -= 1                                                                 # decrement counter to rewrite over failed savefile
                videosave = "{}{:03d}".format(videoname,videoi)                             # reset savefile name

########## Main Loop ##########
BME680()                                                                                    # call bme loop to run
LSM303AGR()                                                                                 # call lsm loop to run
MUON_DETECTOR()                                                                             # call muon detector to run
CAMERA()                                                                                    # call camera to run

try:
    while True:                                                                             
        pass                                                                                # to hold the program from ending
except:
    print('ending')                                                                         # final shutdown procedure